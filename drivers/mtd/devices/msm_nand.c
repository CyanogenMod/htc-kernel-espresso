/* drivers/mtd/devices/msm_nand.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

#include <asm/dma.h>
#include <asm/mach/flash.h>

#include <mach/dma.h>

#define MSM_NAND_BASE 0xA0A00000
#include "msm_nand.h"

#define MSM_NAND_DMA_BUFFER_SIZE SZ_4K
#define MSM_NAND_DMA_BUFFER_SLOTS \
	(MSM_NAND_DMA_BUFFER_SIZE / (sizeof(((atomic_t *)0)->counter) * 8))

#define NAND_CFG0_RAW 0xA80420C0
#define NAND_CFG1_RAW 0x5045D

#define SUPPORT_WRONG_ECC_CONFIG 1
#define IGNORE_ARM9_CONFIG       0
#define VERBOSE 0

static struct nand_hw_info *nand_info;
struct nand_hw_info {
	uint32_t flash_id;
	uint8_t maker_id;
	uint8_t maker_name[10];
	uint8_t width;
	uint32_t size;
	uint32_t block_count;
	uint32_t page_count;
	uint32_t page_size;
};

struct msm_nand_chip {
	struct device *dev;
	wait_queue_head_t wait_queue;
	atomic_t dma_buffer_busy;
	unsigned dma_channel;
	uint8_t *dma_buffer;
	dma_addr_t dma_addr;
	unsigned CFG0, CFG1;
#if SUPPORT_WRONG_ECC_CONFIG
	uint32_t ecc_buf_cfg;
	uint32_t saved_ecc_buf_cfg;
#endif
	struct nand_hw_info dev_info;
};

#define CFG1_WIDE_FLASH (1U << 1)

/* TODO: move datamover code out */

#define SRC_CRCI_NAND_CMD  CMD_SRC_CRCI(DMOV_NAND_CRCI_CMD)
#define DST_CRCI_NAND_CMD  CMD_DST_CRCI(DMOV_NAND_CRCI_CMD)
#define SRC_CRCI_NAND_DATA CMD_SRC_CRCI(DMOV_NAND_CRCI_DATA)
#define DST_CRCI_NAND_DATA CMD_DST_CRCI(DMOV_NAND_CRCI_DATA)

#define msm_virt_to_dma(chip, vaddr) \
	((void)(*(vaddr)), (chip)->dma_addr + \
	 ((uint8_t *)(vaddr) - (chip)->dma_buffer))

/**
 * msm_nand_oob_64 - oob info for large (2KB) page
 */
static struct nand_ecclayout msm_nand_oob_64 = {
	.eccbytes	= 40,
	.eccpos		= {
		0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
		10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
		46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
		},
	.oobavail	= 16,
	.oobfree	= {
		{30, 16},
	}
};

static void *msm_nand_get_dma_buffer(struct msm_nand_chip *chip, size_t size)
{
	unsigned int bitmask, free_bitmask, old_bitmask;
	unsigned int need_mask, current_need_mask;
	int free_index;

	need_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	bitmask = atomic_read(&chip->dma_buffer_busy);
	free_bitmask = ~bitmask;
	do {
		free_index = __ffs(free_bitmask);
		current_need_mask = need_mask << free_index;
		if ((bitmask & current_need_mask) == 0) {
			old_bitmask =
				atomic_cmpxchg(&chip->dma_buffer_busy,
					       bitmask,
					       bitmask | current_need_mask);
			if (old_bitmask == bitmask)
				return chip->dma_buffer +
					free_index * MSM_NAND_DMA_BUFFER_SLOTS;
			free_bitmask = 0; /* force return */
		}
		/* current free range was too small, clear all free bits */
		/* below the top busy bit within current_need_mask */
		free_bitmask &=
			~(~0U >> (32 - fls(bitmask & current_need_mask)));
	} while (free_bitmask);

	return NULL;
}

static void msm_nand_release_dma_buffer(struct msm_nand_chip *chip,
					void *buffer, size_t size)
{
	int index;
	unsigned int used_mask;

	used_mask = (1UL << DIV_ROUND_UP(size, MSM_NAND_DMA_BUFFER_SLOTS)) - 1;
	index = ((uint8_t *)buffer - chip->dma_buffer) /
		MSM_NAND_DMA_BUFFER_SLOTS;
	atomic_sub(used_mask << index, &chip->dma_buffer_busy);

	wake_up(&chip->wait_queue);
}

uint32_t flash_read_id(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		unsigned data[5];
	} *dma_buffer;
	uint32_t rv;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = 0 | 4;
	dma_buffer->data[1] = NAND_CMD_FETCH_ID;
	dma_buffer->data[2] = 1;
	dma_buffer->data[3] = 0xeeeeeeee;
	dma_buffer->data[4] = 0xeeeeeeee;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = 0 | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = NAND_FLASH_CHIP_SELECT;
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = DST_CRCI_NAND_CMD;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[1]);
	dma_buffer->cmd[1].dst = NAND_FLASH_CMD;
	dma_buffer->cmd[1].len = 4;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[2]);
	dma_buffer->cmd[2].dst = NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA;
	dma_buffer->cmd[3].src = NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[3]);
	dma_buffer->cmd[3].len = 4;

	dma_buffer->cmd[4].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[4].src = NAND_READ_ID;
	dma_buffer->cmd[4].dst = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[4].len = 4;
	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	pr_info("status: %x\n", dma_buffer->data[3]);
	pr_info("nandid: %x maker %02x device %02x\n",
		dma_buffer->data[4], dma_buffer->data[4] & 0xff,
		(dma_buffer->data[4] >> 8) & 0xff);
	rv = dma_buffer->data[4];
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
	return rv;
}

int flash_read_config(struct msm_nand_chip *chip)
{
	struct {
		dmov_s cmd[2];
		unsigned cmdptr;
		unsigned cfg0;
		unsigned cfg1;
	} *dma_buffer;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));
	dma_buffer->cfg0 = 0;
	dma_buffer->cfg1 = 0;

	dma_buffer->cmd[0].cmd = CMD_OCB;
	dma_buffer->cmd[0].src = NAND_DEV0_CFG0;
	dma_buffer->cmd[0].dst = msm_virt_to_dma(chip, &dma_buffer->cfg0);
	dma_buffer->cmd[0].len = 4;

	dma_buffer->cmd[1].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[1].src = NAND_DEV0_CFG1;
	dma_buffer->cmd[1].dst = msm_virt_to_dma(chip, &dma_buffer->cfg1);
	dma_buffer->cmd[1].len = 4;
	BUILD_BUG_ON(1 != ARRAY_SIZE(dma_buffer->cmd) - 1);

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	chip->CFG0 = dma_buffer->cfg0;
	chip->CFG1 = dma_buffer->cfg1;
	pr_info("read CFG0 = %x, CFG1 = %x\n", chip->CFG0, chip->CFG1);
	chip->CFG0 = (3 <<  6)  /* 4 codeword per page for 2k nand */
		|  (516 <<  9)  /* 516 user data bytes */
		|   (10 << 19)  /* 10 parity bytes */
		|    (5 << 27)  /* 5 address cycles */
		|    (1 << 30)  /* Read status before data */
		|    (1 << 31)  /* Send read cmd */
		/* 0 spare bytes for 16 bit nand or 1 spare bytes for 8 bit */
		| ((chip->CFG1 & CFG1_WIDE_FLASH) ? (0 << 23) : (1 << 23));
	chip->CFG1 = chip->CFG1
#if IGNORE_ARM9_CONFIG
		/* use ARM11 own setting on CFG1 */
		|    (7 <<  2)  /* 8 recovery cycles */
		|    (0 <<  5)  /* Allow CS deassertion */
		|    (2 << 17)  /* 6 cycle tWB/tRB */
#endif
		|  (465 <<  6)  /* Bad block marker location */
		| (chip->CFG1 & CFG1_WIDE_FLASH); /* preserve wide flag */
	chip->CFG1 = chip->CFG1
		&   ~(1 <<  0)  /* Enable ecc */
		&   ~(1 << 16); /* Bad block in user data area */
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if ((chip->CFG0 == 0) || (chip->CFG1 == 0))
		return -1;

	return 0;
}

unsigned flash_rd_reg(struct msm_nand_chip *chip, unsigned addr)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;
	unsigned rv;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = addr;
	dma_buffer->cmd.dst = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = 0xeeeeeeee;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();
	rv = dma_buffer->data;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	return rv;
}

void flash_wr_reg(struct msm_nand_chip *chip, unsigned addr, unsigned val)
{
	struct {
		dmov_s cmd;
		unsigned cmdptr;
		unsigned data;
	} *dma_buffer;

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->cmd.cmd = CMD_LC;
	dma_buffer->cmd.src = msm_virt_to_dma(chip, &dma_buffer->data);
	dma_buffer->cmd.dst = addr;
	dma_buffer->cmd.len = 4;

	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, &dma_buffer->cmd) >> 3) | CMD_PTR_LP;
	dma_buffer->data = val;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
}

static int msm_nand_read_oob(struct mtd_info *mtd, loff_t from,
			     struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;

	struct {
		dmov_s cmd[4 * 5 + 3];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
#if SUPPORT_WRONG_ECC_CONFIG
			uint32_t ecccfg;
			uint32_t ecccfg_restore;
#endif
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result[4];
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = from / 2048;
	uint32_t oob_len = ops->ooblen;
	uint32_t sectordatasize;
	uint32_t sectoroobsize;
	int err, pageerr, rawerr;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	uint32_t oob_col = 0;
	unsigned page_count;
	unsigned pages_read = 0;
	unsigned start_sector = 0;
	uint32_t ecc_errors;
	uint32_t total_ecc_errors = 0;

	if (from & (mtd->writesize - 1)) {
		pr_err("%s: unsupported from, 0x%llx\n",
		       __func__, from);
		return -EINVAL;
	}
	if (ops->datbuf != NULL && (ops->len % mtd->writesize) != 0) {
		/* when ops->datbuf is NULL, ops->len may refer to ooblen */
		pr_err("%s: unsupported ops->len, %d\n",
		       __func__, ops->len);
		return -EINVAL;
	}
	if (ops->ooblen != 0 && ops->ooboffs != 0) {
		pr_err("%s: unsupported ops->ooboffs, %d\n",
		       __func__, ops->ooboffs);
		return -EINVAL;
	}

	if (ops->oobbuf && !ops->datbuf && ops->mode == MTD_OOB_AUTO)
		start_sector = 3;

	if (ops->oobbuf && !ops->datbuf)
		page_count = ops->ooblen / ((ops->mode == MTD_OOB_AUTO) ?
			mtd->oobavail : mtd->oobsize);
	else
		page_count = ops->len / mtd->writesize;

#if 0 /* yaffs reads more oob data than it needs */
	if (ops->ooblen >= sectoroobsize * 4) {
		pr_err("%s: unsupported ops->ooblen, %d\n",
		       __func__, ops->ooblen);
		return -EINVAL;
	}
#endif

#if VERBOSE
	pr_info("msm_nand_read_oob %llx %p %x %p %x\n",
		from, ops->datbuf, ops->len, ops->oobbuf, ops->ooblen);
#endif
	if (ops->datbuf) {
		/* memset(ops->datbuf, 0x55, ops->len); */
		data_dma_addr_curr = data_dma_addr =
			dma_map_single(chip->dev, ops->datbuf, ops->len,
				       DMA_FROM_DEVICE);
		if (dma_mapping_error(chip->dev, data_dma_addr)) {
			pr_err("msm_nand_read_oob: failed to get dma addr "
			       "for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (ops->oobbuf) {
		memset(ops->oobbuf, 0xff, ops->ooblen);
		oob_dma_addr_curr = oob_dma_addr =
			dma_map_single(chip->dev, ops->oobbuf,
				       ops->ooblen, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(chip->dev, oob_dma_addr)) {
			pr_err("msm_nand_read_oob: failed to get dma addr "
			       "for %p\n", ops->oobbuf);
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}
	}

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	oob_col = start_sector * 0x210;
	if (chip->CFG1 & CFG1_WIDE_FLASH)
		oob_col >>= 1;

	err = 0;
	while (page_count-- > 0) {
		cmd = dma_buffer->cmd;

		/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		dma_buffer->data.cmd = NAND_CMD_PAGE_READ_ECC;
		dma_buffer->data.addr0 = (page << 16) | oob_col;
		/* qc example is (page >> 16) && 0xff !? */
		dma_buffer->data.addr1 = (page >> 16) & 0xff;
		/* flash0 + undoc bit */
		dma_buffer->data.chipsel = 0 | 4;


		dma_buffer->data.cfg0 =
			(chip->CFG0 & ~(7U << 6)) | ((3U - start_sector) << 6);
		dma_buffer->data.cfg1 = chip->CFG1;

		/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;


		BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data.result));

		for (n = start_sector; n < 4; n++) {
			/* flash + buffer status return words */
			dma_buffer->data.result[n].flash_status = 0xeeeeeeee;
			dma_buffer->data.result[n].buffer_status = 0xeeeeeeee;

			/* block on cmd ready, then
			 * write CMD / ADDR0 / ADDR1 / CHIPSEL
			 * regs in a burst
			 */
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = NAND_FLASH_CMD;
			if (n == start_sector)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == start_sector) {
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip,
							&dma_buffer->data.cfg0);
				cmd->dst = NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
#if SUPPORT_WRONG_ECC_CONFIG
				if (chip->saved_ecc_buf_cfg !=
				    chip->ecc_buf_cfg) {
					dma_buffer->data.ecccfg =
						chip->ecc_buf_cfg;
					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip,
						      &dma_buffer->data.ecccfg);
					cmd->dst = NAND_EBI2_ECC_BUF_CFG;
					cmd->len = 4;
					cmd++;
				}
#endif
			}

			/* kick the execute register */
			cmd->cmd = 0;
			cmd->src =
				msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

			/* block on data ready, then
			 * read the status register
			 */
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip,
						   &dma_buffer->data.result[n]);
			/* NAND_FLASH_STATUS + NAND_BUFFER_STATUS */
			cmd->len = 8;
			cmd++;

			/* read data block
			 * (only valid if status says success)
			 */
			if (ops->datbuf) {
				sectordatasize = (n < 3) ? 516 : 500;
				cmd->cmd = 0;
				cmd->src = NAND_FLASH_BUFFER;
				cmd->dst = data_dma_addr_curr;
				data_dma_addr_curr += sectordatasize;
				cmd->len = sectordatasize;
				cmd++;
			}

			if (ops->oobbuf &&
			    (n == 3 || ops->mode != MTD_OOB_AUTO)) {
				cmd->cmd = 0;
				if (n == 3) {
					cmd->src = NAND_FLASH_BUFFER + 500;
					sectoroobsize = 16;
					if (ops->mode != MTD_OOB_AUTO)
						sectoroobsize += 10;
				} else {
					cmd->src = NAND_FLASH_BUFFER + 516;
					sectoroobsize = 10;
				}

				cmd->dst = oob_dma_addr_curr;
				if (sectoroobsize < oob_len)
					cmd->len = sectoroobsize;
				else
					cmd->len = oob_len;
				oob_dma_addr_curr += cmd->len;
				oob_len -= cmd->len;
				if (cmd->len > 0)
					cmd++;
			}
		}
#if SUPPORT_WRONG_ECC_CONFIG
		if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) {
			dma_buffer->data.ecccfg_restore =
				chip->saved_ecc_buf_cfg;
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip,
					      &dma_buffer->data.ecccfg_restore);
			cmd->dst = NAND_EBI2_ECC_BUF_CFG;
			cmd->len = 4;
			cmd++;
		}
#endif

		BUILD_BUG_ON(4 * 5 + 3 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;

		dma_buffer->cmdptr =
			(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3)
			| CMD_PTR_LP;

        dsb();
		msm_dmov_exec_cmd(
			chip->dma_channel, DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(
				msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
        dsb();

		/* if any of the writes failed (0x10), or there
		 * was a protection violation (0x100), we lose
		 */
		pageerr = rawerr = 0;
		for (n = start_sector; n < 4; n++) {
			if (dma_buffer->data.result[n].flash_status & 0x110) {
				rawerr = -EIO;
				break;
			}
		}
		if (rawerr) {
			if (ops->datbuf) {
				uint8_t *datbuf =
					ops->datbuf + pages_read * 2048;

				dma_sync_single_for_cpu(chip->dev,
					data_dma_addr_curr-mtd->writesize,
					mtd->writesize, DMA_BIDIRECTIONAL);

				for (n = 0; n < 2048; n++) {
					/* empty blocks read 0x54 at
					 * these offsets
					 */
					if (n % 516 == 3 && datbuf[n] == 0x54)
						datbuf[n] = 0xff;
					if (datbuf[n] != 0xff) {
						pageerr = rawerr;
						break;
					}
				}

				dma_sync_single_for_device(chip->dev,
					data_dma_addr_curr-mtd->writesize,
					mtd->writesize, DMA_BIDIRECTIONAL);

			}
			if (ops->oobbuf) {
				for (n = 0; n < ops->ooblen; n++) {
					if (ops->oobbuf[n] != 0xff) {
						pageerr = rawerr;
						break;
					}
				}
			}
		}
		if (pageerr) {
			for (n = start_sector; n < 4; n++) {
				if (dma_buffer->data.result[n].buffer_status &
								0x8) {
					/* not thread safe */
					mtd->ecc_stats.failed++;
					pageerr = -EBADMSG;
					break;
				}
			}
		}
		if (!rawerr) { /* check for corretable errors */
			for (n = start_sector; n < 4; n++) {
				ecc_errors = dma_buffer->data.
					result[n].buffer_status & 0x7;
				if (ecc_errors) {
					total_ecc_errors += ecc_errors;
					/* not thread safe */
					mtd->ecc_stats.corrected += ecc_errors;
					if (ecc_errors > 1)
						pageerr = -EUCLEAN;
				}
			}
		}
		if (pageerr && (pageerr != -EUCLEAN || err == 0))
			err = pageerr;

#if VERBOSE
		if (rawerr && !pageerr) {
			pr_err("msm_nand_read_oob %llx %x %x empty page\n",
			       (loff_t)page * mtd->writesize, ops->len,
			       ops->ooblen);
		} else {
			pr_info("status: %x %x %x %x %x %x %x %x\n",
				dma_buffer->data.result[0].flash_status,
				dma_buffer->data.result[0].buffer_status,
				dma_buffer->data.result[1].flash_status,
				dma_buffer->data.result[1].buffer_status,
				dma_buffer->data.result[2].flash_status,
				dma_buffer->data.result[2].buffer_status,
				dma_buffer->data.result[3].flash_status,
				dma_buffer->data.result[3].buffer_status);
		}
#endif
		if (err && err != -EUCLEAN && err != -EBADMSG)
			break;
		pages_read++;
		page++;
	}
	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (ops->oobbuf) {
		dma_unmap_single(chip->dev, oob_dma_addr,
				 ops->ooblen, DMA_FROM_DEVICE);
	}
err_dma_map_oobbuf_failed:
	if (ops->datbuf) {
		dma_unmap_single(chip->dev, data_dma_addr,
				 ops->len, DMA_FROM_DEVICE);
	}

	ops->retlen = mtd->writesize * pages_read;
	ops->oobretlen = ops->ooblen - oob_len;
	if (err)
		pr_err("msm_nand_read_oob %llx %x %x failed %d, corrected %d\n",
		       from, ops->datbuf ? ops->len : 0, ops->ooblen, err,
		       total_ecc_errors);
	return err;
}

static int
msm_nand_read(struct mtd_info *mtd, loff_t from, size_t len,
	      size_t *retlen, u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

	/* printk("msm_nand_read %llx %x\n", from, len); */

	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_read_oob(mtd, from, &ops);
	*retlen = ops.retlen;
	return ret;
}

static int
msm_nand_write_oob(struct mtd_info *mtd, loff_t to, struct mtd_oob_ops *ops)
{
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[4 * 6 + 3];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
#if SUPPORT_WRONG_ECC_CONFIG
			uint32_t ecccfg;
			uint32_t ecccfg_restore;
#endif
			uint32_t flash_status[4];
			uint32_t zeroes;
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	unsigned n;
	unsigned page = to / 2048;
	uint32_t oob_len = ops->ooblen;
	uint32_t sectordatawritesize;
	int err;
	dma_addr_t data_dma_addr = 0;
	dma_addr_t oob_dma_addr = 0;
	dma_addr_t data_dma_addr_curr = 0;
	dma_addr_t oob_dma_addr_curr = 0;
	unsigned page_count;
	unsigned pages_written = 0;

	if (to & (mtd->writesize - 1)) {
		pr_err("%s: unsupported to, 0x%llx\n", __func__, to);
		return -EINVAL;
	}
	if (ops->ooblen != 0 && ops->mode != MTD_OOB_AUTO) {
		pr_err("%s: unsupported ops->mode, %d\n",
		       __func__, ops->mode);
		return -EINVAL;
	}

	if (ops->datbuf == NULL) {
		pr_err("%s: unsupported ops->datbuf == NULL\n", __func__);
		return -EINVAL;
	}
	if ((ops->len % mtd->writesize) != 0) {
		pr_err("%s: unsupported ops->len, %d\n",
		       __func__, ops->len);
		return -EINVAL;
	}
#if 0 /* yaffs writes more oob data than it needs */
	if (ops->ooblen >= sectoroobsize * 4) {
		pr_err("%s: unsupported ops->ooblen, %d\n",
		       __func__, ops->ooblen);
		return -EINVAL;
	}
#endif
	if (ops->ooblen != 0 && ops->ooboffs != 0) {
		pr_err("%s: unsupported ops->ooboffs, %d\n",
		       __func__, ops->ooboffs);
		return -EINVAL;
	}

	if (ops->datbuf) {
		data_dma_addr_curr = data_dma_addr =
			dma_map_single(chip->dev, ops->datbuf,
				       ops->len, DMA_TO_DEVICE);
		if (dma_mapping_error(chip->dev, data_dma_addr)) {
			pr_err("msm_nand_write_oob: failed to get dma addr "
			       "for %p\n", ops->datbuf);
			return -EIO;
		}
	}
	if (ops->oobbuf) {
		oob_dma_addr_curr = oob_dma_addr =
			dma_map_single(chip->dev, ops->oobbuf,
				       ops->ooblen, DMA_TO_DEVICE);
		if (dma_mapping_error(chip->dev, oob_dma_addr)) {
			pr_err("msm_nand_write_oob: failed to get dma addr "
			       "for %p\n", ops->oobbuf);
			err = -EIO;
			goto err_dma_map_oobbuf_failed;
		}
	}

	page_count = ops->len / mtd->writesize;

	wait_event(chip->wait_queue, (dma_buffer =
			msm_nand_get_dma_buffer(chip, sizeof(*dma_buffer))));

	while (page_count-- > 0) {
		cmd = dma_buffer->cmd;

			/* CMD / ADDR0 / ADDR1 / CHIPSEL program values */
		dma_buffer->data.cmd = NAND_CMD_PRG_PAGE;
		dma_buffer->data.addr0 = page << 16;
		dma_buffer->data.addr1 = (page >> 16) & 0xff;
		dma_buffer->data.chipsel = 0 | 4; /* flash0 + undoc bit */
		dma_buffer->data.zeroes = 0;

		dma_buffer->data.cfg0 = chip->CFG0;
		dma_buffer->data.cfg1 = chip->CFG1;

			/* GO bit for the EXEC register */
		dma_buffer->data.exec = 1;

		BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->data.flash_status));

		for (n = 0; n < 4; n++) {
			/* status return words */
			dma_buffer->data.flash_status[n] = 0xeeeeeeee;
			/* block on cmd ready, then
			 * write CMD / ADDR0 / ADDR1 / CHIPSEL regs in a burst
			 */
			cmd->cmd = DST_CRCI_NAND_CMD;
			cmd->src =
				msm_virt_to_dma(chip, &dma_buffer->data.cmd);
			cmd->dst = NAND_FLASH_CMD;
			if (n == 0)
				cmd->len = 16;
			else
				cmd->len = 4;
			cmd++;

			if (n == 0) {
				cmd->cmd = 0;
				cmd->src = msm_virt_to_dma(chip,
							&dma_buffer->data.cfg0);
				cmd->dst = NAND_DEV0_CFG0;
				cmd->len = 8;
				cmd++;
#if SUPPORT_WRONG_ECC_CONFIG
				if (chip->saved_ecc_buf_cfg !=
				    chip->ecc_buf_cfg) {
					dma_buffer->data.ecccfg =
						chip->ecc_buf_cfg;
					cmd->cmd = 0;
					cmd->src = msm_virt_to_dma(chip,
						      &dma_buffer->data.ecccfg);
					cmd->dst = NAND_EBI2_ECC_BUF_CFG;
					cmd->len = 4;
					cmd++;
				}
#endif
			}

				/* write data block */
			sectordatawritesize = (n < 3) ? 516 : 500;
			cmd->cmd = 0;
			cmd->src = data_dma_addr_curr;
			data_dma_addr_curr += sectordatawritesize;
			cmd->dst = NAND_FLASH_BUFFER;
			cmd->len = sectordatawritesize;
			cmd++;

			if (ops->oobbuf) {
				if (n == 3) {
					cmd->cmd = 0;
					cmd->src = oob_dma_addr_curr;
					cmd->dst = NAND_FLASH_BUFFER + 500;
					if (16 < oob_len)
						cmd->len = 16;
					else
						cmd->len = oob_len;
					oob_dma_addr_curr += cmd->len;
					oob_len -= cmd->len;
					if (cmd->len > 0)
						cmd++;
				}
				if (ops->mode != MTD_OOB_AUTO) {
					/* skip ecc bytes in oobbuf */
					if (oob_len < 10) {
						oob_dma_addr_curr += 10;
						oob_len -= 10;
					} else {
						oob_dma_addr_curr += oob_len;
						oob_len = 0;
					}
				}
			}

			/* kick the execute register */
			cmd->cmd = 0;
			cmd->src =
				msm_virt_to_dma(chip, &dma_buffer->data.exec);
			cmd->dst = NAND_EXEC_CMD;
			cmd->len = 4;
			cmd++;

			/* block on data ready, then
			 * read the status register
			 */
			cmd->cmd = SRC_CRCI_NAND_DATA;
			cmd->src = NAND_FLASH_STATUS;
			cmd->dst = msm_virt_to_dma(chip,
					     &dma_buffer->data.flash_status[n]);
			cmd->len = 4;
			cmd++;

			/* clear the status register in case the OP_ERR is set
			 * due to the irite, to work around a h/w bug */
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip,
						   &dma_buffer->data.zeroes);
			cmd->dst = NAND_FLASH_STATUS;
			cmd->len = 4;
			cmd++;
		}
#if SUPPORT_WRONG_ECC_CONFIG
		if (chip->saved_ecc_buf_cfg != chip->ecc_buf_cfg) {
			dma_buffer->data.ecccfg_restore =
				chip->saved_ecc_buf_cfg;
			cmd->cmd = 0;
			cmd->src = msm_virt_to_dma(chip,
					      &dma_buffer->data.ecccfg_restore);
			cmd->dst = NAND_EBI2_ECC_BUF_CFG;
			cmd->len = 4;
			cmd++;
		}
#endif
		dma_buffer->cmd[0].cmd |= CMD_OCB;
		cmd[-1].cmd |= CMD_OCU | CMD_LC;
		BUILD_BUG_ON(4 * 6 + 3 != ARRAY_SIZE(dma_buffer->cmd));
		BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
		dma_buffer->cmdptr =
			(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) |
			CMD_PTR_LP;

        dsb();
		msm_dmov_exec_cmd(chip->dma_channel,
			DMOV_CMD_PTR_LIST | DMOV_CMD_ADDR(
				msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
        dsb();

		/* if any of the writes failed (0x10), or there was a
		 * protection violation (0x100), or the program success
		 * bit (0x80) is unset, we lose
		 */
		err = 0;
		for (n = 0; n < 4; n++) {
			if (dma_buffer->data.flash_status[n] & 0x110) {
				if (dma_buffer->data.flash_status[n] & 0x10)
					pr_err("msm_nand: critical write error,"
					       " 0x%x(%d)\n", page, n);
				err = -EIO;
				break;
			}
			if (!(dma_buffer->data.flash_status[n] & 0x80)) {
				err = -EIO;
				break;
			}
		}

#if VERBOSE
		pr_info("write page %d: status: %x %x %x %x\n", page,
			dma_buffer->data.flash_status[0],
			dma_buffer->data.flash_status[1],
			dma_buffer->data.flash_status[2],
			dma_buffer->data.flash_status[3]);
#endif
		if (err)
			break;
		pages_written++;
		page++;
	}
	ops->retlen = mtd->writesize * pages_written;
	ops->oobretlen = ops->ooblen - oob_len;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));

	if (ops->oobbuf)
		dma_unmap_single(chip->dev, oob_dma_addr,
				 ops->ooblen, DMA_TO_DEVICE);
err_dma_map_oobbuf_failed:
	if (ops->datbuf)
		dma_unmap_single(chip->dev, data_dma_addr, 2048, DMA_TO_DEVICE);
	if (err)
		pr_err("msm_nand_write_oob %llx %x %x failed %d\n",
		       to, ops->len, ops->ooblen, err);
	return err;
}

static int msm_nand_write(struct mtd_info *mtd, loff_t to, size_t len,
			  size_t *retlen, const u_char *buf)
{
	int ret;
	struct mtd_oob_ops ops;

	ops.mode = MTD_OOB_PLACE;
	ops.len = len;
	ops.retlen = 0;
	ops.ooblen = 0;
	ops.datbuf = (uint8_t *)buf;
	ops.oobbuf = NULL;
	ret =  msm_nand_write_oob(mtd, to, &ops);
	*retlen = ops.retlen;
	return ret;
}

static int
msm_nand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	int err;
	struct msm_nand_chip *chip = mtd->priv;
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		unsigned data[9];
	} *dma_buffer;
	unsigned page = instr->addr / 2048;

	if (instr->addr & (mtd->erasesize - 1)) {
		pr_err("%s: unsupported erase address, 0x%llx\n",
		       __func__, instr->addr);
		return -EINVAL;
	}
	if (instr->len != mtd->erasesize) {
		pr_err("%s: unsupported erase len, %lld\n",
		       __func__, instr->len);
		return -EINVAL;
	}

	wait_event(chip->wait_queue,
		   (dma_buffer = msm_nand_get_dma_buffer(
			    chip, sizeof(*dma_buffer))));

	dma_buffer->data[0] = NAND_CMD_BLOCK_ERASE;
	dma_buffer->data[1] = page;
	dma_buffer->data[2] = 0;
	dma_buffer->data[3] = 0 | 4;
	dma_buffer->data[4] = 1;
	dma_buffer->data[5] = 0xeeeeeeee;
	dma_buffer->data[6] = chip->CFG0 & (~(7 << 6));  /* CW_PER_PAGE = 0 */
	dma_buffer->data[7] = chip->CFG1;
	dma_buffer->data[8] = 0;
	BUILD_BUG_ON(8 != ARRAY_SIZE(dma_buffer->data) - 1);

	dma_buffer->cmd[0].cmd = DST_CRCI_NAND_CMD | CMD_OCB;
	dma_buffer->cmd[0].src = msm_virt_to_dma(chip, &dma_buffer->data[0]);
	dma_buffer->cmd[0].dst = NAND_FLASH_CMD;
	dma_buffer->cmd[0].len = 16;

	dma_buffer->cmd[1].cmd = 0;
	dma_buffer->cmd[1].src = msm_virt_to_dma(chip, &dma_buffer->data[6]);
	dma_buffer->cmd[1].dst = NAND_DEV0_CFG0;
	dma_buffer->cmd[1].len = 8;

	dma_buffer->cmd[2].cmd = 0;
	dma_buffer->cmd[2].src = msm_virt_to_dma(chip, &dma_buffer->data[4]);
	dma_buffer->cmd[2].dst = NAND_EXEC_CMD;
	dma_buffer->cmd[2].len = 4;

	dma_buffer->cmd[3].cmd = SRC_CRCI_NAND_DATA;
	dma_buffer->cmd[3].src = NAND_FLASH_STATUS;
	dma_buffer->cmd[3].dst = msm_virt_to_dma(chip, &dma_buffer->data[5]);
	dma_buffer->cmd[3].len = 4;

	/* clear the status register in case the OP_ERR is set
	 * due to the write, to work around a h/w bug */
	dma_buffer->cmd[4].cmd = CMD_OCU | CMD_LC;
	dma_buffer->cmd[4].src = msm_virt_to_dma(chip, &dma_buffer->data[8]);
	dma_buffer->cmd[4].dst = NAND_FLASH_STATUS;
	dma_buffer->cmd[4].len = 4;

	BUILD_BUG_ON(4 != ARRAY_SIZE(dma_buffer->cmd) - 1);
	dma_buffer->cmdptr =
		(msm_virt_to_dma(chip, dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(
		chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	/* we fail if there was an operation error, a mpu error, or the
	 * erase success bit was not set.
	 */

	if (dma_buffer->data[5] & 0x110 || !(dma_buffer->data[5] & 0x80)) {
		if (dma_buffer->data[5] & 0x10)
			pr_warning("msm_nand: critical erase error, 0x%llx\n",
				   instr->addr);
		err = -EIO;
	} else
		err = 0;

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer));
	if (err) {
		pr_err("%s: erase failed, 0x%llx\n", __func__, instr->addr);
		instr->fail_addr = instr->addr;
		instr->state = MTD_ERASE_FAILED;
	} else {
		instr->state = MTD_ERASE_DONE;
		instr->fail_addr = 0xffffffff;
		mtd_erase_callback(instr);
	}
	return err;
}

static int
msm_nand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
	struct msm_nand_chip *chip = mtd->priv;
	int ret;
	struct {
		dmov_s cmd[5];
		unsigned cmdptr;
		struct {
			uint32_t cmd;
			uint32_t addr0;
			uint32_t addr1;
			uint32_t chipsel;
			uint32_t cfg0;
			uint32_t cfg1;
			uint32_t exec;
			uint32_t ecccfg;
			struct {
				uint32_t flash_status;
				uint32_t buffer_status;
			} result;
		} data;
	} *dma_buffer;
	dmov_s *cmd;
	uint8_t *buf;
	unsigned page = 0;
	unsigned cwperpage;

	if (mtd->writesize == 2048)
		page = ofs >> 11;

	if (mtd->writesize == 4096)
		page = ofs >> 12;

	cwperpage = (mtd->writesize >> 9);

	/* Check for invalid offset */
	if (ofs > mtd->size)
		return -EINVAL;
	if (ofs & (mtd->erasesize - 1)) {
		pr_err("%s: unsupported block address, 0x%x\n",
			 __func__, (uint32_t)ofs);
		return -EINVAL;
	}

	wait_event(chip->wait_queue,
		(dma_buffer = msm_nand_get_dma_buffer(chip ,
					 sizeof(*dma_buffer) + 4)));
	buf = (uint8_t *)dma_buffer + sizeof(*dma_buffer);

	/* Read 4 bytes starting from the bad block marker location
	 * in the last code word of the page
	 */

	cmd = dma_buffer->cmd;

	dma_buffer->data.cmd = NAND_CMD_PAGE_READ;
	dma_buffer->data.cfg0 = NAND_CFG0_RAW & ~(7U << 6);
	dma_buffer->data.cfg1 = NAND_CFG1_RAW | (chip->CFG1 & CFG1_WIDE_FLASH);

	if (chip->CFG1 & CFG1_WIDE_FLASH)
		dma_buffer->data.addr0 = (page << 16) |
			((528*(cwperpage-1)) >> 1);
	else
		dma_buffer->data.addr0 = (page << 16) |
			(528*(cwperpage-1));

	dma_buffer->data.addr1 = (page >> 16) & 0xff;
	dma_buffer->data.chipsel = 0 | 4;

	dma_buffer->data.exec = 1;

	dma_buffer->data.result.flash_status = 0xeeeeeeee;
	dma_buffer->data.result.buffer_status = 0xeeeeeeee;

	cmd->cmd = DST_CRCI_NAND_CMD;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cmd);
	cmd->dst = NAND_FLASH_CMD;
	cmd->len = 16;
	cmd++;

	cmd->cmd = 0;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.cfg0);
	cmd->dst = NAND_DEV0_CFG0;
	cmd->len = 8;
	cmd++;

	cmd->cmd = 0;
	cmd->src = msm_virt_to_dma(chip, &dma_buffer->data.exec);
	cmd->dst = NAND_EXEC_CMD;
	cmd->len = 4;
	cmd++;

	cmd->cmd = SRC_CRCI_NAND_DATA;
	cmd->src = NAND_FLASH_STATUS;
	cmd->dst = msm_virt_to_dma(chip, &dma_buffer->data.result);
	cmd->len = 8;
	cmd++;

	cmd->cmd = 0;
	cmd->src = NAND_FLASH_BUFFER + (mtd->writesize - (528*(cwperpage-1)));
	cmd->dst = msm_virt_to_dma(chip, buf);
	cmd->len = 4;
	cmd++;

	BUILD_BUG_ON(5 != ARRAY_SIZE(dma_buffer->cmd));
	BUG_ON(cmd - dma_buffer->cmd > ARRAY_SIZE(dma_buffer->cmd));
	dma_buffer->cmd[0].cmd |= CMD_OCB;
	cmd[-1].cmd |= CMD_OCU | CMD_LC;

	dma_buffer->cmdptr = (msm_virt_to_dma(chip,
				dma_buffer->cmd) >> 3) | CMD_PTR_LP;

	dsb();
	msm_dmov_exec_cmd(chip->dma_channel, DMOV_CMD_PTR_LIST |
		DMOV_CMD_ADDR(msm_virt_to_dma(chip, &dma_buffer->cmdptr)));
	dsb();

	ret = 0;
	if (dma_buffer->data.result.flash_status & 0x110)
		ret = -EIO;

	if (!ret) {
		/* Check for bad block marker byte */
		if (chip->CFG1 & CFG1_WIDE_FLASH) {
			if (buf[0] != 0xFF || buf[1] != 0xFF)
				ret = 1;
		} else {
			if (buf[0] != 0xFF)
				ret = 1;
		}
	}

	msm_nand_release_dma_buffer(chip, dma_buffer, sizeof(*dma_buffer) + 4);
	return ret;
}


static int
msm_nand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
	/* struct msm_nand_chip *this = mtd->priv; */
	int ret;

	ret = msm_nand_block_isbad(mtd, ofs);
	if (ret) {
		/* If it was bad already, return success and do nothing */
		if (ret > 0)
			return 0;
		return ret;
	}

	return -EIO;
}

/**
 * msm_nand_suspend - [MTD Interface] Suspend the msm_nand flash
 * @param mtd		MTD device structure
 */
static int msm_nand_suspend(struct mtd_info *mtd)
{
	return 0;
}

/**
 * msm_nand_resume - [MTD Interface] Resume the msm_nand flash
 * @param mtd		MTD device structure
 */
static void msm_nand_resume(struct mtd_info *mtd)
{
}

/*
 * Export two attributes for HTC SSD HW INFO tool
 */
static int param_get_vendor_name(char *buffer, struct kernel_param *kp)
{
	return sprintf(buffer, "%s", nand_info->maker_name);
}
module_param_call(vendor, NULL, param_get_vendor_name, NULL, S_IRUGO);

static int param_get_nand_info(char *buffer, struct kernel_param *kp)
{
	int result = 0;
	result += sprintf(buffer, "<<  NAND INFO  >>\n");
	result += sprintf(buffer + result, "flash id\t =%X\n",
				nand_info->flash_id);
	result += sprintf(buffer + result, "vendor\t\t =%s\n",
				nand_info->maker_name);
	result += sprintf(buffer + result, "width\t\t =%d bits\n",
				nand_info->width);
	result += sprintf(buffer + result, "size\t\t =%d MB\n",
				nand_info->size>>20);
	result += sprintf(buffer + result, "block count\t =%d\n",
				nand_info->block_count);
	result += sprintf(buffer + result, "page count\t =%d",
				nand_info->page_count);
	return result;
}
module_param_call(info, NULL, param_get_nand_info, NULL, S_IRUGO);

/**
 * msm_nand_scan - [msm_nand Interface] Scan for the msm_nand device
 * @param mtd		MTD device structure
 * @param maxchips	Number of chips to scan for
 *
 * This fills out all the not initialized function pointers
 * with the defaults.
 * The flash ID is read and the mtd/chip structures are
 * filled with the appropriate values.
 */
int msm_nand_scan(struct mtd_info *mtd, int maxchips)
{
	unsigned n;
	struct msm_nand_chip *chip = mtd->priv;
	uint32_t flash_id;


	if (flash_read_config(chip)) {
		pr_err("ERRROR: could not save CFG0 & CFG1 state\n");
		return -ENODEV;
	}
	pr_info("CFG0 = %x, CFG1 = %x\n", chip->CFG0, chip->CFG1);
	pr_info("CFG0: cw/page=%d ud_sz=%d ecc_sz=%d spare_sz=%d "
		"num_addr_cycles=%d\n", (chip->CFG0 >> 6) & 7,
		(chip->CFG0 >> 9) & 0x3ff, (chip->CFG0 >> 19) & 15,
		(chip->CFG0 >> 23) & 15, (chip->CFG0 >> 27) & 7);

	pr_info("NAND_READ_ID = %x\n", flash_rd_reg(chip, NAND_READ_ID));
	flash_wr_reg(chip, NAND_READ_ID, 0x12345678);

	flash_id = flash_read_id(chip);

	n = flash_rd_reg(chip, NAND_DEV0_CFG0);
	pr_info("CFG0: cw/page=%d ud_sz=%d ecc_sz=%d spare_sz=%d\n",
		(n >> 6) & 7, (n >> 9) & 0x3ff, (n >> 19) & 15,
		(n >> 23) & 15);

	n = flash_rd_reg(chip, NAND_DEV_CMD1);
	pr_info("DEV_CMD1: %x\n", n);

	n = flash_rd_reg(chip, NAND_EBI2_ECC_BUF_CFG);
	pr_info(KERN_INFO "NAND_EBI2_ECC_BUF_CFG: %x\n", n);

#if SUPPORT_WRONG_ECC_CONFIG
	chip->ecc_buf_cfg = 0x203;
	chip->saved_ecc_buf_cfg = n;
#endif

	if ((flash_id & 0xffff) == 0xaaec)	/* 2Gbit Samsung chip */
		mtd->size = 256 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xbaad)	/* 2Gbit Hynix chip */
		mtd->size = 256 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xba2c)	/* 2Gbit Micron chip */
		mtd->size = 256 << 20; 		/* * num_chips */

	if ((flash_id & 0xffff) == 0xacec) 	/* 4Gbit Samsung chip */
		mtd->size = 512 << 20; 		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xbcec)	/* 4Gbit Samsung chip */
		mtd->size = 512 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xbcad)	/* 4Gbit Hynix chip */
		mtd->size = 512 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xbc2c)	/* 4Gbit Micron chip */
		mtd->size = 512 << 20;		/* * num_chips */

	if ((flash_id & 0xffff) == 0xb3ec)	/* 8Gbit Samsung chip */
		mtd->size = 1024 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xb3ad)	/* 8Gbit Hynix chip */
		mtd->size = 1024 << 20;		/* * num_chips */
	else if ((flash_id & 0xffff) == 0xb32c)	/* 8Gbit Micron chip */
		mtd->size = 1024 << 20;		/* * num_chips */

	pr_info("flash_id: %x size %llx\n", flash_id, mtd->size);

	mtd->writesize = 2048;
	mtd->oobsize = msm_nand_oob_64.eccbytes + msm_nand_oob_64.oobavail;
	mtd->oobavail = msm_nand_oob_64.oobavail;
	mtd->erasesize = mtd->writesize << 6; /* TODO: check */
	mtd->ecclayout = &msm_nand_oob_64;

	/* Fill in remaining MTD driver data */
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;
	/* mtd->ecctype = MTD_ECC_SW; */
	mtd->erase = msm_nand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = msm_nand_read;
	mtd->write = msm_nand_write;
	mtd->read_oob = msm_nand_read_oob;
	mtd->write_oob = msm_nand_write_oob;
	/* mtd->sync = msm_nand_sync; */
	mtd->lock = NULL;
	/* mtd->unlock = msm_nand_unlock; */
	mtd->suspend = msm_nand_suspend;
	mtd->resume = msm_nand_resume;
	mtd->block_isbad = msm_nand_block_isbad;
	mtd->block_markbad = msm_nand_block_markbad;
	mtd->owner = THIS_MODULE;

	/* Information provides to HTC SSD HW Info tool */
	nand_info = &chip->dev_info;
	nand_info->flash_id = flash_id;
	nand_info->maker_id = (flash_id & 0xff);
	switch(nand_info->maker_id) {
	case 0xec:
		strcpy(nand_info->maker_name, "Samsung");
		break;
	case 0xad:
		strcpy(nand_info->maker_name, "Hynix");
		break;
	case 0x2c:
		strcpy(nand_info->maker_name, "Micron");
		break;
	default:
		strcpy(nand_info->maker_name, "Unknown");
		break;
	}
	nand_info->width = ((chip->CFG1 & CFG1_WIDE_FLASH) ? 16 : 8);
	nand_info->size = mtd->size;
	nand_info->page_size = 2048;
	nand_info->page_count = 64;
	nand_info->block_count = mtd->size;
	do_div(nand_info->block_count, nand_info->page_size * nand_info->page_count);
	/* Unlock whole block */
	/* msm_nand_unlock_all(mtd); */

	/* return this->scan_bbt(mtd); */
	return 0;
}
EXPORT_SYMBOL_GPL(msm_nand_scan);

/**
 * msm_nand_release - [msm_nand Interface] Free resources held by the msm_nand device
 * @param mtd		MTD device structure
 */
void msm_nand_release(struct mtd_info *mtd)
{
	/* struct msm_nand_chip *this = mtd->priv; */

#ifdef CONFIG_MTD_PARTITIONS
	/* Deregister partitions */
	del_mtd_partitions(mtd);
#endif
	/* Deregister the device */
	del_mtd_device(mtd);
}
EXPORT_SYMBOL_GPL(msm_nand_release);

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL,  };
#endif

struct msm_nand_info {
	struct mtd_info		mtd;
	struct mtd_partition	*parts;
	struct msm_nand_chip	msm_nand;
};

static int __devinit msm_nand_probe(struct platform_device *pdev)
{
	struct msm_nand_info *info;
	struct flash_platform_data *pdata = pdev->dev.platform_data;
	int err;

	if (pdev->num_resources != 1) {
		pr_err("invalid num_resources");
		return -ENODEV;
	}
	if (pdev->resource[0].flags != IORESOURCE_DMA) {
		pr_err("invalid resource type");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct msm_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->msm_nand.dev = &pdev->dev;

	init_waitqueue_head(&info->msm_nand.wait_queue);

	info->msm_nand.dma_channel = pdev->resource[0].start;
	/* this currently fails if dev is passed in */
	info->msm_nand.dma_buffer =
		dma_alloc_coherent(/*dev*/ NULL, MSM_NAND_DMA_BUFFER_SIZE,
				   &info->msm_nand.dma_addr, GFP_KERNEL);
	if (info->msm_nand.dma_buffer == NULL) {
		err = -ENOMEM;
		goto out_free_info;
	}

	pr_info("allocated dma buffer at %p, dma_addr %x\n",
		info->msm_nand.dma_buffer, info->msm_nand.dma_addr);

	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->msm_nand;
	info->mtd.owner = THIS_MODULE;

	if (msm_nand_scan(&info->mtd, 1)) {
		err = -ENXIO;
		goto out_free_dma_buffer;
	}

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (err <= 0 && pdata && pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		err = add_mtd_device(&info->mtd);

	dev_set_drvdata(&pdev->dev, info);

	return 0;

out_free_dma_buffer:
	dma_free_coherent(/*dev*/ NULL, SZ_4K, info->msm_nand.dma_buffer,
			  info->msm_nand.dma_addr);
out_free_info:
	kfree(info);

	return err;
}

static int __devexit msm_nand_remove(struct platform_device *pdev)
{
	struct msm_nand_info *info = dev_get_drvdata(&pdev->dev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (info) {
#ifdef CONFIG_MTD_PARTITIONS
		if (info->parts)
			del_mtd_partitions(&info->mtd);
		else
#endif
			del_mtd_device(&info->mtd);

		msm_nand_release(&info->mtd);
		dma_free_coherent(/*dev*/ NULL, SZ_4K,
				  info->msm_nand.dma_buffer,
				  info->msm_nand.dma_addr);
		kfree(info);
	}

	return 0;
}

#define DRIVER_NAME "msm_nand"

static struct platform_driver msm_nand_driver = {
	.probe		= msm_nand_probe,
	.remove		= __devexit_p(msm_nand_remove),
	.driver = {
		.name		= DRIVER_NAME,
	}
};

MODULE_ALIAS(DRIVER_NAME);

static int __init msm_nand_init(void)
{
	return platform_driver_register(&msm_nand_driver);
}

static void __exit msm_nand_exit(void)
{
	platform_driver_unregister(&msm_nand_driver);
}

module_init(msm_nand_init);
module_exit(msm_nand_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("msm_nand flash driver code");
