/* arch/arm/mach-msm/include/mach/msm_iomap.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
 *
 * The MSM peripherals are spread all over across 768MB of physical
 * space, which makes just having a simple IO_ADDRESS macro to slide
 * them into the right virtual location rough.  Instead, we will
 * provide a master phys->virt mapping for peripherals here.
 *
 */

#ifndef __ASM_ARCH_MSM_IOMAP_H
#define __ASM_ARCH_MSM_IOMAP_H

#include <asm/sizes.h>

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * MSM_VIC_BASE must be an value that can be loaded via a "mov"
 * instruction, otherwise entry-macro.S will not compile.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */

#ifdef __ASSEMBLY__
#define IOMEM(x)	x
#else
#define IOMEM(x)	((void __force __iomem *)(x))
#endif

#if defined(CONFIG_ARCH_MSM_ARM11)
#define _PHYS(a11,scorp) (a11)
#endif
#if defined(CONFIG_ARCH_MSM_SCORPION)
#define _PHYS(a11,scorp) (scorp)
#endif

#define MSM_VIC_BASE          IOMEM(0xF8000000)
#define MSM_VIC_PHYS          _PHYS(0xC0000000,0xAC000000)
#define MSM_VIC_SIZE          SZ_4K

#define MSM_CSR_BASE          IOMEM(0xF8001000)
#define MSM_CSR_PHYS          _PHYS(0xC0100000,0xAC100000)
#define MSM_CSR_SIZE          SZ_4K

#define MSM_GPT_PHYS          MSM_CSR_PHYS
#define MSM_GPT_BASE          MSM_CSR_BASE
#define MSM_GPT_SIZE          SZ_4K

#define MSM_DMOV_BASE         IOMEM(0xF8002000)
#define MSM_DMOV_PHYS         0xA9700000
#define MSM_DMOV_SIZE         SZ_4K

#define MSM_GPIO1_BASE        IOMEM(0xF8003000)
#define MSM_GPIO1_PHYS        _PHYS(0xA9200000,0xA9000000)
#define MSM_GPIO1_SIZE        SZ_4K

#define MSM_GPIO2_BASE        IOMEM(0xF8004000)
#define MSM_GPIO2_PHYS        _PHYS(0xA9300000,0xA9100000)
#define MSM_GPIO2_SIZE        SZ_4K

#define MSM_CLK_CTL_BASE      IOMEM(0xF8005000)
#define MSM_CLK_CTL_PHYS      0xA8600000
#define MSM_CLK_CTL_SIZE      SZ_4K

#define MSM_SHARED_RAM_BASE   IOMEM(0xF8100000)
#if defined(CONFIG_ARCH_QSD8X50)
#define MSM_SHARED_RAM_PHYS   0x00100000
#elif defined(CONFIG_ARCH_MSM7225)
#define MSM_SHARED_RAM_PHYS   0x00800000
#elif defined(CONFIG_ARCH_MSM7227)
#define MSM_SHARED_RAM_PHYS   0x00100000
#else
#define MSM_SHARED_RAM_PHYS   0x01F00000
#endif
#define MSM_SHARED_RAM_SIZE   SZ_1M

#define MSM_UART1_PHYS        0xA9A00000
#define MSM_UART1_SIZE        SZ_4K

#define MSM_UART2_PHYS        0xA9B00000
#define MSM_UART2_SIZE        SZ_4K

#define MSM_UART3_PHYS        0xA9C00000
#define MSM_UART3_SIZE        SZ_4K

#ifdef CONFIG_MSM_DEBUG_UART
#define MSM_DEBUG_UART_BASE   0xF9000000
#if CONFIG_MSM_DEBUG_UART == 1
#define MSM_DEBUG_UART_PHYS   MSM_UART1_PHYS
#elif CONFIG_MSM_DEBUG_UART == 2
#define MSM_DEBUG_UART_PHYS   MSM_UART2_PHYS
#elif CONFIG_MSM_DEBUG_UART == 3
#define MSM_DEBUG_UART_PHYS   MSM_UART3_PHYS
#endif
#define MSM_DEBUG_UART_SIZE   SZ_4K
#endif

#define MSM_I2C_PHYS          0xA9900000
#define MSM_I2C_SIZE          SZ_4K

#define MSM_HSUSB_PHYS        0xA0800000
#define MSM_HSUSB_SIZE        SZ_4K

#define MSM_PMDH_PHYS         0xAA600000
#define MSM_PMDH_SIZE         SZ_4K

#define MSM_EMDH_PHYS         0xAA700000
#define MSM_EMDH_SIZE         SZ_4K

#define MSM_MDP_PHYS          0xAA200000
#define MSM_MDP_SIZE          0x000F0000

#define MSM_MDC_BASE	      IOMEM(0xF8200000)
#define MSM_MDC_PHYS	      0xAA500000
#define MSM_MDC_SIZE	      SZ_1M

#define MSM_AD5_BASE          IOMEM(0xF8300000)
#define MSM_AD5_PHYS          0xAC000000
#define MSM_AD5_SIZE          (SZ_1M*13)

#define MSM_VFE_PHYS          0xA0F00000
#define MSM_VFE_SIZE          SZ_1M

#define MSM_UART1DM_PHYS      0xA0200000
#define MSM_UART2DM_PHYS      0xA0300000

#if defined(CONFIG_ARCH_MSM7225)
#define MSM_SSBI_BASE         IOMEM(0xF800A000)
#define MSM_SSBI_PHYS         0xA8100000
#define MSM_SSBI_SIZE         SZ_4K

#define MSM_TSSC_BASE         IOMEM(0xF800B000)
#define MSM_TSSC_PHYS         0xAA300000
#define MSM_TSSC_SIZE         SZ_4K
#endif

#define MSM_SDC1_PHYS         _PHYS(0xA0400000,0xA0300000)
#define MSM_SDC1_SIZE         SZ_4K
#define MSM_SDC2_BASE         IOMEM(0xF800C000)
#define MSM_SDC2_PHYS         _PHYS(0xA0500000,0xA0400000)
#define MSM_SDC2_SIZE         SZ_4K
#define MSM_SDC3_PHYS         _PHYS(0xA0600000,0xA0500000)
#define MSM_SDC3_SIZE         SZ_4K
#define MSM_SDC4_PHYS         _PHYS(0xA0700000,0xA0600000)
#define MSM_SDC4_SIZE         SZ_4K

#if defined(CONFIG_ARCH_MSM_SCORPION)
#define MSM_SIRC_BASE         IOMEM(0xF8006000)
#define MSM_SIRC_PHYS         0xAC200000
#define MSM_SIRC_SIZE         SZ_4K

#define MSM_SCPLL_BASE        IOMEM(0xF8007000)
#define MSM_SCPLL_PHYS        0xA8800000
#define MSM_SCPLL_SIZE        SZ_4K
#endif

#if defined(CONFIG_CACHE_L2X0)
#define MSM_L2CC_BASE         IOMEM(0xF8008000)
#define MSM_L2CC_PHYS         0xC0400000
#define MSM_L2CC_SIZE         SZ_4K
#endif

#if defined(CONFIG_ARCH_MSM7227)
#define MSM_TGPIO1_BASE         IOMEM(0xF8009000)
#define MSM_TGPIO1_PHYS         0xA9000000
#define MSM_TGPIO1_SIZE         SZ_4K
#endif

#if defined(CONFIG_ARCH_QSD8X50) || defined(CONFIG_ARCH_MSM7227)
#define MSM_GPU_REG_PHYS      0xA0000000
#define MSM_GPU_REG_SIZE      0x00020000
#endif

#if defined(CONFIG_ARCH_QSD8X50)
#define MSM_SPI_PHYS          0xA1200000
#define MSM_SPI_SIZE          SZ_4K
#endif

#if defined(CONFIG_ARCH_MSM_SCORPION)
#define MSM_TCSR_BASE  IOMEM(0xF8008000)
#define MSM_TCSR_PHYS  0xA8700000
#define MSM_TCSR_SIZE  SZ_4K
#endif

#endif
