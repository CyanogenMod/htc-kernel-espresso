/**
 * op_model_v7.c
 * ARM V7 (Cortex A8) Event Monitor Driver
 *
 * Copyright 2008 Jean Pihet <jpihet@mvista.com>
 * Copyright 2004 ARM SMP Development Team
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/oprofile.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/smp.h>

#include "op_counter.h"
#include "op_arm_model.h"
#include "op_model_v7.h"

/* #define DEBUG */


/*
 * ARM V7 PMNC support
 */
struct scorpEv {
	u32 evType;
	u32 val;
	u8 grp;
	u32 evTypeAct;
} scEv[] =
			{{0x4f, 0x83000000, 0, 0x4f},
			{0x4e, 0x80030000, 0, 0x4e},
			{0x50, 0x80000009, 1, 0x50},
			{0x51, 0x80000900, 1, 0x51},
			{0x52, 0x80070000, 1, 0x52},
			{0x53, 0x87000000, 1, 0x53},
			{0x60, 0x800b0000, 1, 0x51},
			{0x54, 0x80000003, 2, 0x54},
			{0x55, 0x80000300, 2, 0x55},
			{0x57, 0x83000000, 2, 0x57},
			{0x56, 0x80080000, 2, 0x56},
			{0x61, 0x80000008, 2, 0x54},
			{0x62, 0x80000800, 2, 0x55},
			{0x63, 0x88000000, 2, 0x57},
			{0x64, 0x8000000e, 2, 0x56},
			{0x65, 0x800e0000, 2, 0x56},
			{0x66, 0x8e000000, 2, 0x57},
			{0x67, 0x800f0000, 2, 0x56},
			{0x58, 0x80000001, 3, 0x58},
			{0x59, 0x80000100, 3, 0x59},
			{0x5a, 0x80010000, 3, 0x5a},
			{0x5b, 0x81000000, 3, 0x5b},
			{0x68, 0x80040000, 3, 0x5a},
			{0x69, 0x84000000, 3, 0x5b},
			{0x6a, 0x80000200, 2, 0x55},
			{0x6b, 0x81000000, 2, 0x57},
			{0x6c, 0x80010000, 0, 0x4f},
			{0x6d, 0x81000000, 0, 0x4e}
				};

#define maxEv 28
unsigned int getEvCode(unsigned int evType)
{
	u32 i;
	for (i = 0; i < maxEv; i++) {
		if (scEv[i].evType == evType)
			return scEv[i].val;
	}
	return 0;
}

unsigned int getEvTypeAct(unsigned int evType)
{
	u32 i;
	for (i = 0; i < maxEv; i++) {
		if (scEv[i].evType == evType)
			return scEv[i].evTypeAct;
	}
	return 0;
}

unsigned int getEvGrp(unsigned int evType)
{
	u32 i;
	for (i = 0; i < maxEv; i++) {
		if (scEv[i].evType == evType)
			return scEv[i].grp;
	}
	return 5;
}


static u32 cnt_en[CNTMAX];

static inline void armv7_pmnc_write(u32 val)
{
	val &= PMNC_MASK;
	asm volatile("mcr p15, 0, %0, c9, c12, 0" : : "r" (val));
}

static inline u32 armv7_pmnc_read(void)
{
	u32 val;

	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r" (val));
	return val;
}

static inline u32 armv7_pmnc_enable_counter(unsigned int cnt)
{
	u32 val;

	if (cnt >= CNTMAX) {
		printk(KERN_ERR "oprofile: CPU%u enabling wrong PMNC counter"
			" %d\n", smp_processor_id(), cnt);
		return -1;
	}

	if (cnt == CCNT)
		val = CNTENS_C;
	else
		val = (1 << (cnt - CNT0));

	val &= CNTENS_MASK;
	asm volatile("mcr p15, 0, %0, c9, c12, 1" : : "r" (val));

	return cnt;
}

static inline u32 armv7_pmnc_disable_counter(unsigned int cnt)
{
	u32 val;

	if (cnt >= CNTMAX) {
		printk(KERN_ERR "oprofile: CPU%u disabling wrong PMNC counter"
			" %d\n", smp_processor_id(), cnt);
		return -1;
	}

	if (cnt == CCNT)
		val = CNTENC_C;
	else
		val = (1 << (cnt - CNT0));

	val &= CNTENC_MASK;
	asm volatile("mcr p15, 0, %0, c9, c12, 2" : : "r" (val));

	return cnt;
}

static inline u32 armv7_pmnc_enable_intens(unsigned int cnt)
{
	u32 val;

	if (cnt >= CNTMAX) {
		printk(KERN_ERR "oprofile: CPU%u enabling wrong PMNC counter"
			" interrupt enable %d\n", smp_processor_id(), cnt);
		return -1;
	}

	if (cnt == CCNT)
		val = INTENS_C;
	else
		val = (1 << (cnt - CNT0));

	val &= INTENS_MASK;
	asm volatile("mcr p15, 0, %0, c9, c14, 1" : : "r" (val));

	return cnt;
}

static inline u32 armv7_pmnc_getreset_flags(void)
{
	u32 val;

	/* Read */
	asm volatile("mrc p15, 0, %0, c9, c12, 3" : "=r" (val));

	/* Write to clear flags */
	val &= FLAG_MASK;
	asm volatile("mcr p15, 0, %0, c9, c12, 3" : : "r" (val));

	return val;
}

static inline int armv7_pmnc_select_counter(unsigned int cnt)
{
	u32 val;

	if ((cnt == CCNT) || (cnt >= CNTMAX)) {
		printk(KERN_ERR "oprofile: CPU%u selecting wrong PMNC counteri"
			" %d\n", smp_processor_id(), cnt);
		return -1;
	}

	val = (cnt - CNT0) & SELECT_MASK;
	asm volatile("mcr p15, 0, %0, c9, c12, 5" : : "r" (val));

	return cnt;
}

static inline void armv7_pmnc_write_evtsel(unsigned int cnt, u32 val)
{
	if (armv7_pmnc_select_counter(cnt) == cnt) {
		val &= EVTSEL_MASK;
		asm volatile("mcr p15, 0, %0, c9, c13, 1" : : "r" (val));
	}
}

static void armv7_pmnc_reset_counter(unsigned int cnt)
{
	u32 cpu_cnt = CPU_COUNTER(smp_processor_id(), cnt);
	u32 val = -(u32)counter_config[cpu_cnt].count;

	switch (cnt) {
	case CCNT:
		armv7_pmnc_disable_counter(cnt);

		asm volatile("mcr p15, 0, %0, c9, c13, 0" : : "r" (val));

		if (cnt_en[cnt] != 0)
		    armv7_pmnc_enable_counter(cnt);

		break;

	case CNT0:
	case CNT1:
	case CNT2:
	case CNT3:
		armv7_pmnc_disable_counter(cnt);

		if (armv7_pmnc_select_counter(cnt) == cnt)
		    asm volatile("mcr p15, 0, %0, c9, c13, 2" : : "r" (val));

		if (cnt_en[cnt] != 0)
		    armv7_pmnc_enable_counter(cnt);

		break;

	default:
		printk(KERN_ERR "oprofile: CPU%u resetting wrong PMNC counter"
			" %d\n", smp_processor_id(), cnt);
		break;
	}
}

int armv7_setup_pmnc(void)
{
	unsigned int cnt;
	u32 val = 0;
	u32 gr;
	u32 lpm2val, lpm0val;
	u32 lpm1val, l2pmval;

	lpm2val = 0;
	lpm0val = 0;
	lpm1val = 0;
	l2pmval = 0;

	if (armv7_pmnc_read() & PMNC_E) {
		printk(KERN_ERR "oprofile: CPU%u PMNC still enabled when setup"
			" new event counter.\n", smp_processor_id());
		return -EBUSY;
	}

	/*
	 * Initialize & Reset PMNC: C bit, D bit and P bit.
	 *  Note: Using a slower count for CCNT (D bit: divide by 64) results
	 *   in a more stable system
	 */
	armv7_pmnc_write(PMNC_P | PMNC_C | PMNC_D);


	for (cnt = CCNT; cnt < CNTMAX; cnt++) {
		unsigned long event;
		u32 cpu_cnt = CPU_COUNTER(smp_processor_id(), cnt);

		/*
		 * Disable counter
		 */
		armv7_pmnc_disable_counter(cnt);
		cnt_en[cnt] = 0;

		if (!counter_config[cpu_cnt].enabled)
			continue;

		event = counter_config[cpu_cnt].event & 255;
		if (event >= 0x40)
			event = getEvTypeAct(event);

		/*
		 * Set event (if destined for PMNx counters)
		 * We don't need to set the event if it's a cycle count
		 */
		if (cnt != CCNT) {
			armv7_pmnc_write_evtsel(cnt, event);
			switch (cnt) {
			case 1:
				val = 0x0dc7;
				break;
			case 2:
				val = 0x500dc7;
				break;
			case 3:
				val = 0xa00dc7;
				break;
			case 4:
				val = 0xf00dc7;
				break;
			default:
				printk(KERN_ERR "oprofile: Wrong PMNC "
						"counter %d\n", cnt);
				break;
			}
		}
		if (event >= 0x40) {
			asm volatile("mcr p15, 0, %0, c9, c15, 0" : :
						"r" (val));
			val = getEvCode(event);
			gr = getEvGrp(event);
			switch (gr) {
			case 0:
				lpm0val = lpm0val | val;
				val = lpm0val;
				asm volatile("mcr p15, 0, %0, c15, c0, 0" : :
						"r" (val));
				break;
			case 1:
				lpm1val = lpm1val | val;
				val = lpm1val;
				asm volatile("mcr p15, 1, %0, c15, c0, 0" : :
						"r" (val));
				break;
			case 2:
				lpm2val = lpm2val | val;
				val = lpm2val;
				asm volatile("mcr p15, 2, %0, c15, c0, 0" : :
						"r" (val));
				break;
			case 3:
				l2pmval = l2pmval | val;
				val = l2pmval;
				asm volatile("mcr p15, 3, %0, c15, c2, 0" : :
						"r" (val));
				break;

			}
		}
		/*
		 * Enable interrupt for this counter
		 */
		armv7_pmnc_enable_intens(cnt);

		/*
		 * Reset counter
		 */
		armv7_pmnc_reset_counter(cnt);

		/*
		 * Enable counter
		 */
		armv7_pmnc_enable_counter(cnt);
		cnt_en[cnt] = 1;
	}

	return 0;
}

static inline void armv7_start_pmnc(void)
{
	armv7_pmnc_write(armv7_pmnc_read() | PMNC_E);
}

static inline void armv7_stop_pmnc(void)
{
	armv7_pmnc_write(armv7_pmnc_read() & ~PMNC_E);
}

/*
 * CPU counters' IRQ handler (one IRQ per CPU)
 */
static irqreturn_t armv7_pmnc_interrupt(int irq, void *arg)
{
	struct pt_regs *regs = get_irq_regs();
	unsigned int cnt;
	u32 flags;


	/*
	 * Stop IRQ generation
	 */
	armv7_stop_pmnc();

	/*
	 * Get and reset overflow status flags
	 */
	flags = armv7_pmnc_getreset_flags();

	/*
	 * Cycle counter
	 */
	if (flags & FLAG_C) {
		u32 cpu_cnt = CPU_COUNTER(smp_processor_id(), CCNT);
		armv7_pmnc_reset_counter(CCNT);
		oprofile_add_sample(regs, cpu_cnt);
	}

	/*
	 * PMNC counters 0:3
	 */
	for (cnt = CNT0; cnt < CNTMAX; cnt++) {
		if (flags & (1 << (cnt - CNT0))) {
			u32 cpu_cnt = CPU_COUNTER(smp_processor_id(), cnt);
			armv7_pmnc_reset_counter(cnt);
			oprofile_add_sample(regs, cpu_cnt);
		}
	}

	/*
	 * Allow IRQ generation
	 */
	armv7_start_pmnc();

	return IRQ_HANDLED;
}

int armv7_request_interrupts(int *irqs, int nr)
{
	unsigned int i;
	int ret = 0;

	for (i = 0; i < nr; i++) {
		ret = request_irq(irqs[i], armv7_pmnc_interrupt,
				IRQF_DISABLED, "CP15 PMNC", NULL);
		if (ret != 0) {
			printk(KERN_ERR "oprofile: unable to request IRQ%u"
				" for ARMv7\n",
			       irqs[i]);
			break;
		}
	}

	if (i != nr)
		while (i-- != 0)
			free_irq(irqs[i], NULL);

	return ret;
}

void armv7_release_interrupts(int *irqs, int nr)
{
	unsigned int i;

	for (i = 0; i < nr; i++)
		free_irq(irqs[i], NULL);
}

#ifdef DEBUG
static void armv7_pmnc_dump_regs(void)
{
	u32 val;
	unsigned int cnt;

	printk(KERN_INFO "PMNC registers dump:\n");

	asm volatile("mrc p15, 0, %0, c9, c12, 0" : "=r" (val));
	printk(KERN_INFO "PMNC  =0x%08x\n", val);

	asm volatile("mrc p15, 0, %0, c9, c12, 1" : "=r" (val));
	printk(KERN_INFO "CNTENS=0x%08x\n", val);

	asm volatile("mrc p15, 0, %0, c9, c14, 1" : "=r" (val));
	printk(KERN_INFO "INTENS=0x%08x\n", val);

	asm volatile("mrc p15, 0, %0, c9, c12, 3" : "=r" (val));
	printk(KERN_INFO "FLAGS =0x%08x\n", val);

	asm volatile("mrc p15, 0, %0, c9, c12, 5" : "=r" (val));
	printk(KERN_INFO "SELECT=0x%08x\n", val);

	asm volatile("mrc p15, 0, %0, c9, c13, 0" : "=r" (val));
	printk(KERN_INFO "CCNT  =0x%08x\n", val);

	for (cnt = CNT0; cnt < CNTMAX; cnt++) {
		armv7_pmnc_select_counter(cnt);
		asm volatile("mrc p15, 0, %0, c9, c13, 2" : "=r" (val));
		printk(KERN_INFO "CNT[%d] count =0x%08x\n", cnt-CNT0, val);
		asm volatile("mrc p15, 0, %0, c9, c13, 1" : "=r" (val));
		printk(KERN_INFO "CNT[%d] evtsel=0x%08x\n", cnt-CNT0, val);
	}
}
#endif


static int irqs[] = {
#ifdef CONFIG_ARCH_OMAP3
	INT_34XX_BENCH_MPU_EMUL,
#endif
#ifdef CONFIG_ARCH_MSM_SCORPION
	INT_ARM11_PM,
#endif
};

static void armv7_pmnc_stop(void)
{
#ifdef DEBUG
	armv7_pmnc_dump_regs();
#endif
	armv7_stop_pmnc();
	armv7_release_interrupts(irqs, ARRAY_SIZE(irqs));
}

static int armv7_pmnc_start(void)
{
	int ret;

#ifdef DEBUG
	armv7_pmnc_dump_regs();
#endif
	ret = armv7_request_interrupts(irqs, ARRAY_SIZE(irqs));
	if (ret >= 0)
		armv7_start_pmnc();

	return ret;
}

static int armv7_detect_pmnc(void)
{
	return 0;
}

struct op_arm_model_spec op_armv7_spec = {
	.init		= armv7_detect_pmnc,
	.num_counters	= 5,
	.setup_ctrs	= armv7_setup_pmnc,
	.start		= armv7_pmnc_start,
	.stop		= armv7_pmnc_stop,
	.name		= "arm/armv7",
};
