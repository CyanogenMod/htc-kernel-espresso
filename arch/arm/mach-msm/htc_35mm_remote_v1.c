/*
 *  H2W device detection driver.
 *
 *  Copyright (C) 2007 Laurence Chen <Laurence_Chen@htc.com>
 *  Copyright (C) 2008 Nick Pelly <npelly@google.com>
 *  Copyright (C) 2008 Thomas Tsai <thomas_tsai@htc.com>
 *  Copyright (C) 2008 Farmer Tseng <farmer_tseng@htc.com>
 *  Copyright (C) 2009 Aaron Lin <aaron.hc_lin@htc.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

/*  For detecting HTC 2 Wire devices, such as wired headset.

    Logically, the H2W driver is always present, and H2W state (hi->state)
    indicates what is currently plugged into the H2W interface.

    When the headset is plugged in, CABLE_IN1 is pulled low. When the headset
    button is pressed, CABLE_IN2 is pulled low. These two lines are shared with
    the TX and RX (respectively) of UART3 - used for serial debugging.

    This headset driver keeps the CPLD configured as UART3 for as long as
    possible, so that we can do serial FIQ debugging even when the kernel is
    locked and this driver no longer runs. So it only configures the CPLD to
    GPIO while the headset is plugged in, and for 10ms during detection work.

    Unfortunately we can't leave the CPLD as UART3 while a headset is plugged
    in, UART3 is pullup on TX but the headset is pull-down, causing a 55 mA
    drain on trout.

    The headset detection work involves setting CPLD to GPIO, and then pulling
    CABLE_IN1 high with a stronger pullup than usual. A H2W headset will still
    pull this line low, whereas other attachments such as a serial console
    would get pulled up by this stronger pullup.

    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/h2w/state to be updated.

    Button presses are interpreted as input event (KEY_MEDIA). Button presses
    are ignored if the headset is plugged in, so the buttons on 11 pin -> 3.5mm
    jack adapters do not work until a headset is plugged into the adapter. This
    is to avoid serial RX traffic causing spurious button press events.

    We tend to check the status of CABLE_IN1 a few more times than strictly
    necessary during headset detection, to avoid spurious headset insertion
    events caused by serial debugger TX traffic.
*/


#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <mach/h2w_v1.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include <mach/microp_i2c.h>
#include <mach/drv_callback.h>
#include <linux/jiffies.h>

#ifdef CONFIG_HTC_AUDIOJACK_V1
#include <mach/audio_jack.h>
#endif

/* #define CONFIG_DEBUG_H2W */

#define HTC_35MM_UNPLUG 0
#define HTC_35MM_NO_MIC 1
#define HTC_35MM_MIC 2

/*Delay 200ms when 11pin device plug in*/
#define H2W_DETECT_DELAY	msecs_to_jiffies(200)
#define H2W_NO_DELAY	msecs_to_jiffies(0)

#define BUTTON_H2W_DELAY	msecs_to_jiffies(10)

#define H2WI(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#define H2WE(fmt, arg...) \
	printk(KERN_ERR "[H2W] %s " fmt "\r\n", __func__, ## arg)

#ifdef CONFIG_DEBUG_H2W
#define H2W_DBG(fmt, arg...) \
	printk(KERN_INFO "[H2W] %s " fmt "\r\n", __func__, ## arg)
#else
#define H2W_DBG(fmt, arg...) do {} while (0)
#endif

#define DOCK_STATE_UNDOCKED	0
#define DOCK_STATE_DESK		(1 << 0)
#define DOCK_STATE_CAR		(1 << 1)

#define DEVICE_ACCESSORY_ATTR(_name, _mode, _show, _store) \
struct device_attribute dev_attr_##_name = __ATTR(flag, _mode, _show, _store)

static struct delayed_work init_work;

static struct workqueue_struct *detect_wq;

static void detect_h2w_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(detect_h2w_work, detect_h2w_do_work);

static void insert_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(insert_35mm_work, insert_35mm_do_work);
static void remove_35mm_do_work(struct work_struct *work);
static DECLARE_WORK(remove_35mm_work, remove_35mm_do_work);

static struct workqueue_struct *button_wq;

static void button_35mm_do_work(struct work_struct *w);
static DECLARE_WORK(button_35mm_work, button_35mm_do_work);
static void button_h2w_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(button_h2w_work, button_h2w_do_work);

static int is_11pin_35mm_stable(void);
static void set_11pin_35mm_stable(int stable);

struct h2w_info {
	struct switch_dev sdev;
	struct class *htc_accessory_class;
	struct device *tty_dev;
	struct device *fm_dev;
	struct device *mic_dev;
	struct device *mute_dev;
	struct device *phonein_dev;
	struct mutex mutex_lock;
	struct mutex mutex_rc_lock;
	int tty_enable_flag;
	int fm_flag;
	int mic_switch_flag;
	int rc_flag;

	int is_11pin_35mm_stable;

	unsigned int irq;
	unsigned int irq_btn;

	int h2w_power;
	int cable_in1;
	int cable_in2;
	int h2w_clk;
	int h2w_data;
	int debug_uart;

	void (*configure) (int);
	void (*defconfig) (void);

	void (*set_dat)(int);
	void (*set_clk)(int);
	void (*set_dat_dir)(int);
	void (*set_clk_dir)(int);
	int (*get_dat)(void);
	int (*get_clk)(void);

	int h2w_dev_type;
	int is_wake_lock_ready;

	H2W_INFO h2w_info;
	H2W_SPEED speed;

	spinlock_t spin_lock;

	struct wake_lock headset_wake_lock;
};

struct headset_35mm_info {
	struct switch_dev sdev;
	struct input_dev *input;
	struct work_struct btn_35mm_work;
	struct mutex mutex_lock;

	int key_level_flag;
	atomic_t btn_state;
	int ignore_btn;
	int ext_35mm_status;
	int h2w_35mm_status;
	int is_ext_insert;
	int mic_bias_state;
	int ext_mic_sel;
	unsigned long insert_jiffies;
};

static struct h2w_info *hi;
static struct headset_35mm_info *h35i;

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "Headset\n");
}

static ssize_t car_dock_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "dock\n");
}

static void button_pressed(int type)
{
	printk(KERN_INFO "[H2W] button_pressed %d\n", type);
	atomic_set(&h35i->btn_state, type);
	input_report_key(h35i->input, type, 1);
	input_sync(h35i->input);
}

static void button_released(int type)
{
	printk(KERN_INFO "[H2W] button_released %d\n", type);
	atomic_set(&h35i->btn_state, 0);
	input_report_key(h35i->input, type, 0);
	input_sync(h35i->input);
}

static void enable_h2w_irq(void)
{
	unsigned long irq_flags;
	int ret;

	local_irq_save(irq_flags);
	enable_irq(hi->irq_btn);
	enable_irq(hi->irq);
	ret = set_irq_wake(hi->irq, 1);
	ret = set_irq_wake(hi->irq_btn, 1);
	local_irq_restore(irq_flags);
}

static void disable_h2w_irq(void)
{
	unsigned long irq_flags;
	int ret;

	local_irq_save(irq_flags);
	disable_irq(hi->irq_btn);
	disable_irq(hi->irq);
	ret = set_irq_wake(hi->irq, 0);
	ret = set_irq_wake(hi->irq_btn, 0);
	local_irq_restore(irq_flags);
}

static int h2w_irq_enable(void *argu)
{
	int *enable = (int *) argu;
	if (*enable)
		enable_h2w_irq();
	else
		disable_h2w_irq();
	return 1;
}

/*****************
 * H2W proctocol *
 *****************/
static inline void h2w_begin_command(void)
{
	mutex_lock(&hi->mutex_rc_lock);
	/* Disable H2W interrupt */
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_HIGH);
	disable_irq(hi->irq);
	disable_irq(hi->irq_btn);

	/* Set H2W_CLK as output low */
	hi->set_clk(0);
	hi->set_clk_dir(1);
}

static inline void h2w_end_command(void)
{
	/* Set H2W_CLK as input */
	hi->set_clk_dir(0);

	/* Enable H2W interrupt */
	enable_irq(hi->irq);
	enable_irq(hi->irq_btn);
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
	mutex_unlock(&hi->mutex_rc_lock);
}

static inline void one_clock_write(unsigned short flag)
{
	if (flag)
		hi->set_dat(1);
	else
		hi->set_dat(0);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(hi->speed);
	hi->set_clk(0);
}

static inline void one_clock_write_RWbit(unsigned short flag)
{
	if (flag)
		hi->set_dat(1);
	else
		hi->set_dat(0);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(hi->speed);
	hi->set_clk(0);
	hi->set_dat_dir(0);
	udelay(hi->speed);
}

static inline void h2w_reset(void)
{
	/* Set H2W_DAT as output low */
	hi->set_dat(0);
	hi->set_dat_dir(1);

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(4 * hi->speed);
	hi->set_dat(1);
	udelay(hi->speed);
	hi->set_dat(0);
	udelay(hi->speed);
	hi->set_clk(0);
	udelay(hi->speed);
}

static inline void h2w_start(void)
{
	udelay(hi->speed);
	hi->set_clk(1);
	udelay(2 * hi->speed);
	hi->set_clk(0);
	udelay(hi->speed);
}

static inline int h2w_ack(void)
{
	int retry_times = 0;

ack_resend:
	if (retry_times == MAX_ACK_RESEND_TIMES)
		return -1;

	udelay(hi->speed);
	hi->set_clk(1);
	udelay(2 * hi->speed);

	if (!hi->get_dat()) {
		retry_times++;
		hi->set_clk(0);
		udelay(hi->speed);
		goto ack_resend;
	}

	hi->set_clk(0);
	udelay(hi->speed);
	return 0;
}

static unsigned char h2w_readc(void)
{
	unsigned char h2w_read_data = 0x0;
	int index;

	for (index = 0; index < 8; index++) {
		hi->set_clk(0);
		udelay(hi->speed);
		hi->set_clk(1);
		udelay(hi->speed);
		if (hi->get_dat())
			h2w_read_data |= (1 << (7 - index));
	}
	hi->set_clk(0);
	udelay(hi->speed);

	return h2w_read_data;
}

static int h2w_readc_cmd(H2W_ADDR address)
{
	int ret = -1, retry_times = 0;
	unsigned char read_data;

read_resend:
	if (retry_times == MAX_HOST_RESEND_TIMES)
		goto err_read;

	h2w_reset();
	h2w_start();
	/* Write address */
	one_clock_write(address & 0x1000);
	one_clock_write(address & 0x0800);
	one_clock_write(address & 0x0400);
	one_clock_write(address & 0x0200);
	one_clock_write(address & 0x0100);
	one_clock_write(address & 0x0080);
	one_clock_write(address & 0x0040);
	one_clock_write(address & 0x0020);
	one_clock_write(address & 0x0010);
	one_clock_write(address & 0x0008);
	one_clock_write(address & 0x0004);
	one_clock_write(address & 0x0002);
	one_clock_write(address & 0x0001);
	one_clock_write_RWbit(1);
	if (h2w_ack() < 0) {
		H2W_DBG("Addr NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto read_resend;
	}

	read_data = h2w_readc();

	if (h2w_ack() < 0) {
		H2W_DBG("Data NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto read_resend;
	}
	ret = (int)read_data;

err_read:
	if (ret < 0)
		H2WE("NO ACK.\n");

	return ret;
}

static int h2w_writec_cmd(H2W_ADDR address, unsigned char data)
{
	int ret = -1;
	int retry_times = 0;

write_resend:
	if (retry_times == MAX_HOST_RESEND_TIMES)
		goto err_write;

	h2w_reset();
	h2w_start();

	/* Write address */
	one_clock_write(address & 0x1000);
	one_clock_write(address & 0x0800);
	one_clock_write(address & 0x0400);
	one_clock_write(address & 0x0200);
	one_clock_write(address & 0x0100);
	one_clock_write(address & 0x0080);
	one_clock_write(address & 0x0040);
	one_clock_write(address & 0x0020);
	one_clock_write(address & 0x0010);
	one_clock_write(address & 0x0008);
	one_clock_write(address & 0x0004);
	one_clock_write(address & 0x0002);
	one_clock_write(address & 0x0001);
	one_clock_write_RWbit(0);
	if (h2w_ack() < 0) {
		H2W_DBG("Addr NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto write_resend;
	}

	/* Write data */
	hi->set_dat_dir(1);
	one_clock_write(data & 0x0080);
	one_clock_write(data & 0x0040);
	one_clock_write(data & 0x0020);
	one_clock_write(data & 0x0010);
	one_clock_write(data & 0x0008);
	one_clock_write(data & 0x0004);
	one_clock_write(data & 0x0002);
	one_clock_write_RWbit(data & 0x0001);
	if (h2w_ack() < 0) {
		H2W_DBG("Data NO ACK(%d).\n", retry_times);
		retry_times++;
		hi->set_clk(0);
		mdelay(RESEND_DELAY);
		goto write_resend;
	}
	ret = 0;

err_write:
	if (ret < 0)
		H2WE("NO ACK.\n");

	return ret;
}

static int h2w_enable_phone_in(int enable)
{
	int ret = 0;

	h2w_begin_command();
	if (enable)
		ret = h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_AUDIO_IN |
				     H2W_ASCR_ACT_EN | H2W_ASCR_PHONE_IN);
	else
		ret = h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_AUDIO_IN |
				     H2W_ASCR_ACT_EN);

	udelay(10);
	h2w_end_command();

	return ret;
}

static int h2w_enable_mute_light(int enable)
{
	int ret = 0;

	h2w_begin_command();
	if (enable)
		ret = h2w_writec_cmd(H2W_LEDCT0, H2W_LED_MTL);
	else
		ret = h2w_writec_cmd(H2W_LEDCT0, H2W_LED_OFF);

	udelay(10);
	h2w_end_command();

	return ret;
}

static int h2w_get_fnkey(void)
{
	int ret;
	h2w_begin_command();
	ret = h2w_readc_cmd(H2W_FNKEY_UPDOWN);
	h2w_end_command();
	return ret;
}

static int h2w_dev_init(H2W_INFO *ph2w_info)
{
	int ret = -1;
	unsigned char ascr0 = 0;
	int h2w_sys = 0, maxgpadd = 0, maxadd = 0, key = 0;

	hi->speed = H2W_50KHz;
	h2w_begin_command();

	/* read H2W_SYSTEM */
	h2w_sys = h2w_readc_cmd(H2W_SYSTEM);
	if (h2w_sys == -1) {
		H2WE("read H2W_SYSTEM(0x0000) failed.\n");
		goto err_plugin;
	}
	ph2w_info->ACC_CLASS = (h2w_sys & 0x03);
	ph2w_info->AUDIO_DEVICE  = (h2w_sys & 0x04) > 0 ? 1 : 0;
	ph2w_info->HW_REV = (h2w_sys & 0x18) >> 3;
	ph2w_info->SLEEP_PR  = (h2w_sys & 0x20) >> 5;
	ph2w_info->CLK_SP = (h2w_sys & 0xC0) >> 6;

	/* enter init mode */
	if (h2w_writec_cmd(H2W_ASCR0, H2W_ASCR_DEVICE_INI) < 0) {
		H2WE("write H2W_ASCR0(0x0002) failed.\n");
		goto err_plugin;
	}
	udelay(10);

	/* read H2W_MAX_GP_ADD */
	maxgpadd = h2w_readc_cmd(H2W_MAX_GP_ADD);
	if (maxgpadd == -1) {
		H2WE("write H2W_MAX_GP_ADD(0x0001) failed.\n");
		goto err_plugin;
	}
	ph2w_info->CLK_SP += (maxgpadd & 0x60) >> 3;
	ph2w_info->MAX_GP_ADD = (maxgpadd & 0x1F);

	/* read key group */
	if (ph2w_info->MAX_GP_ADD >= 1) {
		ph2w_info->KEY_MAXADD = h2w_readc_cmd(H2W_KEY_MAXADD);
		if (ph2w_info->KEY_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->KEY_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_ASCII_DOWN);
			if (key < 0)
				goto err_plugin;
			ph2w_info->ASCII_DOWN = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 2) {
			key = h2w_readc_cmd(H2W_ASCII_UP);
			if (key == -1)
				goto err_plugin;
			ph2w_info->ASCII_UP = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 3) {
			key = h2w_readc_cmd(H2W_FNKEY_UPDOWN);
			if (key == -1)
				goto err_plugin;
			ph2w_info->FNKEY_UPDOWN = (key == 0xFF) ? 1 : 0;
		}
		if (ph2w_info->KEY_MAXADD >= 4) {
			key = h2w_readc_cmd(H2W_KD_STATUS);
			if (key == -1)
				goto err_plugin;
			ph2w_info->KD_STATUS = (key == 0x01) ? 1 : 0;
		}
	}

	/* read led group */
	if (ph2w_info->MAX_GP_ADD >= 2) {
		ph2w_info->LED_MAXADD = h2w_readc_cmd(H2W_LED_MAXADD);
		if (ph2w_info->LED_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->LED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_LEDCT0);
			if (key == -1)
				goto err_plugin;
			ph2w_info->LEDCT0 = (key == 0x02) ? 1 : 0;
		}
	}

	/* read group 3, 4, 5 */
	if (ph2w_info->MAX_GP_ADD >= 3) {
		maxadd = h2w_readc_cmd(H2W_CRDL_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}
	if (ph2w_info->MAX_GP_ADD >= 4) {
		maxadd = h2w_readc_cmd(H2W_CARKIT_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}
	if (ph2w_info->MAX_GP_ADD >= 5) {
		maxadd = h2w_readc_cmd(H2W_USBHOST_MAXADD);
		if (maxadd == -1)
			goto err_plugin;
	}

	/* read medical group */
	if (ph2w_info->MAX_GP_ADD >= 6) {
		ph2w_info->MED_MAXADD = h2w_readc_cmd(H2W_MED_MAXADD);
		if (ph2w_info->MED_MAXADD == -1)
			goto err_plugin;
		if (ph2w_info->MED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_MED_CONTROL);
			if (key == -1)
				goto err_plugin;
		ph2w_info->DATA_EN = (key & 0x01);
		ph2w_info->AP_EN = (key & 0x02) >> 1;
		ph2w_info->AP_ID = (key & 0x1c) >> 2;
		}
		if (ph2w_info->MED_MAXADD >= 2) {
			key = h2w_readc_cmd(H2W_MED_IN_DATA);
			if (key == -1)
				goto err_plugin;
		}
	}

	if (ph2w_info->AUDIO_DEVICE)
		ascr0 = H2W_ASCR_AUDIO_IN | H2W_ASCR_ACT_EN;
	else
		ascr0 = H2W_ASCR_ACT_EN;

	if (h2w_writec_cmd(H2W_ASCR0, ascr0) < 0)
		goto err_plugin;
	udelay(10);

	ret = 0;

	/* adjust speed */
	if (ph2w_info->MAX_GP_ADD == 2) {
		/* Remote control */
		hi->speed = H2W_250KHz;
	} else if (ph2w_info->MAX_GP_ADD == 6) {
		if (ph2w_info->MED_MAXADD >= 1) {
			key = h2w_readc_cmd(H2W_MED_CONTROL);
			if (key == -1)
				goto err_plugin;
			ph2w_info->DATA_EN   = (key & 0x01);
			ph2w_info->AP_EN = (key & 0x02) >> 1;
			ph2w_info->AP_ID = (key & 0x1c) >> 2;
		}
	}

err_plugin:
	h2w_end_command();

	return ret;
}

static inline void h2w_dev_power_on(int on)
{
	if (!hi->h2w_power)
		return;

	if (on)
		gpio_set_value(hi->h2w_power, 1);
	else
		gpio_set_value(hi->h2w_power, 0);
}

static int h2w_dev_detect(void)
{
	int ret = -1;
	int retry_times;

	for (retry_times = 5; retry_times; retry_times--) {
		/* Enable H2W Power */
		h2w_dev_power_on(1);
		msleep(100);
		memset(&hi->h2w_info, 0, sizeof(H2W_INFO));
		if (h2w_dev_init(&hi->h2w_info) < 0) {
			h2w_dev_power_on(0);
			msleep(100);
		} else if (hi->h2w_info.MAX_GP_ADD == 2) {
			ret = 0;
			break;
		} else {
			printk(KERN_INFO "h2w_detect: detect error(%d)\n"
				, hi->h2w_info.MAX_GP_ADD);
			h2w_dev_power_on(0);
			msleep(100);
		}
		printk(KERN_INFO "h2w_detect(%d)\n"
				, hi->h2w_info.MAX_GP_ADD);
	}
	H2W_DBG("h2w_detect:(%d)\n", retry_times);
	return ret;
}

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
extern void msm_serial_debug_enable(int);
#endif

static void remove_headset(void)
{
	unsigned long irq_flags;
	int ret = 0;

	H2W_DBG("");

	mutex_lock(&hi->mutex_lock);
	if (!h35i->is_ext_insert)
		switch_set_state(&h35i->sdev, switch_get_state(&h35i->sdev) &
				 ~(BIT_HEADSET | BIT_HEADSET_NO_MIC |
				 BIT_USB_CRADLE));
	else
		switch_set_state(&h35i->sdev, switch_get_state(&h35i->sdev) &
				 ~BIT_USB_CRADLE);
	mutex_unlock(&hi->mutex_lock);

	hi->defconfig();

	/* Disable button */
	switch (hi->h2w_dev_type) {
	case HTC_HEADSET:
		local_irq_save(irq_flags);
		disable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);

		if (atomic_read(&h35i->btn_state))
			button_released(atomic_read(&h35i->btn_state));
		printk(KERN_INFO "Remove HTC_HEADSET\n");
		break;
	case NORMAL_HEARPHONE:
		if (h35i->h2w_35mm_status == HTC_35MM_MIC) {
			if (h35i->mic_bias_state  && !h35i->is_ext_insert) {
				turn_mic_bias_on(0);
				h35i->mic_bias_state = 0;
			}
			cnf_driver_event("unplug_microp_mic", &ret);
		}
		if (atomic_read(&h35i->btn_state))
			button_released(atomic_read(&h35i->btn_state));
		h35i->h2w_35mm_status = HTC_35MM_UNPLUG;
		printk(KERN_INFO "Remove NORMAL_HEARPHONE (3.5mm, 4-in-1)\n");
		break;
	case H2W_DEVICE:
		h2w_dev_power_on(0);
		/*have H2W device*/
		if (!gpio_get_value(hi->cable_in1))
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_HIGH);
		else /*no H2W device*/
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_LOW);
		disable_irq(hi->irq_btn);
		hi->set_clk_dir(0);
		hi->set_dat_dir(0);
		printk(KERN_INFO "Remove H2W_DEVICE\n");
		break;
	case USB_CRADLE:
		printk(KERN_INFO "Remove USB_CRADLE\n");
		switch_set_state(&hi->sdev, DOCK_STATE_UNDOCKED);
		break;
	default:
		printk(KERN_INFO "Unknown Accessory\n");
		return;
	}

	hi->h2w_dev_type = 0;
}

static void insert_headset(int type)
{
	unsigned long irq_flags;
	int state;
	int ret;

	H2W_DBG("");

	if (hi->h2w_dev_type)
		remove_headset();

	hi->h2w_dev_type = type;
	h35i->ignore_btn = 0;
	state = switch_get_state(&h35i->sdev);
	state &= ~(BIT_HEADSET_NO_MIC | BIT_HEADSET | BIT_USB_CRADLE);
	switch (type) {
	case HTC_HEADSET:
		printk(KERN_INFO "Insert HTC_HEADSET\n");
		state |= BIT_HEADSET;
		h35i->ignore_btn = !gpio_get_value(hi->h2w_data);
		/* Enable button irq */
		local_irq_save(irq_flags);
		enable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);
		break;
	case NORMAL_HEARPHONE:
		/* support 3.5mm earphone with mic */
		printk(KERN_INFO "Insert NORMAL_HEARPHONE (3.5mm, 4-in-1)\n");
		/* Turn On Mic Bias */
		if (!h35i->mic_bias_state) {
			turn_mic_bias_on(1);
			h35i->mic_bias_state = 1;
			/* Wait pin be stable */
			msleep(300);
		}
		ret = 0;
		/* Detect headset with or without microphone */
		if (!cnf_driver_event("query_mic_state", &ret)) {
			/* without microphone */
			/*
			if (h35i->mic_bias_state) {
				turn_mic_bias_on(0);
				h35i->mic_bias_state = 0;
			}
			*/
			state |= BIT_HEADSET_NO_MIC;
			h35i->h2w_35mm_status = HTC_35MM_NO_MIC;
			printk(KERN_INFO "11pin_3.5mm without microphone\n");
		} else { /* with microphone */
			state |= BIT_HEADSET;
			h35i->h2w_35mm_status = HTC_35MM_MIC;
			printk(KERN_INFO "11pin_3.5mm with microphone\n");
		}
		break;
	case H2W_DEVICE:
		printk(KERN_INFO "Insert H2W_DEVICE\n");
		if (!hi->set_dat) {
			printk(KERN_INFO "Don't support H2W_DEVICE\n");
			hi->h2w_dev_type = 0;
			return;
		}
		if (h2w_dev_detect() < 0) {
			printk(KERN_INFO "H2W_DEVICE -- Non detected\n");
			hi->set_clk_dir(0);
			hi->set_dat_dir(0);
			hi->h2w_dev_type = 0;
		} else {
			printk(KERN_INFO "H2W_DEVICE -- detected\n");
			local_irq_save(irq_flags);
			set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
			enable_irq(hi->irq_btn);
			local_irq_restore(irq_flags);
			state |= BIT_HEADSET;
		}
		break;
	case USB_CRADLE:
		printk(KERN_INFO "Insert USB_CRADLE\n");
		state |= BIT_USB_CRADLE;
		switch_set_state(&hi->sdev, DOCK_STATE_CAR);
		break;
	case UART_DEBUG:
		printk(KERN_INFO "Insert UART_DEBUG\n");
		disable_h2w_irq();
		hi->configure(hi->debug_uart);
		return;
	default:
		printk(KERN_INFO "Unknown Accessory\n");
		return;
	}
	mutex_lock(&h35i->mutex_lock);
	switch_set_state(&h35i->sdev, state);
	mutex_unlock(&h35i->mutex_lock);
	hi->configure(H2W_GPIO);

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
	msm_serial_debug_enable(false);
#endif

	/* On some non-standard headset adapters (usually those without a
	 * button) the btn line is pulled down at the same time as the detect
	 * line. We can check here by sampling the button line, if it is
	 * low then it is probably a bad adapter so ignore the button.
	 * If the button is released then we stop ignoring the button, so that
	 * the user can recover from the situation where a headset is plugged
	 * in with button held down.
	 */

	 /* hi->debounce_time = ktime_set(0, 200000000);   20 ms */
}

static int is_accessary_pluged_in(void)
{
	int type = 0;
	int clk1 = 0, dat1 = 0, clk2 = 0, dat2 = 0, clk3 = 0, dat3 = 0;

	/* Step1: save H2W_CLK and H2W_DAT */
	/* Delay 10ms for pin stable. */
	msleep(10);
	clk1 = gpio_get_value(hi->h2w_clk);
	dat1 = gpio_get_value(hi->h2w_data);

	/*
	 * Step2: set GPIO_CABLE_IN1 as output high and GPIO_CABLE_IN2 as
	 * input
	 */
	gpio_direction_output(hi->cable_in1, 1);
	gpio_direction_input(hi->cable_in2);
	/* Delay 10ms for pin stable. */
	msleep(10);
	/* Step 3: save H2W_CLK and H2W_DAT */
	clk2 = gpio_get_value(hi->h2w_clk);
	dat2 = gpio_get_value(hi->h2w_data);

	/*
	 * Step 4: set GPIO_CABLE_IN1 as input and GPIO_CABLE_IN2 as output
	 * high
	 */
	gpio_direction_input(hi->cable_in1);
	gpio_direction_output(hi->cable_in2, 1);
	/* Delay 10ms for pin stable. */
	msleep(10);
	/* Step 5: save H2W_CLK and H2W_DAT */
	clk3 = gpio_get_value(hi->h2w_clk);
	dat3 = gpio_get_value(hi->h2w_data);

	/* Step 6: set both GPIO_CABLE_IN1 and GPIO_CABLE_IN2 as input */
	gpio_direction_input(hi->cable_in1);
	gpio_direction_input(hi->cable_in2);

	H2WI("(%d,%d) (%d,%d) (%d,%d)",
		clk1, dat1, clk2, dat2, clk3, dat3);

	if ((clk1 == 0) && (dat1 == 1) &&
	    (clk2 == 0) && (dat2 == 1) &&
	    (clk3 == 0) && (dat3 == 1))
		type = HTC_HEADSET;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 0) && (dat2 == 0) &&
		 (clk3 == 0) &&  (dat3 == 0))
		type = NORMAL_HEARPHONE;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 1) && (dat2 == 0) &&
		 (clk3 == 0) && (dat3 == 1))
		type = H2W_DEVICE;
	else if ((clk1 == 0) && (dat1 == 0) &&
		 (clk2 == 1) && (dat2 == 1) &&
		 (clk3 == 1) && (dat3 == 1))
		type = USB_CRADLE;
	else if ((clk1 == 0) && (dat1 == 1) &&
		 (clk2 == 1) && (dat2 == 1) &&
		 (clk3 == 0) && (dat3 == 1))
		type = UART_DEBUG;
	else
		type = NO_DEVICE;

	return type;
}

static void detect_h2w_do_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int type;

	H2W_DBG("");
	if (h35i->is_ext_insert) {
		set_11pin_35mm_stable(1);
		return;
	}

	if (gpio_get_value(hi->cable_in1) != 0) {
		/* Headset not plugged in */
		if (hi->h2w_dev_type)
			remove_headset();
		set_11pin_35mm_stable(1);
		return;
	}
	/* Switch CPLD to GPIO to do detection */
	hi->configure(H2W_GPIO);

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hi->irq);
	local_irq_restore(irq_flags);

	/* Something plugged in, lets make sure its a headset */
	type = is_accessary_pluged_in();

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hi->irq);
	local_irq_restore(irq_flags);

	insert_headset(type);
	set_11pin_35mm_stable(1);
}

static void headset_button_event(int is_press, int type)
{
	if (!is_press) {
		if (h35i->ignore_btn)
			h35i->ignore_btn = 0;
		else
			button_released(type);
	} else {
		if (!h35i->ignore_btn && !atomic_read(&h35i->btn_state))
			button_pressed(type);
	}
}

static void button_35mm_do_work(struct work_struct *w)
{
	int key;

	if (!h35i->is_ext_insert && !h35i->h2w_35mm_status) {
		H2WI("3.5mm headset is plugged out, skip report key event");
		return;
	}

	if (!is_11pin_35mm_stable()) {
		H2WI(" The H2W pin tremble, skip the button event");
		return;
	}

#ifdef CONFIG_HTC_AUDIOJACK_V1
	if (!is_audio_jack_pin_stable()) {
		H2WI(" The HPIN tremble, skip the button event");
		return;
	}
#endif

	if (h35i->key_level_flag) {
		switch (h35i->key_level_flag) {
		case 1:
			H2WI("3.5mm RC: Play Pressed");
			key = KEY_MEDIA;
			break;
		case 2:
			H2WI("3.5mm RC: BACKWARD Pressed");
			key = KEY_PREVIOUSSONG;
			break;
		case 3:
			H2WI("3.5mm RC: FORWARD Pressed");
			key = KEY_NEXTSONG;
			break;
		default:
			H2WI("3.5mm RC: WRONG Button Pressed");
			return;
		}
		headset_button_event(1, key);
	} else { /* key release */
		if (atomic_read(&h35i->btn_state))
			headset_button_event(0, atomic_read(&h35i->btn_state));
		else
			H2WI("3.5mm RC: WRONG Button Release");
	}
	wake_lock_timeout(&hi->headset_wake_lock, 1.5*HZ);
}

static void button_h2w_do_work(struct work_struct *w)
{
	int key, press, keyname, h2w_key = 1;

	H2W_DBG("");

	if (switch_get_state(&h35i->sdev) & BIT_HEADSET) {
		switch (hi->h2w_dev_type) {
		case HTC_HEADSET:
			if (!gpio_get_value(hi->h2w_data))
				headset_button_event(1, KEY_MEDIA); /* press */
			else
				headset_button_event(0, KEY_MEDIA);
			break;
		case H2W_DEVICE:
			if ((hi->get_dat() == 1) && (hi->get_clk() == 1)) {
				/* Don't do anything because H2W pull out. */
				H2WE("Remote Control pull out.\n");
			} else {
				key = h2w_get_fnkey();
				press = (key > 0x7F) ? 0 : 1;
				keyname = key & 0x7F;
				 /* H2WI("key = %d, press = %d,
					 keyname = %d \n",
					 key, press, keyname); */
				switch (keyname) {
				case H2W_KEY_PLAY:
					H2WI("H2W_KEY_PLAY");
					key = KEY_PLAYPAUSE;
					break;
				case H2W_KEY_FORWARD:
					H2WI("H2W_KEY_FORWARD");
					key = KEY_NEXTSONG;
					break;
				case H2W_KEY_BACKWARD:
					H2WI("H2W_KEY_BACKWARD");
					key = KEY_PREVIOUSSONG;
					break;
				case H2W_KEY_VOLUP:
					H2WI("H2W_KEY_VOLUP");
					key = KEY_VOLUMEUP;
					break;
				case H2W_KEY_VOLDOWN:
					H2WI("H2W_KEY_VOLDOWN");
					key = KEY_VOLUMEDOWN;
					break;
				case H2W_KEY_PICKUP:
					H2WI("H2W_KEY_PICKUP");
					key = KEY_SEND;
					break;
				case H2W_KEY_HANGUP:
					H2WI("H2W_KEY_HANGUP");
					key = KEY_END;
					break;
				case H2W_KEY_MUTE:
					H2WI("H2W_KEY_MUTE");
					key = KEY_MUTE;
					break;
				case H2W_KEY_HOLD:
					H2WI("H2W_KEY_HOLD");
					break;
				default:
				H2WI("default");
					h2w_key = 0;
				}
				if (h2w_key) {
					if (press)
						H2WI("Press\n");
					else
						H2WI("Release\n");
					wake_lock_timeout
					(&hi->headset_wake_lock, 1.5*HZ);
					input_report_key(h35i->input, key, press);
				}
			}
			break;
		} /* end switch */
	}

}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;
	int state = BIT_HEADSET | BIT_HEADSET_NO_MIC;
	H2W_DBG("");
	set_irq_type(hi->irq_btn, IRQF_TRIGGER_LOW);
	do {
		value1 = gpio_get_value(hi->cable_in1);
		set_irq_type(hi->irq, value1 ?
				IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->cable_in1);
	} while (value1 != value2 && retry_limit-- > 0);

	H2WI("value2 = %d (%d retries), device=%d", value2,
		(10-retry_limit), switch_get_state(&h35i->sdev));

	if ((hi->h2w_dev_type == 0) ^ value2) {
		set_11pin_35mm_stable(0);
		if ((switch_get_state(&h35i->sdev) & state) &&
		    !h35i->is_ext_insert)
			h35i->ignore_btn = 1;
		if (hi->is_wake_lock_ready)
			wake_lock_timeout(&hi->headset_wake_lock, 2.5*HZ);
		queue_delayed_work(detect_wq, &detect_h2w_work, H2W_DETECT_DELAY);
	}
	return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	H2W_DBG("");
	do {
		value1 = gpio_get_value(hi->h2w_data);
		if (hi->h2w_dev_type != H2W_DEVICE)
			set_irq_type(hi->irq_btn, value1 ?
				     IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
		value2 = gpio_get_value(hi->h2w_data);
	} while (value1 != value2 && retry_limit-- > 0);

	H2W_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));

	queue_delayed_work(button_wq, &button_h2w_work, BUTTON_H2W_DELAY);
	return IRQ_HANDLED;
}

static void remove_35mm_do_work(struct work_struct *work)
{
	int ret, state;

	wake_lock_timeout(&hi->headset_wake_lock, 2.5*HZ);

	/*To solve the insert, remove, insert headset problem*/
	if (time_before_eq(jiffies, h35i->insert_jiffies))
		msleep(800);
	if (h35i->is_ext_insert) {
		H2WI("Skip 3.5mm headset plug out!!!");
		return;
	}

	printk(KERN_INFO "3.5mm_headset plug out\n");
	mutex_lock(&h35i->mutex_lock);
	state = switch_get_state(&h35i->sdev);

	if (h35i->mic_bias_state) {
		turn_mic_bias_on(0);
		h35i->mic_bias_state = 0;
	}

	ret = 0;
	cnf_driver_event("unplug_microp_mic", &ret);

	if (atomic_read(&h35i->btn_state))
		button_released(atomic_read(&h35i->btn_state));
	h35i->ext_35mm_status = HTC_35MM_UNPLUG;

	if (h35i->ext_mic_sel)
		gpio_direction_output(h35i->ext_mic_sel, 0);

	if (!gpio_get_value(hi->cable_in1) &&
	    hi->h2w_dev_type != USB_CRADLE) {
		state &= ~BIT_35MM_HEADSET;
		switch_set_state(&h35i->sdev, state);
		queue_delayed_work(detect_wq, &detect_h2w_work, 0);
	} else {
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC |
			BIT_35MM_HEADSET);
		switch_set_state(&h35i->sdev, state);
	}

	mutex_unlock(&h35i->mutex_lock);
}

static void insert_35mm_do_work(struct work_struct *work)
{
	int state;
	int ret;
	int i, mic1, mic2;

	h35i->insert_jiffies = jiffies + 1*HZ;
	wake_lock_timeout(&hi->headset_wake_lock, 1.5*HZ);
	if (hi->h2w_dev_type && h35i->is_ext_insert &&
	    hi->h2w_dev_type != USB_CRADLE)
		remove_headset();

	mutex_lock(&h35i->mutex_lock);
	state = switch_get_state(&h35i->sdev);

	if (h35i->is_ext_insert) {
		printk(KERN_INFO "3.5mm_headset plug in\n");
		h35i->ignore_btn = 0;
		state &= ~(BIT_HEADSET | BIT_HEADSET_NO_MIC);
		if (h35i->ext_mic_sel)
			gpio_direction_output(h35i->ext_mic_sel, 1);

		/* Turn On Mic Bias */
		if (!h35i->mic_bias_state) {
			turn_mic_bias_on(1);
			h35i->mic_bias_state = 1;
			/* Wait for pin stable */
			msleep(300);
		}

		/* Detect headset with or without microphone */
		ret = 0;
		for (i = 0; i < 10; i++) {
			mic1 = cnf_driver_event("query_mic_state", &ret);
			msleep(600);
			mic2 = cnf_driver_event("query_mic_state", &ret);
			if (mic1 == mic2)
				break;
		}
		if (mic2 == 0 || mic1 != mic2) {
			/* without microphone */
			state |= BIT_HEADSET_NO_MIC;
			printk(KERN_INFO "3.5mm_headset without microphone\n");
		} else {
			/* with microphone */
			state |= BIT_HEADSET;
			printk(KERN_INFO "3.5mm_headset with microphone\n");
		}

		state |= BIT_35MM_HEADSET;
		switch_set_state(&h35i->sdev, state);

		if (state & BIT_HEADSET_NO_MIC)
			h35i->ext_35mm_status = HTC_35MM_NO_MIC;
		else
			h35i->ext_35mm_status = HTC_35MM_MIC;

	}

	mutex_unlock(&h35i->mutex_lock);
}

static int insert_35mm_headset(void *argu)
{
	h35i->is_ext_insert = *((int *)argu);
	H2WI(" %d", h35i->is_ext_insert);
	if (!h35i->is_ext_insert)
		queue_work(detect_wq, &remove_35mm_work);
	else
		queue_work(detect_wq, &insert_35mm_work);
	return 1;
}

static int microp_ready(void *argu)
{
	if (h35i->is_ext_insert)
		queue_work(detect_wq, &insert_35mm_work);
	if (h35i->h2w_35mm_status)
		 insert_headset(NORMAL_HEARPHONE);
	return 1;
}

static int get_35mm_button_status(void *argu)
{
	int *key_level = (int *) argu;

	h35i->key_level_flag = *key_level;
	if (h35i->ext_35mm_status == HTC_35MM_NO_MIC ||
		h35i->h2w_35mm_status == HTC_35MM_NO_MIC) {

		printk(KERN_INFO "To re-detect mic!!!\n");
		queue_work(detect_wq, &insert_35mm_work);
	} else {
		queue_work(button_wq, &button_35mm_work);
	}
	return 1;
}

static int is_11pin_35mm_stable(void)
{
	int stable = 1;
	unsigned long flags = 0;

	if (!hi)
		return 1;

	spin_lock_irqsave(&hi->spin_lock, flags);
	stable = hi->is_11pin_35mm_stable;
	spin_unlock_irqrestore(&hi->spin_lock, flags);

	H2WI("(stable = %d)", stable);

	return stable;
}

static void set_11pin_35mm_stable(int stable)
{
	unsigned long flags = 0;

	if (!hi)
		return;

	spin_lock_irqsave(&hi->spin_lock, flags);
	hi->is_11pin_35mm_stable = stable;
	spin_unlock_irqrestore(&hi->spin_lock, flags);

	H2WI("(stable = %d)", stable);
}

#if defined(CONFIG_DEBUG_FS)
static int h2w_debug_set(void *data, u64 val)
{
	mutex_lock(&h35i->mutex_lock);
	switch_set_state(&h35i->sdev, (int)val);
	mutex_unlock(&h35i->mutex_lock);
	return 0;
}

static int h2w_debug_get(void *data, u64 *val)
{
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(h2w_debug_fops, h2w_debug_get, h2w_debug_set, "%llu\n");
static int __init h2w_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("h2w", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &h2w_debug_fops);

	return 0;
}

device_initcall(h2w_debug_init);
#endif

static ssize_t tty_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	mutex_lock(&hi->mutex_lock);
	s += sprintf(s, "%d\n", hi->tty_enable_flag);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t tty_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;
	state = switch_get_state(&h35i->sdev);
	state &= ~(BIT_TTY_FULL | BIT_TTY_VCO | BIT_TTY_HCO);

	if (count == (strlen("enable") + 1) &&
	   strncmp(buf, "enable", strlen("enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 1;

		switch_set_state(&h35i->sdev, state | BIT_TTY_FULL);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY FULL\n");
		return count;
	}
	if (count == (strlen("vco_enable") + 1) &&
	   strncmp(buf, "vco_enable", strlen("vco_enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 2;

		switch_set_state(&h35i->sdev, state | BIT_TTY_VCO);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY VCO\n");
		return count;
	}
	if (count == (strlen("hco_enable") + 1) &&
	   strncmp(buf, "hco_enable", strlen("hco_enable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 3;

		switch_set_state(&h35i->sdev, state | BIT_TTY_HCO);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable TTY HCO\n");
		return count;
	}
	if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->tty_enable_flag = 0;

		switch_set_state(&h35i->sdev, state);

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Disable TTY\n");
		return count;
	}
	printk(KERN_ERR "tty_enable_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(tty, 0666, tty_flag_show, tty_flag_store);

static ssize_t fm_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int state;

	mutex_lock(&hi->mutex_lock);
	state = switch_get_state(&h35i->sdev);
	state &= ~(BIT_FM_HEADSET | BIT_FM_SPEAKER);

	if (count == (strlen("fm_headset") + 1) &&
	   strncmp(buf, "fm_headset", strlen("fm_headset")) == 0) {
		hi->fm_flag = 1;
		state |= BIT_FM_HEADSET;
		printk(KERN_INFO "Enable FM HEADSET\n");
	} else if (count == (strlen("fm_speaker") + 1) &&
	   strncmp(buf, "fm_speaker", strlen("fm_speaker")) == 0) {
		hi->fm_flag = 2;
		state |= BIT_FM_SPEAKER;
		printk(KERN_INFO "Enable FM SPEAKER\n");
	} else if (count == (strlen("disable") + 1) &&
	   strncmp(buf, "disable", strlen("disable")) == 0) {
		hi->fm_flag = 0 ;
		printk(KERN_INFO "Disable FM\n");
	} else {
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_ERR "fm_enable_flag_store: invalid argument\n");
		return -EINVAL;
	}
	switch_set_state(&h35i->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	return count;
}

static ssize_t fm_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->fm_flag == 0)
		show_str = "disable";
	if (hi->fm_flag == 1)
		show_str = "fm_headset";
	if (hi->fm_flag == 2)
		show_str = "fm_speaker";

	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}
static DEVICE_ACCESSORY_ATTR(fm, 0666, fm_flag_show, fm_flag_store);

static ssize_t mic_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	char *s = buf;
	char *show_str;
	mutex_lock(&hi->mutex_lock);
	if (hi->mic_switch_flag == 0)
		show_str = "11pin";
	if (hi->mic_switch_flag == 1)
		show_str = "35mm";
	s += sprintf(s, "%s\n", show_str);
	mutex_unlock(&hi->mutex_lock);
	return (s - buf);
}

static ssize_t mic_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("35mm") + 1) &&
	   strncmp(buf, "35mm", strlen("35mm")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->mic_switch_flag = 1;

		if (h35i->ext_mic_sel) {
			gpio_direction_output(h35i->ext_mic_sel, 1);
			printk(KERN_INFO "ext_mic_sel to 1\n");
		}

		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable 3.5mm MIC\n");
		return count;
	}
	if (count == (strlen("11pin") + 1) &&
	   strncmp(buf, "11pin", strlen("11pin")) == 0) {
		mutex_lock(&hi->mutex_lock);
		hi->mic_switch_flag = 0;

		if (h35i->ext_mic_sel) {
			gpio_direction_output(h35i->ext_mic_sel, 0);
			printk(KERN_INFO "ext_mic_sel to 0\n");
		}
		mutex_unlock(&hi->mutex_lock);
		printk(KERN_INFO "Enable 11pin MIC\n");
		return count;
	}
	printk(KERN_ERR "mic_switch_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(mic, 0666, mic_flag_show, mic_flag_store);

static ssize_t mute_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	if ((hi->rc_flag & H2W_MuteLed) == 0)
		show_str = "unmute";
	if ((hi->rc_flag & H2W_MuteLed) == H2W_MuteLed)
		show_str = "mute";
	s += sprintf(s, "%s\n", show_str);
	return (s - buf);
}
static ssize_t mute_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("unmute") + 1) &&
	   strncmp(buf, "unmute", strlen("unmute")) == 0) {
		hi->rc_flag = hi->rc_flag & (!H2W_MuteLed);
		h2w_enable_mute_light(0);
		printk(KERN_INFO "Turn off mute LED\n");
		return count;
	}
	if (count == (strlen("mute") + 1) &&
	   strncmp(buf, "mute", strlen("mute")) == 0) {
		hi->rc_flag = hi->rc_flag | H2W_MuteLed;
		h2w_enable_mute_light(1);
		printk(KERN_INFO "Turn on mute LED\n");
		return count;
	}
	printk(KERN_ERR "mute_flag_store: invalid argument\n");
	return -EINVAL;
}
static DEVICE_ACCESSORY_ATTR(mute, 0666, mute_flag_show, mute_flag_store);

static ssize_t phonein_flag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;
	char *show_str;
	if ((hi->rc_flag & H2W_PhoneIn) == 0)
		show_str = "phoneoff";
	if ((hi->rc_flag & H2W_PhoneIn) == H2W_PhoneIn)
		show_str = "phonein";
	s += sprintf(s, "%s\n", show_str);
	return (s - buf);
}

static ssize_t phonein_flag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (count == (strlen("phoneoff") + 1) &&
	   strncmp(buf, "phoneoff", strlen("phoneoff")) == 0) {
		if (hi->h2w_dev_type == H2W_DEVICE) {
			hi->rc_flag = hi->rc_flag & (!H2W_PhoneIn);
			h2w_enable_phone_in(0);
			printk(KERN_INFO "The phone call is OFF\n");
		}
		return count;
	}
	if (count == (strlen("phonein") + 1) &&
	   strncmp(buf, "phonein", strlen("phonein")) == 0) {
		if (hi->h2w_dev_type == H2W_DEVICE) {
			hi->rc_flag = hi->rc_flag | H2W_PhoneIn;
			h2w_enable_phone_in(1);
			printk(KERN_INFO "The phone call is ON\n");
		}
		return count;
	}
	printk(KERN_ERR "phonein_flag_store: invalid argument\n");
	return -EINVAL;
}

static DEVICE_ACCESSORY_ATTR(phonein, 0666, \
			     phonein_flag_show, phonein_flag_store);

static void init_work_func(struct work_struct *work)
{
	hi->is_wake_lock_ready = 1;
}

static int h2w_probe(struct platform_device *pdev)
{
	int ret;

	struct h2w_platform_data *pdata = pdev->dev.platform_data;
	struct cnf_driver *h2w_extern, *h2w_irq, *h2w_3button;
	struct cnf_driver *h2w_microp_ready;

	printk(KERN_INFO "H2W: htc_35mm_remote Registering H2W (headset) Driver\n");
	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi)
		return -ENOMEM;
	h35i = kzalloc(sizeof(struct headset_35mm_info), GFP_KERNEL);
	if (!h35i)
		return -ENOMEM;

	atomic_set(&h35i->btn_state, 0);
	h35i->ignore_btn = 0;
	hi->h2w_dev_type = 0;
	h35i->ext_35mm_status = 0;
	h35i->h2w_35mm_status = 0;
	h35i->is_ext_insert = 0;
	h35i->mic_bias_state = 0;
	h35i->key_level_flag = -1;

	hi->is_wake_lock_ready = 0;
	hi->tty_enable_flag = 0;
	hi->fm_flag = 0;
	hi->mic_switch_flag = 1;
	hi->rc_flag = 0;
	hi->cable_in1 = pdata->cable_in1;
	hi->cable_in2 = pdata->cable_in2;
	hi->h2w_clk = pdata->h2w_clk;
	hi->h2w_data = pdata->h2w_data;
	hi->debug_uart = pdata->debug_uart;
	hi->configure = pdata->config;
	hi->defconfig = pdata->defconfig;
	hi->set_dat = pdata->set_dat;
	hi->set_clk = pdata->set_clk;
	hi->set_dat_dir	= pdata->set_dat_dir;
	hi->set_clk_dir	= pdata->set_clk_dir;
	hi->get_dat = pdata->get_dat;
	hi->get_clk = pdata->get_clk;
	hi->speed = H2W_50KHz;
	hi->h2w_power = pdata->h2w_power;
	hi->is_11pin_35mm_stable = 1;

	if (hi->h2w_power)
		gpio_direction_output(hi->h2w_power, 0);

	h35i->ext_mic_sel = pdata->ext_mic_sel;
	mutex_init(&hi->mutex_lock);
	mutex_init(&h35i->mutex_lock);
	mutex_init(&hi->mutex_rc_lock);
	hi->htc_accessory_class = class_create(THIS_MODULE, "htc_accessory");
	if (IS_ERR(hi->htc_accessory_class)) {
		ret = PTR_ERR(hi->htc_accessory_class);
		hi->htc_accessory_class = NULL;
		goto err_create_class;
	}

	hi->tty_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "tty");
	if (unlikely(IS_ERR(hi->tty_dev))) {
		ret = PTR_ERR(hi->tty_dev);
		hi->tty_dev = NULL;
		goto err_create_tty_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->tty_dev, &dev_attr_tty);
	if (ret)
		goto err_create_tty_device_file;

	hi->fm_dev = device_create(hi->htc_accessory_class,
				   NULL, 0, "%s", "fm");
	if (unlikely(IS_ERR(hi->fm_dev))) {
		ret = PTR_ERR(hi->fm_dev);
		hi->fm_dev = NULL;
		goto err_create_fm_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->fm_dev, &dev_attr_fm);
	if (ret)
		goto err_create_fm_device_file;

	hi->mic_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "mic");
	if (unlikely(IS_ERR(hi->mic_dev))) {
		ret = PTR_ERR(hi->mic_dev);
		hi->mic_dev = NULL;
		goto err_create_mic_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->mic_dev, &dev_attr_mic);
	if (ret)
		goto err_create_mic_device_file;

	hi->mute_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "mute");
	if (unlikely(IS_ERR(hi->mute_dev))) {
		ret = PTR_ERR(hi->mute_dev);
		hi->mute_dev = NULL;
		goto err_create_mute_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->mute_dev, &dev_attr_mute);
	if (ret)
		goto err_create_mute_device_file;

	hi->phonein_dev = device_create(hi->htc_accessory_class,
				    NULL, 0, "%s", "phonein");
	if (unlikely(IS_ERR(hi->phonein_dev))) {
		ret = PTR_ERR(hi->phonein_dev);
		hi->phonein_dev = NULL;
		goto err_create_phonein_device;
	}

	/* register the attributes */
	ret = device_create_file(hi->phonein_dev, &dev_attr_phonein);
	if (ret)
		goto err_create_phonein_device_file;

	h35i->sdev.name = "h2w";
	h35i->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&h35i->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_DELAYED_WORK(&init_work, init_work_func);

	detect_wq = create_workqueue("detection");
	if (detect_wq  == NULL) {
		ret = -ENOMEM;
		goto err_create_detect_work_queue;
	}
	button_wq = create_workqueue("button");
	if (button_wq  == NULL) {
			ret = -ENOMEM;
			goto err_create_button_work_queue;
	}

	ret = gpio_request(hi->cable_in1, "h2w_detect");
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_request(hi->h2w_data, "h2w_button");
	if (ret < 0)
		goto err_request_button_gpio;

	ret = gpio_direction_input(hi->h2w_data);
	if (ret < 0)
		goto err_set_button_gpio;

	ret = gpio_direction_input(hi->cable_in1);
	if (ret < 0)
		goto err_set_detect_gpio;

	ret = gpio_direction_input(hi->cable_in2);
	if (ret < 0)
		goto err_set_cablein2_gpio;

	hi->irq = gpio_to_irq(hi->cable_in1);
	if (hi->irq < 0) {
		ret = hi->irq;
		goto err_get_h2w_detect_irq_num_failed;
	}

	hi->irq_btn = gpio_to_irq(hi->h2w_data);
	if (hi->irq_btn < 0) {
		ret = hi->irq_btn;
		goto err_get_button_irq_num_failed;
	}

	/* Set CPLD MUX to H2W <-> CPLD GPIO */
	hi->defconfig();

	ret = request_irq(hi->irq, detect_irq_handler,
			  IRQF_TRIGGER_LOW, "h2w_detect", NULL);
	if (ret < 0)
		goto err_request_detect_irq;

	ret = request_irq(hi->irq_btn, button_irq_handler,
			  IRQF_TRIGGER_LOW, "h2w_button", NULL);
	if (ret < 0)
		goto err_request_h2w_headset_button_irq;

	disable_irq(hi->irq_btn);

	ret = set_irq_wake(hi->irq, 1);
	if (ret < 0)
		goto err_request_input_dev;
	ret = set_irq_wake(hi->irq_btn, 1);
	if (ret < 0)
		goto err_request_input_dev;

	if (h35i->ext_mic_sel)
		gpio_direction_output(h35i->ext_mic_sel, 0);

	h35i->input = input_allocate_device();
	if (!h35i->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	h35i->input->name = "h2w headset";
	set_bit(EV_SYN, h35i->input->evbit);
	set_bit(EV_KEY, h35i->input->evbit);
	set_bit(KEY_MEDIA, h35i->input->keybit);
	set_bit(KEY_NEXTSONG, h35i->input->keybit);
	set_bit(KEY_PLAYPAUSE, h35i->input->keybit);
	set_bit(KEY_PREVIOUSSONG, h35i->input->keybit);
	set_bit(KEY_MUTE, h35i->input->keybit);
	set_bit(KEY_VOLUMEUP, h35i->input->keybit);
	set_bit(KEY_VOLUMEDOWN, h35i->input->keybit);
	set_bit(KEY_END, h35i->input->keybit);
	set_bit(KEY_SEND, h35i->input->keybit);

	ret = input_register_device(h35i->input);
	if (ret < 0)
		goto err_register_input_dev;

	wake_lock_init(&hi->headset_wake_lock, WAKE_LOCK_SUSPEND, "headset");

	h2w_extern = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_extern) {
		h2w_extern->name = "H2W_extend_headset";
		h2w_extern->func = &insert_35mm_headset;
	} else {
		printk(KERN_ERR "h2w_extern alloc failed (no memory)\n");
		goto err_alloc_h2w_extern;
	}
	cnf_driver_register(h2w_extern);

	h2w_irq = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_irq) {
		h2w_irq->name = "H2W_enable_irq";
		h2w_irq->func = &h2w_irq_enable;
	} else {
		printk(KERN_ERR "h2w_irq alloc failed (no memory)\n");
		goto err_alloc_h2w_irq;
	}
	cnf_driver_register(h2w_irq);

	h2w_3button = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_3button) {
		h2w_3button->name = "H2W_3button";
		h2w_3button->func = &get_35mm_button_status;
	} else {
		printk(KERN_ERR "H2W_3button alloc failed (no memory)\n");
		goto err_alloc_h2w_3button;
	}
	cnf_driver_register(h2w_3button);

	h2w_microp_ready = kzalloc(sizeof(struct cnf_driver), GFP_KERNEL);
	if (h2w_microp_ready) {
		h2w_microp_ready->name = "H2W_microp_ready";
		h2w_microp_ready->func = &microp_ready;
	} else {
		printk(KERN_ERR "h2w_microp_ready alloc failed (no memory)\n");
		goto err_alloc_h2w_microp_ready;
	}
	cnf_driver_register(h2w_microp_ready);

	hi->sdev.name = "dock";
	hi->sdev.print_name = car_dock_print_name;
	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_car_dock_switch_register;

	/*Fix me, work around: device reboot when call wake_lock_timeout() in detect_irq_handler at boot time*/
	schedule_delayed_work(&init_work, HZ);

	printk(KERN_INFO "H2W: Registering H2W (headset) driver finish\n");
	return 0;

err_car_dock_switch_register:
	kfree(h2w_microp_ready);
err_alloc_h2w_microp_ready:
	kfree(h2w_3button);
err_alloc_h2w_3button:
	kfree(h2w_irq);
err_alloc_h2w_irq:
	kfree(h2w_extern);
err_alloc_h2w_extern:
err_register_input_dev:
	input_free_device(h35i->input);
err_request_input_dev:
	free_irq(hi->irq_btn, 0);
err_request_h2w_headset_button_irq:
	free_irq(hi->irq, 0);
err_request_detect_irq:
err_get_button_irq_num_failed:
err_get_h2w_detect_irq_num_failed:
err_set_cablein2_gpio:
err_set_detect_gpio:
err_set_button_gpio:
	gpio_free(hi->h2w_data);
err_request_button_gpio:
	gpio_free(hi->cable_in1);
err_request_detect_gpio:
	destroy_workqueue(button_wq);
err_create_button_work_queue:
	destroy_workqueue(detect_wq);
err_create_detect_work_queue:
	switch_dev_unregister(&h35i->sdev);
err_switch_dev_register:
	device_remove_file(hi->phonein_dev, &dev_attr_phonein);
err_create_phonein_device_file:
	device_unregister(hi->phonein_dev);
err_create_phonein_device:
	device_remove_file(hi->mute_dev, &dev_attr_mute);
err_create_mute_device_file:
	device_unregister(hi->mute_dev);
err_create_mute_device:
	device_remove_file(hi->mic_dev, &dev_attr_mic);
err_create_mic_device_file:
	device_unregister(hi->mic_dev);
err_create_mic_device:
	device_remove_file(hi->fm_dev, &dev_attr_fm);
err_create_fm_device_file:
	device_unregister(hi->fm_dev);
err_create_fm_device:
	device_remove_file(hi->tty_dev, &dev_attr_tty);
err_create_tty_device_file:
	device_unregister(hi->tty_dev);
err_create_tty_device:
	class_destroy(hi->htc_accessory_class);
err_create_class:

	printk(KERN_ERR "H2W: Failed to register driver\n");

	return ret;
}

static int h2w_remove(struct platform_device *pdev)
{
	H2W_DBG("");
	if ((switch_get_state(&h35i->sdev) &
		(BIT_HEADSET | BIT_HEADSET_NO_MIC)) != 0)
		remove_headset();
	input_unregister_device(h35i->input);
	gpio_free(hi->h2w_data);
	gpio_free(hi->cable_in1);
	free_irq(hi->irq_btn, 0);
	free_irq(hi->irq, 0);

	destroy_workqueue(detect_wq);
	switch_dev_unregister(&hi->sdev);
	switch_dev_unregister(&h35i->sdev);
	device_remove_file(hi->phonein_dev, &dev_attr_phonein);
	device_unregister(hi->phonein_dev);
	device_remove_file(hi->mute_dev, &dev_attr_mute);
	device_unregister(hi->mute_dev);
	device_remove_file(hi->mic_dev, &dev_attr_mic);
	device_unregister(hi->mic_dev);
	device_remove_file(hi->fm_dev, &dev_attr_fm);
	device_unregister(hi->fm_dev);
	device_remove_file(hi->tty_dev, &dev_attr_tty);
	device_unregister(hi->tty_dev);
	class_destroy(hi->htc_accessory_class);

	return 0;
}


static struct platform_driver h2w_driver = {
	.probe		= h2w_probe,
	.remove		= h2w_remove,
	.driver		= {
		.name		= "htc_headset",
		.owner		= THIS_MODULE,
	},
};


static int __init h2w_init(void)
{
	H2W_DBG("");

	return platform_driver_register(&h2w_driver);
}

static void __exit h2w_exit(void)
{
	platform_driver_unregister(&h2w_driver);
}

module_init(h2w_init);
module_exit(h2w_exit);

MODULE_AUTHOR("Laurence Chen <Laurence_Chen@htc.com>");
MODULE_DESCRIPTION("HTC 2 Wire detection driver");
MODULE_LICENSE("GPL");
