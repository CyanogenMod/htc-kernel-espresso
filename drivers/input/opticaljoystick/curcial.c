/* drivers/input/opticaljoystick/curcial.c
 *
 * Copyright (C) 2009 HTC Corporation.
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
 *
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <mach/atmega_microp.h>
#include <linux/earlysuspend.h>
#include <linux/curcial_oj.h>
#include <mach/vreg.h>
#include <asm/mach-types.h>
#include "curcial.h"

#define OJ_POWERON                  1
#define OJ_POWEROFF                 0
#define CURCIAL_OJ_POWER            85
#define BURST_DATA_SIZE             7
#define OJ_DEVICE_ID                0x0D
#define OJ_REGISTER_WRITE           0x7B
#define OJ_REGISTER_REQUEST         0x7C
#define OJ_REGISTER_READ            0x7D
#define OJ_REGISTER_BURST_REQUEST   0x7E
#define OJ_REGISTER_BURST_READ      0x7F
#define OJ_REGISTER_OJ_POLLING      0xA8
#define OJ_MOTION                   0x02
#define OJ_DELTA_Y                  0x03
#define OJ_DELTA_X                  0x04
#define OJ_SQUAL                    0x05
#define OJ_SHU_HIGH                 0x06
#define OJ_SHU_LOW                  0x07
#define OJ_OBSERVATION              0x2E
#define OJ_POLLING_ENABLE           1
#define OJ_POLLING_DISABLE          0
#define OJ_POLLING_INTERVAL         10
#define OJ_POLLING_COUNT            10

#define DELTA_SUM_TIME      40
#define DELTA_SUM_CP        0

static const unsigned short keymap[] = {
	 KEY_RIGHT,
	 KEY_LEFT,
	 KEY_UP,
	 KEY_DOWN /*,
	 KEY_REPLY*/
};

enum {
	MOTION = 0,
	Y,
	X,
	SQUAL,
	SHUTTER_UPPER,
	SHUTTER_LOWER,
	MAXIMUM_PIXEL
};
extern unsigned int system_rev;
static struct proc_dir_entry *oj_proc_entry;
static struct workqueue_struct *curcial_wq;
static struct curcial_oj_platform_data *my_oj;
static uint32_t click_interval = 800;
static uint8_t  polling_delay1;/* use mdelay*/
static uint8_t  polling_delay2;/* use msleep*/
static uint16_t  interval;
static uint8_t  debugflag;
static uint8_t	ap_code;
static uint8_t	send_count;
static int16_t	mSumDeltaX;
static int16_t	mSumDeltaY;
static int8_t	fast_th;
static int8_t	normal_th;
static int8_t	xy_ratio;
static uint8_t continue_count;
static uint8_t continue_th;
static uint8_t continue_max;
static bool suspend;
static uint8_t	x_send_count;
static uint8_t	y_send_count;
static	int16_t	x_sum;
static	int16_t	y_sum;
static	int16_t delta_time;

uint8_t  softclick;


static int __devinit curcial_oj_probe(struct platform_device *pdev);
static int __devexit curcial_oj_remove(struct platform_device *pdev);

static struct platform_driver curcial_oj_device_driver = {
	.probe    = curcial_oj_probe,
	.remove   = __devexit_p(curcial_oj_remove),
	.driver   = {
		.name   = CURCIAL_OJ_NAME,
		.owner  = THIS_MODULE,
	}
};

static uint8_t curcial_oj_register_read(uint8_t reg)
{
	uint8_t cmd[2];

	cmd[0] = 0;
	cmd[1] = reg;
	microp_i2c_write(OJ_REGISTER_REQUEST, cmd, 2);
	microp_i2c_read(OJ_REGISTER_READ, cmd, 2);

	return cmd[1];
}

static void curcial_oj_burst_read(uint8_t *data)
{
	uint8_t cmd[2];

	cmd[0] = 0x01;
	microp_i2c_write(OJ_REGISTER_BURST_REQUEST, cmd, 1);
	microp_i2c_read(OJ_REGISTER_BURST_READ, data, BURST_DATA_SIZE);
}

static void curcial_oj_polling_mode(uint8_t mode)
{
	uint8_t cmd[2];

	cmd[0] = mode;
	microp_i2c_write(OJ_REGISTER_OJ_POLLING, cmd, 1);
}

static void curcial_oj_irq_handler(void)
{
	queue_work(curcial_wq, &my_oj->work);
}

static int curcial_oj_late_init(void)
{
	uint8_t data[BURST_DATA_SIZE];
	uint8_t id;
	uint8_t version;

	microp_i2c_read(MICROP_I2C_RCMD_VERSION, data, 2);
	version = my_oj->microp_version;


	if (data[0] < version) {
		printk("Microp firmware version:%d have to large than %d !\n\
						Stop OJ driver loading!\n", data[0], version);
		return 0;
	}

	if (!my_oj->oj_poweron(OJ_POWERON))
		return 0;

	mdelay(10);
	microp_spi_vote_enable(SPI_OJ, 1);

	/*microp_i2c_read(0x24, data, 2);*/
	my_oj->oj_shutdown(0);
	/* Write 0x5a to register 0x3a */
	data[0] = 0x3a;
	data[1] = 0x5a;
	microp_i2c_write(OJ_REGISTER_WRITE, data, 2);
	mdelay(23);

	/* Read from register 0x02,0x03 and 0x04 one time regardless the state of the motion pin */
	curcial_oj_register_read(OJ_MOTION);
	curcial_oj_register_read(OJ_DELTA_Y);
	curcial_oj_register_read(OJ_DELTA_X);


	id = curcial_oj_register_read(0x00);
	if (id == OJ_DEVICE_ID) {
		printk(KERN_INFO"OpticalJoystick Device ID: %02x\n", OJ_DEVICE_ID);
		id = curcial_oj_register_read(0x01);
		printk(KERN_INFO"OJ Driver: Revision : %02x\n", id);
	} else {
		printk("Can't probe OpticalJoystick Device: %02x!\n", id);
		return 0;
	}

	/* Write 0x10 to register 0x1C. This will activate burst mode. */
	data[0] = 0x1C;
	data[1] = 0x10;
	microp_i2c_write(OJ_REGISTER_WRITE, data, 2);

	curcial_oj_polling_mode(OJ_POLLING_ENABLE);

	return 1;
}

static void curcial_oj_init_callback(void)
{
	if (!curcial_oj_late_init())
		platform_driver_unregister(&curcial_oj_device_driver);
}

static struct microp_oj_callback oj_callback = {
	.oj_init = curcial_oj_init_callback,
	.oj_intr = curcial_oj_irq_handler
};

static int curcial_oj_module_init(void)
{
	if (!microp_register_oj_callback(&oj_callback))
		return curcial_oj_late_init();

	return 1;
}
static OJKeyEvt_T OJ_ProcessNavi(int Ratio, int DeltaMin)
{
	OJKeyEvt_T	tmpKey;

#if 0
	if (abs(mSumDeltaX) > DeltaMin || abs(mSumDeltaY) > DeltaMin) {
		if (abs(mSumDeltaX) > abs(mSumDeltaY)) {
			if (mSumDeltaX > 0) {
				mSumDeltaX -= DeltaMin;
				tmpKey = OJ_KEY_RIGHT;
			} else {
				mSumDeltaX += DeltaMin;
				tmpKey = OJ_KEY_LEFT;
			}
		} else if (abs(mSumDeltaX) < abs(mSumDeltaY)) {
			if (mSumDeltaY > 0) {
				mSumDeltaY -= DeltaMin;
				tmpKey = OJ_KEY_UP;
			} else {
				mSumDeltaY += DeltaMin;
				tmpKey = OJ_KEY_DOWN;
			}
		} else
			tmpKey = OJ_KEY_NONE;
	} else
		tmpKey = OJ_KEY_NONE;

#else
	if ((10*abs(mSumDeltaY) > (Ratio*abs(mSumDeltaX)))
			&& (abs(mSumDeltaY) > DeltaMin)) {
		if (mSumDeltaY > 0)
			tmpKey = OJ_KEY_UP;
		else
			tmpKey = OJ_KEY_DOWN;
	} else	if (abs(mSumDeltaX) > DeltaMin) {
		if (mSumDeltaX > 0)
			tmpKey = OJ_KEY_RIGHT;
		else
			tmpKey = OJ_KEY_LEFT;
	} else
		tmpKey = OJ_KEY_NONE;
#endif
	return tmpKey;
}

static void curcial_oj_work_func(struct work_struct *work)
{
	struct curcial_oj_platform_data *oj = container_of(work, struct curcial_oj_platform_data, work);
	OJData_T  OJData;
	OJTouchEvt_T  OJTouchEvt;
	uint16_t i, j;
	uint8_t data[BURST_DATA_SIZE];
	uint32_t click_time = 0;
	uint8_t	count = 0;
	OJKeyEvt_T	evtKey = OJ_KEY_NONE;
	uint8_t	x_count = 0;
	uint8_t	y_count = 0;
	bool out = false;
	uint8_t pxsum;
	uint16_t sht;

	curcial_oj_polling_mode(OJ_POLLING_DISABLE);

	mSumDeltaX = 0;
	mSumDeltaY = 0;


	if (machine_is_legend())
		i = 0x00;
	else 	if (machine_is_incrediblec())
		i = 0x01; /*incrediblec */

	if (oj->softclick == true && system_rev == i) {
		for (i = 1; i < 31; i++) {
			curcial_oj_burst_read(data);
			OJData.squal = data[SQUAL];
			if (debugflag) {
				printk(KERN_INFO"OJ:M=0x%02x Y=0x%02x X=0x%02x SQUAL=%d\n", data[0], data[1], data[2], data[SQUAL]);
			}

			if (oj->swap == false) {
				OJData.deltaX = (oj->x)*((int8_t) data[X]);
				OJData.deltaY = (oj->y)*((int8_t) data[Y]);
			} else {
				OJData.deltaX = (oj->y)*((int8_t) data[Y]);
				OJData.deltaY = (oj->x)*((int8_t) data[X]);
			}

			mSumDeltaX += -(OJData.deltaX);
			mSumDeltaY += -(OJData.deltaY);
			if (i % 3 == 0)
				evtKey = OJ_ProcessNavi(15, 20);

			if (evtKey != OJ_KEY_NONE) {
				if (evtKey == OJ_KEY_LEFT || evtKey == OJ_KEY_RIGHT) {
					if (1)
						printk(KERN_INFO"softclick:OJ:mSumDeltaX = %d mSumDeltaY = %d \n", mSumDeltaX, mSumDeltaY);
					x_count = abs(mSumDeltaX) / 10;
					if (x_count >= 30)
						x_count = 29;
					for (j = 0; j <= oj->Xsteps[x_count]; j++) {
						if (evtKey == OJ_KEY_LEFT) {
							input_report_rel(oj->input_dev, REL_X, -1);
							/*printk(KERN_INFO"OJ:KEY_LEFT\n");*/
						} else if (evtKey == OJ_KEY_RIGHT) {
							input_report_rel(oj->input_dev, REL_X, 1);
							/*printk(KERN_INFO"OJ:KEY_RIGHT\n");*/
						}
						input_sync(oj->input_dev);
					}
					mSumDeltaX = 0;
					mSumDeltaY = 0;
				} else	if (evtKey == OJ_KEY_DOWN || evtKey == OJ_KEY_UP) {
					if (1)
						printk(KERN_INFO"softclick:OJ:mSumDeltaX = %d mSumDeltaY = %d \n", mSumDeltaX, mSumDeltaY);
					y_count = abs(mSumDeltaY) / 10;
					if (y_count >= 30)
						y_count = 29;
					for (j = 0; j <= oj->Ysteps[y_count]; j++) {
						if (evtKey == OJ_KEY_DOWN) {
							input_report_rel(oj->input_dev, REL_Y, 1);
							/*printk(KERN_INFO"OJ:KEY_DOWN\n");*/
						} else if (evtKey == OJ_KEY_UP) {
							input_report_rel(oj->input_dev, REL_Y, -1);
							/*printk(KERN_INFO"OJ:KEY_UP\n");*/
						}
						input_sync(oj->input_dev);
					}
					mSumDeltaX = 0;
					mSumDeltaY = 0;
				}
				break;
			}

			OJTouchEvt = OJ_SoftClick_Event(&OJData);

			if (OJTouchEvt == OJ_TOUCH_CLICK_EVT) {
				click_time = jiffies_to_msecs(jiffies);
				if (click_time - oj->last_click_time > click_interval)
					oj->click	=	false;

				if (oj->click) {
					if (click_time - oj->last_click_time < click_interval) {
						OJData.key = BTN_MOUSE;
						oj->click = false;
					}
				} else {
					oj->click = true;
					printk(KERN_INFO"%s:CLICK\n", __func__);
				}
				oj->last_click_time = click_time;

			}

			if (data[SQUAL] < 30) {
				if (count++ == 3)
				break;
			}
			if (polling_delay1)
				mdelay(polling_delay1);
			if (polling_delay2)
				msleep(polling_delay2);
		}

		if (OJData.key == BTN_MOUSE) {
			input_report_key(oj->input_dev, OJData.key, 1);
			input_report_key(oj->input_dev, OJData.key, 0);
			printk(KERN_INFO"OJ: BTN_MOUSE\n");
		}
	} else if (ap_code) {
		normal_th = oj->normal_th;
/*
		click_time = jiffies_to_msecs(jiffies);
		if (click_time - oj->last_click_time >  300) {
			x_sum = 0;
			y_sum = 0;
		}
		oj->last_click_time = click_time;
		interval = oj->interval;
*/
		do {
			memset(data, 0x00, sizeof(data));
			out = false;
			curcial_oj_burst_read(data);
			OJData.squal = data[SQUAL];
			pxsum = curcial_oj_register_read(0x09);
			sht =	((data[SHUTTER_UPPER] << 8)|data[SHUTTER_LOWER]);
			if (debugflag) {
				printk(KERN_INFO"OJ1:M=0x%02x Y=0x%02x X=0x%02x SQUAL=0x%02x "
				"SHU_U=0x%02x SHU_L=0x%02x pxsum=%d sht=%d  \n", data[MOTION], data[Y], data[X],
				 data[SQUAL], data[SHUTTER_UPPER], data[SHUTTER_LOWER], pxsum, sht);
			}

			for (i = 1; i < oj->degree; i++) {
				if (((oj->sht_tbl[i-1] < sht) && (sht <= oj->sht_tbl[i])) && (oj->pxsum_tbl[i] < pxsum)) {
					if (debugflag)
						printk("OJ:A.code_condition:%d\n", i);
					out = true;
					break;
				}
			}
			if (!out)
				goto exit;

			oj->oj_adjust_xy(data, &mSumDeltaX, &mSumDeltaY);
			if (debugflag)
				printk(KERN_INFO"check:OJ:mSumDeltaX = %d mSumDeltaY = %d \n", mSumDeltaX, mSumDeltaY);

			evtKey = OJ_ProcessNavi(xy_ratio, normal_th);

			if (evtKey != OJ_KEY_NONE) {
				click_time = jiffies_to_msecs(jiffies);
				/*delta_time = click_time - oj->last_click_time;*/
				if (debugflag)
					printk(KERN_INFO"click_time=%x last_click_time=%x, %x\n", click_time, oj->last_click_time, click_time-oj->last_click_time);
/*
					if (delta_time != 0) {
						if (evtKey == OJ_KEY_LEFT || evtKey == OJ_KEY_RIGHT) {
							x_sum += mSumDeltaX;
						}	else if (evtKey == OJ_KEY_DOWN || evtKey == OJ_KEY_UP) {
							y_sum += mSumDeltaY;
						}
					}
				if (debugflag)
					printk(KERN_INFO"check:OJ:x_sum = %d y_sum = %d delta_time= %d \n", x_sum, y_sum, delta_time);
*/
			}

			if (click_time-oj->last_click_time < interval) {
					evtKey = OJ_KEY_NONE;
					if (debugflag)
						printk(KERN_INFO"OJ_KEY_NONE < %d\n", interval);
					normal_th = oj->fast_th;
			}
			if (evtKey == OJ_KEY_LEFT) {
				x_count = oj->Xsteps[abs(mSumDeltaX) / 10];
				x_sum = 0;
				for (j = 0; j < x_count; j++) {
					input_report_rel(oj->input_dev, REL_X, -1);
					input_sync(oj->input_dev);
				}
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_LEFT:%d\n", x_count);
			} else if (evtKey == OJ_KEY_RIGHT) {
				x_count = oj->Xsteps[abs(mSumDeltaX) / 10];
				x_sum = 0;
				for (j = 0; j < x_count; j++) {
					input_report_rel(oj->input_dev, REL_X, 1);
					input_sync(oj->input_dev);
				}
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_RIGHT:%d\n", x_count);
			} else if (evtKey == OJ_KEY_DOWN) {
				y_count = oj->Ysteps[abs(mSumDeltaY) / 10];
				y_sum = 0;
				for (j = 0; j < y_count; j++) {
					input_report_rel(oj->input_dev, REL_Y, 1);
					input_sync(oj->input_dev);
				}
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_DOWN:%d\n", y_count);
			} else if (evtKey == OJ_KEY_UP) {
				y_count = oj->Ysteps[abs(mSumDeltaY) / 10];
				y_sum = 0;
				for (j = 0; j < y_count; j++) {
					input_report_rel(oj->input_dev, REL_Y, -1);
					input_sync(oj->input_dev);
				}
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_UP:%d\n", y_count);
			}

			if (evtKey != OJ_KEY_NONE) {
				oj->key = evtKey;
				oj->last_click_time = jiffies_to_msecs(jiffies);
			}

			mSumDeltaX = 0;
			mSumDeltaY = 0;
			if (polling_delay1)
				mdelay(polling_delay1);
			if (polling_delay2)
				msleep(polling_delay2);
			} while ((data[0] & 0x80) && (suspend == false));
	} else {
    curcial_oj_burst_read(data);
    OJData.squal = data[SQUAL];

    if (debugflag) {
			printk(KERN_INFO"OJ:M=0x%02x Y=0x%02x X=0x%02x SQUAL=%d\n", data[0], data[1], data[2], data[SQUAL]);
    }

    oj->oj_adjust_xy(data, &mSumDeltaX, &mSumDeltaY);
    if (debugflag)
       printk(KERN_INFO"check:OJ:mSumDeltaX = %d mSumDeltaY = %d \n", mSumDeltaX, mSumDeltaY);

    click_time = jiffies_to_msecs(jiffies);

    if (continue_count > continue_th && (click_time - oj->last_click_time < interval))
			evtKey = OJ_ProcessNavi(xy_ratio, fast_th);
    else
			evtKey = OJ_ProcessNavi(xy_ratio, normal_th);

    if (evtKey == OJ_KEY_LEFT || evtKey == OJ_KEY_RIGHT) {

			if (click_time - oj->last_click_time < interval) {
				continue_count++;
			} else
				continue_count = 0;

			if (continue_max != 0 && continue_count > continue_th + continue_max)
				continue_count = 0;

			if (debugflag)
				printk(KERN_INFO"check:OJ:continue_count = %d \n", continue_count);

			x_count = oj->Xsteps[abs(mSumDeltaX) / 10];

			if (continue_count > continue_th)
				x_count = x_send_count;

			for (j = 0; j < x_count; j++) {
				if (evtKey == OJ_KEY_LEFT) {
					input_report_rel(oj->input_dev, REL_X, -1);
					if (debugflag)
						printk(KERN_INFO"OJ:KEY_LEFT\n");
				} else if (evtKey == OJ_KEY_RIGHT) {
					input_report_rel(oj->input_dev, REL_X, 1);
					if (debugflag)
						printk(KERN_INFO"OJ:KEY_RIGHT\n");
				}
			}
			oj->last_click_time  = jiffies_to_msecs(jiffies);
			input_sync(oj->input_dev);
			mSumDeltaX = 0;
			mSumDeltaY = 0;

     } else if (evtKey == OJ_KEY_DOWN || evtKey == OJ_KEY_UP) {
			if (click_time - oj->last_click_time < interval) {
				continue_count++;
			} else
				continue_count = 0;

      if (continue_max != 0 && continue_count > continue_th + continue_max)
				continue_count = 0;

      if (debugflag)
				printk(KERN_INFO"check:OJ:continue_count = %d \n", continue_count);

			y_count = oj->Ysteps[abs(mSumDeltaY) / 10];

			if (continue_count > continue_th)
				y_count = y_send_count;

			for (j = 0; j < y_count; j++) {
				if (evtKey == OJ_KEY_DOWN) {
					input_report_rel(oj->input_dev, REL_Y, 1);
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_DOWN\n");
				} else if (evtKey == OJ_KEY_UP) {
					input_report_rel(oj->input_dev, REL_Y, -1);
				if (debugflag)
					printk(KERN_INFO"OJ:KEY_UP\n");
			}
		}

		oj->last_click_time  = jiffies_to_msecs(jiffies);
		input_sync(oj->input_dev);
		mSumDeltaX = 0;
		mSumDeltaY = 0;
		}
		if (polling_delay1)
			mdelay(polling_delay1);
		if (polling_delay2)
			msleep(polling_delay2);
	}

exit:
	gTouchEvt = OJ_TOUCH_NONE_EVT;
	if (debugflag)
		printk(KERN_INFO"%s:-\n", __func__);
	if (suspend == false)
		curcial_oj_polling_mode(OJ_POLLING_ENABLE);
	else
		curcial_oj_polling_mode(OJ_POLLING_DISABLE);
}


static ssize_t oj_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	uint8_t i,j = 20;
	while(j--) {
	i = curcial_oj_register_read(0x05);
	printk("SQUAL = 0x%x\n",i);
	i = curcial_oj_register_read(0x00);
	printk("ID = 0x%x\n",i);
	mdelay(10);
	}

	return sprintf(buf,
				"interval=%d fast_th=%d normal_th=%d continue_th=%d x_send_count=%d y_send_count=%d system_rev=%d"
				" debugflag=%d polling_delay1=%d xy_ratio=%d polling_delay2=%d ap_code=%d send_count=%d\n",
				 interval, fast_th, normal_th, continue_th, x_send_count, y_send_count, system_rev, debugflag,
				  polling_delay1, xy_ratio, polling_delay2, ap_code , send_count);

}
static ssize_t oj_send_count_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	send_count = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_ap_code_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	ap_code = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_interval_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	interval = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_fast_th_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	fast_th = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_normal_th_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	normal_th = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_polling_delay1_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	polling_delay1 = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_polling_delay2_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	polling_delay2 = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_xy_ratio_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	xy_ratio = simple_strtoull(buf, NULL, 10);

	return count;
}
static ssize_t oj_debugflag_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	debugflag = simple_strtoull(buf, NULL, 10);

	return count;
}

static ssize_t oj_continue_th_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	continue_th = simple_strtoull(buf, NULL, 10);
	return count;
}
static ssize_t oj_continue_max_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	continue_max = simple_strtoull(buf, NULL, 10);
	return count;
}
static ssize_t oj_x_send_count_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	x_send_count = simple_strtoull(buf, NULL, 10);
	return count;
}
static ssize_t oj_y_send_count_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	y_send_count = simple_strtoull(buf, NULL, 10);
	return count;
 }

static ssize_t oj_xtable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{

	char *buffer,*endptr;
	int i;
	buffer = (char *)buf;

	i= simple_strtoull(buffer, &endptr, 10);
	buffer = endptr+1;

	if (i <= 30)
		my_oj->Xsteps[i-1] = simple_strtoull(buffer, &endptr, 10);

	return count;
}
static ssize_t oj_ytable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	char *buffer,*endptr;
	int i;
	buffer = (char *)buf;;


	i= simple_strtoull(buffer, &endptr, 10);
	buffer = endptr+1;

	if (i <= 30)
		my_oj->Ysteps[i-1] = simple_strtoull(buffer, &endptr, 10);

	return count;
}
static ssize_t oj_xtable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	char log[128];
	int i,p;

	for (i = 0, p = 0; i < 30 ; i++)	{
		p += sprintf(log+p, "%d,",  my_oj->Xsteps[i]);
	}
	return sprintf(buf,"X_table:%s\n",log);

}
static ssize_t oj_ytable_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	char log[128];
	int i,p;

	for (i = 0, p = 0; i < 30 ; i++)	{
		p += sprintf(log+p, "%d,",  my_oj->Ysteps[i]);
	}
	return sprintf(buf,"Y_table:%s\n",log);

}
static DEVICE_ATTR(send_count, 0644, oj_show, oj_send_count_store);
static DEVICE_ATTR(ap_code, 0644, oj_show, oj_ap_code_store);
static DEVICE_ATTR(interval, 0644, oj_show, oj_interval_store);
static DEVICE_ATTR(fast_th, 0644, oj_show, oj_fast_th_store);
static DEVICE_ATTR(normal_th, 0644, oj_show, oj_normal_th_store);
static DEVICE_ATTR(polling_delay1, 0644, oj_show, oj_polling_delay1_store);
static DEVICE_ATTR(polling_delay2, 0644, oj_show, oj_polling_delay2_store);
static DEVICE_ATTR(xy_ratio, 0644, oj_show, oj_xy_ratio_store);
static DEVICE_ATTR(debugflag, 0644, oj_show, oj_debugflag_store);
static DEVICE_ATTR(x_send_count, 0644, oj_show, oj_x_send_count_store);
static DEVICE_ATTR(y_send_count, 0644, oj_show, oj_y_send_count_store);
static DEVICE_ATTR(continue_th, 0644, oj_show, oj_continue_th_store);
static DEVICE_ATTR(continue_max, 0644, oj_show, oj_continue_max_store);
static DEVICE_ATTR(xtable, 0644, oj_xtable_show, oj_xtable_store);
static DEVICE_ATTR(ytable, 0644, oj_ytable_show, oj_ytable_store);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void curcial_oj_early_suspend(struct early_suspend *h)
{
	struct curcial_oj_platform_data *oj;
	suspend = true;
	oj = container_of(h, struct curcial_oj_platform_data, early_suspend);
	printk(KERN_ERR"%s: enter\n", __func__);
	oj->oj_shutdown(1);
	curcial_oj_polling_mode(OJ_POLLING_DISABLE);
	if (oj->share_power == false) {
		oj->oj_poweron(OJ_POWEROFF);
	}
	microp_spi_vote_enable(SPI_OJ, 0);

}

static void curcial_oj_late_resume(struct early_suspend *h)
{
	struct curcial_oj_platform_data	*oj;
	suspend = false;
	oj = container_of(h, struct curcial_oj_platform_data, early_suspend);
	printk(KERN_ERR"%s: enter\n", __func__);
	curcial_oj_module_init();
}
#endif

static int __devinit curcial_oj_probe(struct platform_device *pdev)
{
	struct curcial_oj_platform_data *oj = pdev->dev.platform_data;
	int err;
	int i;

	my_oj = oj;
	if (!curcial_oj_module_init()) {
		printk(KERN_ERR "Unable to init OJ module\n");
		err = -EINVAL;
		goto fail;
	}

	INIT_WORK(&oj->work, curcial_oj_work_func);

	curcial_wq = create_singlethread_workqueue("curcial_wq");
	if (!curcial_wq) {
		err = -ENOMEM;
		goto fail;
	}


	oj->input_dev = input_allocate_device();
	if (!oj->input_dev) {
		printk(KERN_ERR "Unable to allocate device for OJ\n");
		err = -ENOMEM;
		goto fail;
	}

	oj->input_dev->name = "curcial-oj";


	oj->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REL);
	input_set_capability(oj->input_dev, EV_KEY, BTN_MOUSE);
	oj->input_dev->relbit[0] = BIT_MASK(REL_X) | BIT_MASK(REL_Y);


	for(i = 0; i < ARRAY_SIZE(keymap); i++)
		set_bit(keymap[i], oj->input_dev->keybit);

	err = input_register_device(oj->input_dev);
	if (err) {
		printk(KERN_ERR "Unable to register %s input device\n", oj->input_dev->name);
		goto fail;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	oj->early_suspend.suspend = curcial_oj_early_suspend;
	oj->early_suspend.resume = curcial_oj_late_resume;
/*	oj->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;*/
	register_early_suspend(&oj->early_suspend);
#endif
	err = 	device_create_file(&(pdev->dev), &dev_attr_send_count);
	err = 	device_create_file(&(pdev->dev), &dev_attr_ap_code);
	err = 	device_create_file(&(pdev->dev), &dev_attr_interval);
	err = 	device_create_file(&(pdev->dev), &dev_attr_fast_th);
	err = 	device_create_file(&(pdev->dev), &dev_attr_normal_th);
	err = 	device_create_file(&(pdev->dev), &dev_attr_continue_th);
	err = 	device_create_file(&(pdev->dev), &dev_attr_continue_max);
	err = 	device_create_file(&(pdev->dev), &dev_attr_x_send_count);
	err = 	device_create_file(&(pdev->dev), &dev_attr_y_send_count);
	err = 	device_create_file(&(pdev->dev), &dev_attr_polling_delay1);
	err = 	device_create_file(&(pdev->dev), &dev_attr_polling_delay2);
	err = 	device_create_file(&(pdev->dev), &dev_attr_xy_ratio);
	err = 	device_create_file(&(pdev->dev), &dev_attr_debugflag);
	err = 	device_create_file(&(pdev->dev), &dev_attr_xtable);
	err = 	device_create_file(&(pdev->dev), &dev_attr_ytable);

	fast_th = my_oj->fast_th;
	normal_th = my_oj->normal_th;
	continue_th = my_oj->continue_th;
	continue_max = my_oj->continue_max;
	x_send_count = my_oj->x_send_count;
	y_send_count = my_oj->y_send_count;
	xy_ratio = my_oj->xy_ratio;
	interval = my_oj->interval;
	polling_delay1 = my_oj->mdelay_time;
	polling_delay2 = my_oj->msleep_time;
	debugflag = my_oj->debugflag;
	ap_code = my_oj->ap_code;
	send_count = my_oj->send_count;
	printk(KERN_INFO "OJ: driver loaded\n");
	return 0;

fail:
	if (oj->share_power == false) {
		oj->oj_poweron(OJ_POWEROFF);
	}
	microp_spi_vote_enable(SPI_OJ, 0);

	if (oj->input_dev) {
		input_free_device(oj->input_dev);
	}

	if (curcial_wq)
		destroy_workqueue(curcial_wq);

	if (oj_proc_entry)
		remove_proc_entry("oj", NULL);

	return err;
}

static int __devexit curcial_oj_remove(struct platform_device *pdev)
{
	struct curcial_oj_platform_data *oj = pdev->dev.platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (oj->early_suspend.suspend && oj->early_suspend.resume)
		unregister_early_suspend(&oj->early_suspend);
#endif
	if (oj->share_power == false) {
		oj->oj_poweron(OJ_POWEROFF);
	}
	microp_spi_vote_enable(SPI_OJ, 0);

	if (oj->input_dev) {
		input_unregister_device(oj->input_dev);
		input_free_device(oj->input_dev);
	}

	if (curcial_wq)
		destroy_workqueue(curcial_wq);

	if (oj_proc_entry)
		remove_proc_entry("oj", NULL);

	device_remove_file(&(pdev->dev), &dev_attr_send_count);
	device_remove_file(&(pdev->dev), &dev_attr_ap_code);
	device_remove_file(&(pdev->dev), &dev_attr_interval);
	device_remove_file(&(pdev->dev), &dev_attr_fast_th);
	device_remove_file(&(pdev->dev), &dev_attr_normal_th);
	device_remove_file(&(pdev->dev), &dev_attr_polling_delay1);
	device_remove_file(&(pdev->dev), &dev_attr_polling_delay2);
	device_remove_file(&(pdev->dev), &dev_attr_xy_ratio);
	device_remove_file(&(pdev->dev), &dev_attr_debugflag);
	device_remove_file(&(pdev->dev), &dev_attr_x_send_count);
	device_remove_file(&(pdev->dev), &dev_attr_y_send_count);
	device_remove_file(&(pdev->dev), &dev_attr_continue_th);
	device_remove_file(&(pdev->dev), &dev_attr_continue_max);
	device_remove_file(&(pdev->dev), &dev_attr_xtable);
	device_remove_file(&(pdev->dev), &dev_attr_ytable);
	printk(KERN_INFO "OJ: driver unloaded\n");
	return 0;
}

static int __init curcial_oj_init(void)
{
	return platform_driver_register(&curcial_oj_device_driver);
}

static void __exit curcial_oj_exit(void)
{
	platform_driver_unregister(&curcial_oj_device_driver);
}

module_init(curcial_oj_init);
module_exit(curcial_oj_exit);

void curcial_oj_send_key(unsigned int code, int value)
{
	if ((my_oj != NULL) && (my_oj->input_dev != NULL))
		input_report_key(my_oj->input_dev, code, value);
	else
		printk(KERN_WARNING "%s: device not ready...\n", __func__);
}

MODULE_DESCRIPTION("Crucial OpticalJoystick Driver");
MODULE_LICENSE("GPL");
