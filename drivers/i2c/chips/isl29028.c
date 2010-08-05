/*
 * Copyright (C) 2009 HTC, Inc.
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

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <mach/isl29028.h>
#include <linux/capella_cm3602.h>

#define D(x...) pr_info(x)

static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

struct isl29028_info {

	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
	struct i2c_client *ps_i2c_client;

	struct early_suspend early_suspend;
	struct i2c_client *client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int ls_enable;
	int ps_enable;
	uint16_t *adc_table;
	int irq;

	int is_suspend;
	int ls_calibrate;

	int (*power)(int);
};

struct isl29028_info *lp_info;

static int ls_enable_flag;
static int psensor_opened;
static int psensor_irq_flag;

static uint16_t get_ls_adc_value(void)
{
	uint16_t value, tmp_value;
	struct isl29028_info *lpi = lp_info;
	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_LS_DATA1);
	tmp_value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_LS_DATA2);
	value = value | tmp_value << 8;
	return value & 0x0fff;
}

static uint16_t get_ps_adc_value(void)
{
	uint16_t value;
	struct isl29028_info *lpi = lp_info;
	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_PROX_DATA);
	return value & 0xff;
}

static int _isl29028_set_reg_bit(struct i2c_client *client, u8 set, u8 cmd, u8 data)
{
	u8 value;
	int ret = 0;

	value = i2c_smbus_read_byte_data(client, cmd);
	if (value < 0)
		return -EIO;
	if (set)
		value |= data;
	else
		value &= ~data;
	ret = i2c_smbus_write_byte_data(client, cmd, value);
	if (ret < 0)
		return -EIO;

	return ret;
}

static int set_lsensor_range(uint16_t lt, uint16_t ht)
{
	uint16_t value = 0;
	int ret;
	struct isl29028_info *lpi = lp_info;

	value = (lt >> 8) & 0x0f;
	value |= ht << 4;
	value &= 0xff;

	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH1, lt & 0xff);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_LS_TH1 fail\n", __func__);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH2, value);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_LS_TH2 fail\n", __func__);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH3, (ht >> 4) & 0xff);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_LS_TH3 fail\n", __func__);
		return ret;
	}

/*
	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH1);
	D("TH1--------->0x%03X\n", value);
	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH2);
	D("TH2--------->0x%03X\n", value);
	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_LS_TH3);
	D("TH3--------->0x%03X\n", value);
*/
	return ret;
}

static int set_psensor_range(u8 lt, u8 ht)
{
	int ret;
	struct isl29028_info *lpi = lp_info;

	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_PROX_LT, lt);
	if (ret < 0)
		return ret;
	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29828_PROX_HT, ht);
	if (ret < 0)
		return ret;

	return ret;
}


static void report_psensor_input_event(struct isl29028_info *lpi)
{
	int val, ret;
/*
	adc_value = get_ps_adc_value();
	lt = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_PROX_LT);
	ht = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29828_PROX_HT);
	D("%s: ADC = 0x%03X, LT = 0x%03X HT = 0x%03X\n", __func__, adc_value, lt, ht);
	if (adc_value >= ht)
		val = 0;
	else
		val = 1;
*/

	val = gpio_get_value(lpi->intr_pin);

	D("proximity %d\n", val);

	/* 0 is close, 1 is far */
	input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
	input_sync(lpi->ps_input_dev);

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 0, \
		ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
	if (ret < 0)
		pr_err("%s: clear lsensor intr flag fail\n", __func__);
}

static void report_lsensor_input_event(struct isl29028_info *lpi)
{
	uint16_t adc_value;
	int level, i, ret;

	adc_value = get_ls_adc_value();

	for (i = 0; i < 10; i++) {
		if (adc_value <= (*(lpi->adc_table + i))) {
			level = i;
			if (*(lpi->adc_table + i))
				break;
		}
	}
	ret = set_lsensor_range(*(lpi->adc_table + (i - 1)), \
		*(lpi->adc_table + i));
	if (ret < 0)
		printk(KERN_ERR "%s fail\n", __func__);

	D("%s: ADC = 0x%X, Level = %d \n", __func__, adc_value, level);
	input_report_abs(lpi->ls_input_dev,
			ABS_MISC, level);
	input_sync(lpi->ls_input_dev);

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 0, \
		ISL29028_INTERRUPT, ISL29028_INT_ALS_FLAG);
	if (ret < 0)
		pr_err("%s: clear lsensor intr flag fail\n", __func__);

}

static void sensor_irq_do_work(struct work_struct *work)
{
	uint8_t intrrupt;
	int ret;
	struct isl29028_info *lpi = lp_info;

	D("%s\n", __func__);
	intrrupt = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_INTERRUPT);
	if (intrrupt & ISL29028_INT_PROX_FLAG) {
		ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 0, \
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);

		psensor_irq_flag = 1;
		set_irq_type(lpi->irq, IRQF_TRIGGER_RISING);
		report_psensor_input_event(lpi);

	} else if (intrrupt & ISL29028_INT_ALS_FLAG) {
		report_lsensor_input_event(lpi);
	} else if (psensor_irq_flag) {

		psensor_irq_flag = 0;
		report_psensor_input_event(lpi);

		set_irq_type(lpi->irq, IRQF_TRIGGER_FALLING);
		ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 1, \
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);
	}
}

static irqreturn_t isl29028_irq_handler(int irq, void *data)
{
	struct isl29028_info *lpi = data;

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int psensor_enable \
	(struct isl29028_info *lpi)
{
	int ret;

	D("%s\n", __func__);
	if (lpi->ps_enable) {
		D("%s: already enabled\n", __func__);
		return 0;
	}

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 1, \
		ISL29028_CONFIGURE, ISL29028_PROX_EN);
	if (ret < 0) {
		pr_err("%s: enable psensor fail\n", __func__);
		return ret;
	}

	lpi->ps_enable = 1;
	return ret;
}

static int psensor_disable \
	(struct isl29028_info *lpi)
{
	int ret = -EIO;

	D("%s\n", __func__);
	if (!lpi->ps_enable) {
		D("%s: already disabled\n", __func__);
		return 0;
	}

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 0, \
		ISL29028_CONFIGURE, ISL29028_PROX_EN);
	if (ret < 0) {
		pr_err("%s: disable psensor fail\n", __func__);
		return ret;
	}

	lpi->ps_enable = 0;
	return ret;
}

static int psensor_open \
	(struct inode *inode, struct file *file)
{
	D("%s\n", __func__);
	if (psensor_opened)
		return -EBUSY;
	psensor_opened = 1;
	return 0;
}

static int psensor_release \
	(struct inode *inode, struct file *file)
{
	struct isl29028_info *lpi = lp_info;
	D("%s\n", __func__);
	psensor_opened = 0;
	return psensor_disable(lpi);
}

static long psensor_ioctl \
	(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	struct isl29028_info *lpi = lp_info;

	D("%s cmd %d\n", __func__, _IOC_NR(cmd));
	switch (cmd) {
	case CAPELLA_CM3602_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val)
			return psensor_enable(lpi);
		else
			return psensor_disable(lpi);
		break;
	case CAPELLA_CM3602_IOCTL_GET_ENABLED:
		return put_user(lpi->ps_enable, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}
}

static struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};

static int lightsensor_enable(struct isl29028_info *lpi)
{
	int ret;

	pr_info("%s\n", __func__);

	ls_enable_flag = 1;
	if (lpi->is_suspend) {
		pr_err("%s: system is suspended\n", __func__);
		return 0;
	}
	if (!lpi->ls_enable) {
		ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 1, \
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: set auto light sensor fail\n", __func__);
		else {
			lpi->ls_enable = 1;
			/* report an invalid value first to ensure we trigger an event
			* when adc_level is zero.
			*/
			input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
			input_sync(lpi->ls_input_dev);
		}
	}
	return 0;
}

static int lightsensor_disable(struct isl29028_info *lpi)
{
	int ret;

	pr_info("%s\n", __func__);

	ls_enable_flag = 0;
	if (lpi->is_suspend) {
		pr_err("%s: microp is suspended\n", __func__);
		return 0;
	}

	if (lpi->ls_enable) {
		ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 0, \
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: disable auto light sensor fail\n",
		       __func__);
		else
			lpi->ls_enable = 0;
	}
	return 0;
}

DEFINE_MUTEX(ls_i2c_api_lock);
static int lightsensor_opened;

static int lightsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	if (lightsensor_opened) {
		pr_err("%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lightsensor_opened = 1;
	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	pr_debug("%s\n", __func__);
	mutex_lock(&ls_i2c_api_lock);
	lightsensor_opened = 0;
	mutex_unlock(&ls_i2c_api_lock);
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct isl29028_info *lpi = lp_info;

	mutex_lock(&ls_i2c_api_lock);
	pr_debug("%s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		printk(KERN_INFO "%s value = %d\n", __func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->ls_enable;
		pr_debug("%s enabled %d\n", __func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	mutex_unlock(&ls_i2c_api_lock);
	return rc;
}

static struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ps_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;

	value = get_ps_adc_value();
	ret = sprintf(buf, 	"ADC[0x%03X]\n", value);

	return ret;
}

static ssize_t ps_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	uint8_t value;
	int ls_auto;
	int ret;
	struct isl29028_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, (ls_auto ? 1 : 0), \
		ISL29028_CONFIGURE, ISL29028_PROX_EN);
	if (ret < 0)
		pr_err("%s: ps enable fail\n", __func__);

	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_CONFIGURE);
	D("%s: register :[0x%03X]\n", __func__, value);

	return count;
}

static DEVICE_ATTR(ps_adc, 0666, ps_adc_show, ps_enable_store);


static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{

	uint16_t value;
	int ret;

	value = get_ls_adc_value();
	ret = sprintf(buf, 	"ADC[0x%03X]\n", value);

	return ret;
}

static DEVICE_ATTR(ls_adc, 0666, ls_adc_show, NULL);

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	uint8_t value;
	int ret;
	struct isl29028_info *lpi = lp_info;

	value = i2c_smbus_read_byte_data(lpi->ps_i2c_client, ISL29028_CONFIGURE);
	ret = sprintf(buf, "Light sensor Auto Enable = %d\n", (value & 0x04) ? 1 : 0);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{

	int ls_auto;
	int ret;
	struct isl29028_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		lpi->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		lpi->ls_enable = 1;
	} else {
		lpi->ls_calibrate = 0;
		lpi->ls_enable = 0;
	}

	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, (lpi->ls_enable ? 1 : 0), \
		ISL29028_CONFIGURE, ISL29028_ALS_EN);
	if (ret < 0)
		pr_err("%s: ls enable fail\n", __func__);

	return count;
}

static DEVICE_ATTR(ls_auto, 0666, \
	ls_enable_show, ls_enable_store);


static int lightsensor_setup(struct isl29028_info *lpi)
{
	int ret;
	/*Light Sensor*/
	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err("%s: could not allocate ls input device\n", __func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("%s: can not register ls input device\n",
				__func__);
		return ret;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("%s: can not register ls misc device\n",
				__func__);
		return ret;
	}
	return ret;
}

static int psensor_setup(struct isl29028_info *lpi)
{
	int ret;

	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		pr_err("%s: could not allocate ps input device\n", __func__);
		return -ENOMEM;
	}
	lpi->ps_input_dev->name = "proximity";

	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	ret = input_register_device(lpi->ps_input_dev);
	if (ret < 0) {
		pr_err("%s: could not register ps input device\n", __func__);
		return ret;
	}

	ret = misc_register(&psensor_misc);
	if (ret < 0) {
		pr_err("%s: could not register ps misc device\n", __func__);
		return ret;
	}
	return ret;
}


static int isl29028_setup(struct isl29028_info *lpi)
{
	int irq, ret = 0;
	ret = gpio_request(lpi->intr_pin, "gpio_isl29028_intr");
	if (ret < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}

	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, lpi->intr_pin, ret);
		return ret;
	}
	irq = gpio_to_irq(lpi->intr_pin);
	if (irq < 0) {
		pr_err("%s: failed to set gpio to irq (%d)\n",
			__func__, lpi->intr_pin);
		return irq;
	}
	lpi->irq = irq;
	ret = request_irq(irq,
			isl29028_irq_handler,
			IRQF_TRIGGER_FALLING,
			"isl29028",
			lpi);
	if (ret < 0) {
		pr_err("%s: request_irq(%d) failed for gpio %d (%d)\n",
			__func__, irq,
			lpi->intr_pin, ret);
		return ret;
	}

	ret = set_irq_wake(irq, 1);
	if (ret < 0) {
		pr_err("%s: failed to set irq %d as a wake interrupt\n",
			__func__, irq);
		return ret;
	}

	lpi->power(1);
	msleep(200);

	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_CONFIGURE, 0xB0);
	if (ret < 0) {
		pr_err("%s: failed to set LPS Configuration\n", __func__);
		return ret;
	}
	ret = set_lsensor_range(0xF00, 0xFFF);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_LS_TH123 fail\n", __func__);
		return ret;
	}
	ret = set_psensor_range(0x08, 0x20);
	if (ret < 0) {
		printk(KERN_ERR "%s : write ISL29028_PS_LT_HT fail\n", __func__);
		return ret;
	}
	ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 1, \
		ISL29028_INTERRUPT, ISL29028_INT_ALS_PRST);
	if (ret < 0) {
		pr_err("%s: set ALS PRST fail\n", __func__);
	}
	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void isl29028_early_suspend(struct early_suspend *h)
{

	int ret;
	struct isl29028_info *lpi = lp_info;

	lpi->is_suspend = 1;
	ret = i2c_smbus_write_byte_data(lpi->ps_i2c_client, ISL29028_CONFIGURE, 0x30);
	if (ret < 0)
		pr_err("%s: disable sensor fail\n",
			__func__);
	else {
		lpi->ls_enable = 0;
		lpi->ps_enable = 0;
	}
	/*lpi->power(0);*/
}

static void isl29028_late_resume(struct early_suspend *h)
{
	int ret;
	struct isl29028_info *lpi = lp_info;

	/*lpi->power(1);*/
	/*msleep(200);*/
	if (ls_enable_flag) {
		ret = _isl29028_set_reg_bit(lpi->ps_i2c_client, 1, \
			ISL29028_CONFIGURE, ISL29028_ALS_EN);
		if (ret < 0)
			pr_err("%s: enable sensor fail\n",
					__func__);
		else {
			ls_enable_flag = 0;
			lpi->ls_enable = 1;
			msleep(100);
			input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
			input_sync(lpi->ls_input_dev);
		}
	}
	lpi->is_suspend = 0;

}
#endif

static int isl29028_probe(struct i2c_client *client, \
	const struct i2c_device_id *id)
{

	int ret = 0;

	struct isl29028_info *lpi;
	struct isl29028_platform_data *pdata;

	D("%s\n", __func__);
	lpi = kzalloc(sizeof(struct isl29028_info), GFP_KERNEL);
	if (!lpi)
		return -ENOMEM;

	lpi->ps_i2c_client = client;
	pdata = client->dev.platform_data;
	if (!pdata)
		return -EBUSY;
	/*pdata->dev_id = (void *)&client->dev;*/
	i2c_set_clientdata(client, lpi);
	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->power = pdata->power;

	lp_info = lpi;

	ret = lightsensor_setup(lpi);
	if (ret < 0)
		return ret;
	ret = psensor_setup(lpi);
	if (ret < 0)
		return ret;
	lpi->lp_wq = create_singlethread_workqueue("isl29028_wq");
	if (!lpi->lp_wq) {
		pr_err("%s: can't create workqueue\n", __func__);
		return -ENOMEM;
	}
	ret = isl29028_setup(lpi);
	if (ret < 0)
		return ret;
#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level =
			EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = isl29028_early_suspend;
	lpi->early_suspend.resume = isl29028_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif
	ret = device_create_file(&lpi->ps_i2c_client->dev, &dev_attr_ls_adc);
	ret = device_create_file(&lpi->ps_i2c_client->dev, &dev_attr_ls_auto);
	ret = device_create_file(&lpi->ps_i2c_client->dev, &dev_attr_ps_adc);

	return ret;

}

static const struct i2c_device_id isl29028_i2c_id[] = {
	{ISL29028_I2C_NAME, 0},
	{}
};

static struct i2c_driver isl29028_driver = {
	.id_table = isl29028_i2c_id,
	.probe = isl29028_probe,
/*
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = isl29028_early_suspend,
	.resume = isl29028_late_resume,
#endif
*/
	.driver = {
		.name = ISL29028_I2C_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init isl29028_init(void)
{
	return i2c_add_driver(&isl29028_driver);
}

static void __exit isl29028_exit(void)
{
	i2c_del_driver(&isl29028_driver);
}

module_init(isl29028_init);
module_exit(isl29028_exit);

MODULE_DESCRIPTION("ISL29028 Driver");
MODULE_LICENSE("GPL");
