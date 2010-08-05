/* arch/arm/mach-msm/board-latte-microp.c
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
*/
#ifdef CONFIG_MICROP_COMMON
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/atmega_microp.h>
#include <mach/drv_callback.h>

#include "board-latte.h"

static struct i2c_client *latte_microp_client;
/*
static struct led_trigger latte_als_level_trigger = {
	.name     = "auto-backlight-trigger",
};
*/
/*
static int latte_als_intr_enable(struct i2c_client *client,
		uint32_t als_func, uint8_t enable)
{
	struct microp_i2c_client_data *cdata;

	cdata = i2c_get_clientdata(client);

	return microp_write_interrupt(client,
		cdata->int_pin.int_lsensor, enable);
}
*/

/*
static void latte_als_level_change(struct i2c_client *client,
		uint8_t *data)
{
	led_trigger_event(&latte_als_level_trigger, data[2]);
}
*/

/*
static int latte_als_power(int pwr_device, uint8_t enable)
{
	struct i2c_client *client = latte_microp_client;
	struct microp_i2c_platform_data *pdata;
	int value;

	pdata = client->dev.platform_data;
	value = enable ? 1 : 0;
	if (pdata->ls_power)
		pdata->ls_power(value);

	return 0;
}
*/

/*
static int latte_als_table_init(struct i2c_client *client,
			int i, uint32_t kadc, uint32_t gadc)
{
	struct microp_i2c_platform_data *pdata;
	uint8_t data[20];
	int j;

	pdata = client->dev.platform_data;

	for (j = 0; j < 10; j++) {
		data[j] = (uint8_t)(pdata->microp_function[i].levels[j]
				* kadc / gadc >> 8);
		data[j + 10] = (uint8_t)(pdata->microp_function[i].levels[j]
				* kadc / gadc);
	}

	return microp_i2c_write(MICROP_I2C_WCMD_ADC_TABLE, data, 20);
}
*/

static int latte_microp_function_init(struct i2c_client *client)
{
	struct microp_i2c_platform_data *pdata;
	struct microp_i2c_client_data *cdata;
	uint8_t data[20];
	int i, j;
	int ret;

	latte_microp_client = client;
	pdata = client->dev.platform_data;
	cdata = i2c_get_clientdata(client);

	/* Light sensor */
/*
	ret = microp_function_check(client, MICROP_FUNCTION_LSENSOR);
	if (ret >= 0) {
		i = ret;
		pdata->function_node[MICROP_FUNCTION_LSENSOR] = i;
		cdata->int_pin.int_lsensor = pdata->microp_function[i].int_pin;
		microp_get_als_kvalue(i);

		ret = latte_als_table_init(client, i, cdata->als_kadc,
				cdata->als_gadc);
		if (ret < 0)
			goto exit;

		if (pdata->ls_power)
			pdata->ls_power(1);
	}
*/
	/* Headset remote key */
	ret = microp_function_check(client, MICROP_FUNCTION_REMOTEKEY);
	if (ret >= 0) {
		i = ret;
		pdata->function_node[MICROP_FUNCTION_REMOTEKEY] = i;
		cdata->int_pin.int_remotekey =
			pdata->microp_function[i].int_pin;

		for (j = 0; j < 6; j++) {
			data[j] = (uint8_t)(pdata->microp_function[i].levels[j] >> 8);
			data[j + 6] = (uint8_t)(pdata->microp_function[i].levels[j]);
		}
		ret = microp_i2c_write(MICROP_I2C_WCMD_REMOTEKEY_TABLE,
				data, 12);
		if (ret)
			goto exit;
	}

	/* Reset button interrupt */
	ret = microp_write_interrupt(client, (1<<8), 1);
	if (ret)
		goto exit;

	/* OJ interrupt */
	ret = microp_function_check(client, MICROP_FUNCTION_OJ);
	if (ret >= 0) {
		i = ret;
		cdata->int_pin.int_oj = pdata->microp_function[i].int_pin;

		ret = microp_write_interrupt(client, cdata->int_pin.int_oj, 1);
		if (ret)
			goto exit;
	}

	return 0;

exit:
	return ret;
}

static struct microp_ops ops = {
	.init_microp_func = latte_microp_function_init,
/*
	.als_pwr_enable = latte_als_power,
	.als_intr_enable = latte_als_intr_enable,
	.als_level_change = latte_als_level_change,
*/
};

void __init latte_microp_init(void)
{
	/*led_trigger_register(&latte_als_level_trigger);*/
	microp_register_ops(&ops);
}

#endif
