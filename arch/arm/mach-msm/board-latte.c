/* arch/arm/mach-msm/board-latte.c
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-msm.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/switch.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/akm8973.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/sysdev.h>
#include <linux/android_pmem.h>
#include <linux/curcial_oj.h>
#include <linux/gpio_event.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/setup.h>
#include <asm/mach/mmc.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <mach/system.h>
#include <mach/vreg.h>
#include <mach/hardware.h>
#include <mach/gpio_chip.h>
#include <mach/board.h>
#include <mach/board_htc.h>
#include <mach/msm_serial_hs.h>
#include <mach/htc_headset_common.h>
#include <mach/audio_jack.h>
#include <mach/atmega_microp.h>
#include <mach/msm_tssc.h>
#include <mach/htc_battery.h>
#include <mach/htc_pwrsink.h>
#include <mach/perflock.h>
#include <mach/drv_callback.h>
#include <mach/camera.h>
#include <mach/msm_flashlight.h>
#include <mach/msm_serial_debugger.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>

#include "devices.h"
#include "board-latte.h"
#include "proc_comm.h"

void msm_init_irq(void);
void msm_init_gpio(void);
void config_latte_camera_on_gpios(void);
void config_latte_camera_off_gpios(void);
void config_latte_proximity_gpios(int on);
#ifdef CONFIG_MICROP_COMMON
void __init latte_microp_init(void);
#endif

static ssize_t htc_battery_show_attr(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.func_show_batt_attr = htc_battery_show_attr,
//	.gpio_mbat_in = LATTE_GPIO_MBAT_IN,
//	.gpio_mchg_en_n = LATTE_GPIO_MCHG_EN_N,
//	.gpio_iset = LATTE_GPIO_ISET,
	.guage_driver = GUAGE_MODEM,
	.charger = LINEAR_CHARGER,
	.m2a_cable_detect = 1,
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id = -1,
	.dev	= {
		.platform_data = &htc_battery_pdev_data,
	},
};
static int capella_cm3602_power(int pwr_device, uint8_t enable);

static struct microp_function_config microp_functions[] = {
	{
		.name = "light_sensor",
		.category = MICROP_FUNCTION_LSENSOR,
		.levels = { 0, 0x21, 0x4D, 0xDC, 0x134, 0x18D, 0x1E5, 0x3FF, 0x3FF, 0x3FF },
		.channel = 3,
		.int_pin = 1 << 9,
		.golden_adc = 0xC0,
		.ls_power = capella_cm3602_power,
	},
	{
		.name   = "remote-key",
		.category = MICROP_FUNCTION_REMOTEKEY,
		.levels = {0, 33, 38, 82, 95, 167},
		.channel = 1,
		.int_pin = 1 << 5,
	},
	{
		.name   = "microp_intrrupt",
		.category = MICROP_FUNCTION_INTR,
	},
	{
		.name   = "reset-int",
		.category = MICROP_FUNCTION_RESET_INT,
		.int_pin = 1 << 8,
	},
	{
		.name   = "oj",
		.category = MICROP_FUNCTION_OJ,
		.int_pin = 1 << 12,
	},
};

static struct lightsensor_platform_data lightsensor_data = {
	.config = &microp_functions[0],
	.irq = MSM_uP_TO_INT(9),
};

static struct microp_led_config led_config[] = {
	{
		.name   = "amber",
		.type = LED_RGB,
	},
	{
		.name   = "green",
		.type = LED_RGB,
	},
	{
		.name   = "button-backlight",
		.type = LED_GPO,
		.mask_w = {0x00, 0x00, 0x08},
	},
	{
		.name	= "keyboard-backlight",
		.type = LED_GPO,
		.mask_w = {0x00, 0x00, 0x04},
	},
};

static struct microp_led_platform_data microp_leds_data = {
	.num_leds	= ARRAY_SIZE(led_config),
	.led_config	= led_config,
};

static struct bma150_platform_data latte_g_sensor_pdata = {
	.microp_new_cmd = 1,
};

static struct platform_device microp_devices[] = {
	{
		.name = "lightsensor_microp",
		.dev = {
			.platform_data = &lightsensor_data,
		},
	},
	{
		.name = "leds-microp",
		.id = -1,
		.dev = {
			.platform_data = &microp_leds_data,
		},
	},
	{
		.name = BMA150_G_SENSOR_NAME,
		.dev = {
			.platform_data = &latte_g_sensor_pdata,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_functions   = ARRAY_SIZE(microp_functions),
	.microp_function = microp_functions,
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = LATTE_GPIO_UP_RESET_N,
	.spi_devices = SPI_OJ | SPI_GSENSOR,
};

static int latte_ts_power(int on)
{
	printk(KERN_INFO "latte_ts_power:%d\n", on);
	if (on) {
		gpio_set_value(LATTE_TP_5V_EN, 1);
		msleep(250);
		/* enable touch panel level shift */
		gpio_set_value(LATTE_TP_LS_EN, 1);
		msleep(2);
	} else {
		gpio_set_value(LATTE_TP_LS_EN, 0);
		udelay(50);
		gpio_set_value(LATTE_TP_5V_EN, 0);

	}
	return 0;
}

static struct synaptics_i2c_rmi_platform_data latte_ts_rmi_data[] = {
	{
		.version = 0x0100,
		.power = latte_ts_power,
		.sensitivity_adjust = 5,
		.flags = SYNAPTICS_FLIP_Y | SYNAPTICS_SNAP_TO_INACTIVE_EDGE,
		.inactive_left = -1 * 0x10000 / 320,
		.inactive_right = -1 * 0x10000 / 320,
		.inactive_top = -1 * 0x10000 / 480,
		.inactive_bottom = -1 * 0x10000 / 480,
		.display_width = 320,
		.display_height = 480,
		.dup_threshold = 10,
		.margin_inactive_pixel = {8, 8, 0, 0}, /* left, right, top, bottom */
	}
};

static void latte_phy_shutdown(void)
{
	printk(KERN_INFO "%s\n", __func__);
	gpio_set_value(LATTE_GPIO_USBPHY_3V3_EN, 0);
}

static struct akm8973_platform_data compass_platform_data = {
	.layouts = LATTE_LAYOUTS,
	.project_name = LATTE_PROJECT_NAME,
	.reset = LATTE_GPIO_COMPASS_RST_N,
	.intr = LATTE_GPIO_COMPASS_INT_N,
};

static struct i2c_board_info i2c_devices[] = {
	{	/* new panel uses I2C dev addr 0x2C */
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x2C),
		.platform_data = &latte_ts_rmi_data,
		.irq = LATTE_GPIO_TO_INT(LATTE_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x2D),
		.platform_data = &latte_ts_rmi_data,
		.irq = LATTE_GPIO_TO_INT(LATTE_GPIO_TP_ATT_N)
	},
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = LATTE_GPIO_TO_INT(LATTE_GPIO_UP_INT_N)
	},
	{
		I2C_BOARD_INFO(AKM8973_I2C_NAME, 0x1C),
		.platform_data = &compass_platform_data,
		.irq = LATTE_GPIO_TO_INT(LATTE_GPIO_COMPASS_INT_N),
	},
};

static struct h2w_platform_data latte_h2w_data = {
};

static struct platform_device latte_h2w = {
	.name		= "htc_headset",
	.id			= -1,
	.dev		= {
		.platform_data	= &latte_h2w_data,
	},
};

static struct audio_jack_platform_data latte_jack_data = {
	.gpio	= LATTE_GPIO_35MM_HEADSET_DET,
};

static struct platform_device latte_audio_jack = {
	.name		= "audio-jack",
	.id			= -1,
	.dev		= {
		.platform_data	= &latte_jack_data,
	},
};

static struct pwr_sink latte_pwrsink_table[] = {
	{
		.id     = PWRSINK_AUDIO,
		.ua_max = 100000,
	},
	{
		.id     = PWRSINK_BACKLIGHT,
		.ua_max = 125000,
	},
	{
		.id     = PWRSINK_LED_BUTTON,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_LED_KEYBOARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_GP_CLK,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_BLUETOOTH,
		.ua_max = 15000,
	},
	{
		.id     = PWRSINK_CAMERA,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_SDCARD,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_VIDEO,
		.ua_max = 0,
	},
	{
		.id     = PWRSINK_WIFI,
		.ua_max = 200000,
	},
	{
		.id     = PWRSINK_SYSTEM_LOAD,
		.ua_max = 100000,
		.percent_util = 38,
	},
};

static int espresso_phy_init_seq[] = { 0xC, 0x31, 0x31, 0x32, 0x1D, 0x0D, 0x1D, 0x10, -1 };
static struct msm_hsusb_product espresso_usb_products[] = {
	{
		.product_id	= 0x0ff9,
		.functions	= 0x00000001, /* usb_mass_storage */
	},
	{
		.product_id	= 0x0c86,
		.functions	= 0x00000003, /* usb_mass_storage + adb */
	},
	{
		.product_id	= 0x0c03,
		.functions	= 0x00000101, /* modem + mass_storage */
	},
	{
		.product_id	= 0x0c04,
		.functions	= 0x00000103, /* modem + adb + mass_storage */
	},
	{
		.product_id	= 0x0c05,
		.functions	= 0x00000021, /* Projector + mass_storage */
	},
	{
		.product_id	= 0x0c06,
		.functions	= 0x00000023, /* Projector + adb + mass_storage */
	},
	{
		.product_id	= 0x0c07,
		.functions	= 0x0000000B, /* diag + adb + mass_storage */
	},
	{
		.product_id	= 0x0c08,
		.functions	= 0x00000009, /* diag + mass_storage */
	},
	{
		.product_id	= 0x0c88,
		.functions	= 0x0000010B, /* adb + mass_storage + diag + modem */
	},
	{
		.product_id	= 0x0c89,
		.functions	= 0x00000019, /* serial + diag + mass_storage */
	},
	{
		.product_id	= 0x0c8a,
		.functions	= 0x0000001B, /* serial + diag + adb + mass_storage */
	},
	{
		.product_id	= 0x0c93,
		.functions	= 0x00000080, /* mtp */
	},
	{
		.product_id	= 0x0FFE,
		.functions	= 0x00000004, /* internet sharing */
	},
};

static void latte_phy_reset(void)
{
	int ret;
	printk(KERN_INFO "msm_hsusb_phy_reset\n");
	ret = msm_proc_comm(PCOM_MSM_HSUSB_PHY_RESET,
			NULL, NULL);
	if (ret)
		printk(KERN_INFO "%s failed\n", __func__);
}

static int latte_pwrsink_resume_early(struct platform_device *pdev)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
	return 0;
}

static void latte_pwrsink_resume_late(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 38);
}

static void latte_pwrsink_suspend_early(struct early_suspend *h)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 7);
}

static int latte_pwrsink_suspend_late(struct platform_device *pdev, pm_message_t state)
{
	htc_pwrsink_set(PWRSINK_SYSTEM_LOAD, 1);
	return 0;
}

static struct pwr_sink_platform_data latte_pwrsink_data = {
	.num_sinks      = ARRAY_SIZE(latte_pwrsink_table),
	.sinks          = latte_pwrsink_table,
	.suspend_late	= latte_pwrsink_suspend_late,
	.resume_early	= latte_pwrsink_resume_early,
	.suspend_early	= latte_pwrsink_suspend_early,
	.resume_late	= latte_pwrsink_resume_late,
};

static struct platform_device latte_pwr_sink = {
	.name = "htc_pwrsink",
	.id = -1,
	.dev    = {
		.platform_data = &latte_pwrsink_data,
	},
};

static struct msm_pmem_setting pmem_setting = {
	.pmem_start = MSM_PMEM_MDP_BASE,
	.pmem_size = MSM_PMEM_MDP_SIZE,
	.pmem_adsp_start = MSM_PMEM_ADSP_BASE,
	.pmem_adsp_size = MSM_PMEM_ADSP_SIZE,
	.pmem_camera_start = MSM_PMEM_CAMERA_BASE,
	.pmem_camera_size = MSM_PMEM_CAMERA_SIZE,
	.ram_console_start = MSM_RAM_CONSOLE_BASE,
	.ram_console_size = MSM_RAM_CONSOLE_SIZE,
        .kgsl_start = MSM_GPU_MEM_BASE,
        .kgsl_size = MSM_GPU_MEM_SIZE,
};

#ifdef CONFIG_MSM_CAMERA
static struct resource msm_camera_resources[] = {
	{
		.start	= MSM_VFE_PHYS,
		.end	= MSM_VFE_PHYS + MSM_VFE_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_latte_camera_on_gpios,
	.camera_gpio_off = config_latte_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash		= flashlight_control,
	.num_flash_levels	= FLASHLIGHT_NUM,
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "s5k4e1gx",
	.sensor_reset   = LATTE_GPIO_CAM1_RST_N,
	.vcm_pwd        = LATTE_GPIO_VCM_PWDN,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_cfg	= &msm_camera_sensor_flash_cfg,
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name      = "msm_camera_s5k4e1gx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif

static struct platform_device latte_rfkill = {
	.name = "latte_rfkill",
	.id = -1,
};

/* Proximity Sensor (Capella_CM3602)*/
static int __capella_cm3602_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "wlan");
	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2800);

	if (on) {
		config_latte_proximity_gpios(1);
		gpio_direction_output(LATTE_GPIO_PROXIMITY_EN, 1);
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
		gpio_direction_output(LATTE_GPIO_PROXIMITY_EN, 0);
		config_latte_proximity_gpios(0);
	}

	return rc;
}

static DEFINE_MUTEX(capella_cm3602_lock);
static unsigned int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
	ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.p_out = LATTE_GPIO_PROXIMITY_INT,
	.p_en = LATTE_GPIO_PROXIMITY_EN,
	.power = capella_cm3602_power,
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
/* End Proximity Sensor (Capella_CM3602)*/
#define CURCIAL_OJ_POWER            85
static void curcial_oj_shutdown (int	enable)
{

}
static int curcial_oj_poweron(int on)
{
	gpio_set_value(CURCIAL_OJ_POWER, on);

	if (gpio_get_value(CURCIAL_OJ_POWER) != on) {
		printk(KERN_ERR "%s:OJ:power status fail \n", __func__);
		return 0;
	}
		printk(KERN_ERR "%s:OJ:power status ok \n", __func__);
	return 1;
}
#define LATTE_MICROP_VER	0x05
static void curcial_oj_adjust_xy(uint8_t *data, int16_t *mSumDeltaX, int16_t *mSumDeltaY)
{
	int8_t 	deltaX;
	int8_t 	deltaY;


	if (data[2] == 0x80)
		data[2] = 0x81;
	if (data[1] == 0x80)
		data[1] = 0x81;
	if (system_rev <= 2) {
		deltaX = (-1)*((int8_t) data[2]); /*X=2*/
		deltaY = (1)*((int8_t) data[1]); /*Y=1*/
	} else {
		deltaX = (-1)*((int8_t) data[1]);
		deltaY = (-1)*((int8_t) data[2]);
	}
	*mSumDeltaX += -((int16_t)deltaX);
	*mSumDeltaY += -((int16_t)deltaY);
}
static struct curcial_oj_platform_data latte_oj_data = {
	.oj_poweron = curcial_oj_poweron,
	.oj_shutdown = curcial_oj_shutdown,
	.oj_adjust_xy = curcial_oj_adjust_xy,
	.microp_version = LATTE_MICROP_VER,
	.mdelay_time = 15,
	.msleep_time = 0,
	.send_count = 3,
	.fast_th = 1,
	.normal_th = 15,
	.continue_th = 3,
	.continue_max = 0,
	.xy_ratio = 15,
	.interval = 60,
	.softclick = false,
	.swap = false,
	.x = -1,
	.y = 1,
	.share_power = false,
	.Xsteps = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
	.Ysteps = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2},
};

static struct platform_device latte_oj = {
	.name = CURCIAL_OJ_NAME,
	.id = -1,
	.dev = {
		.platform_data	= &latte_oj_data,
	}
};

static struct msm_i2c_device_platform_data latte_i2c_device_data = {
	.i2c_clock = 400000,
	.clock_strength = GPIO_8MA,
	.data_strength = GPIO_4MA,
};

static struct gpio_led espresso_led_list[] = {
	{
		.name = "caps",
		.gpio = LATTE_GPIO_LED_CAP_LED_EN,
	},
	{
		.name = "func",
		.gpio = LATTE_GPIO_LED_FN_LED_EN,
	},
};

static struct gpio_led_platform_data espresso_leds_data = {
	.num_leds	= ARRAY_SIZE(espresso_led_list),
	.leds		= espresso_led_list,
};

static struct platform_device espresso_leds = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &espresso_leds_data,
	},
};

static struct platform_device *devices[] __initdata = {
	&msm_device_i2c,
	&latte_h2w,
	&htc_battery_pdev,
	&latte_audio_jack,
	&msm_camera_sensor_s5k4e1gx,
	&latte_rfkill,
#ifdef CONFIG_HTC_PWRSINK
	&latte_pwr_sink,
#endif
	&latte_oj,
	&capella_cm3602,
	&espresso_leds,
};

extern struct sys_timer msm_timer;

static void __init latte_init_irq(void)
{
	printk("latte_init_irq()\n");
	msm_init_irq();
}

static uint cpld_iset;
static uint cpld_charger_en;
static uint cpld_usb_h2w_sw;
static uint opt_disable_uart3;
static char *keycaps = "";

module_param_named(iset, cpld_iset, uint, 0);
module_param_named(charger_en, cpld_charger_en, uint, 0);
module_param_named(usb_h2w_sw, cpld_usb_h2w_sw, uint, 0);
module_param_named(disable_uart3, opt_disable_uart3, uint, 0);
module_param_named(keycaps, keycaps, charp, 0);

static char bt_chip_id[10] = "brfxxxx";
module_param_string(bt_chip_id, bt_chip_id, sizeof(bt_chip_id), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_chip_id, "BT's chip id");

static char bt_fw_version[10] = "v2.0.38";
module_param_string(bt_fw_version, bt_fw_version, sizeof(bt_fw_version), S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(bt_fw_version, "BT's fw version");

static void latte_reset(void)
{
	gpio_set_value(LATTE_GPIO_PS_HOLD, 0);
}


static struct i2c_board_info i2c_camera_devices[] = {
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x20 >> 1),/*5M samsung bayer sensor driver*/
	},
};

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(21, 0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(21, 0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
};

static uint32_t camera_off_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(0, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(1, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT1 */

	PCOM_GPIO_CFG(2, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* PCLK */
	PCOM_GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), /* MCLK */

	PCOM_GPIO_CFG(27, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* CAM_I2C_SDA */
	PCOM_GPIO_CFG(124, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), /* CAM_I2C_SCL */
};

static uint32_t camera_on_gpio_table[] = {
	/* CAMERA */
	PCOM_GPIO_CFG(2, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	PCOM_GPIO_CFG(3, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	PCOM_GPIO_CFG(4, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	PCOM_GPIO_CFG(5, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	PCOM_GPIO_CFG(6, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	PCOM_GPIO_CFG(7, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	PCOM_GPIO_CFG(8, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	PCOM_GPIO_CFG(9, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	PCOM_GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	PCOM_GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	PCOM_GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	PCOM_GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	PCOM_GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	PCOM_GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

void config_latte_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

void config_latte_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

void config_latte_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

static uint32_t latte_serial_debug_table[] = {
	/* config as serial debug uart */
	PCOM_GPIO_CFG(LATTE_GPIO_UART3_RX, 1,
			GPIO_INPUT, GPIO_PULL_UP, GPIO_8MA),	/* UART3 RX */
	PCOM_GPIO_CFG(LATTE_GPIO_UART3_TX, 1,
			GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* UART3 TX */
};

static void latte_config_serial_debug_gpios(void)
{
	config_gpio_table(latte_serial_debug_table,
			ARRAY_SIZE(latte_serial_debug_table));
}

static void __init config_gpios(void)
{
	latte_config_serial_debug_gpios();
	config_latte_camera_off_gpios();
}

static struct msm_acpu_clock_platform_data latte_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200,
	.wait_for_irq_khz = 200000,
};

static unsigned latte_perf_acpu_table[] = {
	245760000,
	480000000,
	600000000,
};

static struct perflock_platform_data latte_perflock_data = {
	.perf_acpu_table = latte_perf_acpu_table,
	.table_size = ARRAY_SIZE(latte_perf_acpu_table),
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.wakeup_irq = MSM_GPIO_TO_INT(LATTE_GPIO_BT_UART1_RX),
	.inject_rx_on_wakeup = 1,
	.rx_to_inject = 0x32,
	.cpu_lock_supported = 1,
};
#endif

/*Farmer:For H2W power*/
static struct vreg *vreg_h2w;
static int h2w_power_configure(struct gpio_chip *chip,
			       unsigned int gpio,
			       unsigned long flags)
{
	if ((flags & GPIOF_DRIVE_OUTPUT) && !vreg_h2w)
		vreg_h2w = vreg_get(0, LATTE_H2W_POWER_NAME);

	if ((flags & GPIOF_OUTPUT_HIGH) && vreg_h2w)
		vreg_enable(vreg_h2w);
	else if ((flags & GPIOF_OUTPUT_LOW) && vreg_h2w)
		vreg_disable(vreg_h2w);

	return 0;
}

static int h2w_power_get_irq_num(struct gpio_chip *chip,
				 unsigned int gpio,
				 unsigned int *irqp,
				 unsigned long *irqnumflagsp)
{
	return -1;
}

static int h2w_power_read(struct gpio_chip *chip, unsigned n)
{
	return -1;
}
static int h2w_power_write(struct gpio_chip *chip, unsigned n, unsigned on)
{
	if (!vreg_h2w)
		return -1;

	if (on)
		vreg_enable(vreg_h2w);
	else
		vreg_disable(vreg_h2w);
	return 0;
}

static struct gpio_chip latte_h2w_gpio_chip = {
	.start = LATTE_GPIO_H2W_POWER,
	.end = LATTE_GPIO_H2W_POWER,
	.configure = h2w_power_configure,
	.get_irq_num = h2w_power_get_irq_num,
	.read = h2w_power_read,
	.write = h2w_power_write,
};

void latte_init_h2w_power_gpio(void)
{
	register_gpio_chip(&latte_h2w_gpio_chip);
}

static void config_latte_flashlight_gpios(void)
{
	uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(LATTE_GPIO_TORCH_EN, 0, GPIO_OUTPUT,
							GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(LATTE_GPIO_FLASH_EN, 0, GPIO_OUTPUT,
							GPIO_NO_PULL, GPIO_2MA),
	};

	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data latte_flashlight_data = {
	.gpio_init = config_latte_flashlight_gpios,
	.torch = LATTE_GPIO_TORCH_EN,
	.flash = LATTE_GPIO_FLASH_EN,
	.flash_duration_ms = 600,
};

static struct platform_device latte_flashlight_device = {
	.name = FLASHLIGHT_NAME,
	.dev = {
		.platform_data  = &latte_flashlight_data,
	},
};

static void __init latte_init(void)
{
	int rc;
	char *cid = NULL;
	printk("latte_init() revision = 0x%X\n", system_rev);
	board_get_cid_tag(&cid);

	/*
	 * Setup common MSM GPIOS
	 */
	config_gpios();

	/* We need to set this pin to 0 only once on power-up; we will
	 * not actually enable the chip until we apply power to it via
	 * vreg.
	 */
	gpio_direction_output(LATTE_GPIO_LS_EN, 0);
	/* disable power for cm3602 chip */
	__capella_cm3602_power(0);

	msm_hw_reset_hook = latte_reset;

	msm_acpu_clock_init(&latte_clock_data);
	perflock_init(&latte_perflock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	if (!opt_disable_uart3)
		msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1,
				MSM_GPIO_TO_INT(LATTE_GPIO_UART3_RX));
#endif

	msm_add_devices();

#ifdef CONFIG_SERIAL_MSM_HS
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_add_serial_devices(3);
#else
	msm_add_serial_devices(0);
#endif

	msm_add_serial_devices(2);
/*
	msm_change_usb_id(0x0bb4, 0x0c10);
*/
	msm_add_usb_id_pin_gpio(LATTE_GPIO_USB_ID_PIN);

	msm_register_usb_phy_init_seq(espresso_phy_init_seq);
	msm_hsusb_set_product(espresso_usb_products,
		ARRAY_SIZE(espresso_usb_products));
	msm_add_usb_devices(latte_phy_reset, latte_phy_shutdown);


	msm_add_mem_devices(&pmem_setting);

	msm_init_pmic_vibrator();
#ifdef CONFIG_MICROP_COMMON
	latte_microp_init();
#endif

	latte_init_h2w_power_gpio();

	rc = latte_init_mmc(system_rev);
	if (rc)
		printk(KERN_CRIT "%s: MMC init failure (%d)\n", __func__, rc);

	/* register flashlight at new-XA above */
	platform_device_register(&latte_flashlight_device);

	/* probe camera driver */
	i2c_register_board_info(0, i2c_camera_devices, ARRAY_SIZE(i2c_camera_devices));

	msm_device_i2c.dev.platform_data = &latte_i2c_device_data;
	platform_add_devices(devices, ARRAY_SIZE(devices));
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	latte_init_keypad();
}

static void __init latte_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=1;
	mi->bank[0].start = MSM_LINUX_BASE;
	mi->bank[0].node = PHYS_TO_NID(MSM_LINUX_BASE);
	mi->bank[0].size = MSM_LINUX_SIZE;
}

static void __init latte_map_io(void)
{
	printk("latte_init_map_io()\n");
	msm_map_common_io();
	msm_clock_init();
#ifdef CONFIG_CACHE_L2X0
	/* 7x27 has 256KB L2 cache:
	* 64Kb/Way and 4-Way Associativity;
	* R/W latency: 3 cycles;
	* evmon/parity/share disabled.
	*/
	l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

MACHINE_START(LATTE, "latte")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params    = 0x12C00100,
	.fixup          = latte_fixup,
	.map_io         = latte_map_io,
	.init_irq       = latte_init_irq,
	.init_machine   = latte_init,
	.timer          = &msm_timer,
MACHINE_END
