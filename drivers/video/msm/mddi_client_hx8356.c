/* drivers/video/msm_fb/mddi_client_hx8356.c
 *
 * Support for eid mddi client devices with Samsung S6D0154
 *
 * Copyright (C) 2007 HTC Incorporated
 * Author: Wade Wu (wade_wu@htc.com)
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/clk.h>
#include <asm/atomic.h>
#include <asm/mach-types.h>
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>
#define printk(arg,...)
#if 1
#define B(s...) printk(s)
#else
#define B(s...) do {} while (0)
#endif

#define DEBUG_VSYNC_INT 0
#define VSYNC_COUNTER 31

#define VSYNC_CLEAR (MSM_GPIO1_BASE + 0x800 + 0x9c)
#define VSYNC_STATUS (MSM_GPIO1_BASE + 0x800 + 0xac)
#define VSYNC_EN (MSM_GPIO1_BASE + 0x800 + 0x8c)

static DECLARE_WAIT_QUEUE_HEAD(himax_vsync_wait);


struct panel_info {
	struct msm_mddi_client_data *client_data;
	struct platform_device pdev;
	struct msm_panel_data panel_data;
	struct msmfb_callback *fb_callback;
	struct wake_lock idle_lock;
	int himax_got_int;
};

static struct clk *ebi1_clk;

static void himax_dump_vsync(void)
{
//	unsigned long rate = clk_get_rate(ebi1_clk);

	printk(KERN_INFO "STATUS %d %s EBI1 %lu\n",
			readl(VSYNC_STATUS) & 0x04,
			readl(VSYNC_EN) & 0x04 ? "ENABLED" : "DISABLED",
			rate);
}

static void
himax_request_vsync(struct msm_panel_data *panel_data,
		      struct msmfb_callback *callback)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	panel->fb_callback = callback;
	if (panel->himax_got_int) {
		panel->himax_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}
}

static void himax_wait_vsync(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;

	if (panel->himax_got_int) {
		panel->himax_got_int = 0;
		client_data->activate_link(client_data); /* clears interrupt */
	}

	if (wait_event_timeout(himax_vsync_wait, panel->himax_got_int,
				HZ/2) == 0)
		printk(KERN_ERR "timeout waiting for VSYNC\n");

	panel->himax_got_int = 0;
	/* interrupt clears when screen dma starts */
}


/* -------------------------------------------------------------------------- */

/* got called by msmfb_suspend */
static int himax_suspend(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	wake_lock(&panel->idle_lock);
	ret = bridge_data->uninit(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret) {
		B(KERN_INFO "mddi himax client: non zero return from "
		  "uninit\n");
		return ret;
	}
	client_data->suspend(client_data);
	return 0;
}

/* got called by msmfb_resume */
static int himax_resume(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;
	int ret;

	client_data->resume(client_data);

	wake_lock(&panel->idle_lock);
	ret = bridge_data->init(bridge_data, client_data);
	wake_unlock(&panel->idle_lock);

	if (ret)
		return ret;
	return 0;
}

static int himax_blank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->blank(bridge_data, client_data);
}

static int himax_unblank(struct msm_panel_data *panel_data)
{
	struct panel_info *panel = container_of(panel_data, struct panel_info,
						panel_data);
	struct msm_mddi_client_data *client_data = panel->client_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	return bridge_data->unblank(bridge_data, client_data);
}

static irqreturn_t himax_vsync_interrupt(int irq, void *data)
{
	struct panel_info *panel = data;

	panel->himax_got_int = 1;
	if (panel->fb_callback) {
		panel->fb_callback->func(panel->fb_callback);
		panel->fb_callback = NULL;
	}
	wake_up(&himax_vsync_wait);

	return IRQ_HANDLED;
}

static int setup_vsync(struct panel_info *panel, int init)
{
	int ret;
	int gpio = 97;
	uint32_t config;
	unsigned int irq;

	if (!init) {
		ret = 0;
		goto uninit;
	}
	ret = gpio_request(gpio, "vsync");
	if (ret)
		goto err_request_gpio_failed;

	config = PCOM_GPIO_CFG(97, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA);
	ret = msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &config, 0);
	if (ret)
		goto err_gpio_direction_input_failed;

	ret = irq = gpio_to_irq(gpio);
	if (ret < 0)
		goto err_get_irq_num_failed;

	ret = request_irq(irq, himax_vsync_interrupt, IRQF_TRIGGER_HIGH,
			"vsync", panel);
	if (ret)
		goto err_request_irq_failed;

	printk(KERN_INFO "vsync on gpio %d now %d\n", gpio,
			gpio_get_value(gpio));
	return 0;

uninit:
	free_irq(gpio_to_irq(gpio), panel->client_data);
err_request_irq_failed:
err_get_irq_num_failed:
err_gpio_direction_input_failed:
	gpio_free(gpio);
err_request_gpio_failed:
	return ret;
}

static int mddi_himax_probe(struct platform_device *pdev)
{
	int ret, err = -EINVAL;
	struct panel_info *panel;
	struct msm_mddi_client_data *client_data = pdev->dev.platform_data;
	struct msm_mddi_bridge_platform_data *bridge_data =
	    client_data->private_client_data;

	B(KERN_DEBUG "%s: enter\n", __func__);

	panel = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
	if (!panel) {
		err = -ENOMEM;
		goto err_out;
	}

	platform_set_drvdata(pdev, panel);
	ret = setup_vsync(panel, 1);
	if (ret) {
		dev_err(&pdev->dev, "mddi_himax_setup_vsync failed\n");
		err = -EIO;
		goto err_panel;
	}

	panel->client_data = client_data;
	panel->panel_data.suspend = himax_suspend;
	panel->panel_data.resume = himax_resume;
	panel->panel_data.request_vsync = himax_request_vsync;
	panel->panel_data.blank = himax_blank;
	panel->panel_data.unblank = himax_unblank;
	panel->panel_data.dump_vsync = himax_dump_vsync;
	panel->panel_data.fb_data = &bridge_data->fb_data;
	panel->panel_data.caps = ~MSMFB_CAP_PARTIAL_UPDATES;
	panel->pdev.name = "msm_panel";
	panel->pdev.id = pdev->id;
	panel->pdev.resource = client_data->fb_resource;
	panel->pdev.num_resources = 1;
	panel->pdev.dev.platform_data = &panel->panel_data;
	platform_device_register(&panel->pdev);
	wake_lock_init(&panel->idle_lock, WAKE_LOCK_IDLE, "eid_idle_lock");
	/*for debugging vsync issue*/
	ebi1_clk = clk_get(NULL, "ebi1_clk");

	return 0;

err_panel:
	kfree(panel);
err_out:
	return err;
}

static int mddi_himax_remove(struct platform_device *pdev)
{
	struct panel_info *panel = platform_get_drvdata(pdev);

	setup_vsync(panel, 0);
	kfree(panel);
	return 0;
}

static struct platform_driver mddi_client_4858_0000 = {
	.probe = mddi_himax_probe,
	.remove = mddi_himax_remove,
	.driver = {.name = "mddi_c_4858_8356"},
};

static int __init mddi_client_himax_init(void)
{
	return platform_driver_register(&mddi_client_4858_0000);
}

module_init(mddi_client_himax_init);
