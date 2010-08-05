obj-y += io.o irq.o timer.o dma.o memory.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += sirc.o
obj-y += devices.o
obj-y += proc_comm.o
obj-y += vreg.o
obj-y += pmic.o
obj-y += remote_spinlock.o
obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock-arm11.o idle.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += arch-init-scorpion.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += acpuclock-scorpion.o idle-v7.o appsbl_sc_sa_fuse_ro.o

obj-y += clock.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += drv_callback.o
obj-$(CONFIG_ARCH_QSD8X50) += pmic.o htc_wifi_nvs.o htc_bluetooth.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o
obj-$(CONFIG_MSM_SMD) += smem_log.o
obj-$(CONFIG_MSM_SMD) += last_radio_log.o
obj-$(CONFIG_MSM_SMD) += htc_port_list.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_ADSP_COMP) += qdsp5_comp/
obj-$(CONFIG_QSD_AUDIO) += qdsp6/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_HTC_ACOUSTIC) += htc_acoustic.o
obj-$(CONFIG_HTC_ACOUSTIC_QSD) += htc_acoustic_qsd.o
obj-$(CONFIG_SENSORS_AKM8976) += htc_akm_cal.o

# MSM7227 boards
obj-$(CONFIG_MACH_LEGEND) += board-legend-rfkill.o
obj-$(CONFIG_MACH_LEGEND) += board-legend.o board-legend-keypad.o board-legend-panel.o
obj-$(CONFIG_MACH_LEGEND) += board-legend-mmc.o board-legend-wifi.o board-legend-microp.o
obj-$(CONFIG_MACH_LATTE) += board-latte-rfkill.o
obj-$(CONFIG_MACH_LATTE) += board-latte.o board-latte-keypad.o board-latte-panel.o
obj-$(CONFIG_MACH_LATTE) += board-latte-mmc.o board-latte-wifi.o board-latte-microp.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-rfkill.o htc_bluetooth.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty.o board-liberty-keypad.o board-liberty-panel.o
obj-$(CONFIG_MACH_LIBERTY) += board-liberty-mmc.o board-liberty-wifi.o board-liberty-microp.o

obj-$(CONFIG_ARCH_MSM7227) += msm_vibrator.o devices_htc.o htc_wifi_nvs.o htc_awb_cal.o
obj-$(CONFIG_ARCH_MSM_FLASHLIGHT) += msm_flashlight.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery_trout.o
obj-$(CONFIG_HTC_BATTCHG) += htc_battery.o
obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o

obj-$(CONFIG_HTC_HEADSET_V1) += htc_headset_v1.o
obj-$(CONFIG_HTC_35MM_REMOTE_V1) += htc_35mm_remote_v1.o
obj-$(CONFIG_HTC_AUDIOJACK_V1) += htc_audio_jack_v1.o
obj-$(CONFIG_HTC_HEADSET) += htc_35mm_remote.o htc_headset_common.o
obj-$(CONFIG_HTC_AUDIOJACK) += htc_audio_jack.o
obj-$(CONFIG_PERFLOCK) += perflock.o
obj-$(CONFIG_MICROP_COMMON) += atmega_microp_common.o
obj-$(CONFIG_TOUCHSCREEN_TSSC_MANAGER) += htc_tp_cal.o

