#ifndef _CURCIAL_OJ_H
#define _CURCIAL_OJ_H
#include<linux/earlysuspend.h>

#define CURCIAL_OJ_NAME "curcial_oj"

struct curcial_oj_platform_data {
	struct input_dev *input_dev;
	struct work_struct work;
	bool click;
	uint8_t key;
	uint8_t key_count;
	uint32_t last_key_time;
	bool ap_code;
	uint8_t degree;
	uint8_t  debugflag;
	uint32_t last_click_time;
	uint16_t interval;
	uint8_t mdelay_time;
	uint8_t msleep_time;
	uint8_t x_send_count;
	uint8_t y_send_count;
	uint8_t send_count;
	int8_t fast_th;
	int8_t normal_th;
	uint8_t continue_th;
	uint8_t continue_max;
	int8_t xy_ratio;
	void (*oj_shutdown)(int);
	int (*oj_poweron)(int);
	void(*oj_adjust_xy)(uint8_t *, int16_t *, int16_t *);
	int microp_version;
	bool softclick;
	bool share_power;
	bool swap;
	int x;
	int y;
	uint8_t Xsteps[30];
	uint8_t Ysteps[30];
	uint16_t sht_tbl[6];
	uint8_t pxsum_tbl[6];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};
void curcial_oj_send_key(unsigned int code, int value);

#endif