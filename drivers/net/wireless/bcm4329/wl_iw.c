/*
 * Linux Wireless Extensions support
 *
 * Copyright (C) 1999-2009, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: wl_iw.c,v 1.51.4.9.2.6.4.57 2009/10/27 04:43:46 Exp $
 */


#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>

#include <bcmutils.h>
#include <bcmendian.h>
#include <proto/ethernet.h>

#include <linux/if_arp.h>
#include <asm/uaccess.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhdioctl.h>

typedef void wlc_info_t;
typedef void wl_info_t;
typedef const struct si_pub  si_t;
#include <wlioctl.h>

#include <proto/ethernet.h>
#include <dngl_stats.h>
#include <dhd.h>
#define WL_ERROR(x)
#define WL_TRACE(x)
#define WL_ASSOC(x)
#define WL_INFORM(x)
#define WL_WSEC(x)
#define WL_SOFTAP(x) printk x

#include <wl_iw.h>

#include <linux/rtnetlink.h>
#include <linux/mutex.h>

#define WL_IW_USE_ISCAN  1
#define ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS  1
static struct net_device *priv_dev;
#ifdef CONFIG_BCM4329_SOFTAP
static int ap_mode = 0;
#endif


#define WL_IW_IOCTL_CALL(func_call) \
	do {				\
		func_call;		\
	} while (0)

static int		g_onoff = G_WLAN_SET_ON;
static struct mutex	wl_start_lock;

extern bool wl_iw_conn_status_str(uint32 event_type, uint32 status,
	uint32 reason, char* stringBuf, uint buflen);
#include <bcmsdbus.h>
extern void dhd_customer_gpio_wlan_ctrl(int onoff);
extern uint dhd_dev_reset(struct net_device *dev, uint8 flag);
extern void dhd_dev_init_ioctl(struct net_device *dev);

uint wl_msg_level = WL_ERROR_VAL;

#define MAX_WLIW_IOCTL_LEN 1024


#if defined(IL_BIGENDIAN)
#include <bcmendian.h>
#define htod32(i) (bcmswap32(i))
#define htod16(i) (bcmswap16(i))
#define dtoh32(i) (bcmswap32(i))
#define dtoh16(i) (bcmswap16(i))
#define htodchanspec(i) htod16(i)
#define dtohchanspec(i) dtoh16(i)
#else
#define htod32(i) i
#define htod16(i) i
#define dtoh32(i) i
#define dtoh16(i) i
#define htodchanspec(i) i
#define dtohchanspec(i) i
#endif

#ifdef CONFIG_WIRELESS_EXT

extern struct iw_statistics *dhd_get_wireless_stats(struct net_device *dev);
extern int dhd_wait_pend8021x(struct net_device *dev);
#endif

#if WIRELESS_EXT < 19
#define IW_IOCTL_IDX(cmd)	((cmd) - SIOCIWFIRST)
#define IW_EVENT_IDX(cmd)	((cmd) - IWEVFIRST)
#endif

static void *g_scan = NULL;
static uint g_scan_specified_ssid;
#ifdef WLAN_PFN
static char pfn_ssid[32] = {0};
static int pfn_ssid_len = 0;
static int pfn_scan = 0;
struct timer_list *pfn_lock_timer = NULL;	
#endif
#ifdef WLAN_PROTECT
static int wl_iw_busdown = 0;
static int wl_iw_recover = 0;
#endif
static int iw_link_state = 0;

static wlc_ssid_t g_ssid;

wl_iw_ss_cache_ctrl_t g_ss_cache_ctrl;


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#define DAEMONIZE(a) daemonize(a); \
	allow_signal(SIGKILL); \
	allow_signal(SIGTERM);
#else
#define RAISE_RX_SOFTIRQ() \
	cpu_raise_softirq(smp_processor_id(), NET_RX_SOFTIRQ)
#define DAEMONIZE(a) daemonize(); \
	do { if (a) \
		strncpy(current->comm, a, MIN(sizeof(current->comm), (strlen(a) + 1))); \
	} while (0);
#endif

#if defined(WL_IW_USE_ISCAN)

static wlc_ssid_t g_specific_ssid;     /* chache specific ssid request */
#define ISCAN_STATE_IDLE   0
#define ISCAN_STATE_SCANING 1

#define WLC_IW_ISCAN_MAXLEN   2048
typedef struct iscan_buf {
	struct iscan_buf * next;
	char   iscan_buf[WLC_IW_ISCAN_MAXLEN];
} iscan_buf_t;

typedef struct iscan_info {
	struct net_device *dev;
	struct timer_list timer;
	uint32 timer_ms;
	uint32 timer_on;
	int    iscan_state;
	iscan_buf_t * list_hdr;
	iscan_buf_t * list_cur;


	long sysioc_pid;
	struct semaphore sysioc_sem;
	struct completion sysioc_exited;

	uint32 scan_flag;

	char ioctlbuf[WLC_IOCTL_SMLEN];
} iscan_info_t;

static void wl_iw_bt_flag_set(struct net_device *dev, bool set);
static void wl_iw_bt_release(void);

typedef enum bt_coex_status {
	BT_DHCP_IDLE = 0,
	BT_DHCP_START,
	BT_DHCP_OPPORTUNITY_WINDOW,
	BT_DHCP_FLAG_FORCE_TIMEOUT
} coex_status_t;
#define BT_DHCP_OPPORTUNITY_WINDOW_TIEM	2500
#define BT_DHCP_FLAG_FORCE_TIME				5500

typedef struct bt_info {
	struct net_device *dev;
	struct timer_list timer;
	uint32 timer_ms;
	uint32 timer_on;
	int	bt_state;


	long bt_pid;
	struct semaphore bt_sem;
	struct completion bt_exited;
} bt_info_t;

bt_info_t *g_bt = NULL;
static void wl_iw_bt_timerfunc(ulong data);
iscan_info_t *g_iscan = NULL;
static void wl_iw_timerfunc(ulong data);
static void wl_iw_set_event_mask(struct net_device *dev);
static int
wl_iw_iscan(iscan_info_t *iscan, wlc_ssid_t *ssid, uint16 action);
#endif
static int
wl_iw_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
);
static int
wl_iw_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
);

static uint
wl_iw_get_scan_prep(
	wl_scan_results_t *list,
	struct iw_request_info *info,
	char *extra,
	short max_size
);


static void swap_key_from_BE(
	        wl_wsec_key_t *key
)
{
	key->index = htod32(key->index);
	key->len = htod32(key->len);
	key->algo = htod32(key->algo);
	key->flags = htod32(key->flags);
	key->rxiv.hi = htod32(key->rxiv.hi);
	key->rxiv.lo = htod16(key->rxiv.lo);
	key->iv_initialized = htod32(key->iv_initialized);
}

static void swap_key_to_BE(
	        wl_wsec_key_t *key
)
{
	key->index = dtoh32(key->index);
	key->len = dtoh32(key->len);
	key->algo = dtoh32(key->algo);
	key->flags = dtoh32(key->flags);
	key->rxiv.hi = dtoh32(key->rxiv.hi);
	key->rxiv.lo = dtoh16(key->rxiv.lo);
	key->iv_initialized = dtoh32(key->iv_initialized);
}

static int dev_wlc_ioctl_off = 0;

void disable_dev_wlc_ioctl(void)
{
	dev_wlc_ioctl_off = 1;
}

#ifdef WLAN_PROTECT
static void wl_iw_restart(struct net_device *dev)
{
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];
	int cmd = 0;

	WL_TRACE(("Enter %s\n", __FUNCTION__));

	mutex_lock(&wl_start_lock);
#if defined(WL_IW_USE_ISCAN)
	g_iscan->iscan_state = ISCAN_STATE_IDLE;
#endif

	dhd_dev_reset(dev, 1);

#if defined(BCMLXSDMMC)
	sdioh_stop(NULL);
#endif

	dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);

	bcm_mdelay(100);
	dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);

#if defined(BCMLXSDMMC)
	sdioh_start(NULL, 0);
#endif

	dhd_dev_reset(dev, 0);

#if defined(BCMLXSDMMC)
	sdioh_start(NULL, 1);
#endif

	dhd_dev_init_ioctl(dev);

	bcm_mdelay(300);

	if (iw_link_state) {
		cmd = SIOCGIWAP;
		bzero(wrqu.addr.sa_data, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		bzero(&extra, ETHER_ADDR_LEN);
		wireless_send_event(dev, cmd, &wrqu, extra);
		iw_link_state = 0;
	}
	mutex_unlock(&wl_start_lock);
	return;
}
#endif

static int
dev_wlc_ioctl(
	struct net_device *dev,
	int cmd,
	void *arg,
	int len
)
{
	struct ifreq ifr;
	wl_ioctl_t ioc;
	mm_segment_t fs;
	int ret = -EINVAL;

	if (dev_wlc_ioctl_off)
	{
		printk("%s: module removing. skip it.\n", __FUNCTION__);
		return -ENODEV;
	}

	if (g_onoff == G_WLAN_SET_ON) {
#ifdef WLAN_PROTECT
		if (wl_iw_busdown&&!wl_iw_recover) {
			wl_iw_recover = 1;
			wl_iw_restart(dev);
			wl_iw_busdown = 0;
			wl_iw_recover = 0;
		} else if (wl_iw_recover) {
			printk("%s: module removing. skip it.\n", __FUNCTION__);
			return -ENODEV;
		}
#endif
		memset(&ioc, 0, sizeof(ioc));
		ioc.cmd = cmd;
		ioc.buf = arg;
		ioc.len = len;

		strcpy(ifr.ifr_name, dev->name);
		ifr.ifr_data = (caddr_t) &ioc;

		ret = dev_open(dev);
		if (ret) {
			WL_ERROR(("%s: Error dev_open: %d\n", __func__, ret));
			return ret;
		}

		fs = get_fs();
		set_fs(get_ds());
		ret = dev->do_ioctl(dev, &ifr, SIOCDEVPRIVATE);
		set_fs(fs);
	}
	else {
		WL_TRACE(("%s: call after driver stop\n", __FUNCTION__));
	}
	return ret;
}

#ifdef WLAN_PROTECT
void wl_iw_set_busdown(int busdown)
{
	wl_iw_busdown = busdown;
}
#endif

#ifdef WLAN_PFN
static void wl_iw_pfn_set_unlock(void)
{
	struct timer_list **timer;
	timer = &pfn_lock_timer;
	if (*timer) {
		del_timer_sync(*timer);
		kfree(*timer);
		*timer = NULL;
	}
	net_os_wake_unlock(priv_dev);

	return;
}

/* penguin add */
static void wl_iw_set_pfn_timer(int enable)
{
	struct timer_list **timer;

	timer = &pfn_lock_timer;

	if (enable) {
		if (*timer) {
			wl_iw_pfn_set_unlock();
		}
		*timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
		if (!(*timer)) {
			wl_iw_pfn_set_unlock();
			return;
		}
		(*timer)->function = (void *)wl_iw_pfn_set_unlock;
		init_timer(*timer);
		(*timer)->expires = jiffies + PFN_WAKE_TIME * HZ / 1000;	
		add_timer(*timer);
	}
	else {
		if (*timer) {
			wl_iw_pfn_set_unlock();
		}
	}

	return;
}
#endif

static int
dev_wlc_intvar_get_reg(
	struct net_device *dev,
	char *name,
	uint  reg,
	int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	uint len;
	len = bcm_mkiovar(name, (char *)(&reg), sizeof(reg), (char *)(&var), sizeof(var.buf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)&var, len);

	*retval = dtoh32(var.val);
	return (error);
}




static int
dev_wlc_intvar_set(
	struct net_device *dev,
	char *name,
	int val)
{
	char buf[WLC_IOCTL_SMLEN];
	uint len;

	val = htod32(val);
	len = bcm_mkiovar(name, (char *)(&val), sizeof(val), buf, sizeof(buf));
	ASSERT(len);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, buf, len));
}

#if defined(WL_IW_USE_ISCAN)
static int
dev_iw_iovar_setbuf(
	struct net_device *dev,
	char *iovar,
	void *param,
	int paramlen,
	void *bufptr,
	int buflen)
{
	int iolen;

	iolen = bcm_mkiovar(iovar, param, paramlen, bufptr, buflen);
	ASSERT(iolen);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, bufptr, iolen));
}

static int
dev_iw_iovar_getbuf(
	struct net_device *dev,
	char *iovar,
	void *param,
	int paramlen,
	void *bufptr,
	int buflen)
{
	int iolen;

	iolen = bcm_mkiovar(iovar, param, paramlen, bufptr, buflen);
	ASSERT(iolen);

	return (dev_wlc_ioctl(dev, WLC_GET_VAR, bufptr, buflen));
}
#endif


#if WIRELESS_EXT > 17
static int
dev_wlc_bufvar_set(
	struct net_device *dev,
	char *name,
	char *buf, int len)
{
	char ioctlbuf[MAX_WLIW_IOCTL_LEN];
	uint buflen;

	buflen = bcm_mkiovar(name, buf, len, ioctlbuf, sizeof(ioctlbuf));
	ASSERT(buflen);

	return (dev_wlc_ioctl(dev, WLC_SET_VAR, ioctlbuf, buflen));
}
#endif


static int
dev_wlc_bufvar_get(
	struct net_device *dev,
	char *name,
	char *buf, int buflen)
{
	char ioctlbuf[MAX_WLIW_IOCTL_LEN];
	int error;

	uint len;

	len = bcm_mkiovar(name, NULL, 0, ioctlbuf, sizeof(ioctlbuf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)ioctlbuf, MAX_WLIW_IOCTL_LEN);
	if (!error)
		bcopy(ioctlbuf, buf, buflen);

	return (error);
}



static int
dev_wlc_intvar_get(
	struct net_device *dev,
	char *name,
	int *retval)
{
	union {
		char buf[WLC_IOCTL_SMLEN];
		int val;
	} var;
	int error;

	uint len;
	uint data_null;

	len = bcm_mkiovar(name, (char *)(&data_null), 0, (char *)(&var), sizeof(var.buf));
	ASSERT(len);
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, (void *)&var, len);

	*retval = dtoh32(var.val);

	return (error);
}


#if WIRELESS_EXT > 12
static int
wl_iw_set_active_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int as = 0;
	int error = 0;
	char *p = extra;

#if defined(WL_IW_USE_ISCAN)
	if (g_iscan->iscan_state == ISCAN_STATE_IDLE)
#endif
		error = dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as));
#if defined(WL_IW_USE_ISCAN)
	else
		g_iscan->scan_flag = as;
#endif
	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_set_passive_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int ps = 1;
	int error = 0;
	char *p = extra;

#if defined(WL_IW_USE_ISCAN)
	if (g_iscan->iscan_state == ISCAN_STATE_IDLE) {
#endif


		if (g_scan_specified_ssid == 0) {
			error = dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &ps, sizeof(ps));
		}
#if defined(WL_IW_USE_ISCAN)
	}
	else
		g_iscan->scan_flag = ps;
#endif

	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_get_macaddr(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error;
	char buf[128];
	struct ether_addr *id;
	char *p = extra;


	strcpy(buf, "cur_etheraddr");
	error = dev_wlc_ioctl(dev, WLC_GET_VAR, buf, sizeof(buf));
	id = (struct ether_addr *) buf;
	p += snprintf(p, MAX_WX_STRING, "Macaddr = %02X:%02X:%02X:%02X:%02X:%02X\n",
		id->octet[0], id->octet[1], id->octet[2],
		id->octet[3], id->octet[4], id->octet[5]);
	wrqu->data.length = p - extra + 1;

	return error;
}


static int
wl_iw_set_country(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	char country_code[WLC_CNTRY_BUF_SZ];
	int error = 0;
	char *p = extra;
	int country_offset;
	int country_code_size;

	WL_TRACE(("%s\n", __FUNCTION__));
	memset(country_code, 0, sizeof(country_code));

	country_offset = strcspn(extra, " ");
	country_code_size = strlen(extra) - country_offset;


	if (country_offset != 0) {
		strncpy(country_code, extra + country_offset +1,
			MIN(country_code_size, sizeof(country_code)));


		if ((error = dev_wlc_ioctl(dev, WLC_SET_COUNTRY,
			&country_code, sizeof(country_code))) >= 0) {
			p += snprintf(p, MAX_WX_STRING, "OK");
			WL_TRACE(("%s: set country %s OK\n", __FUNCTION__, country_code));
			goto exit;
		}
	}

	WL_ERROR(("%s: set country %s failed code %d\n", __FUNCTION__, country_code, error));
	p += snprintf(p, MAX_WX_STRING, "FAIL");

exit:
	wrqu->data.length = p - extra + 1;
	return error;
}

static int
wl_iw_set_btcoex_dhcp(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error = 0;
	char *p = extra;
	uint val;
#ifdef  CUSTOMER_HW2
	//wl_pkt_filter_enable_t	enable_parm;
	static int  pm = PM_FAST;
	int  pm_local = PM_OFF;
#endif
	char powermode_val = 0;
	char buf_reg66va_dhcp_on[8] = { 66, 00, 00, 00, 0x10, 0x27, 0x00, 0x00 };
	char buf_reg41va_dhcp_on[8] = { 41, 00, 00, 00, 0x33, 0x00, 0x00, 0x00 };
	char buf_reg68va_dhcp_on[8] = { 68, 00, 00, 00, 0x90, 0x01, 0x00, 0x00 };

	char buf_reg66val_defualt[8] = { 66, 00, 00, 00, 0x88, 0x13, 0x00, 0x00 };
	char buf_reg41val_defualt[8] = { 41, 00, 00, 00, 0x13, 0x00, 0x00, 0x00 };
	char buf_reg68val_defualt[8] = { 68, 00, 00, 00, 0x14, 0x00, 0x00, 0x00 };

	char buf_flag7_default[8] =   { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};


	strncpy((char *)&powermode_val, extra + strlen("POWERMODE") +1, 1);


	dev_wlc_intvar_get_reg(dev, "btc_params", 68,  &val);

	if (strnicmp((char *)&powermode_val, "1", strlen("1")) == 0) {

		WL_TRACE(("%s: DHCP session starts\n", __FUNCTION__));

#ifdef  CUSTOMER_HW2

		dev_wlc_ioctl(dev, WLC_GET_PM, &pm, sizeof(pm));

		dev_wlc_ioctl(dev, WLC_SET_PM, &pm_local, sizeof(pm_local));
		/* disable packet filter */
/*		enable_parm.id = htod32(100);
		enable_parm.enable = htod32(0);
		dev_wlc_bufvar_set(dev, "pkt_filter_enable", \
				(char *)&enable_parm, sizeof(enable_parm));
*/
#endif
		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg66va_dhcp_on[0], sizeof(buf_reg66va_dhcp_on));

		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg41va_dhcp_on[0], sizeof(buf_reg41va_dhcp_on));

		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg68va_dhcp_on[0], sizeof(buf_reg68va_dhcp_on));


		g_bt->bt_state = BT_DHCP_START;
		g_bt->timer_on = 1;
		mod_timer(&g_bt->timer, g_bt->timer.expires);
		WL_TRACE(("%s enable BT DHCP Timer\n", __FUNCTION__));

	}
	else if (strnicmp((char *)&powermode_val, "0", strlen("0")) == 0) {

		WL_TRACE(("%s: DHCP session done\n", __FUNCTION__));

#ifdef  CUSTOMER_HW2

		dev_wlc_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm));
		/* enable packet filter */
/*		enable_parm.id = htod32(100);
		enable_parm.enable = htod32(1);
		dev_wlc_bufvar_set(dev, "pkt_filter_enable", \
				(char *)&enable_parm, sizeof(enable_parm));
*/
#endif

		WL_TRACE(("%s disable BT DHCP Timer\n", __FUNCTION__));
		if (g_bt->timer_on) {
			g_bt->timer_on = 0;
			del_timer_sync(&g_bt->timer);
		}


		dev_wlc_bufvar_set(dev, "btc_flags", \
				(char *)&buf_flag7_default[0], sizeof(buf_flag7_default));


		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg66val_defualt[0], sizeof(buf_reg66val_defualt));

		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg41val_defualt[0], sizeof(buf_reg41val_defualt));

		dev_wlc_bufvar_set(dev, "btc_params", \
				   (char *)&buf_reg68val_defualt[0], sizeof(buf_reg68val_defualt));
	}
	else {
		WL_ERROR(("Unkwown yet power setting, ignored\n"));
	}

	p += snprintf(p, MAX_WX_STRING, "OK");

	wrqu->data.length = p - extra + 1;

	return error;
}

int
wl_format_ssid(char* ssid_buf, uint8* ssid, int ssid_len)
{
	int i, c;
	char *p = ssid_buf;

	if (ssid_len > 32) ssid_len = 32;

	for (i = 0; i < ssid_len; i++) {
		c = (int)ssid[i];
		if (c == '\\') {
			*p++ = '\\';
			*p++ = '\\';
		} else if (isprint((uchar)c)) {
			*p++ = (char)c;
		} else {
			p += sprintf(p, "\\x%02X", c);
		}
	}
	*p = '\0';

	return p - ssid_buf;
}

static int
wl_iw_get_link_speed(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int error = 0;
	char *p = extra;
	static int link_speed;


	if (g_onoff == G_WLAN_SET_ON) {
		error = dev_wlc_ioctl(dev, WLC_GET_RATE, &link_speed, sizeof(link_speed));
		link_speed *= 500000;
	}

	p += snprintf(p, MAX_WX_STRING, "LinkSpeed %d", link_speed/1000000);

	wrqu->data.length = p - extra + 1;

	return error;
}

static int
wl_iw_get_rssi(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	static int rssi = 0;
	int error = 0;
	wlc_ssid_t ssid;
	char *p = extra;
	static char ssidbuf[SSID_FMT_BUF_LEN];
	scb_val_t scb_val;

	scb_val.val = 0;

	bzero(&ssid, sizeof(ssid));
	if (g_onoff == G_WLAN_SET_ON) {
		error = dev_wlc_ioctl(dev, WLC_GET_RSSI, &scb_val, sizeof(scb_val_t));
		rssi = dtoh32(scb_val.val);

		error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid));

		ssid.SSID_len = dtoh32(ssid.SSID_len);
	}

	if (!ssid.SSID_len) {
		return 0;
	}

	g_ss_cache_ctrl.last_rssi = rssi;
	/* if rssi is 0, it means we lost the connection */
	if (rssi == 0)
		rssi = WL_IW_RSSI_MINVAL;

	wl_format_ssid(ssidbuf, ssid.SSID, dtoh32(ssid.SSID_len));
	p += snprintf(p, MAX_WX_STRING, "%s rssi %d ", ssidbuf, rssi);
	wrqu->data.length = p - extra + 1;

	return error;
}

static int
wl_iw_send_priv_event(
	struct net_device *dev,
	char *flag
)
{
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];
	int cmd;

	cmd = IWEVCUSTOM;
	memset(&wrqu, 0, sizeof(wrqu));
	if (strlen(flag) > sizeof(extra))
		return -1;

	strcpy(extra, flag);
	wrqu.data.length = strlen(extra);
	wireless_send_event(dev, cmd, &wrqu, extra);
	WL_TRACE(("Send IWEVCUSTOM Event as %s\n", extra));

	return 0;
}

#ifdef WL_IW_USE_THREAD_WL_OFF
static int
_wl_control_sysioc_thread_wl_off(void *data)
{
	struct wl_ctrl *wl_ctl = (struct wl_ctrl *)data;

	DAEMONIZE("wlcontrol_sysioc");

	WL_TRACE(("%s Entered\n", __FUNCTION__));
	net_os_wake_lock(wl_ctl->dev);

	mutex_lock(&wl_start_lock);
	while (down_interruptible(&wl_ctl->timer_sem) == 0) {

		WL_TRACE(("%s Turning off wifi dev\n", __FUNCTION__));

		g_onoff = G_WLAN_SET_OFF;

#if defined(WL_IW_USE_ISCAN)
		g_iscan->iscan_state = ISCAN_STATE_IDLE;
#endif

		dhd_dev_reset(wl_ctl->dev, 1);

#if defined(BCMLXSDMMC)
		sdioh_stop(NULL);
#endif

		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);

		wl_iw_send_priv_event(wl_ctl->dev, "STOP");

		net_os_wake_lock_timeout_enable(wl_ctl->dev);
		break;
	}
	mutex_unlock(&wl_start_lock);
	WL_TRACE(("%s Exited\n", __FUNCTION__));
	net_os_wake_unlock(wl_ctl->dev);

	complete_and_exit(&wl_ctl->sysioc_exited, 0);
	KILL_PROC(wl_ctl->sysioc_pid, SIGTERM);
}
#endif

int
wl_control_wl_start(struct net_device *dev)
{
	int ret = 0;

	WL_TRACE(("Enter %s \n", __FUNCTION__));

	mutex_lock(&wl_start_lock);
	if (g_onoff == G_WLAN_SET_OFF) {
		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_ON);

#if defined(BCMLXSDMMC)
		sdioh_start(NULL, 0);
#endif

		dhd_dev_reset(dev, 0);

#if defined(BCMLXSDMMC)
		sdioh_start(NULL, 1);
#endif

		dhd_dev_init_ioctl(dev);

		g_onoff = G_WLAN_SET_ON;
	}
	WL_TRACE(("Exited %s \n", __FUNCTION__));

	mutex_unlock(&wl_start_lock);
	return ret;
}

#ifdef WL_IW_USE_THREAD_WL_OFF
static void
wl_iw_stop_timerfunc(ulong data)
{
	struct wl_ctrl * wl_ctl = (struct wl_ctrl *)data;

	WL_TRACE(("%s\n", __FUNCTION__));

	del_timer_sync(wl_ctl->timer);

	up(&wl_ctl->timer_sem);
}
#endif

static int
wl_iw_control_wl_off(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;
#ifdef WL_IW_USE_THREAD_WL_OFF
	static struct wl_ctrl ctl;
	static struct timer_list timer;
#endif
	WL_TRACE(("Enter %s\n", __FUNCTION__));

#ifdef WL_IW_USE_THREAD_WL_OFF
	ctl.timer = &timer;
	ctl.dev = dev;
	sema_init(&ctl.timer_sem, 0);
	init_completion(&ctl.sysioc_exited);

	ctl.sysioc_pid = kernel_thread(_wl_control_sysioc_thread_wl_off, &ctl, 0);

	timer.data = (ulong)&ctl;
	timer.function = wl_iw_stop_timerfunc;
	init_timer(&timer);
	timer.expires = jiffies + 2000 * HZ / 1000;
	add_timer(&timer);
#else
	mutex_lock(&wl_start_lock);
	if (g_onoff == G_WLAN_SET_ON) {
		g_onoff = G_WLAN_SET_OFF;
#if defined(WL_IW_USE_ISCAN)
		g_iscan->iscan_state = ISCAN_STATE_IDLE;
#endif

		dhd_dev_reset(dev, 1);

#if defined(BCMLXSDMMC)
		sdioh_stop(NULL);
#endif

		dhd_customer_gpio_wlan_ctrl(WLAN_RESET_OFF);

		wl_iw_send_priv_event(dev, "STOP");

		net_os_wake_lock_timeout_enable(dev);
	}
	/* prevent ON/OFF deadlock */
	bcm_mdelay(1000);
	mutex_unlock(&wl_start_lock);
#endif
	WL_TRACE(("Exited %s\n", __FUNCTION__));

	return ret;
}

static int
wl_iw_control_wl_on(
	struct net_device *dev,
	struct iw_request_info *info
)
{
	int ret = 0;

	WL_TRACE(("Enter %s \n", __FUNCTION__));

	ret = wl_control_wl_start(dev);

	wl_iw_send_priv_event(dev, "START");

	net_os_wake_lock_timeout_enable(dev);

	WL_TRACE(("Exited %s \n", __FUNCTION__));

	return ret;
}


#endif

#if WIRELESS_EXT < 13
struct iw_request_info
{
	__u16		cmd;
	__u16		flags;
};

typedef int (*iw_handler)(struct net_device *dev, struct iw_request_info *info,
	void *wrqu, char *extra);
#endif

static int
wl_iw_config_commit(
	struct net_device *dev,
	struct iw_request_info *info,
	void *zwrq,
	char *extra
)
{
	wlc_ssid_t ssid;
	int error;
	struct sockaddr bssid;

	WL_TRACE(("%s: SIOCSIWCOMMIT\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid))))
		return error;

	ssid.SSID_len = dtoh32(ssid.SSID_len);

	if (!ssid.SSID_len)
		return 0;

	bzero(&bssid, sizeof(struct sockaddr));
	if ((error = dev_wlc_ioctl(dev, WLC_REASSOC, &bssid, ETHER_ADDR_LEN))) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	return 0;
}

static int
wl_iw_get_name(
	struct net_device *dev,
	struct iw_request_info *info,
	char *cwrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWNAME\n", dev->name));

	strcpy(cwrq, "IEEE 802.11-DS");

	return 0;
}

static int
wl_iw_set_freq(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_freq *fwrq,
	char *extra
)
{
	int error, chan;
	uint sf = 0;

	WL_TRACE(("%s: SIOCSIWFREQ\n", dev->name));


	if (fwrq->e == 0 && fwrq->m < MAXCHANNEL) {
		chan = fwrq->m;
	}


	else {

		if (fwrq->e >= 6) {
			fwrq->e -= 6;
			while (fwrq->e--)
				fwrq->m *= 10;
		} else if (fwrq->e < 6) {
			while (fwrq->e++ < 6)
				fwrq->m /= 10;
		}

	if (fwrq->m > 4000 && fwrq->m < 5000)
		sf = WF_CHAN_FACTOR_4_G;

		chan = wf_mhz2channel(fwrq->m, sf);
	}
	chan = htod32(chan);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_CHANNEL, &chan, sizeof(chan))))
		return error;


	return -EINPROGRESS;
}

static int
wl_iw_get_freq(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_freq *fwrq,
	char *extra
)
{
	channel_info_t ci;
	int error;

	WL_TRACE(("%s: SIOCGIWFREQ\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(ci))))
		return error;


	fwrq->m = dtoh32(ci.hw_channel);
	fwrq->e = dtoh32(0);
	return 0;
}

static int
wl_iw_set_mode(
	struct net_device *dev,
	struct iw_request_info *info,
	__u32 *uwrq,
	char *extra
)
{
	int infra = 0, ap = 0, error = 0;

	WL_TRACE(("%s: SIOCSIWMODE\n", dev->name));

	switch (*uwrq) {
	case IW_MODE_MASTER:
		infra = ap = 1;
		break;
	case IW_MODE_ADHOC:
	case IW_MODE_AUTO:
		break;
	case IW_MODE_INFRA:
		infra = 1;
		break;
	default:
		return -EINVAL;
	}
	infra = htod32(infra);
	ap = htod32(ap);

	if ((error = dev_wlc_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(infra))) ||
	    (error = dev_wlc_ioctl(dev, WLC_SET_AP, &ap, sizeof(ap))))
		return error;


	return -EINPROGRESS;
}

static int
wl_iw_get_mode(
	struct net_device *dev,
	struct iw_request_info *info,
	__u32 *uwrq,
	char *extra
)
{
	int error, infra = 0, ap = 0;

	WL_TRACE(("%s: SIOCGIWMODE\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_INFRA, &infra, sizeof(infra))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_AP, &ap, sizeof(ap))))
		return error;

	infra = dtoh32(infra);
	ap = dtoh32(ap);
	*uwrq = infra ? ap ? IW_MODE_MASTER : IW_MODE_INFRA : IW_MODE_ADHOC;

	return 0;
}

static int
wl_iw_get_range(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	struct iw_range *range = (struct iw_range *) extra;
	int channels[MAXCHANNEL+1];
	wl_uint32_list_t *list = (wl_uint32_list_t *) channels;
	wl_rateset_t rateset;
	int error, i, k;
	uint sf, ch;

	int phytype;
	int bw_cap = 0, sgi_tx = 0, nmode = 0;
	channel_info_t ci;
	uint8 nrate_list2copy = 0;
	uint16 nrate_list[4][8] = { {13, 26, 39, 52, 78, 104, 117, 130},
		{14, 29, 43, 58, 87, 116, 130, 144},
		{27, 54, 81, 108, 162, 216, 243, 270},
		{30, 60, 90, 120, 180, 240, 270, 300}};

	WL_TRACE(("%s: SIOCGIWRANGE\n", dev->name));

	if (!extra)
		return -EINVAL;

	dwrq->length = sizeof(struct iw_range);
	memset(range, 0, sizeof(range));


	range->min_nwid = range->max_nwid = 0;


	list->count = htod32(MAXCHANNEL);
	if ((error = dev_wlc_ioctl(dev, WLC_GET_VALID_CHANNELS, channels, sizeof(channels))))
		return error;
	for (i = 0; i < dtoh32(list->count) && i < IW_MAX_FREQUENCIES; i++) {
		range->freq[i].i = dtoh32(list->element[i]);

		ch = dtoh32(list->element[i]);
		if (ch <= CH_MAX_2G_CHANNEL)
			sf = WF_CHAN_FACTOR_2_4_G;
		else
			sf = WF_CHAN_FACTOR_5_G;

		range->freq[i].m = wf_channel2mhz(ch, sf);
		range->freq[i].e = 6;
	}
	range->num_frequency = range->num_channels = i;


	range->max_qual.qual = 5;

	range->max_qual.level = 0x100 - 200;

	range->max_qual.noise = 0x100 - 200;

	range->sensitivity = 65535;

#if WIRELESS_EXT > 11

	range->avg_qual.qual = 3;

	range->avg_qual.level = 0x100 + WL_IW_RSSI_GOOD;

	range->avg_qual.noise = 0x100 - 75;
#endif


	if ((error = dev_wlc_ioctl(dev, WLC_GET_CURR_RATESET, &rateset, sizeof(rateset))))
		return error;
	rateset.count = dtoh32(rateset.count);
	range->num_bitrates = rateset.count;
	for (i = 0; i < rateset.count && i < IW_MAX_BITRATES; i++)
		range->bitrate[i] = (rateset.rates[i]& 0x7f) * 500000;
	dev_wlc_intvar_get(dev, "nmode", &nmode);
	dev_wlc_ioctl(dev, WLC_GET_PHYTYPE, &phytype, sizeof(phytype));

	if (nmode == 1 && phytype == WLC_PHY_TYPE_SSN) {
		dev_wlc_intvar_get(dev, "mimo_bw_cap", &bw_cap);
		dev_wlc_intvar_get(dev, "sgi_tx", &sgi_tx);
		dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(channel_info_t));
		ci.hw_channel = dtoh32(ci.hw_channel);

		if (bw_cap == 0 ||
			(bw_cap == 2 && ci.hw_channel <= 14)) {
			if (sgi_tx == 0)
				nrate_list2copy = 0;
			else
				nrate_list2copy = 1;
		}
		if (bw_cap == 1 ||
			(bw_cap == 2 && ci.hw_channel >= 36)) {
			if (sgi_tx == 0)
				nrate_list2copy = 2;
			else
				nrate_list2copy = 3;
		}
		range->num_bitrates += 8;
		for (k = 0; i < range->num_bitrates; k++, i++) {

			range->bitrate[i] = (nrate_list[nrate_list2copy][k]) * 500000;
		}
	}


	if ((error = dev_wlc_ioctl(dev, WLC_GET_PHYTYPE, &i, sizeof(i))))
		return error;
	i = dtoh32(i);
	if (i == WLC_PHY_TYPE_A)
		range->throughput = 24000000;
	else
		range->throughput = 1500000;


	range->min_rts = 0;
	range->max_rts = 2347;
	range->min_frag = 256;
	range->max_frag = 2346;

	range->max_encoding_tokens = DOT11_MAX_DEFAULT_KEYS;
	range->num_encoding_sizes = 4;
	range->encoding_size[0] = WEP1_KEY_SIZE;
	range->encoding_size[1] = WEP128_KEY_SIZE;
#if WIRELESS_EXT > 17
	range->encoding_size[2] = TKIP_KEY_SIZE;
#else
	range->encoding_size[2] = 0;
#endif
	range->encoding_size[3] = AES_KEY_SIZE;


	range->min_pmp = 0;
	range->max_pmp = 0;
	range->min_pmt = 0;
	range->max_pmt = 0;
	range->pmp_flags = 0;
	range->pm_capa = 0;


	range->num_txpower = 2;
	range->txpower[0] = 1;
	range->txpower[1] = 255;
	range->txpower_capa = IW_TXPOW_MWATT;

#if WIRELESS_EXT > 10
	range->we_version_compiled = WIRELESS_EXT;
	range->we_version_source = 19;


	range->retry_capa = IW_RETRY_LIMIT;
	range->retry_flags = IW_RETRY_LIMIT;
	range->r_time_flags = 0;

	range->min_retry = 1;
	range->max_retry = 255;

	range->min_r_time = 0;
	range->max_r_time = 0;
#endif

#if WIRELESS_EXT > 17
	range->enc_capa = IW_ENC_CAPA_WPA;
	range->enc_capa |= IW_ENC_CAPA_CIPHER_TKIP;
	range->enc_capa |= IW_ENC_CAPA_CIPHER_CCMP;
#ifdef BCMWPA2
	range->enc_capa |= IW_ENC_CAPA_WPA2;
#endif


	IW_EVENT_CAPA_SET_KERNEL(range->event_capa);

	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWAP);
	IW_EVENT_CAPA_SET(range->event_capa, SIOCGIWSCAN);
	IW_EVENT_CAPA_SET(range->event_capa, IWEVTXDROP);
	IW_EVENT_CAPA_SET(range->event_capa, IWEVMICHAELMICFAILURE);
#ifdef BCMWPA2
	IW_EVENT_CAPA_SET(range->event_capa, IWEVPMKIDCAND);
#endif
#endif

	return 0;
}

static int
rssi_to_qual(int rssi)
{
	if (rssi <= WL_IW_RSSI_NO_SIGNAL)
		return 0;
	else if (rssi <= WL_IW_RSSI_VERY_LOW)
		return 1;
	else if (rssi <= WL_IW_RSSI_LOW)
		return 2;
	else if (rssi <= WL_IW_RSSI_GOOD)
		return 3;
	else if (rssi <= WL_IW_RSSI_VERY_GOOD)
		return 4;
	else
		return 5;
}

static int
wl_iw_set_spy(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);
	struct sockaddr *addr = (struct sockaddr *) extra;
	int i;

	WL_TRACE(("%s: SIOCSIWSPY\n", dev->name));

	if (!extra)
		return -EINVAL;

	iw->spy_num = MIN(ARRAYSIZE(iw->spy_addr), dwrq->length);
	for (i = 0; i < iw->spy_num; i++)
		memcpy(&iw->spy_addr[i], addr[i].sa_data, ETHER_ADDR_LEN);
	memset(iw->spy_qual, 0, sizeof(iw->spy_qual));

	return 0;
}

static int
wl_iw_get_spy(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);
	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality *qual = (struct iw_quality *) &addr[iw->spy_num];
	int i;

	WL_TRACE(("%s: SIOCGIWSPY\n", dev->name));

	if (!extra)
		return -EINVAL;

	dwrq->length = iw->spy_num;
	for (i = 0; i < iw->spy_num; i++) {
		memcpy(addr[i].sa_data, &iw->spy_addr[i], ETHER_ADDR_LEN);
		addr[i].sa_family = AF_UNIX;
		memcpy(&qual[i], &iw->spy_qual[i], sizeof(struct iw_quality));
		iw->spy_qual[i].updated = 0;
	}

	return 0;
}

static int
wl_iw_set_wap(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	int error = -EINVAL;
	wl_join_params_t join_params;

	WL_TRACE(("%s: SIOCSIWAP\n", dev->name));

	if (awrq->sa_family != ARPHRD_ETHER) {
		WL_ERROR(("Invalid Header...sa_family\n"));
		return -EINVAL;
	}


	if (ETHER_ISBCAST(awrq->sa_data) || ETHER_ISNULLADDR(awrq->sa_data)) {
		scb_val_t scbval;

		bzero(&scbval, sizeof(scb_val_t));

		(void) dev_wlc_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t));
		return 0;
	}



	memset(&join_params, 0, sizeof(join_params));

#ifdef WLAN_PFN
	dhd_set_pfn_ssid(g_ssid.SSID, g_ssid.SSID_len);
#endif
	memcpy(join_params.ssid.SSID, g_ssid.SSID, g_ssid.SSID_len);
	join_params.ssid.SSID_len = htod32(g_ssid.SSID_len);
	memcpy(&join_params.params.bssid, awrq->sa_data, ETHER_ADDR_LEN);

	if ((error = dev_wlc_ioctl(dev, WLC_SET_SSID, &join_params, sizeof(join_params)))) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}


	memset(&g_ssid, 0, sizeof(g_ssid));
	return 0;
}

static int
wl_iw_get_wap(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWAP\n", dev->name));

	awrq->sa_family = ARPHRD_ETHER;
	memset(awrq->sa_data, 0, ETHER_ADDR_LEN);


	(void) dev_wlc_ioctl(dev, WLC_GET_BSSID, awrq->sa_data, ETHER_ADDR_LEN);

	return 0;
}

#if WIRELESS_EXT > 17
static int
wl_iw_mlme(
	struct net_device *dev,
	struct iw_request_info *info,
	struct sockaddr *awrq,
	char *extra
)
{
	struct iw_mlme *mlme;
	scb_val_t scbval;
	int error  = -EINVAL;

	WL_TRACE(("%s: SIOCSIWMLME DISASSOC/DEAUTH\n", dev->name));

	mlme = (struct iw_mlme *)extra;
	if (mlme == NULL) {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	scbval.val = mlme->reason_code;
	bcopy(&mlme->addr.sa_data, &scbval.ea, ETHER_ADDR_LEN);

	if (mlme->cmd == IW_MLME_DISASSOC) {
		scbval.val = htod32(scbval.val);
		error = dev_wlc_ioctl(dev, WLC_DISASSOC, &scbval, sizeof(scb_val_t));
	}
	else if (mlme->cmd == IW_MLME_DEAUTH) {
		scbval.val = htod32(scbval.val);
		error = dev_wlc_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
			sizeof(scb_val_t));
	}
	else {
		WL_ERROR(("Invalid ioctl data.\n"));
		return error;
	}

	return error;
}
#endif

static int
wl_iw_get_aplist(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality qual[IW_MAX_AP];
	wl_bss_info_t *bi = NULL;
	int error, i;
	uint buflen = dwrq->length;

	WL_TRACE(("%s: SIOCGIWAPLIST\n", dev->name));

	if (!extra)
		return -EINVAL;


	list = kmalloc(buflen, GFP_KERNEL);
	if (!list)
		return -ENOMEM;
	memset(list, 0, buflen);
	list->buflen = htod32(buflen);
	if ((error = dev_wlc_ioctl(dev, WLC_SCAN_RESULTS, list, buflen))) {
		WL_ERROR(("%d: Scan results error %d\n", __LINE__, error));
		kfree(list);
		return error;
	}
	list->buflen = dtoh32(list->buflen);
	list->version = dtoh32(list->version);
	list->count = dtoh32(list->count);
	ASSERT(list->version == WL_BSS_INFO_VERSION);

	for (i = 0, dwrq->length = 0; i < list->count && dwrq->length < IW_MAX_AP; i++) {
		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;
		ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
			buflen));


		if (!(dtoh16(bi->capability) & DOT11_CAP_ESS))
			continue;


		memcpy(addr[dwrq->length].sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		addr[dwrq->length].sa_family = ARPHRD_ETHER;
		qual[dwrq->length].qual = rssi_to_qual(dtoh16(bi->RSSI));
		qual[dwrq->length].level = 0x100 + dtoh16(bi->RSSI);
		qual[dwrq->length].noise = 0x100 + bi->phy_noise;


#if WIRELESS_EXT > 18
		qual[dwrq->length].updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
#else
		qual[dwrq->length].updated = 7;
#endif

		dwrq->length++;
	}

	kfree(list);

	if (dwrq->length) {
		memcpy(&addr[dwrq->length], qual, sizeof(struct iw_quality) * dwrq->length);

		dwrq->flags = 1;
	}

	return 0;
}

#ifdef WL_IW_USE_ISCAN
static int
wl_iw_iscan_get_aplist(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
	iscan_buf_t * buf;
	iscan_info_t *iscan = g_iscan;

	struct sockaddr *addr = (struct sockaddr *) extra;
	struct iw_quality qual[IW_MAX_AP];
	wl_bss_info_t *bi = NULL;
	int i;

	WL_TRACE(("%s: SIOCGIWAPLIST\n", dev->name));

	if (!extra)
		return -EINVAL;

	if ((!iscan) || (iscan->sysioc_pid < 0)) {
		return wl_iw_get_aplist(dev, info, dwrq, extra);
	}

	buf = iscan->list_hdr;

	while (buf) {
		list = &((wl_iscan_results_t*)buf->iscan_buf)->results;
		ASSERT(list->version == WL_BSS_INFO_VERSION);

		bi = NULL;
		for (i = 0, dwrq->length = 0; i < list->count && dwrq->length < IW_MAX_AP; i++) {
			bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length))
			          : list->bss_info;
			ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
				WLC_IW_ISCAN_MAXLEN));


			if (!(dtoh16(bi->capability) & DOT11_CAP_ESS))
				continue;


			memcpy(addr[dwrq->length].sa_data, &bi->BSSID, ETHER_ADDR_LEN);
			addr[dwrq->length].sa_family = ARPHRD_ETHER;
			qual[dwrq->length].qual = rssi_to_qual(dtoh16(bi->RSSI));
			qual[dwrq->length].level = 0x100 + dtoh16(bi->RSSI);
			qual[dwrq->length].noise = 0x100 + bi->phy_noise;


#if WIRELESS_EXT > 18
			qual[dwrq->length].updated = IW_QUAL_ALL_UPDATED | IW_QUAL_DBM;
#else
			qual[dwrq->length].updated = 7;
#endif

			dwrq->length++;
		}
		buf = buf->next;
	}
	if (dwrq->length) {
		memcpy(&addr[dwrq->length], qual, sizeof(struct iw_quality) * dwrq->length);

		dwrq->flags = 1;
	}

	return 0;
}

static int
wl_iw_iscan_prep(wl_scan_params_t *params, wlc_ssid_t *ssid)
{
	int err = 0;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;
	params->nprobes = -1;
	params->active_time = -1;
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;

	params->nprobes = htod32(params->nprobes);
	params->active_time = htod32(params->active_time);
	params->passive_time = htod32(params->passive_time);
	params->home_time = htod32(params->home_time);
	if (ssid && ssid->SSID_len)
		memcpy(&params->ssid, ssid, sizeof(wlc_ssid_t));

	return err;
}

static int
wl_iw_iscan(iscan_info_t *iscan, wlc_ssid_t *ssid, uint16 action)
{
	int params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_iscan_params_t, params));
	wl_iscan_params_t *params;
	int err = 0, retry = 0;

	if (ssid && ssid->SSID_len) {
		params_size += sizeof(wlc_ssid_t);
	}
	params = (wl_iscan_params_t*)kmalloc(params_size, GFP_KERNEL);
	if (params == NULL) {
		return -ENOMEM;
	}
	memset(params, 0, params_size);
	ASSERT(params_size < WLC_IOCTL_SMLEN);

	err = wl_iw_iscan_prep(&params->params, ssid);

	if (!err) {
		params->version = htod32(ISCAN_REQ_VERSION);
		params->action = htod16(action);
		params->scan_duration = htod16(0);


iscan_retry:
		err = dev_iw_iovar_setbuf(iscan->dev, "iscan", params, params_size,
			iscan->ioctlbuf, WLC_IOCTL_SMLEN);
		if (err&&(retry < 2)) {
			retry++;
			WL_ERROR(("penguin, iscan retry!\n"));
			bcm_mdelay(500);
			goto iscan_retry;
		}

		if (err)
			WL_ERROR(("penguin, iscan fail!\n"));
	}

	kfree(params);
	return err;
}

static void
wl_iw_timerfunc(ulong data)
{
	iscan_info_t *iscan = (iscan_info_t *)data;
	if (iscan) {
		iscan->timer_on = 0;
		if (iscan->iscan_state != ISCAN_STATE_IDLE) {
			WL_TRACE(("timer trigger\n"));
			up(&iscan->sysioc_sem);
		}
	}
}
static void wl_iw_set_event_mask(struct net_device *dev)
{
	char eventmask[WL_EVENTING_MASK_LEN];
	char iovbuf[WL_EVENTING_MASK_LEN + 12];

	dev_iw_iovar_getbuf(dev, "event_msgs", "", 0, iovbuf, sizeof(iovbuf));
	bcopy(iovbuf, eventmask, WL_EVENTING_MASK_LEN);
	setbit(eventmask, WLC_E_SCAN_COMPLETE);
	dev_iw_iovar_setbuf(dev, "event_msgs", eventmask, WL_EVENTING_MASK_LEN,
		iovbuf, sizeof(iovbuf));
}

static uint32
wl_iw_iscan_get(iscan_info_t *iscan)
{
	iscan_buf_t * buf;
	iscan_buf_t * ptr;
	wl_iscan_results_t * list_buf;
	wl_iscan_results_t list;
	wl_scan_results_t *results;
	uint32 status;


	if (iscan->list_cur) {
		buf = iscan->list_cur;
		iscan->list_cur = buf->next;
	}
	else {
		buf = kmalloc(sizeof(iscan_buf_t), GFP_KERNEL);
		if (!buf)
			return WL_SCAN_RESULTS_ABORTED;
		buf->next = NULL;
		if (!iscan->list_hdr)
			iscan->list_hdr = buf;
		else {
			ptr = iscan->list_hdr;
			while (ptr->next) {
				ptr = ptr->next;
			}
			ptr->next = buf;
		}
	}
	memset(buf->iscan_buf, 0, WLC_IW_ISCAN_MAXLEN);
	list_buf = (wl_iscan_results_t*)buf->iscan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WLC_IW_ISCAN_MAXLEN);
	(void) dev_iw_iovar_getbuf(
		iscan->dev,
		"iscanresults",
		&list,
		WL_ISCAN_RESULTS_FIXED_SIZE,
		buf->iscan_buf,
		WLC_IW_ISCAN_MAXLEN);
	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	results->count = dtoh32(results->count);
	WL_TRACE(("results->count = %d\n", results->count));

	WL_TRACE(("results->buflen = %d\n", results->buflen));
	status = dtoh32(list_buf->status);
	return status;
}

static void wl_iw_force_specific_scan(iscan_info_t *iscan)
{
	WL_TRACE(("### Force Specific SCAN for %s\n", g_specific_ssid.SSID));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_lock();
#endif
	(void) dev_wlc_ioctl(iscan->dev, WLC_SCAN, &g_specific_ssid, sizeof(g_specific_ssid));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_unlock();
#endif
}

static void wl_iw_send_scan_complete(iscan_info_t *iscan)
{
#ifndef SANDGATE2G
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];

		memset(&wrqu, 0, sizeof(wrqu));
		memset(extra, 0, sizeof(extra));
		wireless_send_event(iscan->dev, SIOCGIWSCAN, &wrqu, extra);
		WL_TRACE(("Send Event SCAN complete\n"));
#endif
}
static int
_iscan_sysioc_thread(void *data)
{
	uint32 status;
	iscan_info_t *iscan = (iscan_info_t *)data;
	static bool iscan_pass_abort = FALSE;

	DAEMONIZE("iscan_sysioc");

	status = WL_SCAN_RESULTS_PARTIAL;
	while (down_interruptible(&iscan->sysioc_sem) == 0) {

		net_os_wake_lock(iscan->dev);

		if (iscan->timer_on) {
			iscan->timer_on = 0;
			del_timer_sync(&iscan->timer);
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
		rtnl_lock();
#endif
		status = wl_iw_iscan_get(iscan);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
		rtnl_unlock();
#endif

		if (g_scan_specified_ssid && (iscan_pass_abort == TRUE)) {
			WL_TRACE(("%s Get results from specific scan sttaus=%d\n", __FUNCTION__, status));
			wl_iw_send_scan_complete(iscan);
			iscan_pass_abort = FALSE;
			status  = -1;
		}
#ifdef WLAN_PFN
		else if (pfn_scan == 1) {
			pfn_scan = 0;
			g_scan_specified_ssid = 1;
			strcpy(g_specific_ssid.SSID, pfn_ssid);
			g_specific_ssid.SSID_len = pfn_ssid_len;
			wl_iw_force_specific_scan(iscan);
			status  = -1;
		}
#endif
		switch (status) {
			case WL_SCAN_RESULTS_PARTIAL:
				WL_TRACE(("iscanresults incomplete\n"));
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
				rtnl_lock();
#endif

				wl_iw_iscan(iscan, NULL, WL_SCAN_ACTION_CONTINUE);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
				rtnl_unlock();
#endif

				mod_timer(&iscan->timer, jiffies + iscan->timer_ms*HZ/1000);
				iscan->timer_on = 1;
				break;
			case WL_SCAN_RESULTS_SUCCESS:
				WL_TRACE(("iscanresults complete\n"));
				iscan->iscan_state = ISCAN_STATE_IDLE;
				wl_iw_send_scan_complete(iscan);
				break;
			case WL_SCAN_RESULTS_PENDING:
				WL_TRACE(("iscanresults pending\n"));

				mod_timer(&iscan->timer, jiffies + iscan->timer_ms*HZ/1000);
				iscan->timer_on = 1;
				break;
			case WL_SCAN_RESULTS_ABORTED:
				WL_TRACE(("iscanresults aborted\n"));
				iscan->iscan_state = ISCAN_STATE_IDLE;
				if (g_scan_specified_ssid == 0)
					wl_iw_send_scan_complete(iscan);
				else {
					iscan_pass_abort = TRUE;
					wl_iw_force_specific_scan(iscan);
				}
				break;
			default:
				WL_TRACE(("iscanresults returned unknown status %d\n", status));
				break;
		}

		net_os_wake_unlock(iscan->dev);
	}

	if (iscan->timer_on) {
		iscan->timer_on = 0;
		del_timer_sync(&iscan->timer);
	}

	complete_and_exit(&iscan->sysioc_exited, 0);
}
#endif



void
wl_iw_init_ss_cache_ctrl(void)
{
	g_ss_cache_ctrl.m_prev_scan_mode = 0;
	g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
	g_ss_cache_ctrl.m_cache_head = NULL;
	g_ss_cache_ctrl.m_link_down = 0;
	g_ss_cache_ctrl.m_timer_expired = 0;
	g_ss_cache_ctrl.m_timer = NULL;
	g_ss_cache_ctrl.last_rssi = -200;
	memset(g_ss_cache_ctrl.m_active_bssid, 0, ETHER_ADDR_LEN);
}



static void
wl_iw_free_ss_cache(void)
{
	wl_iw_ss_cache_t *node, *cur;
	wl_iw_ss_cache_t **spec_scan_head;

	WL_TRACE(("%s called\n", __FUNCTION__));

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;

	for (;node;) {
		WL_TRACE(("%s : SSID - %s\n", __FUNCTION__, node->bss_info->SSID));
		cur = node;
		node = cur->next;
		kfree(cur);
	}
	*spec_scan_head = NULL;
}

static void
wl_iw_set_ss_cache_timer_flag(void)
{
	g_ss_cache_ctrl.m_timer_expired = 1;
	WL_TRACE(("%s called\n", __FUNCTION__));
}


static int
wl_iw_run_ss_cache_timer(int kick_off)
{
	struct timer_list **timer;

	timer = &g_ss_cache_ctrl.m_timer;

	if (kick_off) {
		*timer = kmalloc(sizeof(struct timer_list), GFP_KERNEL);
		if (!(*timer)) {
			return -ENOMEM;
		}
		(*timer)->function = (void *)wl_iw_set_ss_cache_timer_flag;
		init_timer(*timer);
		(*timer)->expires = jiffies + 30000 * HZ / 1000;
		add_timer(*timer);
		WL_TRACE(("%s : timer starts \n", __FUNCTION__));
	}
	else {
		if (*timer) {
			del_timer_sync(*timer);
			kfree(*timer);
			*timer = NULL;
		}
		WL_TRACE(("%s : timer stops \n", __FUNCTION__));
	}

	return 0;
}


void
wl_iw_release_ss_cache_ctrl(void)
{
	wl_iw_free_ss_cache();
	wl_iw_run_ss_cache_timer(0);
}



static void
wl_iw_reset_ss_cache(void)
{
	wl_iw_ss_cache_t *node, *prev, *cur;
	wl_iw_ss_cache_t **spec_scan_head;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;
	prev = node;

	for (;node;) {
		WL_TRACE(("%s : node SSID %s \n", __FUNCTION__, node->bss_info->SSID));
		if (!node->dirty) {
			cur = node;
			if (cur == *spec_scan_head) {
				*spec_scan_head = cur->next;
				prev = *spec_scan_head;
			}
			else {
				prev->next = cur->next;
			}
			node = cur->next;

			WL_TRACE(("%s : Del node : SSID %s\n", __FUNCTION__, cur->bss_info->SSID));
			kfree(cur);
			continue;
		}

		node->dirty = 0;
		prev = node;
		node = node->next;
	}

}


static int
wl_iw_add_bss_to_ss_cache(wl_scan_results_t *ss_list)
{

	wl_iw_ss_cache_t *node, *prev, *leaf;
	wl_iw_ss_cache_t **spec_scan_head;
	wl_bss_info_t *bi = NULL;
	int i;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;

	if (!ss_list->count) {
		return 0;
	}


	for (i = 0; i < ss_list->count; i++) {

		node = *spec_scan_head;
		prev = node;

		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : ss_list->bss_info;

		WL_TRACE(("%s : %dth SSID %s\n", __FUNCTION__, i, bi->SSID));
		for (;node;) {
			if (!memcmp(&node->bss_info->BSSID, &bi->BSSID, ETHER_ADDR_LEN)) {

				WL_TRACE(("dirty marked : SSID %s\n", bi->SSID));
				node->dirty = 1;
				break;
			}
			prev = node;
			node = node->next;
		}

		if (node) {
			continue;
		}
		leaf = kmalloc(WLC_IW_SS_CACHE_MAXLEN, GFP_KERNEL);
		if (!leaf) {
			return -ENOMEM;
		}

		if (bi->length > WLC_IW_BSS_INFO_MAXLEN) {
			WL_TRACE(("bss info length is too long : %d\n", bi->length));
			kfree(leaf);
			continue;
		}

		memcpy(leaf->bss_info, bi, bi->length);
		leaf->next = NULL;
		leaf->dirty = 1;
		leaf->count = 1;
		leaf->version = ss_list->version;

		if (!prev) {
			*spec_scan_head = leaf;
		}
		else {
			prev->next = leaf;
		}
	}
	return 0;

}


static int
wl_iw_merge_scan_cache(struct iw_request_info *info, char *extra, uint buflen_from_user,
__u16 *merged_len)
{
	wl_iw_ss_cache_t *node;
	wl_scan_results_t *list_merge;

	node = g_ss_cache_ctrl.m_cache_head;
	for (;node;) {
		list_merge = (wl_scan_results_t *)node;
		WL_TRACE(("%s: Bcast APs list=%d\n", __FUNCTION__, list_merge->count));
		if (buflen_from_user - *merged_len > 0) {
			*merged_len += (__u16) wl_iw_get_scan_prep(list_merge, info,
				extra + *merged_len, buflen_from_user - *merged_len);
		}
		else {
			break;
		}
		node = node->next;
	}
	return 0;
}


static int
wl_iw_delete_bss_from_ss_cache(void *addr)
{

	wl_iw_ss_cache_t *node, *prev;
	wl_iw_ss_cache_t **spec_scan_head;

	spec_scan_head = &g_ss_cache_ctrl.m_cache_head;
	node = *spec_scan_head;
	prev = node;
	for (;node;) {
		if (!memcmp(&node->bss_info->BSSID, addr, ETHER_ADDR_LEN)) {
			if (node == *spec_scan_head) {
				*spec_scan_head = node->next;
			}
			else {
				prev->next = node->next;
			}

			WL_TRACE(("%s : Del node : %s\n", __FUNCTION__, node->bss_info->SSID));
			kfree(node);
			break;
		}

		prev = node;
		node = node->next;
	}

	memset(addr, 0, ETHER_ADDR_LEN);
	return 0;

}



static int
wl_iw_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	int ret, retry = 0;
	WL_TRACE(("%s: SIOCSIWSCAN\n", dev->name));


	if (g_onoff == G_WLAN_SET_OFF)
		return 0;


	memset(&g_specific_ssid, 0, sizeof(g_specific_ssid));
	g_scan_specified_ssid	= 0;

#if WIRELESS_EXT > 17

	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			g_specific_ssid.SSID_len = MIN(sizeof(g_specific_ssid.SSID), req->essid_len);
			memcpy(g_specific_ssid.SSID, req->essid, g_specific_ssid.SSID_len);
			g_specific_ssid.SSID_len = htod32(g_specific_ssid.SSID_len);
			g_scan_specified_ssid = 1;
			WL_TRACE(("Specific scan ssid=%s len=%d\n", g_specific_ssid.SSID, g_specific_ssid.SSID_len));
		}
	}
#endif

specific_retry:
	ret = dev_wlc_ioctl(dev, WLC_SCAN, &g_specific_ssid, sizeof(g_specific_ssid));

	if (ret&&(retry < 2)) {
		bcm_mdelay(500);
		WL_ERROR(("specific retry!\n"));
		retry++;
		goto specific_retry;
	}
	if (ret)
		WL_ERROR(("penguin, specific scan fail!\n"));
	return 0;
}

#ifdef WL_IW_USE_ISCAN
static int
wl_iw_iscan_set_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	union iwreq_data *wrqu,
	char *extra
)
{
	wlc_ssid_t ssid;
	iscan_info_t *iscan = g_iscan;

	WL_TRACE(("%s: SIOCSIWSCAN\n", dev->name));


	if (g_onoff == G_WLAN_SET_OFF)
		return 0;


	if ((!iscan) || (iscan->sysioc_pid < 0)) {
		return wl_iw_set_scan(dev, info, wrqu, extra);
	}
	if (iscan->iscan_state == ISCAN_STATE_SCANING) {
		return 0;
	}


	memset(&ssid, 0, sizeof(ssid));

	g_scan_specified_ssid = 0;
#if WIRELESS_EXT > 17

	if (wrqu->data.length == sizeof(struct iw_scan_req)) {
		if (wrqu->data.flags & IW_SCAN_THIS_ESSID) {
			int as = 0;
			struct iw_scan_req *req = (struct iw_scan_req *)extra;
			ssid.SSID_len = MIN(sizeof(ssid.SSID), req->essid_len);
			memcpy(ssid.SSID, req->essid, ssid.SSID_len);
			ssid.SSID_len = htod32(ssid.SSID_len);
			g_scan_specified_ssid = 1;
			(void) dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN, &as, sizeof(as));
			return wl_iw_set_scan(dev, info, wrqu, extra);
		}
	}
#endif

	iscan->list_cur = iscan->list_hdr;
	iscan->iscan_state = ISCAN_STATE_SCANING;
	(void) dev_wlc_ioctl(dev, WLC_SET_PASSIVE_SCAN,
		&iscan->scan_flag, sizeof(iscan->scan_flag));
	wl_iw_set_event_mask(dev);
	wl_iw_iscan(iscan, &ssid, WL_SCAN_ACTION_START);
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms*HZ/1000);
	iscan->timer_on = 1;

	return 0;
}
#endif

#if WIRELESS_EXT > 17
static bool
ie_is_wpa_ie(uint8 **wpaie, uint8 **tlvs, int *tlvs_len)
{


	uint8 *ie = *wpaie;


	if ((ie[1] >= 6) &&
		!bcmp((const void *)&ie[2], (const void *)(WPA_OUI "\x01"), 4)) {
		return TRUE;
	}


	ie += ie[1] + 2;

	*tlvs_len -= (int)(ie - *tlvs);

	*tlvs = ie;
	return FALSE;
}

static bool
ie_is_wps_ie(uint8 **wpsie, uint8 **tlvs, int *tlvs_len)
{


	uint8 *ie = *wpsie;


	if ((ie[1] >= 4) &&
		!bcmp((const void *)&ie[2], (const void *)(WPA_OUI "\x04"), 4)) {
		return TRUE;
	}


	ie += ie[1] + 2;

	*tlvs_len -= (int)(ie - *tlvs);

	*tlvs = ie;
	return FALSE;
}
#endif


static int
wl_iw_handle_scanresults_ies(char **event_p, char *end,
	struct iw_request_info *info, wl_bss_info_t *bi)
{
#if WIRELESS_EXT > 17
	struct iw_event	iwe;
	char *event;

	event = *event_p;
	if (bi->ie_length) {

		bcm_tlv_t *ie;
		uint8 *ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
		int ptr_len = bi->ie_length;

#ifdef BCMWPA2
		if ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_RSN_ID))) {
			iwe.cmd = IWEVGENIE;
			iwe.u.data.length = ie->len + 2;
			event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
		}
		ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
#endif

		while ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_WPA_ID))) {

			if (ie_is_wps_ie(((uint8 **)&ie), &ptr, &ptr_len)) {
				iwe.cmd = IWEVGENIE;
				iwe.u.data.length = ie->len + 2;
				event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
				break;
			}
		}

		ptr = ((uint8 *)bi) + sizeof(wl_bss_info_t);
		ptr_len = bi->ie_length;
		while ((ie = bcm_parse_tlvs(ptr, ptr_len, DOT11_MNG_WPA_ID))) {
			if (ie_is_wpa_ie(((uint8 **)&ie), &ptr, &ptr_len)) {
				iwe.cmd = IWEVGENIE;
				iwe.u.data.length = ie->len + 2;
				event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)ie);
				break;
			}
		}

	*event_p = event;
	}
#endif

	return 0;
}

static uint
wl_iw_get_scan_prep(
	wl_scan_results_t *list,
	struct iw_request_info *info,
	char *extra,
	short max_size)
{
	int  i, j;
	struct iw_event  iwe;
	wl_bss_info_t *bi = NULL;
	char *event = extra, *end = extra + max_size - WE_ADD_EVENT_FIX, *value;
	int	ret = 0;

	ASSERT(list);



	for (i = 0; i < list->count && i < IW_MAX_AP; i++)
	{
		ASSERT(list->version == WL_BSS_INFO_VERSION);

		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;


		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_ADDR_LEN);

		iwe.u.data.length = dtoh32(bi->SSID_len);
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.flags = 1;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, bi->SSID);


		if (dtoh16(bi->capability) & (DOT11_CAP_ESS | DOT11_CAP_IBSS)) {
			iwe.cmd = SIOCGIWMODE;
			if (dtoh16(bi->capability) & DOT11_CAP_ESS)
				iwe.u.mode = IW_MODE_INFRA;
			else
				iwe.u.mode = IW_MODE_ADHOC;
			event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_UINT_LEN);
		}


		iwe.cmd = SIOCGIWFREQ;
#ifdef  CUSTOMER_HW2
		if (dtoh32(bi->version) != 107 && bi->n_cap)
		{
		   iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->ctl_ch),
		   CHSPEC_CHANNEL(bi->ctl_ch) <= CH_MAX_2G_CHANNEL ?
		   WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		} else
#endif
		iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->chanspec),
			CHSPEC_CHANNEL(bi->chanspec) <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		iwe.u.freq.e = 6;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_FREQ_LEN);


		iwe.cmd = IWEVQUAL;
		iwe.u.qual.qual = rssi_to_qual(dtoh16(bi->RSSI));
		iwe.u.qual.level = 0x100 + dtoh16(bi->RSSI);
		iwe.u.qual.noise = 0x100 + bi->phy_noise;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_QUAL_LEN);


		 wl_iw_handle_scanresults_ies(&event, end, info, bi);


		iwe.cmd = SIOCGIWENCODE;
		if (dtoh16(bi->capability) & DOT11_CAP_PRIVACY)
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		iwe.u.data.length = 0;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)event);


		if (bi->rateset.count) {
			if (((event - extra) + IW_EV_LCP_LEN) <= (int)end) {
				value = event + IW_EV_LCP_LEN;
				iwe.cmd = SIOCGIWRATE;

				iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
				for (j = 0; j < bi->rateset.count && j < IW_MAX_BITRATES; j++) {
					iwe.u.bitrate.value = (bi->rateset.rates[j] & 0x7f) * 500000;
					value = IWE_STREAM_ADD_VALUE(info, event, value, end, &iwe,
						IW_EV_PARAM_LEN);
				}
				event = value;
			}
		}
	}

	if ((ret = (event - extra)) < 0) {
		WL_ERROR(("==> Wrong size\n"));
		ret = 0;
	}
	WL_TRACE(("%s: size=%d bytes prepared \n", __FUNCTION__, (unsigned int)(event - extra)));
	return (uint)ret;
}

static int
wl_iw_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	channel_info_t ci;
	wl_scan_results_t *list_merge;
	wl_scan_results_t *list = (wl_scan_results_t *) g_scan;
	int error;
	uint buflen_from_user = dwrq->length;
	uint len =  G_SCAN_RESULTS;
	__u16 len_ret = 0;
	__u16 merged_len = 0;
#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = g_iscan;
	iscan_buf_t * p_buf;
#endif

	WL_TRACE(("%s: buflen_from_user %d: \n", dev->name, buflen_from_user));

	if (!extra) {
		WL_TRACE(("%s: wl_iw_get_scan return -EINVAL\n", dev->name));
		return -EINVAL;
	}


	if ((error = dev_wlc_ioctl(dev, WLC_GET_CHANNEL, &ci, sizeof(ci))))
		return error;
	ci.scan_channel = dtoh32(ci.scan_channel);
	if (ci.scan_channel)
		return -EAGAIN;

	if (g_ss_cache_ctrl.m_timer_expired) {
		wl_iw_free_ss_cache();
		g_ss_cache_ctrl.m_timer_expired ^= 1;
	}
	if ((!g_scan_specified_ssid && g_ss_cache_ctrl.m_prev_scan_mode) ||
		g_ss_cache_ctrl.m_cons_br_scan_cnt > 4) {
		g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;

		wl_iw_reset_ss_cache();
	}
	g_ss_cache_ctrl.m_prev_scan_mode = g_scan_specified_ssid;
	if (g_scan_specified_ssid) {
		g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;
	}
	else {
		g_ss_cache_ctrl.m_cons_br_scan_cnt++;
	}



	if (g_scan_specified_ssid) {

		list = kmalloc(len, GFP_KERNEL);
		if (!list) {
			WL_TRACE(("%s: wl_iw_get_scan return -ENOMEM\n", dev->name));
			return -ENOMEM;
		}
	}

	memset(list, 0, len);
	list->buflen = htod32(len);
	if ((error = dev_wlc_ioctl(dev, WLC_SCAN_RESULTS, list, len))) {
		WL_TRACE(("%s: %s : Scan_results too big %d\n", dev->name, __FUNCTION__, len));
		dwrq->length = len;
		if (g_scan_specified_ssid)
			kfree(list);
		return 0;
	}
	list->buflen = dtoh32(list->buflen);
	list->version = dtoh32(list->version);
	list->count = dtoh32(list->count);

	if (list->version != WL_BSS_INFO_VERSION) {
		WL_ERROR(("%s : list->version %d != WL_BSS_INFO_VERSION\n",
				__FUNCTION__, list->version));
		if (g_scan_specified_ssid)
			kfree(list);
		return -EINVAL;
	}

	if (g_scan_specified_ssid) {

		wl_iw_add_bss_to_ss_cache(list);
		kfree(list);
	}

#if defined(WL_IW_USE_ISCAN)
	p_buf = iscan->list_hdr;

	while (p_buf != iscan->list_cur) {
		list_merge = &((wl_iscan_results_t*)p_buf->iscan_buf)->results;
		WL_TRACE(("%s: Bcast APs list=%d\n", __FUNCTION__, list_merge->count));
		if (list_merge->count > 0)
			len_ret += (__u16) wl_iw_get_scan_prep(list_merge, info,
				extra+len_ret, buflen_from_user -len_ret);
		p_buf = p_buf->next;
	}
#else
	list_merge = (wl_scan_results_t *) g_scan;
	len_ret = (__u16) wl_iw_get_scan_prep(list_merge, info, extra, buflen_from_user);
#endif
	if (g_ss_cache_ctrl.m_link_down) {

		wl_iw_delete_bss_from_ss_cache(g_ss_cache_ctrl.m_active_bssid);
	}

	wl_iw_merge_scan_cache(info, extra+len_ret, buflen_from_user-len_ret, &merged_len);
	len_ret += merged_len;
	wl_iw_run_ss_cache_timer(0);
	wl_iw_run_ss_cache_timer(1);



	if ((len_ret + WE_ADD_EVENT_FIX) < buflen_from_user)
		len = len_ret;

	dwrq->length = len;
	dwrq->flags = 0;

	WL_TRACE(("%s return to WE %d bytes APs=%d\n", __FUNCTION__, dwrq->length, list->count));
	return 0;
}

#if defined(WL_IW_USE_ISCAN)
static int
wl_iw_iscan_get_scan(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_scan_results_t *list;
	struct iw_event	iwe;
	wl_bss_info_t *bi = NULL;
	int ii, j;
	int apcnt;
	char *event = extra, *end = extra + dwrq->length, *value;
	iscan_info_t *iscan = g_iscan;
	iscan_buf_t * p_buf;
	uint32  counter = 0;
	__u16 merged_len = 0;
	uint buflen_from_user = dwrq->length;


	WL_TRACE(("%s %s buflen_from_user %d:\n", dev->name, __FUNCTION__, dwrq->length));

	if (!extra) {
		WL_TRACE(("%s: SIOCGIWSCAN GET bad parameter\n", dev->name));
		return -EINVAL;
	}


	if ((!iscan) || (iscan->sysioc_pid < 0)) {
		return wl_iw_get_scan(dev, info, dwrq, extra);
	}



	if (g_ss_cache_ctrl.m_timer_expired) {
		wl_iw_free_ss_cache();
		g_ss_cache_ctrl.m_timer_expired ^= 1;
	}
	if (g_scan_specified_ssid) {
		return wl_iw_get_scan(dev, info, dwrq, extra);
	}
	else {
		if (g_ss_cache_ctrl.m_link_down) {

			wl_iw_delete_bss_from_ss_cache(g_ss_cache_ctrl.m_active_bssid);
		}
		if (g_ss_cache_ctrl.m_prev_scan_mode || g_ss_cache_ctrl.m_cons_br_scan_cnt > 4) {
			g_ss_cache_ctrl.m_cons_br_scan_cnt = 0;

			wl_iw_reset_ss_cache();
		}
		g_ss_cache_ctrl.m_prev_scan_mode = g_scan_specified_ssid;
		g_ss_cache_ctrl.m_cons_br_scan_cnt++;
	}

	WL_TRACE(("%s: SIOCGIWSCAN GET broadcast results\n", dev->name));
	apcnt = 0;
	p_buf = iscan->list_hdr;

	while (p_buf != iscan->list_cur) {
	    list = &((wl_iscan_results_t*)p_buf->iscan_buf)->results;

	    counter += list->count;

	    if (list->version != WL_BSS_INFO_VERSION) {
		WL_ERROR(("list->version %d != WL_BSS_INFO_VERSION\n", list->version));
		return -EINVAL; /* if WL_BSS_INFO_VERSION is corrupted iscan results are garbage */
	    }

	    bi = NULL;
	    for (ii = 0; ii < list->count && apcnt < IW_MAX_AP; apcnt++, ii++) {
		bi = bi ? (wl_bss_info_t *)((uintptr)bi + dtoh32(bi->length)) : list->bss_info;
		ASSERT(((uintptr)bi + dtoh32(bi->length)) <= ((uintptr)list +
			WLC_IW_ISCAN_MAXLEN));


		if (event + ETHER_ADDR_LEN + bi->SSID_len + IW_EV_UINT_LEN + IW_EV_FREQ_LEN +
			IW_EV_QUAL_LEN >= end)
			return -E2BIG;

		iwe.cmd = SIOCGIWAP;
		iwe.u.ap_addr.sa_family = ARPHRD_ETHER;
		memcpy(iwe.u.ap_addr.sa_data, &bi->BSSID, ETHER_ADDR_LEN);
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_ADDR_LEN);


		iwe.u.data.length = dtoh32(bi->SSID_len);
		iwe.cmd = SIOCGIWESSID;
		iwe.u.data.flags = 1;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, bi->SSID);


		if (dtoh16(bi->capability) & (DOT11_CAP_ESS | DOT11_CAP_IBSS)) {
			iwe.cmd = SIOCGIWMODE;
			if (dtoh16(bi->capability) & DOT11_CAP_ESS)
				iwe.u.mode = IW_MODE_INFRA;
			else
				iwe.u.mode = IW_MODE_ADHOC;
			event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_UINT_LEN);
		}


		iwe.cmd = SIOCGIWFREQ;
#ifdef  CUSTOMER_HW2
		if (dtoh32(bi->version) != 107 && bi->n_cap)
		{
		   iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->ctl_ch),
		   CHSPEC_CHANNEL(bi->ctl_ch) <= CH_MAX_2G_CHANNEL ?
		   WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		} else
#endif
		iwe.u.freq.m = wf_channel2mhz(CHSPEC_CHANNEL(bi->chanspec),
			CHSPEC_CHANNEL(bi->chanspec) <= CH_MAX_2G_CHANNEL ?
			WF_CHAN_FACTOR_2_4_G : WF_CHAN_FACTOR_5_G);
		iwe.u.freq.e = 6;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_FREQ_LEN);


		iwe.cmd = IWEVQUAL;
		iwe.u.qual.qual = rssi_to_qual(dtoh16(bi->RSSI));
		iwe.u.qual.level = 0x100 + dtoh16(bi->RSSI);
		iwe.u.qual.noise = 0x100 + bi->phy_noise;
		event = IWE_STREAM_ADD_EVENT(info, event, end, &iwe, IW_EV_QUAL_LEN);


		wl_iw_handle_scanresults_ies(&event, end, info, bi);


		iwe.cmd = SIOCGIWENCODE;
		if (dtoh16(bi->capability) & DOT11_CAP_PRIVACY)
			iwe.u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
		else
			iwe.u.data.flags = IW_ENCODE_DISABLED;
		iwe.u.data.length = 0;
		event = IWE_STREAM_ADD_POINT(info, event, end, &iwe, (char *)event);


		if (bi->rateset.count) {
			if (event + IW_MAX_BITRATES*IW_EV_PARAM_LEN >= end)
				return -E2BIG;

			value = event + IW_EV_LCP_LEN;
			iwe.cmd = SIOCGIWRATE;

			iwe.u.bitrate.fixed = iwe.u.bitrate.disabled = 0;
			for (j = 0; j < bi->rateset.count && j < IW_MAX_BITRATES; j++) {
				iwe.u.bitrate.value = (bi->rateset.rates[j] & 0x7f) * 500000;
				value = IWE_STREAM_ADD_VALUE(info, event, value, end, &iwe,
					IW_EV_PARAM_LEN);
			}
			event = value;
		}
	    }
	    p_buf = p_buf->next;
	}

	dwrq->length = event - extra;
	dwrq->flags = 0;


	wl_iw_merge_scan_cache(info, event, buflen_from_user - dwrq->length, &merged_len);
	dwrq->length += merged_len;
	wl_iw_run_ss_cache_timer(0);
	wl_iw_run_ss_cache_timer(1);

	WL_TRACE(("%s return to WE %d bytes APs=%d\n", __FUNCTION__, dwrq->length, counter));

	return 0;
}
#endif

static int
wl_iw_set_essid(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	int error;

	WL_TRACE(("%s: SIOCSIWESSID\n", dev->name));


	memset(&g_ssid, 0, sizeof(g_ssid));

	if (dwrq->length && extra) {
#if WIRELESS_EXT > 20
		g_ssid.SSID_len = MIN(sizeof(g_ssid.SSID), dwrq->length);
#else
		g_ssid.SSID_len = MIN(sizeof(g_ssid.SSID), dwrq->length-1);
#endif
		memcpy(g_ssid.SSID, extra, g_ssid.SSID_len);
	} else {

		g_ssid.SSID_len = 0;
	}
	g_ssid.SSID_len = htod32(g_ssid.SSID_len);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_SSID, &g_ssid, sizeof(g_ssid))))
		return error;

	return 0;
}

static int
wl_iw_get_essid(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wlc_ssid_t ssid;
	int error;

	WL_TRACE(("%s: SIOCGIWESSID\n", dev->name));

	if (!extra)
		return -EINVAL;

	if ((error = dev_wlc_ioctl(dev, WLC_GET_SSID, &ssid, sizeof(ssid)))) {
		WL_ERROR(("Error getting the SSID\n"));
		return error;
	}

	ssid.SSID_len = dtoh32(ssid.SSID_len);


	memcpy(extra, ssid.SSID, ssid.SSID_len);

	dwrq->length = ssid.SSID_len;

	dwrq->flags = 1;

	return 0;
}

static int
wl_iw_set_nick(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCSIWNICKN\n", dev->name));

	if (!extra)
		return -EINVAL;


	if (dwrq->length > sizeof(iw->nickname))
		return -E2BIG;

	memcpy(iw->nickname, extra, dwrq->length);
	iw->nickname[dwrq->length - 1] = '\0';

	return 0;
}

static int
wl_iw_get_nick(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCGIWNICKN\n", dev->name));

	if (!extra)
		return -EINVAL;

	strcpy(extra, iw->nickname);
	dwrq->length = strlen(extra) + 1;

	return 0;
}

static int wl_iw_set_rate(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	wl_rateset_t rateset;
	int error, rate, i, error_bg, error_a;

	WL_TRACE(("%s: SIOCSIWRATE\n", dev->name));


	if ((error = dev_wlc_ioctl(dev, WLC_GET_CURR_RATESET, &rateset, sizeof(rateset))))
		return error;

	rateset.count = dtoh32(rateset.count);

	if (vwrq->value < 0) {

		rate = rateset.rates[rateset.count - 1] & 0x7f;
	} else if (vwrq->value < rateset.count) {

		rate = rateset.rates[vwrq->value] & 0x7f;
	} else {

		rate = vwrq->value / 500000;
	}

	if (vwrq->fixed) {

		error_bg = dev_wlc_intvar_set(dev, "bg_rate", rate);
		error_a = dev_wlc_intvar_set(dev, "a_rate", rate);

		if (error_bg && error_a)
			return (error_bg | error_a);
	} else {


		error_bg = dev_wlc_intvar_set(dev, "bg_rate", 0);

		error_a = dev_wlc_intvar_set(dev, "a_rate", 0);

		if (error_bg && error_a)
			return (error_bg | error_a);


		for (i = 0; i < rateset.count; i++)
			if ((rateset.rates[i] & 0x7f) > rate)
				break;
		rateset.count = htod32(i);


		if ((error = dev_wlc_ioctl(dev, WLC_SET_RATESET, &rateset, sizeof(rateset))))
			return error;
	}

	return 0;
}

static int wl_iw_get_rate(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, rate;

	WL_TRACE(("%s: SIOCGIWRATE\n", dev->name));


	if ((error = dev_wlc_ioctl(dev, WLC_GET_RATE, &rate, sizeof(rate))))
		return error;
	rate = dtoh32(rate);
	vwrq->value = rate * 500000;

	return 0;
}

static int
wl_iw_set_rts(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, rts;

	WL_TRACE(("%s: SIOCSIWRTS\n", dev->name));

	if (vwrq->disabled)
		rts = DOT11_DEFAULT_RTS_LEN;
	else if (vwrq->value < 0 || vwrq->value > DOT11_DEFAULT_RTS_LEN)
		return -EINVAL;
	else
		rts = vwrq->value;

	if ((error = dev_wlc_intvar_set(dev, "rtsthresh", rts)))
		return error;

	return 0;
}

static int
wl_iw_get_rts(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, rts;

	WL_TRACE(("%s: SIOCGIWRTS\n", dev->name));

	if ((error = dev_wlc_intvar_get(dev, "rtsthresh", &rts)))
		return error;

	vwrq->value = rts;
	vwrq->disabled = (rts >= DOT11_DEFAULT_RTS_LEN);
	vwrq->fixed = 1;

	return 0;
}

static int
wl_iw_set_frag(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, frag;

	WL_TRACE(("%s: SIOCSIWFRAG\n", dev->name));

	if (vwrq->disabled)
		frag = DOT11_DEFAULT_FRAG_LEN;
	else if (vwrq->value < 0 || vwrq->value > DOT11_DEFAULT_FRAG_LEN)
		return -EINVAL;
	else
		frag = vwrq->value;

	if ((error = dev_wlc_intvar_set(dev, "fragthresh", frag)))
		return error;

	return 0;
}

static int
wl_iw_get_frag(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, fragthreshold;

	WL_TRACE(("%s: SIOCGIWFRAG\n", dev->name));

	if ((error = dev_wlc_intvar_get(dev, "fragthresh", &fragthreshold)))
		return error;

	vwrq->value = fragthreshold;
	vwrq->disabled = (fragthreshold >= DOT11_DEFAULT_FRAG_LEN);
	vwrq->fixed = 1;

	return 0;
}

static int
wl_iw_set_txpow(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, disable;
	uint16 txpwrmw;
	WL_TRACE(("%s: SIOCSIWTXPOW\n", dev->name));


	disable = vwrq->disabled ? WL_RADIO_SW_DISABLE : 0;
	disable += WL_RADIO_SW_DISABLE << 16;

	disable = htod32(disable);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_RADIO, &disable, sizeof(disable))))
		return error;


	if (disable & WL_RADIO_SW_DISABLE)
		return 0;


	if (!(vwrq->flags & IW_TXPOW_MWATT))
		return -EINVAL;


	if (vwrq->value < 0)
		return 0;

	if (vwrq->value > 0xffff) txpwrmw = 0xffff;
	else txpwrmw = (uint16)vwrq->value;


	error = dev_wlc_intvar_set(dev, "qtxpower", (int)(bcm_mw_to_qdbm(txpwrmw)));
	return error;
}

static int
wl_iw_get_txpow(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, disable, txpwrdbm;
	uint8 result;

	WL_TRACE(("%s: SIOCGIWTXPOW\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_RADIO, &disable, sizeof(disable))) ||
	    (error = dev_wlc_intvar_get(dev, "qtxpower", &txpwrdbm)))
		return error;

	disable = dtoh32(disable);
	result = (uint8)(txpwrdbm & ~WL_TXPWR_OVERRIDE);
	vwrq->value = (int32)bcm_qdbm_to_mw(result);
	vwrq->fixed = 0;
	vwrq->disabled = (disable & (WL_RADIO_SW_DISABLE | WL_RADIO_HW_DISABLE)) ? 1 : 0;
	vwrq->flags = IW_TXPOW_MWATT;

	return 0;
}

#if WIRELESS_EXT > 10
static int
wl_iw_set_retry(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, lrl, srl;

	WL_TRACE(("%s: SIOCSIWRETRY\n", dev->name));


	if (vwrq->disabled || (vwrq->flags & IW_RETRY_LIFETIME))
		return -EINVAL;


	if (vwrq->flags & IW_RETRY_LIMIT) {


#if WIRELESS_EXT > 20
	if ((vwrq->flags & IW_RETRY_LONG) ||(vwrq->flags & IW_RETRY_MAX) ||
		!((vwrq->flags & IW_RETRY_SHORT) || (vwrq->flags & IW_RETRY_MIN))) {
#else
	if ((vwrq->flags & IW_RETRY_MAX) || !(vwrq->flags & IW_RETRY_MIN)) {
#endif
			lrl = htod32(vwrq->value);
			if ((error = dev_wlc_ioctl(dev, WLC_SET_LRL, &lrl, sizeof(lrl))))
				return error;
		}


#if WIRELESS_EXT > 20
	if ((vwrq->flags & IW_RETRY_SHORT) ||(vwrq->flags & IW_RETRY_MIN) ||
		!((vwrq->flags & IW_RETRY_LONG) || (vwrq->flags & IW_RETRY_MAX))) {
#else
		if ((vwrq->flags & IW_RETRY_MIN) || !(vwrq->flags & IW_RETRY_MAX)) {
#endif
			srl = htod32(vwrq->value);
			if ((error = dev_wlc_ioctl(dev, WLC_SET_SRL, &srl, sizeof(srl))))
				return error;
		}
	}
	return 0;
}

static int
wl_iw_get_retry(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, lrl, srl;

	WL_TRACE(("%s: SIOCGIWRETRY\n", dev->name));

	vwrq->disabled = 0;


	if ((vwrq->flags & IW_RETRY_TYPE) == IW_RETRY_LIFETIME)
		return -EINVAL;


	if ((error = dev_wlc_ioctl(dev, WLC_GET_LRL, &lrl, sizeof(lrl))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_SRL, &srl, sizeof(srl))))
		return error;

	lrl = dtoh32(lrl);
	srl = dtoh32(srl);


	if (vwrq->flags & IW_RETRY_MAX) {
		vwrq->flags = IW_RETRY_LIMIT | IW_RETRY_MAX;
		vwrq->value = lrl;
	} else {
		vwrq->flags = IW_RETRY_LIMIT;
		vwrq->value = srl;
		if (srl != lrl)
			vwrq->flags |= IW_RETRY_MIN;
	}

	return 0;
}
#endif

static int
wl_iw_set_encode(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
	int error, val, wsec;

	WL_TRACE(("%s: SIOCSIWENCODE\n", dev->name));

	memset(&key, 0, sizeof(key));

	if ((dwrq->flags & IW_ENCODE_INDEX) == 0) {

		for (key.index = 0; key.index < DOT11_MAX_DEFAULT_KEYS; key.index++) {
			val = htod32(key.index);
			if ((error = dev_wlc_ioctl(dev, WLC_GET_KEY_PRIMARY, &val, sizeof(val))))
				return error;
			val = dtoh32(val);
			if (val)
				break;
		}

		if (key.index == DOT11_MAX_DEFAULT_KEYS)
			key.index = 0;
	} else {
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;
		if (key.index >= DOT11_MAX_DEFAULT_KEYS)
			return -EINVAL;
	}


	if (!extra || !dwrq->length || (dwrq->flags & IW_ENCODE_NOKEY)) {

		val = htod32(key.index);
		if ((error = dev_wlc_ioctl(dev, WLC_SET_KEY_PRIMARY, &val, sizeof(val))))
			return error;
	} else {
		key.len = dwrq->length;

		if (dwrq->length > sizeof(key.data))
			return -EINVAL;

		memcpy(key.data, extra, dwrq->length);

		key.flags = WL_PRIMARY_KEY;
		switch (key.len) {
		case WEP1_KEY_SIZE:
			key.algo = CRYPTO_ALGO_WEP1;
			break;
		case WEP128_KEY_SIZE:
			key.algo = CRYPTO_ALGO_WEP128;
			break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
		case TKIP_KEY_SIZE:
			key.algo = CRYPTO_ALGO_TKIP;
			break;
#endif
		case AES_KEY_SIZE:
			key.algo = CRYPTO_ALGO_AES_CCM;
			break;
		default:
			return -EINVAL;
		}


		swap_key_from_BE(&key);
		if ((error = dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key))))
			return error;
	}


	val = (dwrq->flags & IW_ENCODE_DISABLED) ? 0 : WEP_ENABLED;

	if ((error = dev_wlc_intvar_get(dev, "wsec", &wsec)))
		return error;

	wsec  &= ~(WEP_ENABLED);
	wsec |= val;

	if ((error = dev_wlc_intvar_set(dev, "wsec", wsec)))
		return error;


	val = (dwrq->flags & IW_ENCODE_RESTRICTED) ? 1 : 0;
	val = htod32(val);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_AUTH, &val, sizeof(val))))
		return error;

	return 0;
}

static int
wl_iw_get_encode(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
	int error, val, wsec, auth;

	WL_TRACE(("%s: SIOCGIWENCODE\n", dev->name));


	bzero(&key, sizeof(wl_wsec_key_t));

	if ((dwrq->flags & IW_ENCODE_INDEX) == 0) {

		for (key.index = 0; key.index < DOT11_MAX_DEFAULT_KEYS; key.index++) {
			val = key.index;
			if ((error = dev_wlc_ioctl(dev, WLC_GET_KEY_PRIMARY, &val, sizeof(val))))
				return error;
			val = dtoh32(val);
			if (val)
				break;
		}
	} else
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;

	if (key.index >= DOT11_MAX_DEFAULT_KEYS)
		key.index = 0;



	if ((error = dev_wlc_ioctl(dev, WLC_GET_WSEC, &wsec, sizeof(wsec))) ||
	    (error = dev_wlc_ioctl(dev, WLC_GET_AUTH, &auth, sizeof(auth))))
		return error;

	swap_key_to_BE(&key);

	wsec = dtoh32(wsec);
	auth = dtoh32(auth);

	dwrq->length = MIN(DOT11_MAX_KEY_SIZE, key.len);


	dwrq->flags = key.index + 1;
	if (!(wsec & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED))) {

		dwrq->flags |= IW_ENCODE_DISABLED;
	}
	if (auth) {

		dwrq->flags |= IW_ENCODE_RESTRICTED;
	}


	if (dwrq->length && extra)
		memcpy(extra, key.data, dwrq->length);

	return 0;
}

static int
wl_iw_set_power(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, pm;

	WL_TRACE(("%s: SIOCSIWPOWER\n", dev->name));

	pm = vwrq->disabled ? PM_OFF : PM_MAX;

	pm = htod32(pm);
	if ((error = dev_wlc_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm))))
		return error;

	return 0;
}

static int
wl_iw_get_power(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error, pm;

	WL_TRACE(("%s: SIOCGIWPOWER\n", dev->name));

	if ((error = dev_wlc_ioctl(dev, WLC_GET_PM, &pm, sizeof(pm))))
		return error;

	pm = dtoh32(pm);
	vwrq->disabled = pm ? 0 : 1;
	vwrq->flags = IW_POWER_ALL_R;

	return 0;
}

#if WIRELESS_EXT > 17
static int
wl_iw_set_wpaie(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *iwp,
	char *extra
)
{
		dev_wlc_bufvar_set(dev, "wpaie", extra, iwp->length);

	return 0;
}

static int
wl_iw_get_wpaie(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *iwp,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWGENIE\n", dev->name));
	iwp->length = 64;
	dev_wlc_bufvar_get(dev, "wpaie", extra, iwp->length);
	return 0;
}

static int
wl_iw_set_encodeext(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *extra
)
{
	wl_wsec_key_t key;
	int error;
	struct iw_encode_ext *iwe;

	WL_TRACE(("%s: SIOCSIWENCODEEXT\n", dev->name));

	memset(&key, 0, sizeof(key));
	iwe = (struct iw_encode_ext *)extra;


	if (dwrq->flags & IW_ENCODE_DISABLED) {

	}


	key.index = 0;
	if (dwrq->flags & IW_ENCODE_INDEX)
		key.index = (dwrq->flags & IW_ENCODE_INDEX) - 1;

	key.len = iwe->key_len;


	if (!ETHER_ISMULTI(iwe->addr.sa_data))
		bcopy((void *)&iwe->addr.sa_data, (char *)&key.ea, ETHER_ADDR_LEN);


	if (key.len == 0) {
		if (iwe->ext_flags & IW_ENCODE_EXT_SET_TX_KEY) {
			WL_WSEC(("Changing the the primary Key to %d\n", key.index));

			key.index = htod32(key.index);
			error = dev_wlc_ioctl(dev, WLC_SET_KEY_PRIMARY,
				&key.index, sizeof(key.index));
			if (error)
				return error;
		}

		else {
			swap_key_from_BE(&key);
			dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));
		}
	}
	else {
		if (iwe->key_len > sizeof(key.data))
			return -EINVAL;

		WL_WSEC(("Setting the key index %d\n", key.index));
		if (iwe->ext_flags & IW_ENCODE_EXT_SET_TX_KEY) {
			WL_WSEC(("key is a Primary Key\n"));
			key.flags = WL_PRIMARY_KEY;
		}

		bcopy((void *)iwe->key, key.data, iwe->key_len);

		if (iwe->alg == IW_ENCODE_ALG_TKIP) {
			uint8 keybuf[8];
			bcopy(&key.data[24], keybuf, sizeof(keybuf));
			bcopy(&key.data[16], &key.data[24], sizeof(keybuf));
			bcopy(keybuf, &key.data[16], sizeof(keybuf));
		}


		if (iwe->ext_flags & IW_ENCODE_EXT_RX_SEQ_VALID) {
			uchar *ivptr;
			ivptr = (uchar *)iwe->rx_seq;
			key.rxiv.hi = (ivptr[5] << 24) | (ivptr[4] << 16) |
				(ivptr[3] << 8) | ivptr[2];
			key.rxiv.lo = (ivptr[1] << 8) | ivptr[0];
			key.iv_initialized = TRUE;
		}

		switch (iwe->alg) {
			case IW_ENCODE_ALG_NONE:
				key.algo = CRYPTO_ALGO_OFF;
				break;
			case IW_ENCODE_ALG_WEP:
				if (iwe->key_len == WEP1_KEY_SIZE)
					key.algo = CRYPTO_ALGO_WEP1;
				else
					key.algo = CRYPTO_ALGO_WEP128;
				break;
			case IW_ENCODE_ALG_TKIP:
				key.algo = CRYPTO_ALGO_TKIP;
				break;
			case IW_ENCODE_ALG_CCMP:
				key.algo = CRYPTO_ALGO_AES_CCM;
				break;
			default:
				break;
		}
		swap_key_from_BE(&key);

		dhd_wait_pend8021x(dev);

		error = dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));
		if (error)
			return error;
	}
	return 0;
}

#if WIRELESS_EXT > 17
#ifdef BCMWPA2
struct {
	pmkid_list_t pmkids;
	pmkid_t foo[MAXPMKID-1];
} pmkid_list;

static int
wl_iw_set_pmksa(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	struct iw_pmksa *iwpmksa;
	uint i;
	int ret = 0;
	char eabuf[ETHER_ADDR_STR_LEN];

	WL_TRACE(("%s: SIOCSIWPMKSA\n", dev->name));
	iwpmksa = (struct iw_pmksa *)extra;
	bzero((char *)eabuf, ETHER_ADDR_STR_LEN);

	if (iwpmksa->cmd == IW_PMKSA_FLUSH) {
		WL_TRACE(("wl_iw_set_pmksa - IW_PMKSA_FLUSH\n"));
		bzero((char *)&pmkid_list, sizeof(pmkid_list));
	}

	else if (iwpmksa->cmd == IW_PMKSA_REMOVE) {
		{
			pmkid_list_t pmkid, *pmkidptr;
			uint j;
			pmkidptr = &pmkid;

			bcopy(&iwpmksa->bssid.sa_data[0], &pmkidptr->pmkid[0].BSSID, ETHER_ADDR_LEN);
			bcopy(&iwpmksa->pmkid[0], &pmkidptr->pmkid[0].PMKID, WPA2_PMKID_LEN);

			WL_TRACE(("wl_iw_set_pmksa,IW_PMKSA_REMOVE - PMKID: %s = ",
				bcm_ether_ntoa(&pmkidptr->pmkid[0].BSSID,
				eabuf)));
			for (j = 0; j < WPA2_PMKID_LEN; j++)
				WL_TRACE(("%02x ", pmkidptr->pmkid[0].PMKID[j]));
			WL_TRACE(("\n"));
		}

		for (i = 0; i < pmkid_list.pmkids.npmkid; i++)
			if (!bcmp(&iwpmksa->bssid.sa_data[0], &pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN))
				break;

		if ((pmkid_list.pmkids.npmkid > 0) && (i < pmkid_list.pmkids.npmkid)) {
			bzero(&pmkid_list.pmkids.pmkid[i], sizeof(pmkid_t));
			for (; i < (pmkid_list.pmkids.npmkid - 1); i++) {
				bcopy(&pmkid_list.pmkids.pmkid[i+1].BSSID,
					&pmkid_list.pmkids.pmkid[i].BSSID,
					ETHER_ADDR_LEN);
				bcopy(&pmkid_list.pmkids.pmkid[i+1].PMKID,
					&pmkid_list.pmkids.pmkid[i].PMKID,
					WPA2_PMKID_LEN);
			}
			pmkid_list.pmkids.npmkid--;
		}
		else
			ret = -EINVAL;
	}

	else if (iwpmksa->cmd == IW_PMKSA_ADD) {
		for (i = 0; i < pmkid_list.pmkids.npmkid; i++)
			if (!bcmp(&iwpmksa->bssid.sa_data[0], &pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN))
				break;
		if (i < MAXPMKID) {
			bcopy(&iwpmksa->bssid.sa_data[0],
				&pmkid_list.pmkids.pmkid[i].BSSID,
				ETHER_ADDR_LEN);
			bcopy(&iwpmksa->pmkid[0], &pmkid_list.pmkids.pmkid[i].PMKID,
				WPA2_PMKID_LEN);
			if (i == pmkid_list.pmkids.npmkid)
				pmkid_list.pmkids.npmkid++;
		}
		else
			ret = -EINVAL;

		{
			uint j;
			uint k;
			k = pmkid_list.pmkids.npmkid;
			WL_TRACE(("wl_iw_set_pmksa,IW_PMKSA_ADD - PMKID: %s = ",
				bcm_ether_ntoa(&pmkid_list.pmkids.pmkid[k].BSSID,
				eabuf)));
			for (j = 0; j < WPA2_PMKID_LEN; j++)
				WL_TRACE(("%02x ", pmkid_list.pmkids.pmkid[k].PMKID[j]));
			WL_TRACE(("\n"));
		}
	}
	WL_TRACE(("PRINTING pmkid LIST - No of elements %d\n", pmkid_list.pmkids.npmkid));
	for (i = 0; i < pmkid_list.pmkids.npmkid; i++) {
		uint j;
		WL_TRACE(("PMKID[%d]: %s = ", i,
			bcm_ether_ntoa(&pmkid_list.pmkids.pmkid[i].BSSID,
			eabuf)));
		for (j = 0; j < WPA2_PMKID_LEN; j++)
			WL_TRACE(("%02x ", pmkid_list.pmkids.pmkid[i].PMKID[j]));
		printf("\n");
	}
	WL_TRACE(("\n"));

	if (!ret)
		ret = dev_wlc_bufvar_set(dev, "pmkid_info", (char *)&pmkid_list, sizeof(pmkid_list));
	return ret;
}
#endif
#endif

static int
wl_iw_get_encodeext(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	WL_TRACE(("%s: SIOCGIWENCODEEXT\n", dev->name));
	return 0;
}

static int
wl_iw_set_wpaauth(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error = 0;
	int paramid;
	int paramval;
	int val = 0;
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCSIWAUTH\n", dev->name));

	paramid = vwrq->flags & IW_AUTH_INDEX;
	paramval = vwrq->value;

	WL_TRACE(("%s: SIOCSIWAUTH, paramid = 0x%0x, paramval = 0x%0x\n",
		dev->name, paramid, paramval));

	switch (paramid) {
	case IW_AUTH_WPA_VERSION:

		if (paramval & IW_AUTH_WPA_VERSION_DISABLED)
			val = WPA_AUTH_DISABLED;
		else if (paramval & (IW_AUTH_WPA_VERSION_WPA))
			val = WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED;
#ifdef BCMWPA2
		else if (paramval & IW_AUTH_WPA_VERSION_WPA2)
			val = WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED;
#endif
		WL_INFORM(("%s: %d: setting wpa_auth to 0x%0x\n", __FUNCTION__, __LINE__, val));
		if ((error = dev_wlc_intvar_set(dev, "wpa_auth", val)))
			return error;
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:


		if (paramval & (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104))
			val = 	WEP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_TKIP)
			val = TKIP_ENABLED;
		if (paramval & IW_AUTH_CIPHER_CCMP)
			val = AES_ENABLED;

		if (paramid == IW_AUTH_CIPHER_PAIRWISE) {
			iw->pwsec = val;
			val |= iw->gwsec;
		}
		else {
			iw->gwsec = val;
			val |= iw->pwsec;
		}

		if ((error = dev_wlc_intvar_set(dev, "wsec", val)))
			return error;

		break;

	case IW_AUTH_KEY_MGMT:
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;

		if (val & (WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED)) {
			if (paramval & IW_AUTH_KEY_MGMT_PSK)
				val = WPA_AUTH_PSK;
			else
				val = WPA_AUTH_UNSPECIFIED;
		}
#ifdef BCMWPA2
		else if (val & (WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED)) {
			if (paramval & IW_AUTH_KEY_MGMT_PSK)
				val = WPA2_AUTH_PSK;
			else
				val = WPA2_AUTH_UNSPECIFIED;
		}
#endif
		WL_INFORM(("%s: %d: setting wpa_auth to %d\n", __FUNCTION__, __LINE__, val));
		if ((error = dev_wlc_intvar_set(dev, "wpa_auth", val)))
			return error;
		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		dev_wlc_bufvar_set(dev, "tkip_countermeasures", (char *)&paramval, 1);
		break;

	case IW_AUTH_80211_AUTH_ALG:

		WL_INFORM(("Setting the D11auth %d\n", paramval));
		if (paramval == IW_AUTH_ALG_OPEN_SYSTEM)
			val = 0;
		else if (paramval == IW_AUTH_ALG_SHARED_KEY)
			val = 1;
		else if (paramval == (IW_AUTH_ALG_OPEN_SYSTEM | IW_AUTH_ALG_SHARED_KEY))
			val = 2;
		else
			error = 1;
		if (!error && (error = dev_wlc_intvar_set(dev, "auth", val)))
			return error;
		break;

	case IW_AUTH_WPA_ENABLED:
		if (paramval == 0) {
			iw->pwsec = 0;
			iw->gwsec = 0;
			if ((error = dev_wlc_intvar_get(dev, "wsec", &val)))
				return error;
			if (val & (TKIP_ENABLED | AES_ENABLED)) {
				val &= ~(TKIP_ENABLED | AES_ENABLED);
				dev_wlc_intvar_set(dev, "wsec", val);
			}
			val = 0;
		WL_INFORM(("%s: %d: setting wpa_auth to %d\n", __FUNCTION__, __LINE__, val));
			dev_wlc_intvar_set(dev, "wpa_auth", 0);
			return error;
		}


		break;

	case IW_AUTH_DROP_UNENCRYPTED:
		dev_wlc_bufvar_set(dev, "wsec_restrict", (char *)&paramval, 1);
		break;

	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		dev_wlc_bufvar_set(dev, "rx_unencrypted_eapol", (char *)&paramval, 1);
		break;

#if WIRELESS_EXT > 17
	case IW_AUTH_ROAMING_CONTROL:
		WL_INFORM(("%s: IW_AUTH_ROAMING_CONTROL\n", __FUNCTION__));

		break;
	case IW_AUTH_PRIVACY_INVOKED:
		WL_INFORM(("%s: IW_AUTH_PRIVACY_INVOKED\n", __FUNCTION__));
		break;
#endif
	default:
		break;
	}
	return 0;
}
#ifdef BCMWPA2
#define VAL_PSK(_val) (((_val) & WPA_AUTH_PSK) || ((_val) & WPA2_AUTH_PSK))
#else
#define VAL_PSK(_val) (((_val) & WPA_AUTH_PSK))
#endif

static int
wl_iw_get_wpaauth(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_param *vwrq,
	char *extra
)
{
	int error;
	int paramid;
	int paramval = 0;
	int val;
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(dev);

	WL_TRACE(("%s: SIOCGIWAUTH\n", dev->name));

	paramid = vwrq->flags & IW_AUTH_INDEX;

	switch (paramid) {
	case IW_AUTH_WPA_VERSION:

		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (val & (WPA_AUTH_NONE | WPA_AUTH_DISABLED))
			paramval = IW_AUTH_WPA_VERSION_DISABLED;
		else if (val & (WPA_AUTH_PSK | WPA_AUTH_UNSPECIFIED))
			paramval = IW_AUTH_WPA_VERSION_WPA;
#ifdef BCMWPA2
		else if (val & (WPA2_AUTH_PSK | WPA2_AUTH_UNSPECIFIED))
			paramval = IW_AUTH_WPA_VERSION_WPA2;
#endif
		break;
	case IW_AUTH_CIPHER_PAIRWISE:
	case IW_AUTH_CIPHER_GROUP:
		if (paramid == IW_AUTH_CIPHER_PAIRWISE)
			val = iw->pwsec;
		else
			val = iw->gwsec;

		paramval = 0;
		if (val) {
			if (val & WEP_ENABLED)
				paramval |= (IW_AUTH_CIPHER_WEP40 | IW_AUTH_CIPHER_WEP104);
			if (val & TKIP_ENABLED)
				paramval |= (IW_AUTH_CIPHER_TKIP);
			if (val & AES_ENABLED)
				paramval |= (IW_AUTH_CIPHER_CCMP);
		}
		else
			paramval = IW_AUTH_CIPHER_NONE;
		break;
	case IW_AUTH_KEY_MGMT:

		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (VAL_PSK(val))
			paramval = IW_AUTH_KEY_MGMT_PSK;
		else
			paramval = IW_AUTH_KEY_MGMT_802_1X;

		break;
	case IW_AUTH_TKIP_COUNTERMEASURES:
		dev_wlc_bufvar_get(dev, "tkip_countermeasures", (char *)&paramval, 1);
		break;

	case IW_AUTH_DROP_UNENCRYPTED:
		dev_wlc_bufvar_get(dev, "wsec_restrict", (char *)&paramval, 1);
		break;

	case IW_AUTH_RX_UNENCRYPTED_EAPOL:
		dev_wlc_bufvar_get(dev, "rx_unencrypted_eapol", (char *)&paramval, 1);
		break;

	case IW_AUTH_80211_AUTH_ALG:

		if ((error = dev_wlc_intvar_get(dev, "auth", &val)))
			return error;
		if (!val)
			paramval = IW_AUTH_ALG_OPEN_SYSTEM;
		else
			paramval = IW_AUTH_ALG_SHARED_KEY;
		break;
	case IW_AUTH_WPA_ENABLED:
		if ((error = dev_wlc_intvar_get(dev, "wpa_auth", &val)))
			return error;
		if (val)
			paramval = TRUE;
		else
			paramval = FALSE;
		break;
#if WIRELESS_EXT > 17
	case IW_AUTH_ROAMING_CONTROL:
		WL_ERROR(("%s: IW_AUTH_ROAMING_CONTROL\n", __FUNCTION__));

		break;
	case IW_AUTH_PRIVACY_INVOKED:
		WL_ERROR(("%s: IW_AUTH_PRIVACY_INVOKED\n", __FUNCTION__));
		break;
#endif
	}
	vwrq->value = paramval;
	return 0;
}
#endif

#ifdef CONFIG_BCM4329_SOFTAP
//penguin, add for softap
#define SSID_LEN	33
#define SEC_LEN		16
#define KEY_LEN		65
#define PROFILE_OFFSET	32
struct ap_profile {
	uint8	ssid[SSID_LEN];
	uint8	sec[SEC_LEN];
	uint8	key[KEY_LEN];
	uint32	channel; /* 0 for auto */
	uint32	preamble;
	uint32	max_scb;	/*maxmium number of station */
};

#define MACLIST_MODE_DISABLED	0
#define MACLIST_MODE_ENABLED	1
#define MACLIST_MODE_ALLOW		2
struct mflist {
	uint count;
	struct ether_addr ea[16];
};
struct mac_list_set {
	uint32	mode;
	struct mflist white_list;
	struct mflist black_list;
};

static int ap_macmode = MACLIST_MODE_DISABLED;
static int ap_txpower_default = 0;
static struct mflist ap_black_list;
static int
wl_iw_parse_wep(char *keystr, wl_wsec_key_t *key)
{
	char hex[] = "XX";
	unsigned char *data = key->data;

	switch (strlen(keystr)) {
	case 5:
	case 13:
	case 16:
		key->len = strlen(keystr);
		memcpy(data, keystr, key->len + 1);
		break;
	case 12:
	case 28:
	case 34:
	case 66:
		/* strip leading 0x */
		if (!strnicmp(keystr, "0x", 2))
			keystr += 2;
		else
			return -1;
		/* fall through */
	case 10:
	case 26:
	case 32:
	case 64:
		key->len = strlen(keystr) / 2;
		while (*keystr) {
			strncpy(hex, keystr, 2);
			*data++ = (char) bcm_strtoul(hex, NULL, 16);
			keystr += 2;
		}
		break;
	default:
		return -1;
	}

	switch (key->len) {
	case 5:
		key->algo = CRYPTO_ALGO_WEP1;
		break;
	case 13:
		key->algo = CRYPTO_ALGO_WEP128;
		break;
	case 16:
		/* default to AES-CCM */
		key->algo = CRYPTO_ALGO_AES_CCM;
		break;
	case 32:
		key->algo = CRYPTO_ALGO_TKIP;
		break;
	default:
		return -1;
	}

	/* Set as primary key by default */
	key->flags |= WL_PRIMARY_KEY;

	return 0;
}

/* anthony: move the passhash function up to here. */
#define SHA1HashSize 20
extern void pbkdf2_sha1(const char *passphrase, const char *ssid, size_t ssid_len,
		 int iterations, u8 *buf, size_t buflen);
static int
wl_iw_setap(struct net_device *dev, struct ap_profile *ap)
{
	int updown = 0;
	int channel = 0;
	int wsec = 0;
	int wpa_auth = 0;
	int apstate = 1;
	wlc_ssid_t ap_ssid;
	wlc_ssid_t null_ssid;
	int max_assoc = 8;
	//char deauth_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
 	int i;
	char * ptr;
	int mpc = 0;
	int wep = 0;
	wl_wsec_key_t key;
#ifdef  CUSTOMER_HW2 /* HTC need limited the selected channel */
	int start_channel, end_channel;
#endif

	memset(&null_ssid, 0, sizeof(wlc_ssid_t));
	WL_SOFTAP(("wl_iw: set ap profile:\n"));
	WL_SOFTAP(("	ssid = %s\n", ap->ssid));
	WL_SOFTAP(("	security = %s\n", ap->sec));
	if (ap->key[0] != '\0')
		WL_SOFTAP(("	key = %s\n", ap->key));
	WL_SOFTAP(("	channel = %d\n", ap->channel));
	WL_SOFTAP(("	max scb = %d\n", ap->max_scb));


		dev_wlc_ioctl(dev, WLC_DOWN, &updown, sizeof(updown));
		apstate = 0;
		dev_wlc_ioctl(dev, WLC_SET_AP, &apstate, sizeof(apstate));
		apstate = 1;
		dev_wlc_ioctl(dev, WLC_SET_AP, &apstate, sizeof(apstate));
		/* check auto channel later */
#ifdef  CUSTOMER_HW2 /* HTC need limited the selected channel */
		if ((ap->channel >> 8) || (ap->channel == 0))
#else
		if (ap->channel == 0)
#endif
#if 1
		{
			int chosen = 0;
			wl_uint32_list_t request;
			int rescan = 0;
			int retry = 0;
			int ret = 0;
			dev_wlc_ioctl(dev, WLC_UP, &updown, sizeof(updown));
			dev_wlc_ioctl(dev, WLC_SET_SSID, &null_ssid, sizeof(null_ssid));
auto_channel_retry:
			request.count = htod32(0);
			ret = dev_wlc_ioctl(dev, WLC_START_CHANNEL_SEL, &request, sizeof(request));
			if (ret < 0) {
				WL_SOFTAP(("can't start auto channel scan\n"));
				return -1;
			}
get_channel_retry:
			bcm_mdelay(500);
			ret = dev_wlc_ioctl(dev, WLC_GET_CHANNEL_SEL, &chosen, sizeof(chosen));
			if (ret < 0 || dtoh32(chosen) == 0) {
				if (retry++ < 3)
					goto get_channel_retry;
				else {
					WL_SOFTAP(("can't get auto channel sel, err = %d, chosen = %d\n", ret, chosen));
					return -1;
				}
			}
			if ((chosen == 1) && (!rescan++))
				goto auto_channel_retry;
#ifdef  CUSTOMER_HW2 /* HTC need limited the selected channel */
			if (ap->channel == 0)
				ap->channel = chosen;
			else {
				start_channel = (ap->channel >> 8) & 0xff;
				end_channel = ap->channel & 0xff;
				WL_SOFTAP(("channel range from %d to %d\n", start_channel, end_channel));
				if (chosen > end_channel) {
					if (chosen <= 6)
						chosen = end_channel;
					else
						chosen = start_channel;
				} else if (chosen < start_channel)
					chosen = start_channel;

				ap->channel = chosen;
			}
#else
			ap->channel = chosen;
#endif
			WL_SOFTAP(("Set auto channel = %d\n", chosen));
			dev_wlc_ioctl(dev, WLC_DOWN, &updown, sizeof(updown));
		}
#else
		ap->channel = 6;
#endif
		channel = ap->channel;
		dev_wlc_ioctl(dev, WLC_SET_CHANNEL, &channel, sizeof(channel));

	if (strnicmp(ap->sec, "open", strlen("open")) == 0) {
		wsec = 0;
		dev_wlc_intvar_set(dev, "wsec", wsec);
		wpa_auth = WPA_AUTH_DISABLED;
		dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);
	} else if (strnicmp(ap->sec, "wep", strlen("wep")) == 0) {
		memset(&key, 0, sizeof(key));

		wsec = WEP_ENABLED;
		dev_wlc_intvar_set(dev, "wsec", wsec);

		key.index = 0;
		if (wl_iw_parse_wep(ap->key, &key)) {
			WL_SOFTAP(("wep key parse err!\n"));
			return -1;
		}

		key.index = htod32(key.index);
		key.len = htod32(key.len);
		key.algo = htod32(key.algo);
		key.flags = htod32(key.flags);

		wep = 1;
		wpa_auth = WPA_AUTH_DISABLED;
		dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);
	} else if (strnicmp(ap->sec, "wpa2-psk", strlen("wpa2-psk")) == 0) {

		wsec_pmk_t psk;
		size_t key_len;

		wsec = AES_ENABLED;
		dev_wlc_intvar_set(dev, "wsec", wsec);

		key_len = strlen(ap->key);
		if (key_len < WSEC_MIN_PSK_LEN || key_len > WSEC_MAX_PSK_LEN) {
			WL_SOFTAP(("passphrase must be between %d and %d characters long\n",
		       WSEC_MIN_PSK_LEN, WSEC_MAX_PSK_LEN));
			return -1;
		}

		/* anthony: turn all psk-key to 64 char wide in default. */
		if ( key_len < WSEC_MAX_PSK_LEN ){
			unsigned char output[2*SHA1HashSize];
			char key_str_buf[WSEC_MAX_PSK_LEN+1];

			/* anthony: call passhash to make hash */
			pbkdf2_sha1(ap->key, ap->ssid, strlen(ap->ssid), 4096, output, 32);
			/* anthony: turn hex digit to char string */
			ptr = key_str_buf;
			for (i=0; i<(WSEC_MAX_PSK_LEN/8); i++){
				/* anthony: the order is comfirmed in big-endian. It maybe different in little-endian? */
				sprintf(ptr, "%02x%02x%02x%02x", (uint)output[i*4], (uint)output[i*4+1], (uint)output[i*4+2], (uint)output[i*4+3]);
				ptr += 8;
			}
			printk("%s: passphase = %s\n", __FUNCTION__, key_str_buf);

			psk.key_len = htod16((ushort)WSEC_MAX_PSK_LEN);
			memcpy(psk.key, key_str_buf, psk.key_len);
		} else {
			psk.key_len = htod16((ushort) key_len);
			memcpy(psk.key, ap->key, key_len);
		}
		psk.flags = htod16(WSEC_PASSPHRASE);
		dev_wlc_ioctl(dev, WLC_SET_WSEC_PMK, &psk, sizeof(psk));

		wpa_auth = WPA2_AUTH_PSK;
		dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);
	} else if (strnicmp(ap->sec, "wpa-psk", strlen("wpa-psk")) == 0) {

		wsec_pmk_t psk;
		size_t key_len;

		wsec = TKIP_ENABLED;
		dev_wlc_intvar_set(dev, "wsec", wsec);

		key_len = strlen(ap->key);
		if (key_len < WSEC_MIN_PSK_LEN || key_len > WSEC_MAX_PSK_LEN) {
			WL_SOFTAP(("passphrase must be between %d and %d characters long\n",
		       WSEC_MIN_PSK_LEN, WSEC_MAX_PSK_LEN));
			return -1;
		}

		/* anthony: turn all psk-key to 64 char wide in default. */
		if ( key_len < WSEC_MAX_PSK_LEN ){
			unsigned char output[2*SHA1HashSize];
			char key_str_buf[WSEC_MAX_PSK_LEN+1];

			printk("%s: do passhash...\n", __FUNCTION__);
			/* anthony: call passhash to make hash */
			pbkdf2_sha1(ap->key, ap->ssid, strlen(ap->ssid), 4096, output, 32);
			/* anthony: turn hex digit to char string */
			ptr = key_str_buf;
			for (i=0; i<(WSEC_MAX_PSK_LEN/8); i++)
			{
				printk("[%02d]: %08x\n", i, *((unsigned int*)&output[i*4]));
				/* anthony: the order is comfirmed in big-endian. It maybe different in little-endian? */
				sprintf(ptr, "%02x%02x%02x%02x", (uint)output[i*4], (uint)output[i*4+1], (uint)output[i*4+2], (uint)output[i*4+3]);
				ptr += 8;
			}
			printk("%s: passphase = %s\n", __FUNCTION__, key_str_buf);

			psk.key_len = htod16((ushort)WSEC_MAX_PSK_LEN);
			memcpy(psk.key, key_str_buf, psk.key_len);
		} else {
			psk.key_len = htod16((ushort) key_len);
			memcpy(psk.key, ap->key, key_len);
		}

		psk.flags = htod16(WSEC_PASSPHRASE);
		dev_wlc_ioctl(dev, WLC_SET_WSEC_PMK, &psk, sizeof(psk));

		wpa_auth = WPA_AUTH_PSK;
		dev_wlc_intvar_set(dev, "wpa_auth", wpa_auth);
	}

	WL_SOFTAP(("ap setup done\n"));
	updown = 0;
	dev_wlc_ioctl(dev, WLC_UP, &updown, sizeof(updown));

	max_assoc = ap->max_scb;
	dev_wlc_intvar_set(dev, "maxassoc", max_assoc);

	ap_mode = 1;
	ap_ssid.SSID_len = strlen(ap->ssid);
	strncpy(ap_ssid.SSID, ap->ssid, ap_ssid.SSID_len);
	dev_wlc_ioctl(dev, WLC_SET_SSID, &ap_ssid, sizeof(ap_ssid));
	dev_wlc_intvar_set(dev, "mpc", mpc);

	if (wep)
		dev_wlc_ioctl(dev, WLC_SET_KEY, &key, sizeof(key));

	return 0;

}

static int
wl_iw_settxpower(struct net_device *dev, int *value)
{
	if (ap_txpower_default == 0) {
		/* get the default txpower */
		dev_wlc_intvar_get(priv_dev, "qtxpower", &ap_txpower_default);
		WL_SOFTAP(("default tx power is %d\n", ap_txpower_default));
		ap_txpower_default |= WL_TXPWR_OVERRIDE;
	}

	if (*value == 99) {
		/* set to default value */
		*value = ap_txpower_default;
	} else {
		*value |= WL_TXPWR_OVERRIDE;
	}

	WL_SOFTAP(("set tx power to 0x%x\n", *value));
	dev_wlc_intvar_set(dev, "qtxpower", *value);

	return 0;
}
static int
wl_iw_get_assoc_list(struct net_device *dev, char *buf, int len)
{
	struct maclist *maclist = (struct maclist *) buf;

	dev_wlc_ioctl(dev, WLC_GET_ASSOCLIST, maclist, len);

	return 0;
}

static int
wl_iw_set_mac_list(struct net_device *dev, char *buf)
{
	struct mac_list_set *mac_list_set = (struct mac_list_set *)buf;
	struct maclist *white_maclist = (struct maclist *)&mac_list_set->white_list;
	struct maclist *black_maclist = (struct maclist *)&mac_list_set->black_list;
	int mac_mode = mac_list_set->mode;
	int length;
	int i;

	ap_macmode = mac_mode;
	if (mac_mode == MACLIST_MODE_DISABLED) {
		/* clear the black list */
		bzero(&ap_black_list, sizeof(struct mflist));

		/* set mac_mode to firmware */
		dev_wlc_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode));
	} else {
		scb_val_t scbval;
		char mac_buf[256] = {0};
		struct maclist *assoc_maclist = (struct maclist *) mac_buf;

		mac_mode = MACLIST_MODE_ALLOW;
		/* set mac_mode to firmware */
		dev_wlc_ioctl(dev, WLC_SET_MACMODE, &mac_mode, sizeof(mac_mode));

		/* set the white list */
		length = sizeof(white_maclist->count)+white_maclist->count*ETHER_ADDR_LEN;
		dev_wlc_ioctl(dev, WLC_SET_MACLIST, white_maclist, length);
		WL_SOFTAP(("White List, length %d:\n", length));
		for (i = 0; i < white_maclist->count; i++)
			WL_SOFTAP(("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, white_maclist->ea[i].octet[0], white_maclist->ea[i].octet[1], white_maclist->ea[i].octet[2],
				white_maclist->ea[i].octet[3], white_maclist->ea[i].octet[4], white_maclist->ea[i].octet[5]));

		/* set the black list */
		bcopy(black_maclist, &ap_black_list, sizeof(ap_black_list));

		WL_SOFTAP(("Black List, size %d:\n", sizeof(ap_black_list)));
		for (i = 0; i < ap_black_list.count; i++)
			WL_SOFTAP(("mac %d: %02X:%02X:%02X:%02X:%02X:%02X\n",
				i, ap_black_list.ea[i].octet[0], ap_black_list.ea[i].octet[1], ap_black_list.ea[i].octet[2],
				ap_black_list.ea[i].octet[3], ap_black_list.ea[i].octet[4], ap_black_list.ea[i].octet[5]));

		/* deauth if there is associated station not in list */
		dev_wlc_ioctl(dev, WLC_GET_ASSOCLIST, assoc_maclist, 256);
		if (assoc_maclist->count) {
			int j;
			for (i = 0; i < assoc_maclist->count; i++) {
				for (j = 0; j < white_maclist->count; j++) {
					if (!bcmp(&assoc_maclist->ea[i], &white_maclist->ea[j], ETHER_ADDR_LEN)) {
						WL_SOFTAP(("match allow, let it be\n"));
						break;
					}
				}
				if (j == white_maclist->count) {
						WL_SOFTAP(("match black, deauth it\n"));
						scbval.val = htod32(1);
						bcopy(&assoc_maclist->ea[i], &scbval.ea, ETHER_ADDR_LEN);
						dev_wlc_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scbval,
							sizeof(scb_val_t));
				}
			}
		}


	}


	return 0;
}
#endif //CONFIG_BCM4329_SOFTAP


struct dd_pkt_filter_s
{
	int add;
	int id;
	int offset;
	char mask[256];
	char pattern[256];
};
extern int dhd_set_pktfilter(int add, int id, int offset, char *mask, char *pattern);
int wl_iw_set_pktfilter(struct net_device *dev, struct dd_pkt_filter_s *data)
{
	printk("%s: add: %d, id: %d, offset: %d, mask: %s, pattern: %s\n", __func__,
		data->add, data->id, data->offset, data->mask, data->pattern);

	return dhd_set_pktfilter(data->add, data->id, data->offset, data->mask, data->pattern);
}

static int
wl_iw_set_priv(
	struct net_device *dev,
	struct iw_request_info *info,
	struct iw_point *dwrq,
	char *ext
)
{
	int ret = 0;
	char * extra;

	if (!(extra = kmalloc(dwrq->length, GFP_KERNEL)))
	    return -ENOMEM;

	if (copy_from_user(extra, dwrq->pointer, dwrq->length)) {
	    kfree(extra);
	    return -EFAULT;
	}

	WL_TRACE(("%s: SIOCSIWPRIV requst = %s\n",
		dev->name, extra));

	net_os_wake_lock(dev);

	if (dwrq->length && extra) {
		if (strnicmp(extra, "START", strlen("START")) == 0) {
			wl_iw_control_wl_on(dev, info);
			WL_TRACE(("%s, Received regular START command\n", __FUNCTION__));
		}

		if (g_onoff == G_WLAN_SET_OFF) {
			WL_TRACE(("%s, missing START, Fail\n", __FUNCTION__));
			kfree(extra);
			net_os_wake_unlock(dev);
			return -EFAULT;
		}

	    if (strnicmp(extra, "SCAN-ACTIVE", strlen("SCAN-ACTIVE")) == 0) {
#ifdef ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS
			WL_TRACE(("%s: active scan setting supppressed\n", dev->name));
#else
			ret = wl_iw_set_active_scan(dev, info, (union iwreq_data *)dwrq, extra);
#endif
	    }
	    else if (strnicmp(extra, "SCAN-PASSIVE", strlen("SCAN-PASSIVE")) == 0)
#ifdef ENABLE_ACTIVE_PASSIVE_SCAN_SUPPRESS
			WL_TRACE(("%s: passive scan setting supppressed\n", dev->name));
#else
			ret = wl_iw_set_passive_scan(dev, info, (union iwreq_data *)dwrq, extra);
#endif
	    else if (strnicmp(extra, "RSSI", strlen("RSSI")) == 0)
			ret = wl_iw_get_rssi(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "LINKSPEED", strlen("LINKSPEED")) == 0)
			ret = wl_iw_get_link_speed(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "MACADDR", strlen("MACADDR")) == 0)
			ret = wl_iw_get_macaddr(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "COUNTRY", strlen("COUNTRY")) == 0)
			ret = wl_iw_set_country(dev, info, (union iwreq_data *)dwrq, extra);
	    else if (strnicmp(extra, "STOP", strlen("STOP")) == 0)
			ret = wl_iw_control_wl_off(dev, info);
	    else if (strnicmp(extra, "POWERMODE", strlen("POWERMODE")) == 0)
			ret = wl_iw_set_btcoex_dhcp(dev, info, (union iwreq_data *)dwrq, extra);
#ifdef CONFIG_BCM4329_SOFTAP
	    else if (strnicmp(extra, "AP_TXPOWER_SET", strlen("AP_TXPOWER_SET")) == 0) {
			WL_SOFTAP(("penguin, AP_TXPOWER_SET\n"));
			wl_iw_settxpower(dev, (int *)(extra + PROFILE_OFFSET));
	    }
	    else if (strnicmp(extra, "AP_PROFILE_SET", strlen("AP_PROFILE_SET")) == 0) {
			WL_SOFTAP(("penguin, get AP_PROFILE_SET\n"));
			wl_iw_setap(dev, (struct ap_profile *)(extra + PROFILE_OFFSET));
	    }
		else if (strnicmp(extra, "AP_ASSOC_LIST_GET", strlen("AP_ASSOC_LIST_GET")) == 0) {
			WL_SOFTAP(("penguin, get AP_ASSOC_LIST_GET\n"));
			wl_iw_get_assoc_list(dev, extra, dwrq->length);
	    }
		else if (strnicmp(extra, "AP_MAC_LIST_SET", strlen("AP_MAC_LIST_SET")) == 0) {
			WL_SOFTAP(("penguin, set AP_MAC_LIST_SET\n"));
			wl_iw_set_mac_list(dev, (extra + PROFILE_OFFSET));
	    }
#endif
		/* for packet filter rule
		 */
		else if (strnicmp(extra, "RXFILTER-ADD", strlen("RXFILTER-ADD")) == 0 ||
			strnicmp(extra, "RXFILTER-REMOVE", strlen("RXFILTER-REMOVE")) == 0) {

			wl_iw_set_pktfilter(dev, (struct dd_pkt_filter_s *)&extra[32]);
		}

	    else {
			snprintf(extra, MAX_WX_STRING, "OK");
			dwrq->length = strlen("OK") + 1;
			WL_TRACE(("Unkown PRIVATE command , ignored\n"));
		}
	}

	net_os_wake_unlock(dev);

	if (extra) {
	    if (copy_to_user(dwrq->pointer, extra, dwrq->length)) {
			kfree(extra);
			return -EFAULT;
	    }

	    kfree(extra);
	}

	return ret;
}

static const iw_handler wl_iw_handler[] =
{
	(iw_handler) wl_iw_config_commit,
	(iw_handler) wl_iw_get_name,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_set_freq,
	(iw_handler) wl_iw_get_freq,
	(iw_handler) wl_iw_set_mode,
	(iw_handler) wl_iw_get_mode,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_get_range,
	(iw_handler) wl_iw_set_priv,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_set_spy,
	(iw_handler) wl_iw_get_spy,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_set_wap,
	(iw_handler) wl_iw_get_wap,
#if WIRELESS_EXT > 17
	(iw_handler) wl_iw_mlme,
#else
	(iw_handler) NULL,
#endif
#if defined(WL_IW_USE_ISCAN)
	(iw_handler) wl_iw_iscan_get_aplist,
#else
	(iw_handler) wl_iw_get_aplist,
#endif
#if WIRELESS_EXT > 13
#if defined(WL_IW_USE_ISCAN)
	(iw_handler) wl_iw_iscan_set_scan,
	(iw_handler) wl_iw_iscan_get_scan,
#else
	(iw_handler) wl_iw_set_scan,
	(iw_handler) wl_iw_get_scan,
#endif
#else
	(iw_handler) NULL,
	(iw_handler) NULL,
#endif
	(iw_handler) wl_iw_set_essid,
	(iw_handler) wl_iw_get_essid,
	(iw_handler) wl_iw_set_nick,
	(iw_handler) wl_iw_get_nick,
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_set_rate,
	(iw_handler) wl_iw_get_rate,
	(iw_handler) wl_iw_set_rts,
	(iw_handler) wl_iw_get_rts,
	(iw_handler) wl_iw_set_frag,
	(iw_handler) wl_iw_get_frag,
	(iw_handler) wl_iw_set_txpow,
	(iw_handler) wl_iw_get_txpow,
#if WIRELESS_EXT > 10
	(iw_handler) wl_iw_set_retry,
	(iw_handler) wl_iw_get_retry,
#endif
	(iw_handler) wl_iw_set_encode,
	(iw_handler) wl_iw_get_encode,
	(iw_handler) wl_iw_set_power,
	(iw_handler) wl_iw_get_power,
#if WIRELESS_EXT > 17
	(iw_handler) NULL,
	(iw_handler) NULL,
	(iw_handler) wl_iw_set_wpaie,
	(iw_handler) wl_iw_get_wpaie,
	(iw_handler) wl_iw_set_wpaauth,
	(iw_handler) wl_iw_get_wpaauth,
	(iw_handler) wl_iw_set_encodeext,
	(iw_handler) wl_iw_get_encodeext,
#ifdef BCMWPA2
	(iw_handler) wl_iw_set_pmksa,
#endif
#endif
};

#if WIRELESS_EXT > 12
static const iw_handler wl_iw_priv_handler[] = {
	NULL,
	(iw_handler)wl_iw_set_active_scan,
	NULL,
	(iw_handler)wl_iw_get_rssi,
	NULL,
	(iw_handler)wl_iw_set_passive_scan,
	NULL,
	(iw_handler)wl_iw_get_link_speed,
	NULL,
	(iw_handler)wl_iw_get_macaddr,
	NULL,
	(iw_handler)wl_iw_control_wl_off,
	NULL,
	(iw_handler)wl_iw_control_wl_on
};

static const struct iw_priv_args wl_iw_priv_args[] = {
	{
		WL_IW_SET_ACTIVE_SCAN,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"SCAN-ACTIVE"
	},
	{
		WL_IW_GET_RSSI,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"RSSI"
	},
	{
		WL_IW_SET_PASSIVE_SCAN,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"SCAN-PASSIVE"
	},
	{
		WL_IW_GET_LINK_SPEED,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"LINKSPEED"
	},
	{
		WL_IW_GET_CURR_MACADDR,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"Macaddr"
	},
	{
		WL_IW_SET_STOP,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"STOP"
	},
	{
		WL_IW_SET_START,
		0,
		IW_PRIV_TYPE_CHAR | IW_PRIV_SIZE_FIXED | MAX_WX_STRING,
		"START"
	}
};

const struct iw_handler_def wl_iw_handler_def =
{
	.num_standard = ARRAYSIZE(wl_iw_handler),
	.standard = (iw_handler *) wl_iw_handler,
	.num_private = ARRAYSIZE(wl_iw_priv_handler),
	.num_private_args = ARRAY_SIZE(wl_iw_priv_args),
	.private = (iw_handler *)wl_iw_priv_handler,
	.private_args = (void *) wl_iw_priv_args,

#if WIRELESS_EXT >= 19
	get_wireless_stats: dhd_get_wireless_stats,
#endif
	};
#endif

int
wl_iw_ioctl(
	struct net_device *dev,
	struct ifreq *rq,
	int cmd
)
{
	struct iwreq *wrq = (struct iwreq *) rq;
	struct iw_request_info info;
	iw_handler handler;
	char *extra = NULL;
	int token_size = 1, max_tokens = 0, ret = 0;

	if (cmd < SIOCIWFIRST ||
		IW_IOCTL_IDX(cmd) >= ARRAYSIZE(wl_iw_handler) ||
		!(handler = wl_iw_handler[IW_IOCTL_IDX(cmd)]))
		return -EOPNOTSUPP;

	switch (cmd) {

	case SIOCSIWESSID:
	case SIOCGIWESSID:
	case SIOCSIWNICKN:
	case SIOCGIWNICKN:
		max_tokens = IW_ESSID_MAX_SIZE + 1;
		break;

	case SIOCSIWENCODE:
	case SIOCGIWENCODE:
#if WIRELESS_EXT > 17
	case SIOCSIWENCODEEXT:
	case SIOCGIWENCODEEXT:
#endif
		max_tokens = IW_ENCODING_TOKEN_MAX;
		break;

	case SIOCGIWRANGE:
		max_tokens = sizeof(struct iw_range);
		break;

	case SIOCGIWAPLIST:
		token_size = sizeof(struct sockaddr) + sizeof(struct iw_quality);
		max_tokens = IW_MAX_AP;
		break;

#if WIRELESS_EXT > 13
	case SIOCGIWSCAN:
#if defined(WL_IW_USE_ISCAN)
	if (g_iscan)
		max_tokens = wrq->u.data.length;
	else
#endif
		max_tokens = IW_SCAN_MAX_DATA;
		break;
#endif

	case SIOCSIWSPY:
		token_size = sizeof(struct sockaddr);
		max_tokens = IW_MAX_SPY;
		break;

	case SIOCGIWSPY:
		token_size = sizeof(struct sockaddr) + sizeof(struct iw_quality);
		max_tokens = IW_MAX_SPY;
		break;

	case SIOCSIWPRIV:
		max_tokens = wrq->u.data.length;
		break;
	}

	if (max_tokens && wrq->u.data.pointer) {
		if (wrq->u.data.length > max_tokens)
			return -E2BIG;

		if (!(extra = kmalloc(max_tokens * token_size, GFP_KERNEL)))
			return -ENOMEM;

		if (copy_from_user(extra, wrq->u.data.pointer, wrq->u.data.length * token_size)) {
			kfree(extra);
			return -EFAULT;
		}
	}

	info.cmd = cmd;
	info.flags = 0;

	ret = handler(dev, &info, &wrq->u, extra);

	if (extra) {
		if (copy_to_user(wrq->u.data.pointer, extra, wrq->u.data.length * token_size)) {
			kfree(extra);
			return -EFAULT;
		}

		kfree(extra);
	}

	return ret;
}


bool
wl_iw_conn_status_str(uint32 event_type, uint32 status, uint32 reason,
	char* stringBuf, uint buflen)
{
	typedef struct conn_fail_event_map_t {
		uint32 inEvent;
		uint32 inStatus;
		uint32 inReason;
		const char* outName;
		const char* outCause;
	} conn_fail_event_map_t;


#	define WL_IW_DONT_CARE	9999
	const conn_fail_event_map_t event_map [] = {


		{WLC_E_SET_SSID,     WLC_E_STATUS_SUCCESS,   WL_IW_DONT_CARE,
		"Conn", "Success"},
		{WLC_E_SET_SSID,     WLC_E_STATUS_NO_NETWORKS, WL_IW_DONT_CARE,
		"Conn", "NoNetworks"},
		{WLC_E_SET_SSID,     WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "ConfigMismatch"},
		{WLC_E_PRUNE,        WL_IW_DONT_CARE,        WLC_E_PRUNE_ENCR_MISMATCH,
		"Conn", "EncrypMismatch"},
		{WLC_E_PRUNE,        WL_IW_DONT_CARE,        WLC_E_RSN_MISMATCH,
		"Conn", "RsnMismatch"},
		{WLC_E_AUTH,         WLC_E_STATUS_TIMEOUT,   WL_IW_DONT_CARE,
		"Conn", "AuthTimeout"},
		{WLC_E_AUTH,         WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "AuthFail"},
		{WLC_E_AUTH,         WLC_E_STATUS_NO_ACK,    WL_IW_DONT_CARE,
		"Conn", "AuthNoAck"},
		{WLC_E_REASSOC,      WLC_E_STATUS_FAIL,      WL_IW_DONT_CARE,
		"Conn", "ReassocFail"},
		{WLC_E_REASSOC,      WLC_E_STATUS_TIMEOUT,   WL_IW_DONT_CARE,
		"Conn", "ReassocTimeout"},
		{WLC_E_REASSOC,      WLC_E_STATUS_ABORT,     WL_IW_DONT_CARE,
		"Conn", "ReassocAbort"},
		{WLC_E_PSK_SUP,      WLC_SUP_KEYED,          WL_IW_DONT_CARE,
		"Sup", "ConnSuccess"},
		{WLC_E_PSK_SUP,      WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Sup", "WpaHandshakeFail"},
		{WLC_E_DEAUTH_IND,   WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "Deauth"},
		{WLC_E_DISASSOC_IND, WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "DisassocInd"},
		{WLC_E_DISASSOC,     WL_IW_DONT_CARE,        WL_IW_DONT_CARE,
		"Conn", "Disassoc"}
	};

	const char* name = "";
	const char* cause = NULL;
	int i;


	for (i = 0;  i < sizeof(event_map)/sizeof(event_map[0]);  i++) {
		const conn_fail_event_map_t* row = &event_map[i];
		if (row->inEvent == event_type &&
		    (row->inStatus == status || row->inStatus == WL_IW_DONT_CARE) &&
		    (row->inReason == reason || row->inReason == WL_IW_DONT_CARE)) {
			name = row->outName;
			cause = row->outCause;
			break;
		}
	}


	if (cause) {
		memset(stringBuf, 0, buflen);
		snprintf(stringBuf, buflen, "%s %s %02d %02d",
			name, cause, status, reason);
		WL_INFORM(("Connection status: %s\n", stringBuf));
		return TRUE;
	} else {
		return FALSE;
	}
}

#if WIRELESS_EXT > 14

static bool
wl_iw_check_conn_fail(wl_event_msg_t *e, char* stringBuf, uint buflen)
{
	uint32 event = ntoh32(e->event_type);
	uint32 status =  ntoh32(e->status);
	uint32 reason =  ntoh32(e->reason);

	if (wl_iw_conn_status_str(event, status, reason, stringBuf, buflen)) {
		return TRUE;
	}
	else
		return FALSE;
}
#endif

#ifndef IW_CUSTOM_MAX
#define IW_CUSTOM_MAX 256
#endif

void
wl_iw_event(struct net_device *dev, wl_event_msg_t *e, void* data)
{
#if WIRELESS_EXT > 13
	union iwreq_data wrqu;
	char extra[IW_CUSTOM_MAX + 1];
	int cmd = 0;
	uint32 event_type = ntoh32(e->event_type);
	uint16 flags =  ntoh16(e->flags);
	uint32 datalen = ntoh32(e->datalen);
	uint32 status =  ntoh32(e->status);
		uint32 toto;

	memset(&wrqu, 0, sizeof(wrqu));
	memset(extra, 0, sizeof(extra));


	switch (event_type) {
#ifdef CONFIG_BCM4329_SOFTAP
	case WLC_E_PRUNE:
		if (ap_mode) {
			char *macaddr = (char *)&e->addr;
			WL_SOFTAP(("PRUNE received, %02X:%02X:%02X:%02X:%02X:%02X!\n",
				macaddr[0],macaddr[1],macaddr[2],macaddr[3],macaddr[4],macaddr[5]));

			/* send the message if macmode enabled */
			if (ap_macmode)
			{
				int i;
				for (i = 0; i < ap_black_list.count; i++) {
					if (!bcmp(macaddr, &ap_black_list.ea[i], sizeof(struct ether_addr))) {
						WL_SOFTAP(("mac in black list, ignore it\n"));
						break;
					}
				}

				if (i == ap_black_list.count) { /* mac not found in black list, send notify */
					char mac_buf[32] = {0};
					sprintf(mac_buf, "STA_BLOCK %02X:%02X:%02X:%02X:%02X:%02X",
						macaddr[0], macaddr[1], macaddr[2],
						macaddr[3], macaddr[4], macaddr[5]);
					wl_iw_send_priv_event(priv_dev, mac_buf);
				}
			}
		}
		break;
#endif
	case WLC_E_TXFAIL:
		cmd = IWEVTXDROP;
		memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		break;
#if WIRELESS_EXT > 14
	case WLC_E_JOIN:
	case WLC_E_ASSOC_IND:
	case WLC_E_REASSOC_IND:
#ifdef CONFIG_BCM4329_SOFTAP
		WL_SOFTAP(("connect received %d\n", event_type));
		if (ap_mode) {
			wl_iw_send_priv_event(priv_dev, "STA_JOIN");
			return;
		}
#endif
		memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		cmd = IWEVREGISTERED;
		break;
	case WLC_E_DEAUTH_IND:
	case WLC_E_DISASSOC_IND:
#ifdef CONFIG_BCM4329_SOFTAP
		WL_SOFTAP(("disconnect received %d\n", event_type));
		if (ap_mode) {
			//wl_iw_send_priv_event(priv_dev, "STA_LEAVE");
			char *macaddr = (char *)&e->addr;
			char mac_buf[32] = {0};
			sprintf(mac_buf, "STA_LEAVE %02X:%02X:%02X:%02X:%02X:%02X",
				macaddr[0], macaddr[1], macaddr[2],
				macaddr[3], macaddr[4], macaddr[5]);
			WL_SOFTAP(("leave received, %02X:%02X:%02X:%02X:%02X:%02X!\n",
				macaddr[0],macaddr[1],macaddr[2],macaddr[3],macaddr[4],macaddr[5]));
			wl_iw_send_priv_event(priv_dev, mac_buf);
			return;
		}
#endif
		cmd = SIOCGIWAP;
		bzero(wrqu.addr.sa_data, ETHER_ADDR_LEN);
		wrqu.addr.sa_family = ARPHRD_ETHER;
		bzero(&extra, ETHER_ADDR_LEN);
		break;
	case WLC_E_LINK:
	case WLC_E_NDIS_LINK:
		cmd = SIOCGIWAP;
		if (!(flags & WLC_EVENT_MSG_LINK)) {
			if (g_ss_cache_ctrl.last_rssi == 0)
				g_ss_cache_ctrl.m_link_down = 1;

			WL_TRACE(("Link Down\n"));
			iw_link_state = 0;

			bzero(wrqu.addr.sa_data, ETHER_ADDR_LEN);
			bzero(&extra, ETHER_ADDR_LEN);
		}
		else {

			memcpy(wrqu.addr.sa_data, &e->addr, ETHER_ADDR_LEN);
			g_ss_cache_ctrl.m_link_down = 0;

			memcpy(g_ss_cache_ctrl.m_active_bssid, &e->addr, ETHER_ADDR_LEN);
#ifdef CONFIG_BCM4329_SOFTAP
		if (ap_mode) {
			WL_SOFTAP(("send AP_UP\n"));
			wl_iw_send_priv_event(priv_dev, "AP_UP");
			return;
		}
#endif
			WL_TRACE(("Link UP\n"));
			iw_link_state = 1;

		}
		wrqu.addr.sa_family = ARPHRD_ETHER;
		break;
	case WLC_E_ACTION_FRAME:
		cmd = IWEVCUSTOM;
		if (datalen + 1 <= sizeof(extra)) {
			wrqu.data.length = datalen + 1;
			extra[0] = WLC_E_ACTION_FRAME;
			memcpy(&extra[1], data, datalen);
			printf("WLC_E_ACTION_FRAME len %d \n", wrqu.data.length);
		}
		break;

	case WLC_E_ACTION_FRAME_COMPLETE:
		cmd = IWEVCUSTOM;
		memcpy(&toto, data, 4);
		if (sizeof(status) + 1 <= sizeof(extra)) {
			wrqu.data.length = sizeof(status) + 1;
			extra[0] = WLC_E_ACTION_FRAME_COMPLETE;
			memcpy(&extra[1], &status, sizeof(status));
			printf("wl_iw_event status %d PacketId %d \n", status, toto);
			printf("WLC_E_ACTION_FRAME_COMPLETE len %d \n", wrqu.data.length);
		}
		break;
#endif
#if WIRELESS_EXT > 17
	case WLC_E_MIC_ERROR: {
		struct	iw_michaelmicfailure  *micerrevt = (struct  iw_michaelmicfailure  *)&extra;
		cmd = IWEVMICHAELMICFAILURE;
		wrqu.data.length = sizeof(struct iw_michaelmicfailure);
		if (flags & WLC_EVENT_MSG_GROUP)
			micerrevt->flags |= IW_MICFAILURE_GROUP;
		else
			micerrevt->flags |= IW_MICFAILURE_PAIRWISE;
		memcpy(micerrevt->src_addr.sa_data, &e->addr, ETHER_ADDR_LEN);
		micerrevt->src_addr.sa_family = ARPHRD_ETHER;

		break;
	}
#ifdef BCMWPA2
	case WLC_E_PMKID_CACHE: {
		if (data)
		{
			struct iw_pmkid_cand *iwpmkidcand = (struct iw_pmkid_cand *)&extra;
			pmkid_cand_list_t *pmkcandlist;
			pmkid_cand_t	*pmkidcand;
			int count;

			cmd = IWEVPMKIDCAND;
			pmkcandlist = data;
			count = ntoh32_ua((uint8 *)&pmkcandlist->npmkid_cand);
			ASSERT(count >= 0);
			wrqu.data.length = sizeof(struct iw_pmkid_cand);
			pmkidcand = pmkcandlist->pmkid_cand;
			while (count) {
				bzero(iwpmkidcand, sizeof(struct iw_pmkid_cand));
				if (pmkidcand->preauth)
					iwpmkidcand->flags |= IW_PMKID_CAND_PREAUTH;
				bcopy(&pmkidcand->BSSID, &iwpmkidcand->bssid.sa_data,
					ETHER_ADDR_LEN);
#ifndef SANDGATE2G
				wireless_send_event(dev, cmd, &wrqu, extra);
#endif
				pmkidcand++;
				count--;
			}
		}
		return;
	}
#endif
#endif

#ifdef WLAN_PFN
	case WLC_E_PFN_NET_FOUND:
	{
		wlc_ssid_t			* ssid;
		if (iw_link_state)
			break;
		pfn_scan = 1;
		net_os_wake_lock(dev);
		wl_iw_set_pfn_timer(1);
		ssid = (wlc_ssid_t *)data;
		printk("pfn scan ssid = %s, len = %d\n", ssid->SSID, ssid->SSID_len);
		memset(pfn_ssid, 0, sizeof(pfn_ssid));
		pfn_ssid_len = strlen(ssid->SSID);
		strncpy(pfn_ssid, ssid->SSID, pfn_ssid_len);
		up(&g_iscan->sysioc_sem);
		break;
	}
#endif
	case WLC_E_SCAN_COMPLETE:
#if defined(WL_IW_USE_ISCAN)
		if ((g_iscan) && (g_iscan->sysioc_pid >= 0) &&
			(g_iscan->iscan_state != ISCAN_STATE_IDLE))
		{
			up(&g_iscan->sysioc_sem);
		} else {
			cmd = SIOCGIWSCAN;
			wrqu.data.length = strlen(extra);
			WL_TRACE(("Event WLC_E_SCAN_COMPLETE from specific scan\n"));
		}
#else
		cmd = SIOCGIWSCAN;
		wrqu.data.length = strlen(extra);
		WL_TRACE(("Event WLC_E_SCAN_COMPLETE\n"));
#endif
		break;

	default:

		WL_TRACE(("Unknown Event %d: ignoring\n", event_type));
		break;
	}
#ifndef SANDGATE2G
	if (cmd)
		wireless_send_event(dev, cmd, &wrqu, extra);
#endif

#if WIRELESS_EXT > 14

	memset(extra, 0, sizeof(extra));
	if (wl_iw_check_conn_fail(e, extra, sizeof(extra))) {
		cmd = IWEVCUSTOM;
		wrqu.data.length = strlen(extra);
#ifndef SANDGATE2G
		wireless_send_event(dev, cmd, &wrqu, extra);
#endif
	}
#endif

#endif
}

int wl_iw_get_wireless_stats(struct net_device *dev, struct iw_statistics *wstats)
{
	int res = 0;
	wl_cnt_t cnt;
	int phy_noise;
	int rssi;
	scb_val_t scb_val;

	phy_noise = 0;
	if ((res = dev_wlc_ioctl(dev, WLC_GET_PHY_NOISE, &phy_noise, sizeof(phy_noise))))
		goto done;

	phy_noise = dtoh32(phy_noise);
	WL_TRACE(("wl_iw_get_wireless_stats phy noise=%d\n *****", phy_noise));

	scb_val.val = 0;
	if ((res = dev_wlc_ioctl(dev, WLC_GET_RSSI, &scb_val, sizeof(scb_val_t))))
		goto done;

	rssi = dtoh32(scb_val.val);
	WL_TRACE(("wl_iw_get_wireless_stats rssi=%d ****** \n", rssi));
	if (rssi <= WL_IW_RSSI_NO_SIGNAL)
		wstats->qual.qual = 0;
	else if (rssi <= WL_IW_RSSI_VERY_LOW)
		wstats->qual.qual = 1;
	else if (rssi <= WL_IW_RSSI_LOW)
		wstats->qual.qual = 2;
	else if (rssi <= WL_IW_RSSI_GOOD)
		wstats->qual.qual = 3;
	else if (rssi <= WL_IW_RSSI_VERY_GOOD)
		wstats->qual.qual = 4;
	else
		wstats->qual.qual = 5;


	wstats->qual.level = 0x100 + rssi;
	wstats->qual.noise = 0x100 + phy_noise;
#if WIRELESS_EXT > 18
	wstats->qual.updated |= (IW_QUAL_ALL_UPDATED | IW_QUAL_DBM);
#else
	wstats->qual.updated |= 7;
#endif

#if WIRELESS_EXT > 11
	WL_TRACE(("wl_iw_get_wireless_stats counters=%d\n *****", (int)sizeof(wl_cnt_t)));

	memset(&cnt, 0, sizeof(wl_cnt_t));
	res = dev_wlc_bufvar_get(dev, "counters", (char *)&cnt, sizeof(wl_cnt_t));
	if (res)
	{
		WL_ERROR(("wl_iw_get_wireless_stats counters failed error=%d ****** \n", res));
		goto done;
	}

	cnt.version = dtoh16(cnt.version);
	if (cnt.version != WL_CNT_T_VERSION) {
		WL_TRACE(("\tIncorrect version of counters struct: expected %d; got %d\n",
			WL_CNT_T_VERSION, cnt.version));
		goto done;
	}

	wstats->discard.nwid = 0;
	wstats->discard.code = dtoh32(cnt.rxundec);
	wstats->discard.fragment = dtoh32(cnt.rxfragerr);
	wstats->discard.retries = dtoh32(cnt.txfail);
	wstats->discard.misc = dtoh32(cnt.rxrunt) + dtoh32(cnt.rxgiant);
	wstats->miss.beacon = 0;

	WL_TRACE(("wl_iw_get_wireless_stats counters txframe=%d txbyte=%d\n",
		dtoh32(cnt.txframe), dtoh32(cnt.txbyte)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxfrmtoolong=%d\n", dtoh32(cnt.rxfrmtoolong)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxbadplcp=%d\n", dtoh32(cnt.rxbadplcp)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxundec=%d\n", dtoh32(cnt.rxundec)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxfragerr=%d\n", dtoh32(cnt.rxfragerr)));
	WL_TRACE(("wl_iw_get_wireless_stats counters txfail=%d\n", dtoh32(cnt.txfail)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxrunt=%d\n", dtoh32(cnt.rxrunt)));
	WL_TRACE(("wl_iw_get_wireless_stats counters rxgiant=%d\n", dtoh32(cnt.rxgiant)));

#endif

done:
	return res;
}
static void
wl_iw_bt_flag_set(
	struct net_device *dev,
	bool set)
{
	char buf_flag7_dhcp_on[8] = { 7, 00, 00, 00, 0x1, 0x0, 0x00, 0x00 };
	char buf_flag7_default[8]   = { 7, 00, 00, 00, 0x0, 0x00, 0x00, 0x00};

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_lock();
#endif

	if (set == TRUE) {

		dev_wlc_bufvar_set(dev, "btc_flags", \
					(char *)&buf_flag7_dhcp_on[0], sizeof(buf_flag7_dhcp_on));
	}
	else  {

		dev_wlc_bufvar_set(dev, "btc_flags", \
					(char *)&buf_flag7_default[0], sizeof(buf_flag7_default));
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27))
	rtnl_unlock();
#endif
}

static void
wl_iw_bt_timerfunc(ulong data)
{
	bt_info_t  *bt_local = (bt_info_t *)data;
	bt_local->timer_on = 0;
	WL_TRACE(("%s\n", __FUNCTION__));

	up(&bt_local->bt_sem);
}

static int
_bt_dhcp_sysioc_thread(void *data)
{
	DAEMONIZE("dhcp_sysioc");

	while (down_interruptible(&g_bt->bt_sem) == 0) {

		net_os_wake_lock(g_bt->dev);

		if (g_bt->timer_on) {
			g_bt->timer_on = 0;
			del_timer_sync(&g_bt->timer);
		}

		switch (g_bt->bt_state) {
			case BT_DHCP_START:

				g_bt->bt_state = BT_DHCP_OPPORTUNITY_WINDOW;
				mod_timer(&g_bt->timer, jiffies + BT_DHCP_OPPORTUNITY_WINDOW_TIEM*HZ/1000);
				g_bt->timer_on = 1;
				break;
			case BT_DHCP_OPPORTUNITY_WINDOW:

				WL_TRACE(("%s waiting for %d msec expired, force bt flag\n", \
						__FUNCTION__, BT_DHCP_OPPORTUNITY_WINDOW_TIEM));
				if (g_bt->dev) wl_iw_bt_flag_set(g_bt->dev, TRUE);
				g_bt->bt_state = BT_DHCP_FLAG_FORCE_TIMEOUT;
				mod_timer(&g_bt->timer, jiffies + BT_DHCP_FLAG_FORCE_TIME*HZ/1000);
				g_bt->timer_on = 1;
				break;
			case BT_DHCP_FLAG_FORCE_TIMEOUT:

				WL_TRACE(("%s waiting for %d msec expired remove bt flag\n", \
						__FUNCTION__, BT_DHCP_FLAG_FORCE_TIME));

				if (g_bt->dev)  wl_iw_bt_flag_set(g_bt->dev, FALSE);
				g_bt->bt_state = BT_DHCP_IDLE;
				g_bt->timer_on = 0;
				break;
			default:
				WL_ERROR(("%s error g_status=%d !!!\n", __FUNCTION__, \
				          g_bt->bt_state));
				if (g_bt->dev) wl_iw_bt_flag_set(g_bt->dev, FALSE);
				g_bt->bt_state = BT_DHCP_IDLE;
				g_bt->timer_on = 0;
				break;
		 }

		net_os_wake_unlock(g_bt->dev);
	}

	if (g_bt->timer_on) {
		g_bt->timer_on = 0;
		del_timer_sync(&g_bt->timer);
	}

	complete_and_exit(&g_bt->bt_exited, 0);
}

static void
wl_iw_bt_release(void)
{
	bt_info_t *bt_local = g_bt;

	if (!bt_local) {
		return;
	}

	if (bt_local->bt_pid >= 0) {
		KILL_PROC(bt_local->bt_pid, SIGTERM);
		wait_for_completion(&bt_local->bt_exited);
	}
	kfree(bt_local);
	g_bt = NULL;
}

static int
wl_iw_bt_init(struct net_device *dev)
{
	bt_info_t *bt_dhcp = NULL;

	bt_dhcp = kmalloc(sizeof(bt_info_t), GFP_KERNEL);
	if (!bt_dhcp)
		return -ENOMEM;

	memset(bt_dhcp, 0, sizeof(bt_info_t));
	bt_dhcp->bt_pid = -1;
	g_bt = bt_dhcp;
	bt_dhcp->dev = dev;
	bt_dhcp->bt_state = BT_DHCP_IDLE;


	bt_dhcp->timer_ms    = 10;
	init_timer(&bt_dhcp->timer);
	bt_dhcp->timer.data = (ulong)bt_dhcp;
	bt_dhcp->timer.function = wl_iw_bt_timerfunc;

	sema_init(&bt_dhcp->bt_sem, 0);
	init_completion(&bt_dhcp->bt_exited);
	bt_dhcp->bt_pid = kernel_thread(_bt_dhcp_sysioc_thread, bt_dhcp, 0);
	if (bt_dhcp->bt_pid < 0) {
		WL_ERROR(("Failed in %s\n", __FUNCTION__));
		return -ENOMEM;
	}

	return 0;
}

int wl_iw_attach(struct net_device *dev, void * dhdp)
{
	wl_iw_t *iw;
#if defined(WL_IW_USE_ISCAN)
	iscan_info_t *iscan = NULL;

	if (!dev)
		return 0;

	iscan = kmalloc(sizeof(iscan_info_t), GFP_KERNEL);
	if (!iscan)
		return -ENOMEM;
	memset(iscan, 0, sizeof(iscan_info_t));
	iscan->sysioc_pid = -1;

	g_iscan = iscan;
	iscan->dev = dev;
	iscan->iscan_state = ISCAN_STATE_IDLE;
	g_iscan->scan_flag = 0;


	iscan->timer_ms    = 3000;
	init_timer(&iscan->timer);
	iscan->timer.data = (ulong)iscan;
	iscan->timer.function = wl_iw_timerfunc;

	sema_init(&iscan->sysioc_sem, 0);
	init_completion(&iscan->sysioc_exited);
	iscan->sysioc_pid = kernel_thread(_iscan_sysioc_thread, iscan, 0);
	if (iscan->sysioc_pid < 0)
		return -ENOMEM;
#endif
	mutex_init(&wl_start_lock);

	iw = *(wl_iw_t **)netdev_priv(dev);
	iw->pub = (dhd_pub_t *)dhdp;
	g_scan = NULL;


	g_scan = (void *)kmalloc(G_SCAN_RESULTS, GFP_KERNEL);
	if (!g_scan)
		return -ENOMEM;

	memset(g_scan, 0, G_SCAN_RESULTS);
	g_scan_specified_ssid = 0;

	wl_iw_init_ss_cache_ctrl();

	wl_iw_bt_init(dev);

	priv_dev = dev;
	return 0;
}

void wl_iw_detach(void)
{
#if defined(WL_IW_USE_ISCAN)
	iscan_buf_t  *buf;
	iscan_info_t *iscan = g_iscan;

	if (!iscan)
		return;
	if (iscan->sysioc_pid >= 0) {
		KILL_PROC(iscan->sysioc_pid, SIGTERM);
		wait_for_completion(&iscan->sysioc_exited);
	}

	while (iscan->list_hdr) {
		buf = iscan->list_hdr->next;
		kfree(iscan->list_hdr);
		iscan->list_hdr = buf;
	}
	kfree(iscan);
	g_iscan = NULL;
#endif

	if (g_scan)
		kfree(g_scan);

	g_scan = NULL;
	wl_iw_release_ss_cache_ctrl();
	wl_iw_bt_release();

#ifdef CONFIG_BCM4329_SOFTAP
	if (ap_mode)
		wl_iw_send_priv_event(priv_dev, "AP_DOWN");
#endif
#ifdef WLAN_PFN
	wl_iw_set_pfn_timer(0);
#endif

}
