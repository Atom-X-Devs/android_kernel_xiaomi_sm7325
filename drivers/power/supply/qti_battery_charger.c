// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt)	"BATTERY_CHG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/soc/qcom/battery_charger.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#ifdef CONFIG_MACH_XIAOMI
#include <linux/hwid.h>
#include <linux/ktime.h>
#include <linux/thermal.h>
#include <linux/string.h>

#include <drm/mi_disp_notifier.h>
#endif

#define MSG_OWNER_BC			32778
#define MSG_TYPE_REQ_RESP		1
#define MSG_TYPE_NOTIFY			2

/* opcode for battery charger */
#define BC_SET_NOTIFY_REQ		0x04
#define BC_DISABLE_NOTIFY_REQ		0x05
#define BC_NOTIFY_IND			0x07
#define BC_BATTERY_STATUS_GET		0x30
#define BC_BATTERY_STATUS_SET		0x31
#define BC_USB_STATUS_GET		0x32
#define BC_USB_STATUS_SET		0x33
#define BC_WLS_STATUS_GET		0x34
#define BC_WLS_STATUS_SET		0x35
#define BC_SHIP_MODE_REQ_SET		0x36
#ifdef CONFIG_MACH_XIAOMI
#define BC_SHUTDOWN_REQ_SET		0x37
#endif
#define BC_WLS_FW_CHECK_UPDATE		0x40
#define BC_WLS_FW_PUSH_BUF_REQ		0x41
#define BC_WLS_FW_UPDATE_STATUS_RESP	0x42
#define BC_WLS_FW_PUSH_BUF_RESP		0x43
#define BC_WLS_FW_GET_VERSION		0x44
#define BC_SHUTDOWN_NOTIFY		0x47
#ifdef CONFIG_MACH_XIAOMI
#define BC_XM_STATUS_GET		0x50
#define BC_XM_STATUS_SET		0x51
#endif
#define BC_GENERIC_NOTIFY		0x80

/* Generic definitions */
#define MAX_STR_LEN			128
#define BC_WAIT_TIME_MS			1000
#define WLS_FW_PREPARE_TIME_MS		300
#define WLS_FW_WAIT_TIME_MS		500
#define WLS_FW_UPDATE_TIME_MS		1000
#define WLS_FW_BUF_SIZE			128
#define DEFAULT_RESTRICT_FCC_UA		1000000

#ifdef CONFIG_MACH_XIAOMI
#if defined(CONFIG_BQ_FG_1S)
#define BATTERY_DIGEST_LEN 32
#else
#define BATTERY_DIGEST_LEN 20
#endif
#define BATTERY_SS_AUTH_DATA_LEN 4

#define ADAP_TYPE_SDP		1
#define ADAP_TYPE_CDP		3
#define ADAP_TYPE_DCP		2
#define ADAP_TYPE_PD		6

#define USBPD_UVDM_SS_LEN		4
#define USBPD_UVDM_VERIFIED_LEN		1

#define MAX_THERMAL_LEVEL		16

enum uvdm_state {
	USBPD_UVDM_DISCONNECT,
	USBPD_UVDM_CHARGER_VERSION,
	USBPD_UVDM_CHARGER_VOLTAGE,
	USBPD_UVDM_CHARGER_TEMP,
	USBPD_UVDM_SESSION_SEED,
	USBPD_UVDM_AUTHENTICATION,
	USBPD_UVDM_VERIFIED,
	USBPD_UVDM_REMOVE_COMPENSATION,
	USBPD_UVDM_REVERSE_AUTHEN,
	USBPD_UVDM_CONNECT,
};
#endif

enum psy_type {
	PSY_TYPE_BATTERY,
	PSY_TYPE_USB,
	PSY_TYPE_WLS,
#ifdef CONFIG_MACH_XIAOMI
	PSY_TYPE_XM,
#endif
	PSY_TYPE_MAX,
};

enum ship_mode_type {
	SHIP_MODE_PMIC,
	SHIP_MODE_PACK_SIDE,
};

/* property ids */
enum battery_property_id {
	BATT_STATUS,
	BATT_HEALTH,
	BATT_PRESENT,
	BATT_CHG_TYPE,
	BATT_CAPACITY,
	BATT_SOH,
	BATT_VOLT_OCV,
	BATT_VOLT_NOW,
	BATT_VOLT_MAX,
	BATT_CURR_NOW,
	BATT_CHG_CTRL_LIM,
	BATT_CHG_CTRL_LIM_MAX,
#ifdef CONFIG_MACH_XIAOMI
	BATT_CONSTANT_CURRENT,
#endif
	BATT_TEMP,
	BATT_TECHNOLOGY,
	BATT_CHG_COUNTER,
	BATT_CYCLE_COUNT,
	BATT_CHG_FULL_DESIGN,
	BATT_CHG_FULL,
	BATT_MODEL_NAME,
	BATT_TTF_AVG,
	BATT_TTE_AVG,
	BATT_RESISTANCE,
	BATT_POWER_NOW,
	BATT_POWER_AVG,
	BATT_PROP_MAX,
};

enum usb_property_id {
	USB_ONLINE,
	USB_VOLT_NOW,
	USB_VOLT_MAX,
	USB_CURR_NOW,
	USB_CURR_MAX,
	USB_INPUT_CURR_LIMIT,
	USB_TYPE,
	USB_ADAP_TYPE,
	USB_MOISTURE_DET_EN,
	USB_MOISTURE_DET_STS,
	USB_TEMP,
	USB_REAL_TYPE,
	USB_TYPEC_COMPLIANT,
	USB_PROP_MAX,
};

enum wireless_property_id {
	WLS_ONLINE,
	WLS_VOLT_NOW,
	WLS_VOLT_MAX,
	WLS_CURR_NOW,
	WLS_CURR_MAX,
	WLS_TYPE,
	WLS_BOOST_EN,
#ifdef CONFIG_MACH_XIAOMI
	WLS_FW_VER,
	WLS_TX_ADAPTER,
	WLS_REGISTER,
	WLS_INPUT_CURR,
#endif
	WLS_PROP_MAX,
};

#ifdef CONFIG_MACH_XIAOMI
enum xm_property_id {
	XM_PROP_RESISTANCE_ID,
	XM_PROP_VERIFY_DIGEST,
	XM_PROP_CONNECTOR_TEMP,
	XM_PROP_AUTHENTIC,
	XM_PROP_CHIP_OK,
	XM_PROP_SOC_DECIMAL,
	XM_PROP_SOC_DECIMAL_RATE,
	XM_PROP_SHUTDOWN_DELAY,
	XM_PROP_VBUS_DISABLE,
	XM_PROP_CC_ORIENTATION,
	XM_PROP_SLAVE_BATT_PRESENT,
#ifdef CONFIG_BQ2597X_BATTERY_CHARGER
	XM_PROP_BQ2597X_CHIP_OK,
	XM_PROP_BQ2597X_SLAVE_CHIP_OK,
	XM_PROP_BQ2597X_BUS_CURRENT,
	XM_PROP_BQ2597X_SLAVE_BUS_CURRENT,
	XM_PROP_BQ2597X_BUS_DELTA,
	XM_PROP_BQ2597X_BUS_VOLTAGE,
	XM_PROP_BQ2597X_BATTERY_PRESENT,
	XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT,
	XM_PROP_BQ2597X_BATTERY_VOLTAGE,
	XM_PROP_COOL_MODE,
#endif
	XM_PROP_BQ2597X_SLAVE_CONNECTOR,
	XM_PROP_BT_TRANSFER_START,
	XM_PROP_MASTER_SMB1396_ONLINE,
	XM_PROP_MASTER_SMB1396_IIN,
	XM_PROP_SLAVE_SMB1396_ONLINE,
	XM_PROP_SLAVE_SMB1396_IIN,
	XM_PROP_SMB_IIN_DIFF,
	/* wireless charge infor */
	XM_PROP_TX_MACL,
	XM_PROP_TX_MACH,
	XM_PROP_RX_CRL,
	XM_PROP_RX_CRH,
	XM_PROP_RX_CEP,
	XM_PROP_BT_STATE,
	XM_PROP_REVERSE_CHG_MODE,
	XM_PROP_REVERSE_CHG_STATE,
	XM_PROP_WLS_FW_STATE,
	XM_PROP_RX_VOUT,
	XM_PROP_RX_VRECT,
	XM_PROP_RX_IOUT,
	XM_PROP_TX_ADAPTER,
	XM_PROP_OP_MODE,
	XM_PROP_WLS_DIE_TEMP,
	XM_PROP_WLS_CAR_ADAPTER,
	XM_PROP_WLS_TX_SPEED,
	/**********************/
	XM_PROP_INPUT_SUSPEND,
	XM_PROP_REAL_TYPE,
	/*used for pd authentic*/
	XM_PROP_VERIFY_PROCESS,
	XM_PROP_VDM_CMD_CHARGER_VERSION,
	XM_PROP_VDM_CMD_CHARGER_VOLTAGE,
	XM_PROP_VDM_CMD_CHARGER_TEMP,
	XM_PROP_VDM_CMD_SESSION_SEED,
	XM_PROP_VDM_CMD_AUTHENTICATION,
	XM_PROP_VDM_CMD_VERIFIED,
	XM_PROP_VDM_CMD_REMOVE_COMPENSATION,
	XM_PROP_VDM_CMD_REVERSE_AUTHEN,
	XM_PROP_CURRENT_STATE,
	XM_PROP_ADAPTER_ID,
	XM_PROP_ADAPTER_SVID,
	XM_PROP_PD_VERIFED,
	XM_PROP_PDO2,
	XM_PROP_UVDM_STATE,
	/*****************/
	XM_PROP_WLS_BIN,
	XM_PROP_FASTCHGMODE,
	XM_PROP_APDO_MAX,
	XM_PROP_THERMAL_REMOVE,
	XM_PROP_VOTER_DEBUG,
	XM_PROP_FG_RM,
	XM_PROP_WLSCHARGE_CONTROL_LIMIT,
	XM_PROP_MTBF_CURRENT,
	XM_PROP_FAKE_TEMP,
	XM_PROP_QBG_VBAT,
	XM_PROP_QBG_VPH_PWR,
	XM_PROP_QBG_TEMP,
	XM_PROP_FB_BLANK_STATE,
	XM_PROP_THERMAL_TEMP,
	XM_PROP_TYPEC_MODE,
	XM_PROP_NIGHT_CHARGING,
	XM_PROP_SMART_BATT,
	XM_PROP_FG1_QMAX,
	XM_PROP_FG1_RM,
	XM_PROP_FG1_FCC,
	XM_PROP_FG1_SOH,
	XM_PROP_FG1_FCC_SOH,
	XM_PROP_FG1_CYCLE,
	XM_PROP_FG1_FAST_CHARGE,
	XM_PROP_FG1_CURRENT_MAX,
	XM_PROP_FG1_VOL_MAX,
	XM_PROP_FG1_TSIM,
	XM_PROP_FG1_TAMBIENT,
	XM_PROP_FG1_TREMQ,
	XM_PROP_FG1_TFULLQ,
	XM_PROP_FG1_SEAL_STATE,
	XM_PROP_FG1_SEAL_SET,
	XM_PROP_FG1_DF_CHECK,
	XM_PROP_FG1_VOLTAGE_MAX,
	XM_PROP_FG1_Charge_Current_MAX,
	XM_PROP_FG1_Discharge_Current_MAX,
	XM_PROP_FG1_TEMP_MAX,
	XM_PROP_FG1_TEMP_MIN,
	XM_PROP_FG1_TIME_HT,
	XM_PROP_FG1_TIME_OT,
	XM_PROP_FG1_TIME_UT,
	XM_PROP_FG1_TIME_LT,
#ifdef CONFIG_BQ_CLOUD_AUTHENTICATION
	XM_PROP_SERVER_SN,
	XM_PROP_SERVER_RESULT,
	XM_PROP_ADSP_RESULT,
#endif
	XM_PROP_SHIPMODE_COUNT_RESET,
	XM_PROP_SPORT_MODE,
	XM_PROP_CELL1_VOLT,
	XM_PROP_CELL2_VOLT,
	XM_PROP_FG_VENDOR_ID,
#ifdef CONFIG_AI_RSOC
	XM_PROP_FG1_RSOC,
	XM_PROP_FG1_AI,
#endif
	XM_PROP_MAX,
};
#endif

enum {
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP = 0x80,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5,
#ifdef CONFIG_MACH_XIAOMI
	QTI_POWER_SUPPLY_USB_TYPE_USB_FLOAT,
	QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3_CLASSB,
#endif
};

struct battery_charger_set_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			power_state;
	u32			low_capacity;
	u32			high_capacity;
};

struct battery_charger_notify_msg {
	struct pmic_glink_hdr	hdr;
	u32			notification;
};

struct battery_charger_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			battery_id;
	u32			property_id;
	u32			value;
};

struct battery_charger_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			value;
	u32			ret_code;
};

struct battery_model_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	char			model[MAX_STR_LEN];
};

#ifdef CONFIG_MACH_XIAOMI
struct wls_fw_resp_msg {
	struct pmic_glink_hdr   hdr;
	u32                     property_id;
	u32			value;
	char                    version[MAX_STR_LEN];
};

struct xm_verify_digest_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u8			digest[BATTERY_DIGEST_LEN];
	bool			slave_fg;
};

struct xm_set_wls_bin_req_msg {
	struct pmic_glink_hdr hdr;
	u32 property_id;
	u16 total_length;
	u8 serial_number;
	u8 fw_area;
	u8 wls_fw_bin[MAX_STR_LEN];
};  /* Message */
#endif

struct wireless_fw_check_req {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
	u32			fw_size;
	u32			fw_crc;
};

struct wireless_fw_check_resp {
	struct pmic_glink_hdr	hdr;
	u32			ret_code;
};

struct wireless_fw_push_buf_req {
	struct pmic_glink_hdr	hdr;
	u8			buf[WLS_FW_BUF_SIZE];
	u32			fw_chunk_id;
};

struct wireless_fw_push_buf_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_status;
};

struct wireless_fw_update_status {
	struct pmic_glink_hdr	hdr;
	u32			fw_update_done;
};

struct wireless_fw_get_version_req {
	struct pmic_glink_hdr	hdr;
};

struct wireless_fw_get_version_resp {
	struct pmic_glink_hdr	hdr;
	u32			fw_version;
};

struct battery_charger_ship_mode_req_msg {
	struct pmic_glink_hdr	hdr;
	u32			ship_mode_type;
};

#ifdef CONFIG_MACH_XIAOMI
struct battery_charger_shutdown_req_msg {
	struct pmic_glink_hdr	hdr;
};

struct xm_ss_auth_resp_msg {
	struct pmic_glink_hdr	hdr;
	u32			property_id;
	u32			data[BATTERY_SS_AUTH_DATA_LEN];
};
#endif

struct psy_state {
	struct power_supply	*psy;
	char			*model;
#ifdef CONFIG_MACH_XIAOMI
	char			*version;
#endif
	const int		*map;
	u32			*prop;
	u32			prop_count;
	u32			opcode_get;
	u32			opcode_set;
};

struct battery_chg_dev {
	struct device			*dev;
	struct class			battery_class;
	struct pmic_glink_client	*client;
	struct mutex			rw_lock;
	struct completion		ack;
	struct completion		fw_buf_ack;
	struct completion		fw_update_ack;
	struct psy_state		psy_list[PSY_TYPE_MAX];
	struct dentry			*debugfs_dir;
	void				*notifier_cookie;
	u32				*thermal_levels;
	const char			*wls_fw_name;
	int				curr_thermal_level;
	int				num_thermal_levels;
	atomic_t			state;
	struct work_struct		subsys_up_work;
	struct work_struct		usb_type_work;
#ifdef CONFIG_MACH_XIAOMI
	struct work_struct		notify_blankstate_work;
	int				blank_state;
#endif
	int				fake_soc;
	bool				block_tx;
	bool				ship_mode_en;
	bool				debug_battery_detected;
	bool				wls_fw_update_reqd;
	u32				wls_fw_version;
	u16				wls_fw_crc;
	struct notifier_block		reboot_notifier;
#ifdef CONFIG_MACH_XIAOMI
	struct notifier_block		shutdown_notifier;
	struct notifier_block 		blankstate_notifier;
#endif
	u32				thermal_fcc_ua;
	u32				restrict_fcc_ua;
	u32				last_fcc_ua;
	u32				usb_icl_ua;
#ifdef CONFIG_MACH_XIAOMI
	struct delayed_work		xm_prop_change_work;
	u8				*digest;
	u32				*ss_auth_data;
	u32				hw_version_build;
	/*shutdown delay is supported*/
	bool				shutdown_delay_en;
	/*dual battery authentic flag*/
	bool				slave_fg_verify_flag;
#endif
	bool				restrict_chg_en;
	/* To track the driver initialization status */
	bool				initialized;
	bool				notify_en;
};

static const int battery_prop_map[BATT_PROP_MAX] = {
	[BATT_STATUS]		= POWER_SUPPLY_PROP_STATUS,
	[BATT_HEALTH]		= POWER_SUPPLY_PROP_HEALTH,
	[BATT_PRESENT]		= POWER_SUPPLY_PROP_PRESENT,
	[BATT_CHG_TYPE]		= POWER_SUPPLY_PROP_CHARGE_TYPE,
	[BATT_CAPACITY]		= POWER_SUPPLY_PROP_CAPACITY,
	[BATT_VOLT_OCV]		= POWER_SUPPLY_PROP_VOLTAGE_OCV,
	[BATT_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[BATT_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[BATT_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[BATT_CHG_CTRL_LIM]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	[BATT_CHG_CTRL_LIM_MAX]	= POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
#ifdef CONFIG_MACH_XIAOMI
	[BATT_CONSTANT_CURRENT]	= POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
#endif
	[BATT_TEMP]		= POWER_SUPPLY_PROP_TEMP,
	[BATT_TECHNOLOGY]	= POWER_SUPPLY_PROP_TECHNOLOGY,
	[BATT_CHG_COUNTER]	= POWER_SUPPLY_PROP_CHARGE_COUNTER,
	[BATT_CYCLE_COUNT]	= POWER_SUPPLY_PROP_CYCLE_COUNT,
	[BATT_CHG_FULL_DESIGN]	= POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	[BATT_CHG_FULL]		= POWER_SUPPLY_PROP_CHARGE_FULL,
	[BATT_MODEL_NAME]	= POWER_SUPPLY_PROP_MODEL_NAME,
	[BATT_TTF_AVG]		= POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	[BATT_TTE_AVG]		= POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	[BATT_POWER_NOW]	= POWER_SUPPLY_PROP_POWER_NOW,
	[BATT_POWER_AVG]	= POWER_SUPPLY_PROP_POWER_AVG,
};

static const int usb_prop_map[USB_PROP_MAX] = {
	[USB_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[USB_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[USB_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[USB_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[USB_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
	[USB_INPUT_CURR_LIMIT]	= POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	[USB_ADAP_TYPE]		= POWER_SUPPLY_PROP_USB_TYPE,
	[USB_TEMP]		= POWER_SUPPLY_PROP_TEMP,
};

static const int wls_prop_map[WLS_PROP_MAX] = {
	[WLS_ONLINE]		= POWER_SUPPLY_PROP_ONLINE,
	[WLS_VOLT_NOW]		= POWER_SUPPLY_PROP_VOLTAGE_NOW,
	[WLS_VOLT_MAX]		= POWER_SUPPLY_PROP_VOLTAGE_MAX,
	[WLS_CURR_NOW]		= POWER_SUPPLY_PROP_CURRENT_NOW,
	[WLS_CURR_MAX]		= POWER_SUPPLY_PROP_CURRENT_MAX,
};

#ifdef CONFIG_MACH_XIAOMI
static const int xm_prop_map[XM_PROP_MAX] = { };
#endif

#ifndef CONFIG_MACH_XIAOMI
/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "SDP", "DCP", "CDP", "ACA", "C",
	"PD", "PD_DRP", "PD_PPS", "BrickID"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5"
};
#else
/* Standard usb_type definitions similar to power_supply_sysfs.c */
static const char * const power_supply_usb_type_text[] = {
	"Unknown", "USB", "USB_DCP", "USB_CDP", "USB_ACA", "USB_C",
	"USB_PD", "PD_DRP", "PD_PPS", "BrickID", "USB_HVDCP",
	"USB_HVDCP3","USB_HVDCP3P5", "USB_FLOAT"
};

/* Custom usb_type definitions */
static const char * const qc_power_supply_usb_type_text[] = {
	"HVDCP", "HVDCP_3", "HVDCP_3P5", "USB_FLOAT", "HVDCP_3"
};

static const char * const power_supply_usbc_text[] = {
	"Nothing attached",
	"Source attached (default current)",
	"Source attached (medium current)",
	"Source attached (high current)",
	"Non compliant",
	"Sink attached",
	"Powered cable w/ sink",
	"Debug Accessory",
	"Audio Adapter",
	"Powered cable w/o sink",
};

int StringToHex(char *str, unsigned char *out, unsigned int *outlen)
{
	char *p = str;
	char high = 0, low = 0;
	int tmplen = strlen(p), cnt = 0;

	tmplen = strlen(p);
	while(cnt < (tmplen / 2)) {
		high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
		low = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
		out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
		p ++;
		cnt ++;
	}

	if (tmplen % 2 != 0)
		out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;

	if (outlen != NULL)
		*outlen = tmplen / 2 + tmplen % 2;

	return tmplen / 2 + tmplen % 2;
}
#endif

static int battery_chg_fw_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return -ENOTCONN;
	}

	reinit_completion(&bcdev->fw_buf_ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->fw_buf_ack,
					msecs_to_jiffies(WLS_FW_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			return -ETIMEDOUT;
		}

		rc = 0;
	}

	return rc;
}

static int battery_chg_write(struct battery_chg_dev *bcdev, void *data,
				int len)
{
	int rc;

	/*
	 * When the subsystem goes down, it's better to return the last
	 * known values until it comes back up. Hence, return 0 so that
	 * pmic_glink_write() is not attempted until pmic glink is up.
	 */
	if (atomic_read(&bcdev->state) == PMIC_GLINK_STATE_DOWN) {
		pr_debug("glink state is down\n");
		return 0;
	}

	if (bcdev->debug_battery_detected && bcdev->block_tx)
		return 0;

	mutex_lock(&bcdev->rw_lock);
	reinit_completion(&bcdev->ack);
	rc = pmic_glink_write(bcdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bcdev->ack,
					msecs_to_jiffies(BC_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			mutex_unlock(&bcdev->rw_lock);
			return -ETIMEDOUT;
		}

		rc = 0;
	}
	mutex_unlock(&bcdev->rw_lock);

	return rc;
}

static int write_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32 val)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = val;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u val: %u\n", pst->psy->desc->name,
			req_msg.property_id, val);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct battery_charger_req_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.battery_id = 0;
	req_msg.value = 0;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	if (pst->psy)
		pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
			req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#ifdef CONFIG_MACH_XIAOMI
static int write_ss_auth_prop_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u32* buff)
{
	struct xm_ss_auth_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;
	memcpy(req_msg.data, buff, BATTERY_SS_AUTH_DATA_LEN*sizeof(u32));

	pr_debug("psy: prop_id:%d size:%d data[0]:0x%x data[1]:0x%x data[2]:0x%x data[3]:0x%x\n",
		req_msg.property_id, sizeof(req_msg), req_msg.data[0], req_msg.data[1], req_msg.data[2], req_msg.data[3]);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_ss_auth_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct xm_ss_auth_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;

	pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
		req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int write_verify_digest_prop_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id, u8* buff)
{
	struct xm_verify_digest_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_set;
	req_msg.slave_fg = bcdev->slave_fg_verify_flag;
	memcpy(req_msg.digest, buff, BATTERY_DIGEST_LEN);

	pr_debug("psy: prop_id:%d size:%d\n", req_msg.property_id, sizeof(req_msg));

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

static int read_verify_digest_property_id(struct battery_chg_dev *bcdev,
			struct psy_state *pst, u32 prop_id)
{
	struct xm_verify_digest_resp_msg req_msg = { { 0 } };

	req_msg.property_id = prop_id;
	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = pst->opcode_get;
	req_msg.slave_fg = bcdev->slave_fg_verify_flag;

	pr_debug("psy: %s prop_id: %u\n", pst->psy->desc->name,
		req_msg.property_id);

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}
#endif

static int get_property_id(struct psy_state *pst,
			enum power_supply_property prop)
{
	u32 i;

	for (i = 0; i < pst->prop_count; i++)
		if (pst->map[i] == prop)
			return i;

	if (pst->psy)
		pr_err("No property id for property %d in psy %s\n", prop,
			pst->psy->desc->name);

	return -ENOENT;
}

static void battery_chg_notify_disable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (bcdev->notify_en) {
		/* Send request to disable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_DISABLE_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0)
			pr_err("Failed to disable notification rc=%d\n", rc);
		else
			bcdev->notify_en = false;
	}
}

static void battery_chg_notify_enable(struct battery_chg_dev *bcdev)
{
	struct battery_charger_set_notify_msg req_msg = { { 0 } };
	int rc;

	if (!bcdev->notify_en) {
		/* Send request to enable notification */
		req_msg.hdr.owner = MSG_OWNER_BC;
		req_msg.hdr.type = MSG_TYPE_NOTIFY;
		req_msg.hdr.opcode = BC_SET_NOTIFY_REQ;

		rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
		if (rc < 0)
			pr_err("Failed to enable notification rc=%d\n", rc);
		else
			bcdev->notify_en = true;
	}
}

static void battery_chg_state_cb(void *priv, enum pmic_glink_state state)
{
	struct battery_chg_dev *bcdev = priv;

	pr_debug("state: %d\n", state);

	atomic_set(&bcdev->state, state);
	if (state == PMIC_GLINK_STATE_UP)
		schedule_work(&bcdev->subsys_up_work);
	else if (state == PMIC_GLINK_STATE_DOWN)
		bcdev->notify_en = false;
}

/**
 * qti_battery_charger_get_prop() - Gets the property being requested
 *
 * @name: Power supply name
 * @prop_id: Property id to be read
 * @val: Pointer to value that needs to be updated
 *
 * Return: 0 if success, negative on error.
 */
int qti_battery_charger_get_prop(const char *name,
				enum battery_charger_prop prop_id, int *val)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev;
	struct psy_state *pst;
	int rc = 0;

	if (prop_id >= BATTERY_CHARGER_PROP_MAX)
		return -EINVAL;

	if (strcmp(name, "battery") && strcmp(name, "usb") &&
	    strcmp(name, "wireless"))
		return -EINVAL;

	psy = power_supply_get_by_name(name);
	if (!psy)
		return -ENODEV;

	bcdev = power_supply_get_drvdata(psy);
	if (!bcdev)
		return -ENODEV;

	power_supply_put(psy);

	switch (prop_id) {
	case BATTERY_RESISTANCE:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
		if (!rc)
			*val = pst->prop[BATT_RESISTANCE];
		break;
	default:
		break;
	}

	return rc;
}
EXPORT_SYMBOL(qti_battery_charger_get_prop);

static bool validate_message(struct battery_charger_resp_msg *resp_msg,
				size_t len)
{
#ifdef CONFIG_MACH_XIAOMI
	struct xm_verify_digest_resp_msg *verify_digest_resp_msg = (struct xm_verify_digest_resp_msg *)resp_msg;
	struct xm_ss_auth_resp_msg *ss_auth_resp_msg = (struct xm_ss_auth_resp_msg *)resp_msg;

	if (len == sizeof(*verify_digest_resp_msg) || len == sizeof(*ss_auth_resp_msg))
		return true;
#endif

	if (len != sizeof(*resp_msg)) {
		pr_err("Incorrect response length %zu for opcode %#x\n", len,
			resp_msg->hdr.opcode);
		return false;
	}

	if (resp_msg->ret_code) {
		pr_err("Error in response for opcode %#x prop_id %u, rc=%d\n",
			resp_msg->hdr.opcode, resp_msg->property_id,
			(int)resp_msg->ret_code);
		return false;
	}

	return true;
}

#ifdef CONFIG_MACH_XIAOMI
extern struct power_supply_desc usb_psy_desc;
#endif

#define MODEL_DEBUG_BOARD	"Debug_Board"
static void handle_message(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_resp_msg *resp_msg = data;
	struct battery_model_resp_msg *model_resp_msg = data;
#ifdef CONFIG_MACH_XIAOMI
	struct xm_verify_digest_resp_msg *verify_digest_resp_msg = data;
	struct xm_ss_auth_resp_msg *ss_auth_resp_msg = data;
	struct wls_fw_resp_msg *wls_fw_ver_resp_msg = data;
#endif
	struct wireless_fw_check_resp *fw_check_msg;
	struct wireless_fw_push_buf_resp *fw_resp_msg;
	struct wireless_fw_update_status *fw_update_msg;
	struct wireless_fw_get_version_resp *fw_ver_msg;
	struct psy_state *pst;
	bool ack_set = false;

	switch (resp_msg->hdr.opcode) {
	case BC_BATTERY_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

		/* Handle model response uniquely as it's a string */
		if (pst->model && len == sizeof(*model_resp_msg)) {
			memcpy(pst->model, model_resp_msg->model, MAX_STR_LEN);
			ack_set = true;
			bcdev->debug_battery_detected = !strcmp(pst->model,
					MODEL_DEBUG_BOARD);
			break;
		}

		/* Other response should be of same type as they've u32 value */
		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
#ifdef CONFIG_MACH_XIAOMI
			if (resp_msg->property_id == USB_ADAP_TYPE) {
				if (resp_msg->value == ADAP_TYPE_DCP)
					usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
				else if (resp_msg->value == ADAP_TYPE_CDP)
					usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
				else if (resp_msg->value == ADAP_TYPE_PD)
					usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
				else if (resp_msg->value == ADAP_TYPE_SDP)
					usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
			}
#endif
			ack_set = true;
		}

		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];

#ifdef CONFIG_MACH_XIAOMI
		/* Handle model response uniquely as it's a string */
		if (pst->version && len == sizeof(*wls_fw_ver_resp_msg)) {
			memcpy(pst->version, wls_fw_ver_resp_msg->version, MAX_STR_LEN);
			ack_set = true;
			break;
		}
#endif

		if (validate_message(resp_msg, len) &&
		    resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
#ifdef CONFIG_MACH_XIAOMI
	case BC_XM_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_XM];

		/* Handle digest response uniquely as it's a string */
		if (bcdev->digest && len == sizeof(*verify_digest_resp_msg)) {
			memcpy(bcdev->digest, verify_digest_resp_msg->digest, BATTERY_DIGEST_LEN);
			ack_set = true;
			break;
		}
		if (bcdev->ss_auth_data && len == sizeof(*ss_auth_resp_msg)) {
			memcpy(bcdev->ss_auth_data, ss_auth_resp_msg->data, BATTERY_SS_AUTH_DATA_LEN*sizeof(u32));
			ack_set = true;
			break;
		}
		if (validate_message(resp_msg, len) &&
			resp_msg->property_id < pst->prop_count) {
			pst->prop[resp_msg->property_id] = resp_msg->value;
			ack_set = true;
		}

		break;
#endif
	case BC_BATTERY_STATUS_SET:
	case BC_USB_STATUS_SET:
	case BC_WLS_STATUS_SET:
#ifdef CONFIG_MACH_XIAOMI
	case BC_XM_STATUS_SET:
#endif
		if (validate_message(data, len))
			ack_set = true;

		break;
	case BC_SET_NOTIFY_REQ:
	case BC_DISABLE_NOTIFY_REQ:
	case BC_SHUTDOWN_NOTIFY:
	case BC_SHIP_MODE_REQ_SET:
		/* Always ACK response for notify or ship_mode request */
		ack_set = true;
		break;
	case BC_WLS_FW_CHECK_UPDATE:
		if (len == sizeof(*fw_check_msg)) {
			fw_check_msg = data;
			if (fw_check_msg->ret_code == 1)
				bcdev->wls_fw_update_reqd = true;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_check_update\n",
				len);
		}
		break;
	case BC_WLS_FW_PUSH_BUF_RESP:
		if (len == sizeof(*fw_resp_msg)) {
			fw_resp_msg = data;
			if (fw_resp_msg->fw_update_status == 1)
				complete(&bcdev->fw_buf_ack);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_push_buf_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_UPDATE_STATUS_RESP:
		if (len == sizeof(*fw_update_msg)) {
			fw_update_msg = data;
			if (fw_update_msg->fw_update_done == 1)
				complete(&bcdev->fw_update_ack);
			else
				pr_err("Wireless FW update not done %d\n",
					(int)fw_update_msg->fw_update_done);
		} else {
			pr_err("Incorrect response length %zu for wls_fw_update_status_resp\n",
				len);
		}
		break;
	case BC_WLS_FW_GET_VERSION:
		if (len == sizeof(*fw_ver_msg)) {
			fw_ver_msg = data;
			bcdev->wls_fw_version = fw_ver_msg->fw_version;
			ack_set = true;
		} else {
			pr_err("Incorrect response length %zu for wls_fw_get_version\n",
				len);
		}
		break;
	default:
		pr_err("Unknown opcode: %u\n", resp_msg->hdr.opcode);
		break;
	}

	if (ack_set)
		complete(&bcdev->ack);
}

#ifndef CONFIG_MACH_XIAOMI
static struct power_supply_desc usb_psy_desc;
#endif

static void battery_chg_update_usb_type_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, usb_type_work);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read USB_ADAP_TYPE rc=%d\n", rc);
		return;
	}

	/* Reset usb_icl_ua whenever USB adapter type changes */
	if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
	    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
		bcdev->usb_icl_ua = 0;

	pr_debug("usb_adap_type: %u\n", pst->prop[USB_ADAP_TYPE]);

	switch (pst->prop[USB_ADAP_TYPE]) {
	case POWER_SUPPLY_USB_TYPE_SDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
		break;
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3:
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5:
#ifdef CONFIG_MACH_XIAOMI
	case QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3_CLASSB:
#endif
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case POWER_SUPPLY_USB_TYPE_C:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_TYPE_C;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
	case POWER_SUPPLY_USB_TYPE_PD_DRP:
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB_PD;
		break;
	default:
#ifndef CONFIG_MACH_XIAOMI
		usb_psy_desc.type = POWER_SUPPLY_TYPE_USB;
#else
		usb_psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
		break;
	}
}

static void handle_notification(struct battery_chg_dev *bcdev, void *data,
				size_t len)
{
	struct battery_charger_notify_msg *notify_msg = data;
	struct psy_state *pst = NULL;

	if (len != sizeof(*notify_msg)) {
		pr_err("Incorrect response length %zu\n", len);
		return;
	}

	pr_debug("notification: %#x\n", notify_msg->notification);

	switch (notify_msg->notification) {
	case BC_BATTERY_STATUS_GET:
	case BC_GENERIC_NOTIFY:
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		break;
	case BC_USB_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_USB];
		schedule_work(&bcdev->usb_type_work);
		break;
	case BC_WLS_STATUS_GET:
		pst = &bcdev->psy_list[PSY_TYPE_WLS];
		break;
#ifdef CONFIG_MACH_XIAOMI
	case BC_XM_STATUS_GET:
		schedule_delayed_work(&bcdev->xm_prop_change_work, 0);
		break;
#endif
	default:
		break;
	}
	if (pst && pst->psy) {
		/*
		 * For charger mode, keep the device awake at least for 50 ms
		 * so that device won't enter suspend when a non-SDP charger
		 * is removed. This would allow the userspace process like
		 * "charger" to be able to read power supply uevents to take
		 * appropriate actions (e.g. shutting down when the charger is
		 * unplugged).
		 */
		power_supply_changed(pst->psy);
		pm_wakeup_dev_event(bcdev->dev, 50, true);
	}
}

static int battery_chg_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct battery_chg_dev *bcdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	if (!bcdev->initialized) {
		pr_debug("Driver initialization failed: Dropping glink callback message: state %d\n",
			 bcdev->state);
		return 0;
	}

	if (hdr->opcode == BC_NOTIFY_IND)
		handle_notification(bcdev, data, len);
	else
		handle_message(bcdev, data, len);

	return 0;
}

static int wls_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int prop_id, rc;

	pval->intval = -ENODATA;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];

	return 0;
}

static int wls_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	return 0;
}

static int wls_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	return 0;
}

static enum power_supply_property wls_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc wls_psy_desc = {
	.name			= "wireless",
	.type			= POWER_SUPPLY_TYPE_WIRELESS,
	.properties		= wls_props,
	.num_properties		= ARRAY_SIZE(wls_props),
	.get_property		= wls_psy_get_prop,
	.set_property		= wls_psy_set_prop,
	.property_is_writeable	= wls_psy_prop_is_writeable,
};

static const char *get_usb_type_name(u32 usb_type)
{
	u32 i;

	if (usb_type >= QTI_POWER_SUPPLY_USB_TYPE_HVDCP &&
#ifdef CONFIG_MACH_XIAOMI
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3_CLASSB) {
#else
	    usb_type <= QTI_POWER_SUPPLY_USB_TYPE_HVDCP_3P5) {
#endif
		for (i = 0; i < ARRAY_SIZE(qc_power_supply_usb_type_text);
		     i++) {
			if (i == (usb_type - QTI_POWER_SUPPLY_USB_TYPE_HVDCP))
				return qc_power_supply_usb_type_text[i];
		}
		return "Unknown";
	}

	for (i = 0; i < ARRAY_SIZE(power_supply_usb_type_text); i++) {
		if (i == usb_type)
			return power_supply_usb_type_text[i];
	}

	return "Unknown";
}

static int usb_psy_set_icl(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	u32 temp;
	int rc;

	rc = read_property_id(bcdev, pst, USB_ADAP_TYPE);
	if (rc < 0) {
		pr_err("Failed to read prop USB_ADAP_TYPE, rc=%d\n", rc);
		return rc;
	}

	/* Allow this only for SDP or USB_PD and not for other charger types */
	if (pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_SDP &&
	    pst->prop[USB_ADAP_TYPE] != POWER_SUPPLY_USB_TYPE_PD)
		return -EINVAL;

	/*
	 * Input current limit (ICL) can be set by different clients. E.g. USB
	 * driver can request for a current of 500/900 mA depending on the
	 * port type. Also, clients like EUD driver can pass 0 or -22 to
	 * suspend or unsuspend the input for its use case.
	 */

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (rc < 0) {
		pr_err("Failed to set ICL (%u uA) rc=%d\n", temp, rc);
	} else {
		pr_debug("Set ICL to %u\n", temp);
		bcdev->usb_icl_ua = temp;
	}

	return rc;
}

#ifdef CONFIG_MACH_XIAOMI
typedef enum {
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP2			= 0x80,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3			= 0x81,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3P5			= 0x82,
	POWER_SUPPLY_USB_REAL_TYPE_USB_FLOAT 		= 0x83,
	POWER_SUPPLY_USB_REAL_TYPE_HVDCP3_CLASSB	= 0x84,
} power_supply_usb_type;

struct quick_charge {
	int adap_type;
	enum power_supply_quick_charge_type adap_cap;
};

struct quick_charge adapter_cap[11] = {
	{ POWER_SUPPLY_USB_TYPE_SDP,	QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_DCP,	QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_CDP,	QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_ACA,	QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_REAL_TYPE_USB_FLOAT,	QUICK_CHARGE_NORMAL },
	{ POWER_SUPPLY_USB_TYPE_PD,		QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP2,	QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3,	QUICK_CHARGE_FAST },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3_CLASSB,	QUICK_CHARGE_FLASH },
	{ POWER_SUPPLY_USB_REAL_TYPE_HVDCP3P5,	QUICK_CHARGE_FLASH },
	{0, 0},
};

static u8 get_quick_charge_type(struct battery_chg_dev *bcdev)
{
	int i = 0, verify_digiest = 0;
	u8 rc;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	enum power_supply_usb_type real_charger_type = 0;
	int	batt_health;
	u32 apdo_max;

	rc = read_property_id(bcdev, pst, BATT_HEALTH);
	if (rc < 0)
		return rc;

	batt_health = pst->prop[BATT_HEALTH];

	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;

	real_charger_type = pst->prop[USB_REAL_TYPE];
	pst = &bcdev->psy_list[PSY_TYPE_XM];
	rc = read_property_id(bcdev, pst, XM_PROP_PD_VERIFED);
	verify_digiest = pst->prop[XM_PROP_PD_VERIFED];

	rc = read_property_id(bcdev, pst, XM_PROP_APDO_MAX);
	apdo_max =  pst->prop[XM_PROP_APDO_MAX];

	if ((batt_health == POWER_SUPPLY_HEALTH_COLD)
		|| (batt_health == POWER_SUPPLY_HEALTH_WARM)
		|| (batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
		|| (batt_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE))
		return QUICK_CHARGE_NORMAL;

	if (real_charger_type == POWER_SUPPLY_USB_TYPE_PD_PPS && verify_digiest == 1) {
		if (apdo_max >= 50)
			return QUICK_CHARGE_SUPER;
		else
			return QUICK_CHARGE_TURBE;
	} else if(real_charger_type == POWER_SUPPLY_USB_TYPE_PD_PPS)
		return QUICK_CHARGE_FAST;

	while (adapter_cap[i].adap_type != 0) {
		if (real_charger_type == adapter_cap[i].adap_type)
			return adapter_cap[i].adap_cap;

		i++;
	}

	return QUICK_CHARGE_NORMAL;
}

static int battery_psy_set_fcc(struct battery_chg_dev *bcdev, u32 prop_id, int val)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	u32 temp;
	int rc;

	temp = val;
	if (val < 0)
		temp = UINT_MAX;

	rc = write_property_id(bcdev, pst, prop_id, temp);
	if (!rc)
		pr_debug("Set FCC to %u\n", temp);

	return rc;
}
#endif

static int usb_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc;

	pval->intval = -ENODATA;
#ifdef CONFIG_MACH_XIAOMI
	if (prop == POWER_SUPPLY_PROP_QUICK_CHARGE_TYPE) {
		pval->intval = get_quick_charge_type(bcdev);
		return 0;
	}
#endif

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	pval->intval = pst->prop[prop_id];
	if (prop == POWER_SUPPLY_PROP_TEMP)
		pval->intval = DIV_ROUND_CLOSEST((int)pval->intval, 10);

	return 0;
}

static int usb_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		rc = usb_psy_set_icl(bcdev, prop_id, pval->intval);
		break;
	default:
		break;
	}

	return rc;
}

static int usb_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_MACH_XIAOMI
	POWER_SUPPLY_PROP_QUICK_CHARGE_TYPE,
#endif
};

static enum power_supply_usb_type usb_psy_supported_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_C,
	POWER_SUPPLY_USB_TYPE_PD,
	POWER_SUPPLY_USB_TYPE_PD_DRP,
	POWER_SUPPLY_USB_TYPE_PD_PPS,
	POWER_SUPPLY_USB_TYPE_APPLE_BRICK_ID,
};

#ifndef CONFIG_MACH_XIAOMI
static
#endif
struct power_supply_desc usb_psy_desc = {
	.name			= "usb",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= usb_props,
	.num_properties		= ARRAY_SIZE(usb_props),
	.get_property		= usb_psy_get_prop,
	.set_property		= usb_psy_set_prop,
	.usb_types		= usb_psy_supported_types,
	.num_usb_types		= ARRAY_SIZE(usb_psy_supported_types),
	.property_is_writeable	= usb_psy_prop_is_writeable,
};

static int __battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					u32 fcc_ua)
{
	int rc;

	if (bcdev->restrict_chg_en) {
		fcc_ua = min_t(u32, fcc_ua, bcdev->restrict_fcc_ua);
		fcc_ua = min_t(u32, fcc_ua, bcdev->thermal_fcc_ua);
	}

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, fcc_ua);
	if (rc < 0) {
		pr_err("Failed to set FCC %u, rc=%d\n", fcc_ua, rc);
	} else {
		pr_debug("Set FCC to %u uA\n", fcc_ua);
		bcdev->last_fcc_ua = fcc_ua;
	}

	return rc;
}

static int battery_psy_set_charge_current(struct battery_chg_dev *bcdev,
					int val)
{
	int rc;
#ifndef CONFIG_MACH_XIAOMI
	u32 fcc_ua, prev_fcc_ua;
#endif

	if (!bcdev->num_thermal_levels)
		return 0;

	if (bcdev->num_thermal_levels < 0) {
		pr_err("Incorrect num_thermal_levels\n");
		return -EINVAL;
	}

#ifdef CONFIG_MACH_XIAOMI
	pr_debug("set thermal-level: %d num_thermal_levels: %d \n", val, bcdev->num_thermal_levels);

	if (val < 0 || val >= bcdev->num_thermal_levels)
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_BATTERY],
				BATT_CHG_CTRL_LIM, val);
	if (rc < 0)
		pr_err("Failed to set ccl:%d, rc=%d\n", val, rc);

	bcdev->curr_thermal_level = val;
#else
	if (val < 0 || val > bcdev->num_thermal_levels)
		return -EINVAL;

	fcc_ua = bcdev->thermal_levels[val];
	prev_fcc_ua = bcdev->thermal_fcc_ua;
	bcdev->thermal_fcc_ua = fcc_ua;

	rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
	if (!rc)
		bcdev->curr_thermal_level = val;
	else
		bcdev->thermal_fcc_ua = prev_fcc_ua;
#endif

	return rc;
}

static int battery_psy_get_prop(struct power_supply *psy,
		enum power_supply_property prop,
		union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc;

	pval->intval = -ENODATA;

	/*
	 * The prop id of TIME_TO_FULL_NOW and TIME_TO_FULL_AVG is same.
	 * So, map the prop id of TIME_TO_FULL_AVG for TIME_TO_FULL_NOW.
	 */
	if (prop == POWER_SUPPLY_PROP_TIME_TO_FULL_NOW)
		prop = POWER_SUPPLY_PROP_TIME_TO_FULL_AVG;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;

	rc = read_property_id(bcdev, pst, prop_id);
	if (rc < 0)
		return rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		pval->strval = pst->model;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_BQ_FUEL_GAUGE
		pval->intval = pst->prop[prop_id] / 100;
#else
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
#endif
		if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) &&
		   (bcdev->fake_soc >= 0 && bcdev->fake_soc <= 100))
			pval->intval = bcdev->fake_soc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifdef CONFIG_BQ_FUEL_GAUGE
		pval->intval = pst->prop[prop_id];
		pval->intval = pval->intval / 10;
#else
		pval->intval = DIV_ROUND_CLOSEST((int)pst->prop[prop_id], 10);
#endif
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		pval->intval = bcdev->curr_thermal_level;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		pval->intval = bcdev->num_thermal_levels;
		break;
#ifdef CONFIG_MACH_XIAOMI
	// Ghost-riderreborn workaround for charge counter
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		pval->intval = DIV_ROUND_CLOSEST(pst->prop[prop_id], 100);
		break;
#endif
	default:
		pval->intval = pst->prop[prop_id];
		break;
	}

	return rc;
}

static int battery_psy_set_prop(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *pval)
{
	struct battery_chg_dev *bcdev = power_supply_get_drvdata(psy);
#ifdef CONFIG_MACH_XIAOMI
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int prop_id, rc = 0;

	prop_id = get_property_id(pst, prop);
	if (prop_id < 0)
		return prop_id;
#endif

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		return battery_psy_set_charge_current(bcdev, pval->intval);
#ifdef CONFIG_MACH_XIAOMI
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		rc = battery_psy_set_fcc(bcdev, prop_id, pval->intval);
		break;
#endif
	default:
		return -EINVAL;
	}

	return 0;
}

static int battery_psy_prop_is_writeable(struct power_supply *psy,
		enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
#ifdef CONFIG_MACH_XIAOMI
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
#endif
		return 1;
	default:
		break;
	}

	return 0;
}

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
#ifdef CONFIG_MACH_XIAOMI
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
#endif
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_POWER_AVG,
};

static const struct power_supply_desc batt_psy_desc = {
	.name			= "battery",
#ifdef CONFIG_BQ_FUEL_GAUGE
	.no_thermal		= true,
#endif
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= battery_props,
	.num_properties		= ARRAY_SIZE(battery_props),
	.get_property		= battery_psy_get_prop,
	.set_property		= battery_psy_set_prop,
	.property_is_writeable	= battery_psy_prop_is_writeable,
};

#ifdef CONFIG_BQ_FUEL_GAUGE
static int power_supply_read_temp(struct thermal_zone_device *tzd,
		int *temp)
{
	struct power_supply *psy;
	struct battery_chg_dev *bcdev = NULL;
	struct psy_state *pst = NULL;
	struct psy_state *batt_pst = NULL;
	int rc = 0, batt_temp;
	static int last_temp;
	ktime_t time_now;
	static ktime_t last_read_time;
	s64 delta;

	WARN_ON(tzd == NULL);
	psy = tzd->devdata;
	bcdev = power_supply_get_drvdata(psy);
	pst = &bcdev->psy_list[PSY_TYPE_XM];
	batt_pst = &bcdev->psy_list[PSY_TYPE_BATTERY];

	time_now = ktime_get();
	delta = ktime_ms_delta(time_now, last_read_time);
	if(delta < 5000)
		batt_temp = last_temp;
	else {
		rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_TEMP);
		batt_temp = pst->prop[XM_PROP_THERMAL_TEMP];
		last_temp = batt_temp;
		last_read_time = time_now;
	}

	*temp = batt_temp * 1000;
	pr_info("batt_thermal: temp: %d, delta: %ld, blank_state: %d, chg_type: %s, tl: %d,  ffc: %d, pd_verifed: %d, r: %d\n",
		batt_temp, delta, bcdev->blank_state, power_supply_usb_type_text[pst->prop[XM_PROP_REAL_TYPE]],
		bcdev->curr_thermal_level, pst->prop[XM_PROP_FASTCHGMODE], pst->prop[XM_PROP_PD_VERIFED],
		(batt_pst->prop[BATT_CHG_COUNTER]/(batt_pst->prop[BATT_CHG_FULL]/1000)));

	return 0;
}

static struct thermal_zone_device_ops psy_tzd_ops = {
	.get_temp = power_supply_read_temp,
};
#endif

static int battery_chg_init_psy(struct battery_chg_dev *bcdev)
{
	struct power_supply_config psy_cfg = {};
	int rc;
#ifdef CONFIG_BQ_FUEL_GAUGE
	struct power_supply *psy;
#endif

	psy_cfg.drv_data = bcdev;
	psy_cfg.of_node = bcdev->dev->of_node;
	bcdev->psy_list[PSY_TYPE_BATTERY].psy =
		devm_power_supply_register(bcdev->dev, &batt_psy_desc,
						&psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_BATTERY].psy);
		pr_err("Failed to register battery power supply, rc=%d\n", rc);
		return rc;
	}

#ifdef CONFIG_BQ_FUEL_GAUGE
	psy = bcdev->psy_list[PSY_TYPE_BATTERY].psy;
	psy->tzd = thermal_zone_device_register(psy->desc->name,
					0, 0, psy, &psy_tzd_ops, NULL, 0, 0);
#endif

	bcdev->psy_list[PSY_TYPE_USB].psy =
		devm_power_supply_register(bcdev->dev, &usb_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_USB].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_USB].psy);
		pr_err("Failed to register USB power supply, rc=%d\n", rc);
		return rc;
	}

	bcdev->psy_list[PSY_TYPE_WLS].psy =
		devm_power_supply_register(bcdev->dev, &wls_psy_desc, &psy_cfg);
	if (IS_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy)) {
		rc = PTR_ERR(bcdev->psy_list[PSY_TYPE_WLS].psy);
		pr_err("Failed to register wireless power supply, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static void battery_chg_subsys_up_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, subsys_up_work);
	int rc;

	battery_chg_notify_enable(bcdev);

	/*
	 * Give some time after enabling notification so that USB adapter type
	 * information can be obtained properly which is essential for setting
	 * USB ICL.
	 */
	msleep(200);

	if (bcdev->last_fcc_ua) {
		rc = __battery_psy_set_charge_current(bcdev,
				bcdev->last_fcc_ua);
		if (rc < 0)
			pr_err("Failed to set FCC (%u uA), rc=%d\n",
				bcdev->last_fcc_ua, rc);
	}

	if (bcdev->usb_icl_ua) {
		rc = usb_psy_set_icl(bcdev, USB_INPUT_CURR_LIMIT,
				bcdev->usb_icl_ua);
		if (rc < 0)
			pr_err("Failed to set ICL(%u uA), rc=%d\n",
				bcdev->usb_icl_ua, rc);
	}
}

static int wireless_fw_send_firmware(struct battery_chg_dev *bcdev,
					const struct firmware *fw)
{
	struct wireless_fw_push_buf_req msg = {};
	const u8 *ptr;
	u32 i, num_chunks, partial_chunk_size;
	int rc;

	num_chunks = fw->size / WLS_FW_BUF_SIZE;
	partial_chunk_size = fw->size % WLS_FW_BUF_SIZE;

	if (!num_chunks)
		return -EINVAL;

	pr_debug("Updating FW...\n");

	ptr = fw->data;
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_WLS_FW_PUSH_BUF_REQ;

	for (i = 0; i < num_chunks; i++, ptr += WLS_FW_BUF_SIZE) {
		msg.fw_chunk_id = i + 1;
		memcpy(msg.buf, ptr, WLS_FW_BUF_SIZE);

		pr_debug("sending FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	if (partial_chunk_size) {
		msg.fw_chunk_id = i + 1;
		memset(msg.buf, 0, WLS_FW_BUF_SIZE);
		memcpy(msg.buf, ptr, partial_chunk_size);

		pr_debug("sending partial FW chunk %u\n", i + 1);
		rc = battery_chg_fw_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			return rc;
	}

	return 0;
}

static int wireless_fw_check_for_update(struct battery_chg_dev *bcdev,
					u32 version, size_t size)
{
	struct wireless_fw_check_req req_msg = {};

	bcdev->wls_fw_update_reqd = false;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_CHECK_UPDATE;
	req_msg.fw_version = version;
	req_msg.fw_size = size;
	req_msg.fw_crc = bcdev->wls_fw_crc;

	return battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
}

#define IDT_FW_MAJOR_VER_OFFSET		0x94
#define IDT_FW_MINOR_VER_OFFSET		0x96
static int wireless_fw_update(struct battery_chg_dev *bcdev, bool force)
{
	const struct firmware *fw;
	struct psy_state *pst;
	u32 version;
	u16 maj_ver, min_ver;
	int rc;

	pm_stay_awake(bcdev->dev);

	/*
	 * Check for USB presence. If nothing is connected, check whether
	 * battery SOC is at least 50% before allowing FW update.
	 */
	pst = &bcdev->psy_list[PSY_TYPE_USB];
	rc = read_property_id(bcdev, pst, USB_ONLINE);
	if (rc < 0)
		goto out;

	if (!pst->prop[USB_ONLINE]) {
		pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
		rc = read_property_id(bcdev, pst, BATT_CAPACITY);
		if (rc < 0)
			goto out;

		if ((pst->prop[BATT_CAPACITY] / 100) < 50) {
			pr_err("Battery SOC should be at least 50%% or connect charger\n");
			rc = -EINVAL;
			goto out;
		}
	}

	rc = firmware_request_nowarn(&fw, bcdev->wls_fw_name, bcdev->dev);
	if (rc) {
		pr_err("Couldn't get firmware rc=%d\n", rc);
		goto out;
	}

	if (!fw || !fw->data || !fw->size) {
		pr_err("Invalid firmware\n");
		rc = -EINVAL;
		goto release_fw;
	}

	if (fw->size < SZ_16K) {
		pr_err("Invalid firmware size %zu\n", fw->size);
		rc = -EINVAL;
		goto release_fw;
	}

	maj_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MAJOR_VER_OFFSET));
	min_ver = le16_to_cpu(*(__le16 *)(fw->data + IDT_FW_MINOR_VER_OFFSET));
	version = maj_ver << 16 | min_ver;

	if (force)
		version = UINT_MAX;

	pr_debug("FW size: %zu version: %#x\n", fw->size, version);

	rc = wireless_fw_check_for_update(bcdev, version, fw->size);
	if (rc < 0) {
		pr_err("Wireless FW update not needed, rc=%d\n", rc);
		goto release_fw;
	}

	if (!bcdev->wls_fw_update_reqd) {
		pr_warn("Wireless FW update not required\n");
		goto release_fw;
	}

	/* Wait for IDT to be setup by charger firmware */
	msleep(WLS_FW_PREPARE_TIME_MS);

	reinit_completion(&bcdev->fw_update_ack);
	rc = wireless_fw_send_firmware(bcdev, fw);
	if (rc < 0) {
		pr_err("Failed to send FW chunk, rc=%d\n", rc);
		goto release_fw;
	}

	rc = wait_for_completion_timeout(&bcdev->fw_update_ack,
				msecs_to_jiffies(WLS_FW_UPDATE_TIME_MS));
	if (!rc) {
		pr_err("Error, timed out updating firmware\n");
		rc = -ETIMEDOUT;
		goto release_fw;
	} else {
		rc = 0;
	}

	pr_info("Wireless FW update done\n");

release_fw:
	bcdev->wls_fw_crc = 0;
	release_firmware(fw);
out:
	pm_relax(bcdev->dev);

	return rc;
}

static ssize_t wireless_fw_crc_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	u16 val;

	if (kstrtou16(buf, 0, &val) || !val)
		return -EINVAL;

	bcdev->wls_fw_crc = val;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_crc);

static ssize_t wireless_fw_version_show(struct class *c,
					struct class_attribute *attr,
					char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct wireless_fw_get_version_req req_msg = {};
	int rc;

	req_msg.hdr.owner = MSG_OWNER_BC;
	req_msg.hdr.type = MSG_TYPE_REQ_RESP;
	req_msg.hdr.opcode = BC_WLS_FW_GET_VERSION;

	rc = battery_chg_write(bcdev, &req_msg, sizeof(req_msg));
	if (rc < 0) {
		pr_err("Failed to get FW version rc=%d\n", rc);
		return rc;
	}

	return scnprintf(buf, PAGE_SIZE, "%#x\n", bcdev->wls_fw_version);
}
static CLASS_ATTR_RO(wireless_fw_version);

static ssize_t wireless_fw_force_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, true);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_force_update);

static ssize_t wireless_fw_update_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;
	int rc;

	if (kstrtobool(buf, &val) || !val)
		return -EINVAL;

	rc = wireless_fw_update(bcdev, false);
	if (rc < 0)
		return rc;

	return count;
}
static CLASS_ATTR_WO(wireless_fw_update);

static ssize_t usb_typec_compliant_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_TYPEC_COMPLIANT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			(int)pst->prop[USB_TYPEC_COMPLIANT]);
}
static CLASS_ATTR_RO(usb_typec_compliant);

static ssize_t usb_real_type_show(struct class *c,
				struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_REAL_TYPE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			get_usb_type_name(pst->prop[USB_REAL_TYPE]));
}
static CLASS_ATTR_RO(usb_real_type);

static ssize_t restrict_cur_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	u32 fcc_ua, prev_fcc_ua;

	if (kstrtou32(buf, 0, &fcc_ua) || fcc_ua > bcdev->thermal_fcc_ua)
		return -EINVAL;

	prev_fcc_ua = bcdev->restrict_fcc_ua;
	bcdev->restrict_fcc_ua = fcc_ua;
	if (bcdev->restrict_chg_en) {
		rc = __battery_psy_set_charge_current(bcdev, fcc_ua);
		if (rc < 0) {
			bcdev->restrict_fcc_ua = prev_fcc_ua;
			return rc;
		}
	}

	return count;
}

static ssize_t restrict_cur_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->restrict_fcc_ua);
}
static CLASS_ATTR_RW(restrict_cur);

static ssize_t restrict_chg_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->restrict_chg_en = val;
	rc = __battery_psy_set_charge_current(bcdev, bcdev->restrict_chg_en ?
			bcdev->restrict_fcc_ua : bcdev->thermal_fcc_ua);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t restrict_chg_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->restrict_chg_en);
}
static CLASS_ATTR_RW(restrict_chg);

static ssize_t fake_soc_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	bcdev->fake_soc = val;
	pr_debug("Set fake soc to %d\n", val);

	if (IS_ENABLED(CONFIG_QTI_PMIC_GLINK_CLIENT_DEBUG) && pst->psy)
		power_supply_changed(pst->psy);

	return count;
}

static ssize_t fake_soc_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->fake_soc);
}
static CLASS_ATTR_RW(fake_soc);

static ssize_t wireless_boost_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_WLS],
				WLS_BOOST_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t wireless_boost_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_WLS];
	int rc;

	rc = read_property_id(bcdev, pst, WLS_BOOST_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[WLS_BOOST_EN]);
}
static CLASS_ATTR_RW(wireless_boost_en);

static ssize_t moisture_detection_en_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_USB],
				USB_MOISTURE_DET_EN, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t moisture_detection_en_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_EN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_EN]);
}
static CLASS_ATTR_RW(moisture_detection_en);

static ssize_t moisture_detection_status_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_USB];
	int rc;

	rc = read_property_id(bcdev, pst, USB_MOISTURE_DET_STS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			pst->prop[USB_MOISTURE_DET_STS]);
}
static CLASS_ATTR_RO(moisture_detection_status);

static ssize_t resistance_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_RESISTANCE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[BATT_RESISTANCE]);
}
static CLASS_ATTR_RO(resistance);

static ssize_t soh_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
	int rc;

	rc = read_property_id(bcdev, pst, BATT_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[BATT_SOH]);
}
static CLASS_ATTR_RO(soh);

static ssize_t ship_mode_en_store(struct class *c, struct class_attribute *attr,
				const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
#ifdef CONFIG_MACH_XIAOMI
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	int rc = 0;
#endif

	if (kstrtobool(buf, &bcdev->ship_mode_en))
		return -EINVAL;

#ifdef CONFIG_MACH_XIAOMI
	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;
	rc = battery_chg_write(bcdev, &msg, sizeof(msg));
	if (rc < 0)
		pr_err("Failed to write ship mode: %d\n", rc);
#endif

	return count;
}

static ssize_t ship_mode_en_show(struct class *c, struct class_attribute *attr,
				char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%d\n", bcdev->ship_mode_en);
}
static CLASS_ATTR_RW(ship_mode_en);

#ifdef CONFIG_MACH_XIAOMI
static ssize_t real_type_show(struct class *c, struct class_attribute *attr,
			char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_REAL_TYPE);
	if (rc < 0)
		return rc;

	/* sanity check to avoid real_type value above maxium 13(USB_FLOAT) to cause kernel crash */
	if (pst->prop[XM_PROP_REAL_TYPE] > 13)
		pst->prop[XM_PROP_REAL_TYPE] = 0;

	return scnprintf(buf, PAGE_SIZE, "%s\n", power_supply_usb_type_text[pst->prop[XM_PROP_REAL_TYPE]]);
}
static CLASS_ATTR_RO(real_type);

static ssize_t resistance_id_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_RESISTANCE_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_RESISTANCE_ID]);
}
static CLASS_ATTR_RO(resistance_id);

static ssize_t verify_digest_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = 
			container_of(c, struct battery_chg_dev,	battery_class);
	u8 random[BATTERY_DIGEST_LEN] = {0};
	char kbuf[70] = {0};
	int rc;
	int i;

	memset(kbuf, 0, sizeof(kbuf));
	strncpy(kbuf, buf, count - 1);
	StringToHex(kbuf, random, &i);
	pr_err("verify_digest_store  1s:%s \n", random);
	rc = write_verify_digest_prop_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
		XM_PROP_VERIFY_DIGEST, random);

	if (rc < 0)
		return rc;

	return count;
}

static ssize_t verify_digest_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u8 digest_buf[4];
	int i;
	int len;

	rc = read_verify_digest_property_id(bcdev, pst, XM_PROP_VERIFY_DIGEST);
	if (rc < 0)
		return rc;

	for (i = 0; i < BATTERY_DIGEST_LEN; i++) {
		memset(digest_buf, 0, sizeof(digest_buf));
		snprintf(digest_buf, sizeof(digest_buf) - 1, "%02x", bcdev->digest[i]);
		strlcat(buf, digest_buf, BATTERY_DIGEST_LEN * 2 + 1);
	}
	len = strlen(buf);
	buf[len] = '\0';
	pr_debug("verify_digest_show: %s\n", buf);

	return strlen(buf) + 1;
}
static CLASS_ATTR_RW(verify_digest);

static ssize_t verify_slave_flag_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	bcdev->slave_fg_verify_flag = val;
	pr_debug("verify_digest_flag: %d\n", val);

	return count;
}
static ssize_t verify_slave_flag_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);

	return scnprintf(buf, PAGE_SIZE, "%u\n", bcdev->slave_fg_verify_flag);
}
static CLASS_ATTR_RW(verify_slave_flag);

static ssize_t connector_temp_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_CONNECTOR_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t connector_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CONNECTOR_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CONNECTOR_TEMP]);
}
static CLASS_ATTR_RW(connector_temp);

static ssize_t authentic_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	pr_debug("authentic_store: %d\n", val);
	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_AUTHENTIC, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t authentic_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_AUTHENTIC);
	if (rc < 0)
		return rc;

	pr_debug("authentic_show: %d\n", pst->prop[XM_PROP_AUTHENTIC]);
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_AUTHENTIC]);
}
static CLASS_ATTR_RW(authentic);

static ssize_t chip_ok_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CHIP_OK]);
}
static CLASS_ATTR_RO(chip_ok);

static ssize_t vbus_disable_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_VBUS_DISABLE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t vbus_disable_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_VBUS_DISABLE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_VBUS_DISABLE]);
}
static CLASS_ATTR_RW(vbus_disable);

static ssize_t cc_orientation_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CC_ORIENTATION);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CC_ORIENTATION]);
}
static CLASS_ATTR_RO(cc_orientation);

static ssize_t slave_batt_present_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SLAVE_BATT_PRESENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SLAVE_BATT_PRESENT]);
}
static CLASS_ATTR_RO(slave_batt_present);

#if defined(CONFIG_BQ2597X_BATTERY_CHARGER)
static ssize_t bq2597x_chip_ok_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_CHIP_OK]);
}
static CLASS_ATTR_RO(bq2597x_chip_ok);

static ssize_t bq2597x_slave_chip_ok_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_CHIP_OK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_CHIP_OK]);
}
static CLASS_ATTR_RO(bq2597x_slave_chip_ok);

static ssize_t bq2597x_bus_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_CURRENT]);
}
static CLASS_ATTR_RO(bq2597x_bus_current);

static ssize_t bq2597x_slave_bus_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_BUS_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_BUS_CURRENT]);
}
static CLASS_ATTR_RO(bq2597x_slave_bus_current);

static ssize_t bq2597x_bus_delta_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_DELTA);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_DELTA]);
}
static CLASS_ATTR_RO(bq2597x_bus_delta);

static ssize_t bq2597x_bus_voltage_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BUS_VOLTAGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BUS_VOLTAGE]);
}
static CLASS_ATTR_RO(bq2597x_bus_voltage);

static ssize_t bq2597x_battery_present_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BATTERY_PRESENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BATTERY_PRESENT]);
}
static CLASS_ATTR_RO(bq2597x_battery_present);

static ssize_t bq2597x_slave_battery_present_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_BATTERY_PRESENT]);
}
static CLASS_ATTR_RO(bq2597x_slave_battery_present);

static ssize_t bq2597x_battery_voltage_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_BATTERY_VOLTAGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_BATTERY_VOLTAGE]);
}
static CLASS_ATTR_RO(bq2597x_battery_voltage);

static ssize_t cool_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_COOL_MODE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t cool_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_COOL_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_COOL_MODE]);
}
static CLASS_ATTR_RW(cool_mode);
#endif

static ssize_t bq2597x_slave_connector_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_BQ2597X_SLAVE_CONNECTOR);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BQ2597X_SLAVE_CONNECTOR]);
}
static CLASS_ATTR_RO(bq2597x_slave_connector);

static ssize_t bt_transfer_start_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	pr_info("transfer_start_store %d build: 0x%x\n", val, bcdev->hw_version_build);

	switch (bcdev->hw_version_build){
		case 0x110001:
		case 0x10001:
		case 0x120000:
		case 0x20000:
		case 0x120005:
		case 0x120001:
		case 0x20001:
		case 0x90002:
			rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_BT_TRANSFER_START, val);
			break;
		default:
			break;
	}

	return count;
}

static ssize_t bt_transfer_start_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;


	rc = read_property_id(bcdev, pst, XM_PROP_BT_TRANSFER_START);
	if (rc < 0)
		return rc;

	pr_info("transfer_start_show %d\n", pst->prop[XM_PROP_BT_TRANSFER_START]);
	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_BT_TRANSFER_START]);
}
static CLASS_ATTR_RW(bt_transfer_start);

static ssize_t master_smb1396_online_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MASTER_SMB1396_ONLINE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_MASTER_SMB1396_ONLINE]);
}
static CLASS_ATTR_RO(master_smb1396_online);

static ssize_t master_smb1396_iin_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MASTER_SMB1396_IIN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_MASTER_SMB1396_IIN]);
}
static CLASS_ATTR_RO(master_smb1396_iin);

static ssize_t slave_smb1396_online_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SLAVE_SMB1396_ONLINE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SLAVE_SMB1396_ONLINE]);
}
static CLASS_ATTR_RO(slave_smb1396_online);

static ssize_t slave_smb1396_iin_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SLAVE_SMB1396_IIN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SLAVE_SMB1396_IIN]);
}
static CLASS_ATTR_RO(slave_smb1396_iin);

static ssize_t smb_iin_diff_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMB_IIN_DIFF);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMB_IIN_DIFF]);
}
static CLASS_ATTR_RO(smb_iin_diff);

static ssize_t soc_decimal_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SOC_DECIMAL);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SOC_DECIMAL]);
}
static CLASS_ATTR_RO(soc_decimal);

static ssize_t soc_decimal_rate_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SOC_DECIMAL_RATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SOC_DECIMAL_RATE]);
}
static CLASS_ATTR_RO(soc_decimal_rate);

static ssize_t shutdown_delay_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SHUTDOWN_DELAY);
	if (rc < 0)
		return rc;

	if (!bcdev->shutdown_delay_en)
		pst->prop[XM_PROP_SHUTDOWN_DELAY] = 0;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_SHUTDOWN_DELAY]);
}

static ssize_t shutdown_delay_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	bcdev->shutdown_delay_en = val;
	pr_info("use contral shutdown delay featue enable= %d\n", bcdev->shutdown_delay_en);

	return count;
}
static CLASS_ATTR_RW(shutdown_delay);

static ssize_t voter_debug_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_VOTER_DEBUG, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t voter_debug_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_VOTER_DEBUG);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u", pst->prop[XM_PROP_VOTER_DEBUG]);
}
static CLASS_ATTR_RW(voter_debug);

static ssize_t power_max_show(struct class *c,
			struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
				battery_class);
	struct psy_state *xm_pst = &bcdev->psy_list[PSY_TYPE_XM];
	union power_supply_propval val = {0, };
	struct power_supply *usb_psy = NULL;
	int rc, usb_present = 0;

	usb_psy = bcdev->psy_list[PSY_TYPE_USB].psy;
	if (usb_psy != NULL) {
		rc = usb_psy_get_prop(usb_psy, POWER_SUPPLY_PROP_ONLINE, &val);
		if (!rc)
		      usb_present = val.intval;
		else
		      usb_present = 0;
		pr_err("usb_present: %d\n", usb_present);
	}

	if (usb_present) {
		rc = read_property_id(bcdev, xm_pst, XM_PROP_APDO_MAX);
		if (rc < 0)
		      return rc;
		return scnprintf(buf, PAGE_SIZE, "%u", xm_pst->prop[XM_PROP_APDO_MAX]);
	}
	pr_err("tx_adapter:%d\n", xm_pst->prop[XM_PROP_TX_ADAPTER]);

	return scnprintf(buf, PAGE_SIZE, "%u", 0);
}
static CLASS_ATTR_RO(power_max);

static ssize_t input_suspend_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	pr_info("set charger input suspend %d\n", val);

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_INPUT_SUSPEND, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t input_suspend_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_INPUT_SUSPEND);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_INPUT_SUSPEND]);
}
static CLASS_ATTR_RW(input_suspend);

static ssize_t night_charging_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	pr_err("set charger night charging %d\n", val);

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_NIGHT_CHARGING, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t night_charging_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_NIGHT_CHARGING);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_NIGHT_CHARGING]);
}
static CLASS_ATTR_RW(night_charging);

static ssize_t smart_batt_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	pr_err("set smart batt charging %d\n", val);

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SMART_BATT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t smart_batt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SMART_BATT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SMART_BATT]);
}
static CLASS_ATTR_RW(smart_batt);

static ssize_t verify_process_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_VERIFY_PROCESS, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t verify_process_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_VERIFY_PROCESS);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_VERIFY_PROCESS]);
}
static CLASS_ATTR_RW(verify_process);

#define BSWAP_32(x) \
	(u32)((((u32)(x) & 0xff000000) >> 24) | \
			(((u32)(x) & 0x00ff0000) >> 8) | \
			(((u32)(x) & 0x0000ff00) << 8) | \
			(((u32)(x) & 0x000000ff) << 24))

static void usbpd_sha256_bitswap32(unsigned int *array, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		array[i] = BSWAP_32(array[i]);
	}
}

static void usbpd_request_vdm_cmd(struct battery_chg_dev *bcdev, enum uvdm_state cmd, unsigned int *data)
{
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	u32 prop_id, val = 0;
	int rc;

	pr_err("usbpd_request_vdm_cmd:cmd = %d, data = %d\n", cmd, *data);
	switch (cmd) {
	case USBPD_UVDM_CHARGER_VERSION:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VOLTAGE;
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		prop_id = XM_PROP_VDM_CMD_CHARGER_TEMP;
		break;
	case USBPD_UVDM_SESSION_SEED:
		prop_id = XM_PROP_VDM_CMD_SESSION_SEED;
		usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
		val = *data;
		pr_info("SESSION_SEED: data = %d\n", val);
		break;
	case USBPD_UVDM_AUTHENTICATION:
		prop_id = XM_PROP_VDM_CMD_AUTHENTICATION;
		usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
		val = *data;
		pr_info("AUTHENTICATION: data = %d\n", val);
		break;
	case USBPD_UVDM_REVERSE_AUTHEN:
		prop_id = XM_PROP_VDM_CMD_REVERSE_AUTHEN;
		usbpd_sha256_bitswap32(data, USBPD_UVDM_SS_LEN);
		val = *data;
		pr_info("AUTHENTICATION: data = %d\n", val);
		break;
	case USBPD_UVDM_REMOVE_COMPENSATION:
		prop_id = XM_PROP_VDM_CMD_REMOVE_COMPENSATION;
		val = *data;
		break;
	case USBPD_UVDM_VERIFIED:
		prop_id = XM_PROP_VDM_CMD_VERIFIED;
		val = *data;
		break;
	default:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		pr_info("cmd:%d is not support\n", cmd);
		break;
	}

	if (cmd == USBPD_UVDM_SESSION_SEED
		|| cmd == USBPD_UVDM_AUTHENTICATION
		|| cmd == USBPD_UVDM_REVERSE_AUTHEN) {
		rc = write_ss_auth_prop_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				prop_id, data);
	} else
		rc = write_property_id(bcdev, pst, prop_id, val);
}

static ssize_t request_vdm_cmd_store(struct class *c,
					struct class_attribute *attr, const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int cmd, ret;
	unsigned char buffer[64];
	unsigned char data[32];
	int ccount;

	ret = sscanf(buf, "%d,%s\n", &cmd, buffer);

	pr_info("%s:buf:%s cmd:%d, buffer:%s\n", __func__, buf, cmd, buffer);

	StringToHex(buffer, data, &ccount);
	usbpd_request_vdm_cmd(bcdev, cmd, (unsigned int *)data);
	return count;
}

static ssize_t request_vdm_cmd_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	u32 prop_id = 0;
	int i;
	char data[16], str_buf[128] = {0};
	enum uvdm_state cmd;

	rc = read_property_id(bcdev, pst, XM_PROP_UVDM_STATE);
	if (rc < 0)
		return rc;

	cmd = pst->prop[XM_PROP_UVDM_STATE];
	pr_info("request_vdm_cmd_show uvdm_state: %d\n", cmd);

	switch (cmd){
	case USBPD_UVDM_CHARGER_VERSION:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VERSION;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
		break;
	case USBPD_UVDM_CHARGER_TEMP:
		prop_id = XM_PROP_VDM_CMD_CHARGER_TEMP;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
		break;
	case USBPD_UVDM_CHARGER_VOLTAGE:
		prop_id = XM_PROP_VDM_CMD_CHARGER_VOLTAGE;
		rc = read_property_id(bcdev, pst, prop_id);
		return snprintf(buf, PAGE_SIZE, "%d,%d", cmd, pst->prop[prop_id]);
		break;
	case USBPD_UVDM_CONNECT:
	case USBPD_UVDM_DISCONNECT:
	case USBPD_UVDM_SESSION_SEED:
	case USBPD_UVDM_VERIFIED:
	case USBPD_UVDM_REMOVE_COMPENSATION:
	case USBPD_UVDM_REVERSE_AUTHEN:
		return snprintf(buf, PAGE_SIZE, "%d,Null", cmd);
		break;
	case USBPD_UVDM_AUTHENTICATION:
		prop_id = XM_PROP_VDM_CMD_AUTHENTICATION;
		rc = read_ss_auth_property_id(bcdev, pst, prop_id);
		if (rc < 0)
			return rc;
		pr_info("auth:0x%x 0x%x 0x%x 0x%x\n",
			bcdev->ss_auth_data[0],bcdev->ss_auth_data[1],bcdev->ss_auth_data[2],bcdev->ss_auth_data[3]);
		for (i = 0; i < USBPD_UVDM_SS_LEN; i++) {
			memset(data, 0, sizeof(data));
			snprintf(data, sizeof(data), "%08lx", bcdev->ss_auth_data[i]);
			strlcat(str_buf, data, sizeof(str_buf));
		}
		return snprintf(buf, PAGE_SIZE, "%d,%s", cmd, str_buf);
		break;
	default:
		pr_info("feedbak cmd:%d is not support\n", cmd);
		break;
	}

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[prop_id]);
}
static CLASS_ATTR_RW(request_vdm_cmd);

static const char * const usbpd_state_strings[] = {
	"UNKNOWN",
	"SNK_Startup",
	"SNK_Ready",
	"SRC_Ready",
};

static ssize_t current_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CURRENT_STATE);
	if (rc < 0)
		return rc;
	if (pst->prop[XM_PROP_CURRENT_STATE] == 25)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[1]);
	else if (pst->prop[XM_PROP_CURRENT_STATE] == 31)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[2]);
	else if (pst->prop[XM_PROP_CURRENT_STATE] == 5)
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[3]);
	else
		return snprintf(buf, PAGE_SIZE, "%s", usbpd_state_strings[0]);

}
static CLASS_ATTR_RO(current_state);

static ssize_t adapter_id_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ADAPTER_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%08x", pst->prop[XM_PROP_ADAPTER_ID]);
}
static CLASS_ATTR_RO(adapter_id);

static ssize_t adapter_svid_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ADAPTER_SVID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%04x", pst->prop[XM_PROP_ADAPTER_SVID]);
}
static CLASS_ATTR_RO(adapter_svid);

static ssize_t pd_verifed_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	bool val;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_PD_VERIFED, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t pd_verifed_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PD_VERIFED);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_PD_VERIFED]);
}
static CLASS_ATTR_RW(pd_verifed);

static ssize_t pdo2_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_PDO2);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%08x\n", pst->prop[XM_PROP_PDO2]);
}
static CLASS_ATTR_RO(pdo2);

static ssize_t fastchg_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FASTCHGMODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FASTCHGMODE]);
}
static CLASS_ATTR_RO(fastchg_mode);

static ssize_t apdo_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_APDO_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_APDO_MAX]);
}
static CLASS_ATTR_RO(apdo_max);

static ssize_t thermal_remove_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_THERMAL_REMOVE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t thermal_remove_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_THERMAL_REMOVE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_THERMAL_REMOVE]);
}
static CLASS_ATTR_RW(thermal_remove);

static ssize_t fg_rm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG_RM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FG_RM]);
}
static CLASS_ATTR_RO(fg_rm);


static ssize_t mtbf_current_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_MTBF_CURRENT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t mtbf_current_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_MTBF_CURRENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_MTBF_CURRENT]);
}
static CLASS_ATTR_RW(mtbf_current);

static ssize_t fake_temp_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FAKE_TEMP, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fake_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FAKE_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FAKE_TEMP]);
}
static CLASS_ATTR_RW(fake_temp);

static ssize_t qbg_vbat_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_QBG_VBAT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_QBG_VBAT]);
}
static CLASS_ATTR_RO(qbg_vbat);

static ssize_t vph_pwr_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_QBG_VPH_PWR);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_QBG_VPH_PWR]);
}
static CLASS_ATTR_RO(vph_pwr);

static ssize_t qbg_temp_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_QBG_TEMP);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_QBG_TEMP]);
}
static CLASS_ATTR_RO(qbg_temp);

static ssize_t typec_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_TYPEC_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%s\n", power_supply_usbc_text[pst->prop[XM_PROP_TYPEC_MODE]]);
}
static CLASS_ATTR_RO(typec_mode);

static ssize_t fg1_qmax_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_QMAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_QMAX]);
}
static CLASS_ATTR_RO(fg1_qmax);

static ssize_t fg1_rm_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_RM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_RM]);
}
static CLASS_ATTR_RO(fg1_rm);

static ssize_t fg1_fcc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FCC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FCC]);
}
static CLASS_ATTR_RO(fg1_fcc);

static ssize_t fg1_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_SOH]);
}
static CLASS_ATTR_RO(fg1_soh);

#ifdef CONFIG_AI_RSOC
static ssize_t fg1_rsoc_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_RSOC);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_RSOC]);
}
static CLASS_ATTR_RO(fg1_rsoc);

static ssize_t fg1_ai_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_AI);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_AI]);
}
static CLASS_ATTR_RO(fg1_ai);
#endif

static ssize_t fg1_seal_set_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	pr_err("seal set %d\n", val);

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_FG1_SEAL_SET, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t fg1_seal_set_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SEAL_SET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_FG1_SEAL_SET]);
}
static CLASS_ATTR_RW(fg1_seal_set);

static ssize_t fg1_seal_state_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_SEAL_STATE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_SEAL_STATE]);
}
static CLASS_ATTR_RO(fg1_seal_state);

static ssize_t fg1_df_check_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_DF_CHECK);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_DF_CHECK]);
}
static CLASS_ATTR_RO(fg1_df_check);

static ssize_t fg1_voltage_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_VOLTAGE_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_VOLTAGE_MAX]);
}
static CLASS_ATTR_RO(fg1_voltage_max);

static ssize_t fg1_charge_current_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_Charge_Current_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_Charge_Current_MAX]);
}
static CLASS_ATTR_RO(fg1_charge_current_max);

static ssize_t fg1_discharge_current_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_Discharge_Current_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_Discharge_Current_MAX]);
}
static CLASS_ATTR_RO(fg1_discharge_current_max);

static ssize_t fg1_temp_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TEMP_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TEMP_MAX]);
}
static CLASS_ATTR_RO(fg1_temp_max);

static ssize_t fg1_temp_min_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TEMP_MIN);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TEMP_MIN]);
}
static CLASS_ATTR_RO(fg1_temp_min);

static ssize_t fg1_time_ht_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_HT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_HT]);
}
static CLASS_ATTR_RO(fg1_time_ht);

static ssize_t fg1_time_ot_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_OT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_OT]);
}
static CLASS_ATTR_RO(fg1_time_ot);

static ssize_t fg1_time_ut_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_UT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_UT]);
}
static CLASS_ATTR_RO(fg1_time_ut);

static ssize_t fg1_time_lt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TIME_LT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TIME_LT]);
}
static CLASS_ATTR_RO(fg1_time_lt);

static ssize_t fg1_fcc_soh_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FCC_SOH);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FCC_SOH]);
}
static CLASS_ATTR_RO(fg1_fcc_soh);

static ssize_t fg1_cycle_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_CYCLE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_CYCLE]);
}
static CLASS_ATTR_RO(fg1_cycle);

static ssize_t fg1_fastcharge_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_FAST_CHARGE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_FAST_CHARGE]);
}
static CLASS_ATTR_RO(fg1_fastcharge);

static ssize_t fg1_current_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_CURRENT_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_CURRENT_MAX]);
}
static CLASS_ATTR_RO(fg1_current_max);

static ssize_t fg1_vol_max_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_VOL_MAX);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_VOL_MAX]);
}
static CLASS_ATTR_RO(fg1_vol_max);

static ssize_t fg1_tsim_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TSIM);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TSIM]);
}
static CLASS_ATTR_RO(fg1_tsim);

static ssize_t fg1_tambient_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TAMBIENT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TAMBIENT]);
}
static CLASS_ATTR_RO(fg1_tambient);

static ssize_t fg1_tremq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TREMQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TREMQ]);
}
static CLASS_ATTR_RO(fg1_tremq);

static ssize_t fg1_tfullq_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG1_TFULLQ);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG1_TFULLQ]);
}
static CLASS_ATTR_RO(fg1_tfullq);

#if defined(CONFIG_BQ_CLOUD_AUTHENTICATION)
static ssize_t server_sn_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;
	int test[8] = {0};
	int i = 0;

	for(i = 0; i < 8; i++) {
		rc = read_property_id(bcdev, pst, XM_PROP_SERVER_SN);
		if (rc < 0)
			return rc;

		test[i] = pst->prop[XM_PROP_SERVER_SN];
	}

	return scnprintf(buf, PAGE_SIZE, "0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x 0x%0x\n", 
		(test[0]>>24)&0xff, (test[0]>>16)&0xff, (test[0]>>8)&0xff, (test[0]>>0)&0xff,
		(test[1]>>24)&0xff, (test[1]>>16)&0xff, (test[1]>>8)&0xff, (test[1]>>0)&0xff,
		(test[2]>>24)&0xff, (test[2]>>16)&0xff, (test[2]>>8)&0xff, (test[2]>>0)&0xff,
		(test[3]>>24)&0xff, (test[3]>>16)&0xff, (test[3]>>8)&0xff, (test[3]>>0)&0xff,
		(test[4]>>24)&0xff, (test[4]>>16)&0xff, (test[4]>>8)&0xff, (test[4]>>0)&0xff,
		(test[5]>>24)&0xff, (test[5]>>16)&0xff, (test[5]>>8)&0xff, (test[5]>>0)&0xff,
		(test[6]>>24)&0xff, (test[6]>>16)&0xff, (test[6]>>8)&0xff, (test[6]>>0)&0xff,
		(test[7]>>24)&0xff, (test[7]>>16)&0xff, (test[7]>>8)&0xff, (test[7]>>0)&0xff);
}
static CLASS_ATTR_RO(server_sn);

static ssize_t server_result_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	bool val;
	int rc;

	if (kstrtobool(buf, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, pst, XM_PROP_SERVER_RESULT, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t server_result_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SERVER_RESULT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_SERVER_RESULT]);
}
static CLASS_ATTR_RW(server_result);

static ssize_t adsp_result_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_ADSP_RESULT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_ADSP_RESULT]);
}
static CLASS_ATTR_RO(adsp_result);
#endif


static ssize_t shipmode_count_reset_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SHIPMODE_COUNT_RESET, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t shipmode_count_reset_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SHIPMODE_COUNT_RESET);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SHIPMODE_COUNT_RESET]);
}
static CLASS_ATTR_RW(shipmode_count_reset);


static ssize_t sport_mode_store(struct class *c,
					struct class_attribute *attr,
					const char *buf, size_t count)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	int rc;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
				XM_PROP_SPORT_MODE, val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t sport_mode_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_SPORT_MODE);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_SPORT_MODE]);
}
static CLASS_ATTR_RW(sport_mode);

static ssize_t cell1_volt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CELL1_VOLT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CELL1_VOLT]);
}
static CLASS_ATTR_RO(cell1_volt);

static ssize_t cell2_volt_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_CELL2_VOLT);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%u\n", pst->prop[XM_PROP_CELL2_VOLT]);
}
static CLASS_ATTR_RO(cell2_volt);

static ssize_t fg_vendor_show(struct class *c,
					struct class_attribute *attr, char *buf)
{
	struct battery_chg_dev *bcdev = container_of(c, struct battery_chg_dev,
						battery_class);
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_XM];
	int rc;

	rc = read_property_id(bcdev, pst, XM_PROP_FG_VENDOR_ID);
	if (rc < 0)
		return rc;

	return scnprintf(buf, PAGE_SIZE, "%d\n", pst->prop[XM_PROP_FG_VENDOR_ID]);
}
static CLASS_ATTR_RO(fg_vendor);
#endif

static struct attribute *battery_class_attrs[] = {
	&class_attr_soh.attr,
	&class_attr_resistance.attr,
	&class_attr_moisture_detection_status.attr,
	&class_attr_moisture_detection_en.attr,
	&class_attr_wireless_boost_en.attr,
	&class_attr_fake_soc.attr,
	&class_attr_wireless_fw_update.attr,
	&class_attr_wireless_fw_force_update.attr,
	&class_attr_wireless_fw_version.attr,
	&class_attr_wireless_fw_crc.attr,
	&class_attr_ship_mode_en.attr,
#ifdef CONFIG_MACH_XIAOMI
	&class_attr_real_type.attr,
	&class_attr_resistance_id.attr,
	&class_attr_verify_digest.attr,
	&class_attr_verify_slave_flag.attr,
	&class_attr_connector_temp.attr,
	&class_attr_authentic.attr,
	&class_attr_chip_ok.attr,
	&class_attr_vbus_disable.attr,
	&class_attr_cc_orientation.attr,
	&class_attr_slave_batt_present.attr,
#if defined(CONFIG_BQ2597X_BATTERY_CHARGER)
	&class_attr_bq2597x_chip_ok.attr,
	&class_attr_bq2597x_slave_chip_ok.attr,
	&class_attr_bq2597x_bus_current.attr,
	&class_attr_bq2597x_slave_bus_current.attr,
	&class_attr_bq2597x_bus_delta.attr,
	&class_attr_bq2597x_bus_voltage.attr,
	&class_attr_bq2597x_battery_present.attr,
	&class_attr_bq2597x_slave_battery_present.attr,
	&class_attr_bq2597x_battery_voltage.attr,
	&class_attr_cool_mode.attr,
#endif
	&class_attr_bq2597x_slave_connector.attr,
	&class_attr_bt_transfer_start.attr,
	&class_attr_master_smb1396_online.attr,
	&class_attr_master_smb1396_iin.attr,
	&class_attr_slave_smb1396_online.attr,
	&class_attr_slave_smb1396_iin.attr,
	&class_attr_smb_iin_diff.attr,
	&class_attr_soc_decimal.attr,
	&class_attr_shutdown_delay.attr,
	&class_attr_soc_decimal_rate.attr,
	/*****************************/
	&class_attr_input_suspend.attr,
	&class_attr_verify_process.attr,
	&class_attr_request_vdm_cmd.attr,
	&class_attr_current_state.attr,
	&class_attr_adapter_id.attr,
	&class_attr_adapter_svid.attr,
	&class_attr_pd_verifed.attr,
	&class_attr_pdo2.attr,
#endif
	&class_attr_restrict_chg.attr,
	&class_attr_restrict_cur.attr,
	&class_attr_usb_real_type.attr,
	&class_attr_usb_typec_compliant.attr,
#ifdef CONFIG_MACH_XIAOMI
	&class_attr_fastchg_mode.attr,
	&class_attr_apdo_max.attr,
	&class_attr_thermal_remove.attr,
	&class_attr_voter_debug.attr,
	&class_attr_fg_rm.attr,
	&class_attr_mtbf_current.attr,
	&class_attr_fake_temp.attr,
	&class_attr_qbg_vbat.attr,
	&class_attr_vph_pwr.attr,
	&class_attr_qbg_temp.attr,
	&class_attr_typec_mode.attr,
	&class_attr_night_charging.attr,
	&class_attr_smart_batt.attr,
	&class_attr_fg1_qmax.attr,
	&class_attr_fg1_rm.attr,
	&class_attr_fg1_fcc.attr,
	&class_attr_fg1_soh.attr,
#ifdef CONFIG_AI_RSOC
	&class_attr_fg1_rsoc.attr,
	&class_attr_fg1_ai.attr,
#endif
	&class_attr_fg1_fcc_soh.attr,
	&class_attr_fg1_cycle.attr,
	&class_attr_fg1_fastcharge.attr,
	&class_attr_fg1_current_max.attr,
	&class_attr_fg1_vol_max.attr,
	&class_attr_fg1_tsim.attr,
	&class_attr_fg1_tambient.attr,
	&class_attr_fg1_tremq.attr,
	&class_attr_fg1_tfullq.attr,
	&class_attr_fg1_seal_state.attr,
	&class_attr_fg1_seal_set.attr,
	&class_attr_fg1_df_check.attr,
	&class_attr_fg1_voltage_max.attr,
	&class_attr_fg1_charge_current_max.attr,
	&class_attr_fg1_discharge_current_max.attr,
	&class_attr_fg1_temp_max.attr,
	&class_attr_fg1_temp_min.attr,
	&class_attr_fg1_time_ht.attr,
	&class_attr_fg1_time_ot.attr,
	&class_attr_fg1_time_ut.attr,
	&class_attr_fg1_time_lt.attr,
#if defined(CONFIG_BQ_CLOUD_AUTHENTICATION)
	&class_attr_server_sn.attr,
	&class_attr_server_result.attr,
	&class_attr_adsp_result.attr,
#endif
	&class_attr_shipmode_count_reset.attr,
	&class_attr_sport_mode.attr,
	&class_attr_power_max.attr,
	&class_attr_cell1_volt.attr,
	&class_attr_cell2_volt.attr,
	&class_attr_fg_vendor.attr,
#endif
	NULL,
};
ATTRIBUTE_GROUPS(battery_class);

#ifdef CONFIG_DEBUG_FS
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev)
{
	int rc;
	struct dentry *dir;

	dir = debugfs_create_dir("battery_charger", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create charger debugfs directory, rc=%d\n",
			rc);
		return;
	}

	bcdev->debugfs_dir = dir;
	debugfs_create_bool("block_tx", 0600, dir, &bcdev->block_tx);
}
#else
static void battery_chg_add_debugfs(struct battery_chg_dev *bcdev) { }
#endif

#ifdef CONFIG_MACH_XIAOMI
#define MAX_UEVENT_LENGTH 50
static void generate_xm_charge_uvent(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work, struct battery_chg_dev, xm_prop_change_work.work);

	static char uevent_string[][MAX_UEVENT_LENGTH+1] = {
		"POWER_SUPPLY_SOC_DECIMAL=\n",	//length=31+8
		"POWER_SUPPLY_SOC_DECIMAL_RATE=\n",	//length=31+8
		"POWER_SUPPLY_SHUTDOWN_DELAY=\n",//28+8
	};
	static char *envp[] = {
		uevent_string[0],
		uevent_string[1],
		uevent_string[2],
		NULL,

	};
	char *prop_buf = NULL;

	prop_buf = (char *)get_zeroed_page(GFP_KERNEL);
	if (!prop_buf)
		return;

	soc_decimal_show( &(bcdev->battery_class), NULL, prop_buf);
	strncpy( uevent_string[0]+25, prop_buf,MAX_UEVENT_LENGTH-25);

	soc_decimal_rate_show( &(bcdev->battery_class), NULL, prop_buf);
	strncpy( uevent_string[1]+30, prop_buf,MAX_UEVENT_LENGTH-30);

	shutdown_delay_show( &(bcdev->battery_class), NULL, prop_buf);
	strncpy( uevent_string[2]+28, prop_buf,MAX_UEVENT_LENGTH-28);

	dev_err(bcdev->dev,"uevent test : %s\n %s\n %s\n",
			envp[0], envp[1], envp[2]);

	/*add our prop end*/

	kobject_uevent_env(&bcdev->dev->kobj, KOBJ_CHANGE, envp);

	free_page((unsigned long)prop_buf);
	return;
}
#endif

static int battery_chg_parse_dt(struct battery_chg_dev *bcdev)
{
	struct device_node *node = bcdev->dev->of_node;
	struct psy_state *pst = &bcdev->psy_list[PSY_TYPE_BATTERY];
#ifndef CONFIG_MACH_XIAOMI
	int i, rc, len;
	u32 prev, val;
#else
	int rc, len;
#endif


	of_property_read_string(node, "qcom,wireless-fw-name",
				&bcdev->wls_fw_name);

	rc = of_property_count_elems_of_size(node, "qcom,thermal-mitigation",
						sizeof(u32));
	if (rc <= 0)
		return 0;

	len = rc;

#ifndef CONFIG_MACH_XIAOMI
	rc = read_property_id(bcdev, pst, BATT_CHG_CTRL_LIM_MAX);
	if (rc < 0) {
		pr_err("Failed to read prop BATT_CHG_CTRL_LIM_MAX, rc=%d\n",
			rc);
		return rc;
	}

	prev = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	for (i = 0; i < len; i++) {
		rc = of_property_read_u32_index(node, "qcom,thermal-mitigation",
						i, &val);
		if (rc < 0)
			return rc;

		if (val > prev) {
			pr_err("Thermal levels should be in descending order\n");
			bcdev->num_thermal_levels = -EINVAL;
			return 0;
		}

		prev = val;
	}
#endif

	bcdev->thermal_levels = devm_kcalloc(bcdev->dev, len + 1,
					sizeof(*bcdev->thermal_levels),
					GFP_KERNEL);
	if (!bcdev->thermal_levels)
		return -ENOMEM;

	/*
	 * Element 0 is for normal charging current. Elements from index 1
	 * onwards is for thermal mitigation charging currents.
	 */

	bcdev->thermal_levels[0] = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	rc = of_property_read_u32_array(node, "qcom,thermal-mitigation",
					&bcdev->thermal_levels[1], len);
	if (rc < 0) {
		pr_err("Error in reading qcom,thermal-mitigation, rc=%d\n", rc);
		return rc;
	}

#ifndef CONFIG_MACH_XIAOMI
	bcdev->num_thermal_levels = len;
#else
	bcdev->num_thermal_levels = MAX_THERMAL_LEVEL;
#endif
	bcdev->thermal_fcc_ua = pst->prop[BATT_CHG_CTRL_LIM_MAX];

	return 0;
}

static int battery_chg_ship_mode(struct notifier_block *nb, unsigned long code,
		void *unused)
{
#ifndef CONFIG_MACH_XIAOMI
	struct battery_charger_notify_msg msg_notify = { { 0 } };
#endif
	struct battery_charger_ship_mode_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     reboot_notifier);
	int rc;

#ifndef CONFIG_MACH_XIAOMI
	msg_notify.hdr.owner = MSG_OWNER_BC;
	msg_notify.hdr.type = MSG_TYPE_NOTIFY;
	msg_notify.hdr.opcode = BC_SHUTDOWN_NOTIFY;

	rc = battery_chg_write(bcdev, &msg_notify, sizeof(msg_notify));
	if (rc < 0)
		pr_err("Failed to send shutdown notification rc=%d\n", rc);
#endif

	if (!bcdev->ship_mode_en)
		return NOTIFY_DONE;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHIP_MODE_REQ_SET;
	msg.ship_mode_type = SHIP_MODE_PMIC;

	if (code == SYS_POWER_OFF) {
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}

#ifdef CONFIG_MACH_XIAOMI
static int battery_chg_shutdown(struct notifier_block *nb, unsigned long code,
		void *unused)
{
	struct battery_charger_shutdown_req_msg msg = { { 0 } };
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     shutdown_notifier);
	int rc;

	msg.hdr.owner = MSG_OWNER_BC;
	msg.hdr.type = MSG_TYPE_REQ_RESP;
	msg.hdr.opcode = BC_SHUTDOWN_REQ_SET;

	if (code == SYS_POWER_OFF || code == SYS_RESTART) {
		pr_err("start adsp shutdown\n");
		rc = battery_chg_write(bcdev, &msg, sizeof(msg));
		if (rc < 0)
			pr_emerg("Failed to write ship mode: %d\n", rc);
	}

	return NOTIFY_DONE;
}

static void notify_blankstate_changed_work(struct work_struct *work)
{
	struct battery_chg_dev *bcdev = container_of(work,
					struct battery_chg_dev, notify_blankstate_work);
	int rc;

	rc = write_property_id(bcdev, &bcdev->psy_list[PSY_TYPE_XM],
			XM_PROP_FB_BLANK_STATE, bcdev->blank_state);
	if (rc < 0)
		pr_err("%s:write BLANK_STATE failed\n", __func__);

	pr_debug("%s:write BLANK_STATE succeed\n", __func__);
}

static int mi_disp_notifier_callback(struct notifier_block *nb,
				unsigned long val, void *data)
{
	struct battery_chg_dev *bcdev = container_of(nb, struct battery_chg_dev,
						     blankstate_notifier);
	struct mi_disp_notifier *evdata = data;
	unsigned int blank;

	if (val != MI_DISP_DPMS_EVENT)
		return NOTIFY_OK;

	if (evdata && evdata->data && bcdev) {
		blank = *(int *)(evdata->data);
		pr_debug("val:%lu, blank:%u\n", val, blank);

		bcdev->blank_state = (blank == MI_DISP_DPMS_ON) ? 0 : 1;

		schedule_work(&bcdev->notify_blankstate_work);
	}
	return NOTIFY_OK;
}

#endif

static void panel_event_notifier_callback(enum panel_event_notifier_tag tag,
			struct panel_event_notification *notification, void *data)
{
	struct battery_chg_dev *bcdev = data;

	if (!notification) {
		pr_debug("Invalid panel notification\n");
		return;
	}

	pr_debug("panel event received, type: %d\n", notification->notif_type);
	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_BLANK:
		battery_chg_notify_disable(bcdev);
		break;
	case DRM_PANEL_EVENT_UNBLANK:
		battery_chg_notify_enable(bcdev);
		break;
	default:
		pr_debug("Ignore panel event: %d\n", notification->notif_type);
		break;
	}
}

static int battery_chg_register_panel_notifier(struct battery_chg_dev *bcdev)
{
	struct device_node *np = bcdev->dev->of_node;
	struct device_node *pnode;
	struct drm_panel *panel, *active_panel = NULL;
	void *cookie = NULL;
	int i, count, rc;

	count = of_count_phandle_with_args(np, "qcom,display-panels", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		pnode = of_parse_phandle(np, "qcom,display-panels", i);
		if (!pnode)
			return -ENODEV;

		panel = of_drm_find_panel(pnode);
		of_node_put(pnode);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			break;
		}
	}

	if (!active_panel) {
		rc = PTR_ERR(panel);
		if (rc != -EPROBE_DEFER)
			dev_err(bcdev->dev, "Failed to find active panel, rc=%d\n");
		return rc;
	}

	cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_BATTERY_CHARGER,
			active_panel,
			panel_event_notifier_callback,
			(void *)bcdev);
	if (IS_ERR(cookie)) {
		rc = PTR_ERR(cookie);
		dev_err(bcdev->dev, "Failed to register panel event notifier, rc=%d\n", rc);
		return rc;
	}

	pr_debug("register panel notifier successful\n");
	bcdev->notifier_cookie = cookie;
	return 0;
}

static int battery_chg_probe(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev;
	struct device *dev = &pdev->dev;
	struct pmic_glink_client_data client_data = { };
	int rc, i;

	bcdev = devm_kzalloc(&pdev->dev, sizeof(*bcdev), GFP_KERNEL);
	if (!bcdev)
		return -ENOMEM;

	bcdev->psy_list[PSY_TYPE_BATTERY].map = battery_prop_map;
	bcdev->psy_list[PSY_TYPE_BATTERY].prop_count = BATT_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_get = BC_BATTERY_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_BATTERY].opcode_set = BC_BATTERY_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_USB].map = usb_prop_map;
	bcdev->psy_list[PSY_TYPE_USB].prop_count = USB_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_USB].opcode_get = BC_USB_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_USB].opcode_set = BC_USB_STATUS_SET;
	bcdev->psy_list[PSY_TYPE_WLS].map = wls_prop_map;
	bcdev->psy_list[PSY_TYPE_WLS].prop_count = WLS_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_get = BC_WLS_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_WLS].opcode_set = BC_WLS_STATUS_SET;
#ifdef CONFIG_MACH_XIAOMI
	bcdev->psy_list[PSY_TYPE_XM].map = xm_prop_map;
	bcdev->psy_list[PSY_TYPE_XM].prop_count = XM_PROP_MAX;
	bcdev->psy_list[PSY_TYPE_XM].opcode_get = BC_XM_STATUS_GET;
	bcdev->psy_list[PSY_TYPE_XM].opcode_set = BC_XM_STATUS_SET;
#endif

	for (i = 0; i < PSY_TYPE_MAX; i++) {
		bcdev->psy_list[i].prop =
			devm_kcalloc(&pdev->dev, bcdev->psy_list[i].prop_count,
					sizeof(u32), GFP_KERNEL);
		if (!bcdev->psy_list[i].prop)
			return -ENOMEM;
	}

	bcdev->psy_list[PSY_TYPE_BATTERY].model =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
	if (!bcdev->psy_list[PSY_TYPE_BATTERY].model)
		return -ENOMEM;
#ifdef CONFIG_MACH_XIAOMI
	bcdev->digest=
		devm_kzalloc(&pdev->dev, BATTERY_DIGEST_LEN, GFP_KERNEL);
	if (!bcdev->digest)
		return -ENOMEM;
	bcdev->ss_auth_data=
		devm_kzalloc(&pdev->dev, BATTERY_SS_AUTH_DATA_LEN * sizeof(u32), GFP_KERNEL);
	if (!bcdev->ss_auth_data)
		return -ENOMEM;

	bcdev->psy_list[PSY_TYPE_WLS].version =
		devm_kzalloc(&pdev->dev, MAX_STR_LEN, GFP_KERNEL);
#endif

	mutex_init(&bcdev->rw_lock);
	init_completion(&bcdev->ack);
	init_completion(&bcdev->fw_buf_ack);
	init_completion(&bcdev->fw_update_ack);
	INIT_WORK(&bcdev->subsys_up_work, battery_chg_subsys_up_work);
	INIT_WORK(&bcdev->usb_type_work, battery_chg_update_usb_type_work);
#ifdef CONFIG_MACH_XIAOMI
	INIT_WORK(&bcdev->notify_blankstate_work, notify_blankstate_changed_work);
#endif
	atomic_set(&bcdev->state, PMIC_GLINK_STATE_UP);
	bcdev->dev = dev;

	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_charger";
	client_data.msg_cb = battery_chg_callback;
	client_data.priv = bcdev;
	client_data.state_cb = battery_chg_state_cb;

	bcdev->client = pmic_glink_register_client(dev, &client_data);
	if (IS_ERR(bcdev->client)) {
		rc = PTR_ERR(bcdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}

	bcdev->initialized = true;
	bcdev->reboot_notifier.notifier_call = battery_chg_ship_mode;
	bcdev->reboot_notifier.priority = 255;
	register_reboot_notifier(&bcdev->reboot_notifier);

#ifdef CONFIG_MACH_XIAOMI
	/* register shutdown notifier to do something before shutdown */
	bcdev->shutdown_notifier.notifier_call = battery_chg_shutdown;
	bcdev->shutdown_notifier.priority = 255;
	register_reboot_notifier(&bcdev->shutdown_notifier);

	bcdev->blankstate_notifier.notifier_call = mi_disp_notifier_callback;
	rc = mi_disp_register_client(&bcdev->blankstate_notifier);
	if (rc < 0) {
		dev_err(dev, "Failed to register disp notifier rc=%d\n", rc);
		goto error;
	}

#endif

	rc = battery_chg_parse_dt(bcdev);
	if (rc < 0) {
		dev_err(dev, "Failed to parse dt rc=%d\n", rc);
		goto error;
	}

	rc = battery_chg_register_panel_notifier(bcdev);
	if (rc < 0)
		goto error;

	bcdev->restrict_fcc_ua = DEFAULT_RESTRICT_FCC_UA;
	platform_set_drvdata(pdev, bcdev);
	bcdev->fake_soc = -EINVAL;
	rc = battery_chg_init_psy(bcdev);
	if (rc < 0)
		goto error;

	bcdev->battery_class.name = "qcom-battery";
	bcdev->battery_class.class_groups = battery_class_groups;
	rc = class_register(&bcdev->battery_class);
	if (rc < 0) {
		dev_err(dev, "Failed to create battery_class rc=%d\n", rc);
		goto error;
	}

	battery_chg_add_debugfs(bcdev);
	bcdev->notify_en = false;
	battery_chg_notify_enable(bcdev);
	device_init_wakeup(bcdev->dev, true);
	schedule_work(&bcdev->usb_type_work);

#ifdef CONFIG_MACH_XIAOMI
	INIT_DELAYED_WORK(&bcdev->xm_prop_change_work, generate_xm_charge_uvent);

	bcdev->slave_fg_verify_flag = false;
	bcdev->shutdown_delay_en = true;
	bcdev->hw_version_build = 0;
#endif
#ifdef CONFIG_BQ_FUEL_GAUGE
	bcdev->hw_version_build = get_hw_id_value();
#endif

	return 0;
error:
	bcdev->initialized = false;
	complete(&bcdev->ack);
	pmic_glink_unregister_client(bcdev->client);
#ifdef CONFIG_MACH_XIAOMI
	mi_disp_unregister_client(&bcdev->blankstate_notifier);
	unregister_reboot_notifier(&bcdev->shutdown_notifier);
#endif
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	return rc;
}

static int battery_chg_remove(struct platform_device *pdev)
{
	struct battery_chg_dev *bcdev = platform_get_drvdata(pdev);
	int rc;

	if (bcdev->notifier_cookie)
		panel_event_notifier_unregister(bcdev->notifier_cookie);

	device_init_wakeup(bcdev->dev, false);
	debugfs_remove_recursive(bcdev->debugfs_dir);
	class_unregister(&bcdev->battery_class);
#ifdef CONFIG_MACH_XIAOMI
	mi_disp_unregister_client(&bcdev->blankstate_notifier);
	unregister_reboot_notifier(&bcdev->shutdown_notifier);
#endif
	unregister_reboot_notifier(&bcdev->reboot_notifier);
	rc = pmic_glink_unregister_client(bcdev->client);
	if (rc < 0) {
		pr_err("Error unregistering from pmic_glink, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static const struct of_device_id battery_chg_match_table[] = {
	{ .compatible = "qcom,battery-charger" },
	{},
};

static struct platform_driver battery_chg_driver = {
	.driver = {
		.name = "qti_battery_charger",
		.of_match_table = battery_chg_match_table,
	},
	.probe = battery_chg_probe,
	.remove = battery_chg_remove,
};
module_platform_driver(battery_chg_driver);

MODULE_DESCRIPTION("QTI Glink battery charger driver");
MODULE_LICENSE("GPL v2");
