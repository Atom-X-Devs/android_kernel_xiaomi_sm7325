// SPDX-License-Identifier: GPL-2.0-only

#ifndef __XIAOMI__TOUCH_H
#define __XIAOMI__TOUCH_H

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/mempolicy.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/rtc.h>
#include <linux/seq_file.h>

/*CUR,DEFAULT,MIN,MAX*/
#define VALUE_TYPE_SIZE 6
#define VALUE_GRIP_SIZE 9
#define MAX_BUF_SIZE 256
#define BTN_INFO 0x152
#define MAX_TOUCH_ID 10
#define RAW_BUF_NUM 4

enum MODE_CMD {
	SET_CUR_VALUE = 0,
	GET_CUR_VALUE,
	GET_DEF_VALUE,
	GET_MIN_VALUE,
	GET_MAX_VALUE,
	GET_MODE_VALUE,
	RESET_MODE,
	SET_LONG_VALUE,
};

enum MODE_TYPE {
	Touch_Game_Mode				= 0,
	Touch_Active_MODE      		= 1,
	Touch_UP_THRESHOLD			= 2,
	Touch_Tolerance				= 3,
	Touch_Aim_Sensitivity       = 4,
	Touch_Tap_Stability         = 5,
	Touch_Expert_Mode           = 6,
	Touch_Edge_Filter      		= 7,
	Touch_Panel_Orientation 	= 8,
	Touch_Report_Rate      		= 9,
	Touch_Fod_Enable       		= 10,
	Touch_Aod_Enable       		= 11,
	Touch_Resist_RF        		= 12,
	Touch_Idle_Time        		= 13,
	Touch_Doubletap_Mode   		= 14,
	Touch_Grip_Mode        		= 15,
	Touch_FodIcon_Enable   		= 16,
	Touch_Nonui_Mode       		= 17,
	Touch_Debug_Level      		= 18,
	Touch_Power_Status     		= 19,
	Touch_Mode_NUM         		= 20,
};

struct xiaomi_touch_interface {
	int thp_cmd_buf[MAX_BUF_SIZE];
	int thp_cmd_size;
	int touch_mode[Touch_Mode_NUM][VALUE_TYPE_SIZE];
	int (*setModeValue)(int Mode, int value);
	int (*getModeValue)(int Mode, int value_type);
	int (*getModeAll)(int Mode, int *modevalue);
	int (*resetMode)(int Mode);
	int (*palm_sensor_write)(int on);
	u8 (*panel_vendor_read)(void);
	u8 (*panel_color_read)(void);
	u8 (*panel_display_read)(void);
	char (*touch_vendor_read)(void);

	int thp_downthreshold;
	int thp_upthreshold;
	int thp_movethreshold;
	int thp_noisefilter;
	int thp_islandthreshold;
	int thp_smooth;
	int thp_dump_raw;
};

struct xiaomi_touch {
	struct miscdevice 	misc_dev;
	struct device *dev;
	struct class *class;
	struct attribute_group attrs;
	struct mutex  mutex;
	struct mutex  palm_mutex;
	struct mutex  prox_mutex;
	wait_queue_head_t 	wait_queue;
};

#define LAST_TOUCH_EVENTS_MAX 512

enum touch_state {
	EVENT_INIT,
	EVENT_DOWN,
	EVENT_UP,
};

struct touch_event {
	u32 slot;
	enum touch_state state;
	struct timespec64 touch_time;
};

struct last_touch_event {
	int head;
	struct touch_event touch_event_buf[LAST_TOUCH_EVENTS_MAX];
};

struct xiaomi_touch_pdata{
	struct xiaomi_touch *device;
	struct xiaomi_touch_interface *touch_data[2];
	int suspend_state;
	dma_addr_t phy_base;
	int raw_head;
	int raw_tail;
	int raw_len;
	unsigned int *raw_buf[RAW_BUF_NUM];
	unsigned int *raw_data;
	spinlock_t raw_lock;
	int palm_value;
	bool palm_changed;
	int prox_value;
	bool prox_changed;
	struct proc_dir_entry  *last_touch_events_proc;
	struct last_touch_event *last_touch_events;
};

struct xiaomi_touch *xiaomi_touch_dev_get(int minor);

extern struct class *get_xiaomi_touch_class(void);

extern int update_palm_sensor_value(int value);

extern int xiaomitouch_register_modedata(int touchId, struct xiaomi_touch_interface *data);

extern void last_touch_events_collect(int slot, int state);


#endif
