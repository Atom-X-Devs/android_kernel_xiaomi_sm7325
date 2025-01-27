/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, Focaltech Ltd. All rights reserved.
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
/*****************************************************************************
*
* File Name: focaltech_core.h

* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract:
*
* Reference:
*
*****************************************************************************/

#ifndef __LINUX_FOCALTECH_CORE_H__
#define __LINUX_FOCALTECH_CORE_H__
/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include "focaltech_common.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_MAX_TOUCH_BUF 					4096
#define FTS_MAX_POINTS_SUPPORT              10 /* constant value, can't be changed */
#define FTS_MAX_KEYS                        4
#define FTS_KEY_DIM                         10
#define FTS_ONE_TCH_LEN                     6
#define FTS_TOUCH_DATA_LEN  (FTS_MAX_POINTS_SUPPORT * FTS_ONE_TCH_LEN + 3)

#define FTS_GESTURE_POINTS_MAX              6
#define FTS_GESTURE_DATA_LEN               (FTS_GESTURE_POINTS_MAX * 4 + 4)

#define FTS_MAX_ID                          0x0A
#define FTS_TOUCH_OFF_E_XH 					0
#define FTS_TOUCH_OFF_XL 					1
#define FTS_TOUCH_OFF_ID_YH 				2
#define FTS_TOUCH_OFF_YL 					3
#define FTS_TOUCH_OFF_PRE 					4
#define FTS_TOUCH_OFF_AREA 					5
#define FTS_TOUCH_E_NUM 					1
#define FTS_COORDS_ARR_SIZE                 4
#define FTS_X_MIN_DISPLAY_DEFAULT           0
#define FTS_Y_MIN_DISPLAY_DEFAULT           0
#define FTS_X_MAX_DISPLAY_DEFAULT           720
#define FTS_Y_MAX_DISPLAY_DEFAULT           1280

#define FTS_TOUCH_DOWN                      0
#define FTS_TOUCH_UP                        1
#define FTS_TOUCH_CONTACT                   2
#define EVENT_DOWN(flag)                    ((FTS_TOUCH_DOWN == flag) || (FTS_TOUCH_CONTACT == flag))
#define EVENT_UP(flag)                      (FTS_TOUCH_UP == flag)
#define EVENT_NO_DOWN(data)                 (!data->point_num)

#define FTX_MAX_COMPATIBLE_TYPE             4
#define FTX_MAX_COMMMAND_LENGTH             16


/*****************************************************************************
*  Alternative mode (When something goes wrong, the modules may be able to solve the problem.)
*****************************************************************************/
/*
 * For commnication error in PM(deep sleep) state
 */
#define FTS_PATCH_COMERR_PM                     1
#define FTS_TIMEOUT_COMERR_PM                   700

#define FTS_TOUCHSCREEN_FOD

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/
struct ftxxxx_proc {
	struct proc_dir_entry *proc_entry;
	u8 opmode;
	u8 cmd_len;
	u8 cmd[FTX_MAX_COMMMAND_LENGTH];
};

struct fts_ts_platform_data {
	u32 type;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	bool have_key;
	u32 key_number;
	u32 keys[FTS_MAX_KEYS];
	u32 key_y_coords[FTS_MAX_KEYS];
	u32 key_x_coords[FTS_MAX_KEYS];
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 max_touch_number;
	u32 super_resolution_factor;
};

struct ts_event {
	int x;      /*x coordinate */
	int y;      /*y coordinate */
	int p;      /* pressure */
	int flag;   /* touch event flag: 0 -- down; 1-- up; 2 -- contact */
	int id;     /*touch ID */
	int area;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct spi_device *spi;
	struct device *dev;
	struct input_dev *input_dev;
	struct fts_ts_platform_data *pdata;
	struct ts_ic_info ic_info;
	struct workqueue_struct *ts_workqueue;
	struct work_struct fwupg_work;
	struct delayed_work esdcheck_work;
	struct delayed_work prc_work;
	struct notifier_block power_supply_notifier;
	struct work_struct power_supply_work;
	struct work_struct resume_work;
	struct work_struct suspend_work;
	wait_queue_head_t ts_waitqueue;

	struct ftxxxx_proc proc;
	spinlock_t irq_lock;
	struct mutex report_mutex;
	struct mutex bus_lock;
	int irq;
	int log_level;
	int fw_is_running;      /* confirm fw is running when using spi:default 0 */
	int dummy_byte;
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	struct completion pm_completion;
	bool pm_suspend;
#endif
	bool suspended;
	bool fw_loading;
	bool irq_disabled;
	bool power_disabled;
	bool glove_mode;
	bool cover_mode;
	bool charger_mode;
	bool gesture_mode;      /* gesture enable or disable, default: disable */
	bool poweroff_on_sleep;
	int report_rate;
	int charger_status;
	u8 gesture_bmode; /*gesture buffer mode*/

	/* multi-touch */
	int bus_type;
	u8 *bus_tx_buf;
	u8 *bus_rx_buf;
	u8 gesture_status;
	u8 *touch_buf;
	u8 touch_addr;
	u32 touch_size;
	void *notifier_cookie;
	int pnt_buf_size;
	int touchs;
	int key_state;
	int touch_point;
	int point_num;
#ifdef FTS_TOUCHSCREEN_FOD
	u8 old_point_id;
	int overlap_area;
	bool finger_in_fod;
	bool fod_finger_skip;
	bool point_id_changed;
#endif
	struct ts_event *events;
	struct regulator *avdd;
	struct regulator *iovdd;
#if FTS_PINCTRL_EN
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_suspend;
	struct pinctrl_state *pins_release;
#endif
#if defined(CONFIG_DRM)
	struct notifier_block fb_notif;
#endif
};

enum _FTS_BUS_TYPE {
	BUS_TYPE_NONE,
	BUS_TYPE_I2C,
	BUS_TYPE_SPI,
	BUS_TYPE_SPI_V2,
};

enum _FTS_TOUCH_ETYPE {
	TOUCH_DEFAULT = 0x00,
	TOUCH_EVENT_NUM = 0x02,
	TOUCH_EXTRA_MSG = 0x08,
	TOUCH_PEN = 0x0B,
	TOUCH_GESTURE = 0x80,
	TOUCH_FW_INIT = 0x81,
	TOUCH_IGNORE = 0xFE,
	TOUCH_ERROR = 0xFF,
};

enum _FTS_GESTURE_BMODE {
	GESTURE_BM_REG,
	GESTURE_BM_TOUCH,
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct fts_ts_data *fts_data;

/* communication interface */
int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen);
int fts_read_reg(u8 addr, u8 *value);
int fts_write(u8 *writebuf, u32 writelen);
int fts_write_reg(u8 addr, u8 value);
void fts_hid2std(void);
int fts_bus_init(struct fts_ts_data *ts_data);
int fts_bus_exit(struct fts_ts_data *ts_data);

/* Gesture functions */
int fts_gesture_init(struct fts_ts_data *ts_data);
int fts_gesture_exit(struct fts_ts_data *ts_data);
void fts_gesture_recovery(struct fts_ts_data *ts_data);
int fts_gesture_readdata(struct fts_ts_data *ts_data, u8 *data);
int fts_gesture_suspend(struct fts_ts_data *ts_data);
int fts_gesture_resume(struct fts_ts_data *ts_data);
int fts_gesture_reg_write(u8 reg, u8 mask, bool enable);

/* ADB functions */
int fts_create_sysfs(struct fts_ts_data *ts_data);
int fts_remove_sysfs(struct fts_ts_data *ts_data);

/* ESD */
#if FTS_ESDCHECK_EN
int fts_esdcheck_init(struct fts_ts_data *ts_data);
int fts_esdcheck_exit(struct fts_ts_data *ts_data);
int fts_esdcheck_switch(bool enable);
int fts_esdcheck_proc_busy(bool proc_debug);
int fts_esdcheck_set_intr(bool intr);
int fts_esdcheck_suspend(void);
int fts_esdcheck_resume(void);
#endif

/* Other */
int fts_reset_proc(int hdelayms);
int fts_wait_tp_to_valid(void);
void fts_release_all_finger(void);
void fts_tp_state_recovery(struct fts_ts_data *ts_data);
int fts_ex_mode_init(struct fts_ts_data *ts_data);
int fts_ex_mode_exit(struct fts_ts_data *ts_data);
int fts_ex_mode_recovery(struct fts_ts_data *ts_data);

void fts_irq_disable(void);
void fts_irq_enable(void);

extern int power_supply_reg_notifier(struct notifier_block *nb);
extern int power_supply_is_system_supplied(void);

#endif /* __LINUX_FOCALTECH_CORE_H__ */
