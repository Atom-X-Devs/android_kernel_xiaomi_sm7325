/*
 *
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2012-2019, FocalTech Systems, Ltd., all rights reserved.
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
* File Name: focaltech_core.c
*
* Author: Focaltech Driver Team
*
* Created: 2016-08-08
*
* Abstract: entrance for focaltech ts driver
*
* Version: V1.0
*
*****************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>
#include <linux/of_irq.h>
#include <linux/soc/qcom/panel_event_notifier.h>
#include "focaltech_core.h"


/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/
#define FTS_DRIVER_NAME                     "fts_ts"
#define FTS_DRIVER_PEN_NAME                 "fts_ts,pen"
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */
#if FTS_POWER_SOURCE_CUST_EN
#define FTS_VTG_MIN_UV                      3000000
#define FTS_VTG_MAX_UV                      3300000
#define FTS_I2C_VTG_MIN_UV                  1800000
#define FTS_I2C_VTG_MAX_UV                  1800000
#endif

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
struct fts_ts_data *fts_data;

#if defined(CONFIG_DRM)
static struct drm_panel *active_panel;
static void fts_ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *event, void *client_data);
#endif

static struct ft_chip_t ctype[] = {
	{ 0x8A, 0x56, 0x62, 0x56, 0x62, 0x56, 0xE2, 0x00, 0x00 },
};

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int fts_ts_suspend(struct device *dev);
static int fts_ts_resume(struct device *dev);

#ifdef CONFIG_DRM
static void fts_ts_register_for_panel_events(struct device_node *dp,
					struct fts_ts_data *ts_data)
{
	const char *touch_type;
	int rc = 0;
	void *cookie = NULL;

	rc = of_property_read_string(dp, "focaltech,touch-type",
						&touch_type);
	if (rc) {
		dev_warn(&fts_data->client->dev,
			"%s: No touch type\n", __func__);
		return;
	}
	if (strcmp(touch_type, "primary")) {
		pr_err("Invalid touch type\n");
		return;
	}

	cookie = panel_event_notifier_register(
			PANEL_EVENT_NOTIFICATION_PRIMARY,
			PANEL_EVENT_NOTIFIER_CLIENT_PRIMARY_TOUCH,
			active_panel, &fts_ts_panel_notifier_callback, ts_data);
	if (!cookie) {
		pr_err("Failed to register for panel events\n");
		return;
	}

	FTS_DEBUG("registered for panel notifications panel: 0x%x\n",
			active_panel);

	ts_data->notifier_cookie = cookie;
}
#endif

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
	int ret = 0, cnt = 0;
	u8 idh = 0;
	u8 chip_idh = fts_data->ic_info.ids.chip_idh;

	do {
		ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
		if (idh == chip_idh) {
			FTS_INFO("TP Ready,Device ID:0x%02x%02x", idh, idl);
			return 0;
		}
		FTS_DEBUG("TP Not Ready,ReadData:0x%02x%02x", idh, idl);
		cnt++;
		msleep(INTERVAL_READ_REG);
	} while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

	return -EIO;
}

/*****************************************************************************
*  Name: fts_tp_state_recovery
*  Brief: Need execute this function when reset
*  Input:
*  Output:
*  Return:
*****************************************************************************/
void fts_tp_state_recovery(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();
	/* wait tp stable */
	fts_wait_tp_to_valid();
	/* recover TP charger state 0x8B */
	/* recover TP glove state 0xC0 */
	/* recover TP cover state 0xC1 */
	fts_ex_mode_recovery(ts_data);
	/* recover TP gesture state 0xD0 */
	fts_gesture_recovery(ts_data);
	FTS_FUNC_EXIT();
}

int fts_reset_proc(int hdelayms)
{
	gpio_direction_output(fts_data->pdata->reset_gpio, 0);
	msleep(1);
	gpio_direction_output(fts_data->pdata->reset_gpio, 1);
	if (hdelayms)
		msleep(hdelayms);

	return 0;
}

void fts_irq_disable(void)
{
	unsigned long irqflags;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (!fts_data->irq_disabled) {
		disable_irq_nosync(fts_data->irq);
		fts_data->irq_disabled = true;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_irq_enable(void)
{
	unsigned long irqflags = 0;

	FTS_FUNC_ENTER();
	spin_lock_irqsave(&fts_data->irq_lock, irqflags);

	if (fts_data->irq_disabled) {
		enable_irq(fts_data->irq);
		fts_data->irq_disabled = false;
	}

	spin_unlock_irqrestore(&fts_data->irq_lock, irqflags);
	FTS_FUNC_EXIT();
}

void fts_hid2std(void)
{
	int ret = 0;
	u8 buf[3] = {0xEB, 0xAA, 0x09};

	if (fts_data->bus_type != BUS_TYPE_I2C)
		return;

	ret = fts_write(buf, 3);
	if (ret < 0) {
		FTS_ERROR("hid2std cmd write fail");
		return;
	}

	msleep(20);
	buf[0] = buf[1] = buf[2] = 0;
	ret = fts_read(NULL, 0, buf, 3);
	if (ret < 0)
		FTS_ERROR("hid2std cmd read fail");
	else if ((buf[0] == 0xEB) && (buf[1] == 0xAA) && (buf[2] == 0x08))
		FTS_DEBUG("hidi2c change to stdi2c successful");
	else
		FTS_DEBUG("hidi2c change to stdi2c not support or fail");

}

static int fts_get_chip_types(
	struct fts_ts_data *ts_data,
	u8 id_h, u8 id_l, bool fw_valid)
{
	int i = 0;
	u32 ctype_entries = sizeof(ctype) / sizeof(struct ft_chip_t);

	if ((0x0 == id_h) || (0x0 == id_l)) {
		FTS_ERROR("id_h/id_l is 0");
		return -EINVAL;
	}

	FTS_DEBUG("verify id:0x%02x%02x", id_h, id_l);
	for (i = 0; i < ctype_entries; i++) {
		if (VALID == fw_valid) {
			if ((id_h == ctype[i].chip_idh) && (id_l == ctype[i].chip_idl))
				break;
		} else {
			if (((id_h == ctype[i].rom_idh) && (id_l == ctype[i].rom_idl))
				|| ((id_h == ctype[i].pb_idh) && (id_l == ctype[i].pb_idl))
				|| ((id_h == ctype[i].bl_idh) && (id_l == ctype[i].bl_idl)))
			break;
		}
	}

	if (i >= ctype_entries)
		return -ENODATA;

	ts_data->ic_info.ids = ctype[i];
	return 0;
}

static int fts_read_bootid(struct fts_ts_data *ts_data, u8 *id)
{
	int ret = 0;
	u8 chip_id[2] = { 0 };
	u8 id_cmd[4] = { 0 };
	u32 id_cmd_len = 0;

	id_cmd[0] = FTS_CMD_START1;
	id_cmd[1] = FTS_CMD_START2;
	ret = fts_write(id_cmd, 2);
	if (ret < 0) {
		FTS_ERROR("start cmd write fail");
		return ret;
	}

	msleep(FTS_CMD_START_DELAY);
	id_cmd[0] = FTS_CMD_READ_ID;
	id_cmd[1] = id_cmd[2] = id_cmd[3] = 0x00;
	if (ts_data->ic_info.is_incell)
		id_cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
	else
		id_cmd_len = FTS_CMD_READ_ID_LEN;

	ret = fts_read(id_cmd, id_cmd_len, chip_id, 2);
	if ((ret < 0) || (0x0 == chip_id[0]) || (0x0 == chip_id[1])) {
		FTS_ERROR("read boot id fail,read:0x%02x%02x", chip_id[0], chip_id[1]);
		return -EIO;
	}

	id[0] = chip_id[0];
	id[1] = chip_id[1];
	return 0;
}

/*****************************************************************************
* Name: fts_get_ic_information
* Brief: read chip id to get ic information, after run the function, driver w-
*        ill know which IC is it.
*        If cant get the ic information, maybe not focaltech's touch IC, need
*        unregister the driver
* Input:
* Output:
* Return: return 0 if get correct ic information, otherwise return error code
*****************************************************************************/
static int fts_get_ic_information(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int cnt = 0;
	u8 chip_id[2] = { 0 };
	u32 type = ts_data->pdata->type;

	ts_data->ic_info.is_incell = FTS_CHIP_IDC(type);
	ts_data->ic_info.hid_supported = FTS_HID_SUPPORTTED(type);

	do {
		fts_reset_proc(0);
		mdelay(FTS_CMD_START_DELAY + (cnt * 8));

		ret = fts_read_bootid(ts_data, &chip_id[0]);
		if (ret < 0) {
			FTS_DEBUG("read boot id fail,retry:%d", cnt);
			continue;
		}

		ret = fts_get_chip_types(ts_data, chip_id[0], chip_id[1], INVALID);
		if (ret < 0) {
			FTS_DEBUG("can't get ic informaton,retry:%d", cnt);
			continue;
		}
		break;
	} while (++cnt < 3);

	if (cnt >= 3) {
		FTS_ERROR("get ic informaton fail");
		return -EIO;
	}

	FTS_INFO("get ic information, chip id = 0x%02x%02x(cid type=0x%x)",
		 ts_data->ic_info.ids.chip_idh, ts_data->ic_info.ids.chip_idl,
		 ts_data->ic_info.cid.type);

	return 0;
}

/*****************************************************************************
*  Reprot related
*****************************************************************************/
// static void fts_show_touch_buffer(u8 *data, int datalen)
// {
// 	int i = 0;
// 	int count = 0;
// 	char *tmpbuf = NULL;

// 	tmpbuf = kzalloc(1024, GFP_KERNEL);
// 	if (!tmpbuf) {
// 		FTS_ERROR("tmpbuf zalloc fail");
// 		return;
// 	}

// 	for (i = 0; i < datalen; i++) {
// 		count += snprintf(tmpbuf + count, 1024 - count, "%02X,", data[i]);
// 		if (count >= 1024)
// 			break;
// 	}
// 	FTS_DEBUG("point buffer:%s", tmpbuf);

// 	if (tmpbuf) {
// 		kfree(tmpbuf);
// 		tmpbuf = NULL;
// 	}
// }

void fts_release_all_finger(void)
{
	struct input_dev *input_dev = fts_data->input_dev;
	u32 max_touches = fts_data->pdata->max_touch_number;
	u32 finger_count = 0;

	FTS_FUNC_ENTER();
	mutex_lock(&fts_data->report_mutex);
	for (finger_count = 0; finger_count < max_touches; finger_count++) {
		input_mt_slot(input_dev, finger_count);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_sync(input_dev);

	fts_data->touchs = 0;
	fts_data->key_state = 0;
	mutex_unlock(&fts_data->report_mutex);
	FTS_FUNC_EXIT();
}

/*****************************************************************************
* Name: fts_input_report_key
* Brief: process key events,need report key-event if key enable.
*        if point's coordinate is in (x_dim-50,y_dim-50) ~ (x_dim+50,y_dim+50),
*        need report it to key event.
*        x_dim: parse from dts, means key x_coordinate, dimension:+-50
*        y_dim: parse from dts, means key y_coordinate, dimension:+-50
* Input:
* Output:
* Return: return 0 if it's key event, otherwise return error code
*****************************************************************************/
static int fts_input_report_key(struct fts_ts_data *data, int index)
{
	int i = 0;
	int x = data->events[index].x;
	int y = data->events[index].y;
	int *x_dim = &data->pdata->key_x_coords[0];
	int *y_dim = &data->pdata->key_y_coords[0];

	if (!data->pdata->have_key)
		return -EINVAL;

	for (i = 0; i < data->pdata->key_number; i++) {
		if ((x >= x_dim[i] - FTS_KEY_DIM) && (x <= x_dim[i] + FTS_KEY_DIM) &&
			(y >= y_dim[i] - FTS_KEY_DIM) && (y <= y_dim[i] + FTS_KEY_DIM)) {
			if (EVENT_DOWN(data->events[index].flag)
				&& !(data->key_state & (1 << i))) {
				input_report_key(data->input_dev, data->pdata->keys[i], 1);
				data->key_state |= (1 << i);
				FTS_DEBUG("Key%d(%d,%d) DOWN!", i, x, y);
			} else if (EVENT_UP(data->events[index].flag)
				&& (data->key_state & (1 << i))) {
				input_report_key(data->input_dev, data->pdata->keys[i], 0);
				data->key_state &= ~(1 << i);
				FTS_DEBUG("Key%d(%d,%d) Up!", i, x, y);
			}
			return 0;
		}
	}
	return -EINVAL;
}

static int fts_input_report(struct fts_ts_data *data)
{
	int i = 0;
	int touchs = 0;
	bool va_reported = false;
	u32 max_touch_num = data->pdata->max_touch_number;
	struct ts_event *events = data->events;

	for (i = 0; i < data->touch_point; i++) {
		if (fts_input_report_key(data, i) == 0)
			continue;

		va_reported = true;
		input_mt_slot(data->input_dev, events[i].id);

		if (EVENT_DOWN(events[i].flag)) {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);

#if FTS_REPORT_PRESSURE_EN
			if (events[i].p <= 0)
				events[i].p = 0x3f;

			input_report_abs(data->input_dev, ABS_MT_PRESSURE, events[i].p);
#endif
			if (events[i].area <= 0)
				events[i].area = 0x09;

			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, data->overlap_area);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR, data->overlap_area);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, events[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, events[i].y);

			touchs |= BIT(events[i].id);
			data->touchs |= BIT(events[i].id);

			if ((data->log_level >= 2) ||
				((1 == data->log_level) && (FTS_TOUCH_DOWN == events[i].flag))) {
				FTS_DEBUG("[B]P%d(%d, %d)[p:%d,tm:%d] DOWN!",
					events[i].id,
					events[i].x, events[i].y,
					events[i].p, events[i].area);
			}
		} else {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(events[i].id);
			if (data->log_level >= 1)
				FTS_DEBUG("[B]P%d UP!", events[i].id);
		}
	}

	if (unlikely(data->touchs ^ touchs)) {
		for (i = 0; i < max_touch_num; i++)  {
			if (BIT(i) & (data->touchs ^ touchs)) {
				if (data->log_level >= 1) {
					FTS_DEBUG("[B]P%d UP!", i);
				}
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}

	if (touchs) {
		input_report_key(data->input_dev, BTN_TOUCH, 1);
	} else if (va_reported || data->touchs) {
		if (data->log_level >= 1)
			FTS_DEBUG("[B]Points All Up!");
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	}

	data->touchs = touchs;
	input_sync(data->input_dev);
	return 0;
}

static int fts_read_and_report_foddata(struct fts_ts_data *data)
{
	u8 buf[9] = { 0 };
	int x, y, z;

	/*
	 * buf[0]: point id
	 * buf[1]:event type， 0x24 is doubletap, 0x25 is single tap, 0x26 is fod pointer event
	 * buf[2]: touch area/fod sensor area
	 * buf[3]: touch area
	 * buf[4-7]: x,y position
	 * buf[8]:pointer up or down, 0 is down, 1 is up
	 */
	switch (buf[1]) {
	case 0x24:
		FTS_INFO("DoubleClick Gesture detected, Wakeup panel\n");
		input_report_key(data->input_dev, KEY_WAKEUP, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_WAKEUP, 0);
		input_sync(data->input_dev);
		break;
	case 0x25:
		FTS_INFO("FOD status report KEY_GOTO\n");
		input_report_key(data->input_dev, KEY_GOTO, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GOTO, 0);
		input_sync(data->input_dev);
		break;
	case 0x26:
		x = (buf[4] << 8) | buf[5];
		y = (buf[6] << 8) | buf[7];
		z = buf[3];
		FTS_INFO(
			"FTS:read fod data: 0x%2x 0x%2x 0x%2x 0x%2x 0x%2x anxis_x: %d anxis_y: %d\n",
			buf[0], buf[1], buf[2], buf[3], buf[8], x, y);

		if (buf[8] == 0) {
			if (!data->fod_finger_skip)
				data->overlap_area = 100;
			if (data->old_point_id != buf[0]) {
				if (data->old_point_id == 0xff)
					data->old_point_id = buf[0];
				else
					data->point_id_changed = true;
			}
			data->finger_in_fod = true;
			if (!data->suspended) {
				/* report value and 0x152 in @fts_input_report_b */
				FTS_INFO("FTS:touch is not in suspend state, report x,y value by touch nomal report\n");
				mutex_unlock(&data->report_mutex);
				return -EINVAL;
			}

			if (!data->fod_finger_skip) {
				mutex_lock(&data->report_mutex);
				input_mt_slot(data->input_dev, buf[0]);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 1);
				input_report_key(data->input_dev, BTN_INFO, 1);
				input_report_key(data->input_dev, BTN_TOUCH, 1);
				input_report_key(data->input_dev, BTN_TOOL_FINGER, 1);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, data->overlap_area);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR, data->overlap_area);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, z);
				input_sync(data->input_dev);
				FTS_INFO("Report_0x152 suspend DOWN report_area %d success for miui", data->overlap_area);
				mutex_unlock(&data->report_mutex);
			}
		} else {
			input_report_key(data->input_dev, BTN_INFO, 0);
			input_sync(data->input_dev);
			data->finger_in_fod = false;
			data->fod_finger_skip = false;
			data->old_point_id = 0xff;
			data->point_id_changed = false;
			FTS_INFO("Report_0x152 UP for FingerPrint\n");
			data->overlap_area = 0;
			if (!data->suspended) {
				FTS_INFO("FTS:touch is not in suspend state, report x,y value by touch nomal report\n");
				return -EINVAL;
			}
			mutex_lock(&data->report_mutex);
			input_mt_slot(data->input_dev, buf[0]);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
			input_report_key(data->input_dev, BTN_TOUCH, 0);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
			input_sync(data->input_dev);
			mutex_unlock(&data->report_mutex);
		}
		break;
	default:
		data->overlap_area = 0;
		if (!data->suspended)
			return -EINVAL;

		break;
	}
	return 0;
}

static int fts_read_touchdata(struct fts_ts_data *ts_data, u8 *buf)
{
	int ret = 0;

	ts_data->touch_addr = 0x01;
	ret = fts_read(&ts_data->touch_addr, 1, buf, ts_data->touch_size);

	fts_read_and_report_foddata(ts_data);

	if ((buf[1] == 0xEF) && (buf[2] == 0xEF) && (buf[3] == 0xEF)) {
		ret = 1;
		goto exit;
	} else if (ret < 0) {
		if (buf[0] == 0xEF) {
			ret = 1;
			goto exit;
		}

		FTS_ERROR("read touchdata failed, ret:%d", ret);
		return ret;
	}

	return 0;

exit:
	fts_release_all_finger();
	ts_data->fw_is_running = true;
	return ret;
}

static int fts_parse_touchdata(struct fts_ts_data *ts_data, u8 *touch_buf)
{
	int ret = 0;
	u8 gesture_en = 0xFF;

	memset(touch_buf, 0xFF, FTS_MAX_TOUCH_BUF);

	/*read touch data*/
	ret = fts_read_touchdata(ts_data, touch_buf);
	if (ret < 0) {
		FTS_ERROR("read touch data fails");
		return TOUCH_ERROR;
	}

	if (ret)
		return TOUCH_IGNORE;

	/*gesture*/
	if (ts_data->suspended && ts_data->gesture_mode) {
		ret = fts_read_reg(FTS_REG_GESTURE_EN, &gesture_en);
		if ((ret >= 0) && (gesture_en == ENABLE))
			return TOUCH_GESTURE;
		FTS_DEBUG("gesture not enable in fw, don't process gesture");
	}

	if ((touch_buf[1] == 0xFF) && (touch_buf[2] == 0xFF) &&
	    (touch_buf[3] == 0xFF) && (touch_buf[4] == 0xFF)) {
		FTS_INFO("touch buff is 0xff, need recovery state");
		return TOUCH_FW_INIT;
	}

	return ((touch_buf[FTS_TOUCH_E_NUM] >> 4) & 0x0F);
}

static int fts_read_parse_touchdata(struct fts_ts_data *data)
{
	int i = 0;
	int max_touch_num = data->pdata->max_touch_number;
	int touch_etype = 0;
	u8 finger_num = 0;
	u8 pointid = 0;
	u8 base = 0;
	u8 *buf = data->touch_buf;
	struct ts_event *events = data->events;

	touch_etype = fts_parse_touchdata(data, buf);
	switch (touch_etype) {
	case TOUCH_DEFAULT:
		finger_num = buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (finger_num > max_touch_num) {
			FTS_ERROR("invalid point_num(%d)", finger_num);
			return -EIO;
		}

		for (i = 0; i < max_touch_num; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= FTS_MAX_ID)
				break;
			else if (pointid >= max_touch_num) {
				FTS_ERROR("ID(%d) beyond max_touch_number",
					  pointid);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			if (data->pdata->super_resolution_factor == 10) {
				events[i].area = buf[FTS_TOUCH_OFF_AREA + base] & 0x7F;
				events[i].p = buf[FTS_TOUCH_OFF_PRE + base] & 0x0F;

				events[i].x =
					((buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 11) +
					((buf[FTS_TOUCH_OFF_XL + base] & 0xFF) << 3) +
					(((buf[FTS_TOUCH_OFF_PRE + base] & 0xC0) >> 6) << 1) +
					((buf[FTS_TOUCH_OFF_E_XH + base] & 0x20) >> 5);
				events[i].y =
					((buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 11) +
					((buf[FTS_TOUCH_OFF_YL + base] & 0xFF) << 3) +
					(((buf[FTS_TOUCH_OFF_PRE + base] & 0x30) >> 4) << 1) +
					((buf[FTS_TOUCH_OFF_ID_YH + base] & 0x10) >> 4);
			} else {
				events[i].p = buf[FTS_TOUCH_OFF_PRE + base];
				events[i].area = buf[FTS_TOUCH_OFF_AREA + base];

				events[i].x =
					((buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) +
					(buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
				events[i].y =
					((buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) +
					(buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
			}
			FTS_DEBUG("x:%d,y:%d", events[i].x, events[i].y);

			data->touch_point++;
			if (EVENT_DOWN(events[i].flag) && (finger_num == 0)) {
				FTS_INFO("abnormal touch data from fw");
				return -EIO;
			}
		}

		if (data->touch_point == 0) {
			FTS_INFO("no touch point information(%02x)",
				 buf[2]);
			return -EIO;
		}

		mutex_lock(&data->report_mutex);
		fts_input_report(data);
		mutex_unlock(&data->report_mutex);
		break;
	case TOUCH_EVENT_NUM:
		data->touch_point = buf[FTS_TOUCH_E_NUM] & 0x0F;
		if (!data->touch_point || (data->touch_point > max_touch_num)) {
			FTS_ERROR("invalid touch event num(%d)", data->touch_point);
			return -EIO;
		}

		for (i = 0; i < data->touch_point; i++) {
			base = FTS_ONE_TCH_LEN * i + 2;
			pointid = (buf[FTS_TOUCH_OFF_ID_YH + base]) >> 4;
			if (pointid >= max_touch_num) {
				FTS_ERROR(
					"touch point ID(%d) beyond max_touch_number(%d)",
					pointid, max_touch_num);
				return -EINVAL;
			}

			events[i].id = pointid;
			events[i].flag = buf[FTS_TOUCH_OFF_E_XH + base] >> 6;
			events[i].p = buf[FTS_TOUCH_OFF_PRE + base];
			events[i].area = buf[FTS_TOUCH_OFF_AREA + base];
			events[i].x =
				((buf[FTS_TOUCH_OFF_E_XH + base] & 0x0F) << 8) +
				(buf[FTS_TOUCH_OFF_XL + base] & 0xFF);
			events[i].y =
				((buf[FTS_TOUCH_OFF_ID_YH + base] & 0x0F) << 8) +
				(buf[FTS_TOUCH_OFF_YL + base] & 0xFF);
		}

		mutex_lock(&data->report_mutex);
		fts_input_report(data);
		mutex_unlock(&data->report_mutex);
		break;
	case TOUCH_GESTURE:
		if (fts_gesture_readdata(data, buf) == 0)
			FTS_INFO("succuss to get gesture data in irq handler");
		break;
	case TOUCH_FW_INIT:
		fts_release_all_finger();
		fts_tp_state_recovery(data);
		break;
	case TOUCH_IGNORE:
	case TOUCH_ERROR:
		break;
	default:
		FTS_INFO("unknown touch event(%d)", touch_etype);
		break;
	}

	return 0;
}

static void fts_irq_read_report(void)
{
	struct fts_ts_data *ts_data = fts_data;

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(1);
#endif

#if FTS_POINT_REPORT_CHECK_EN
	fts_prc_queue_work(ts_data);
#endif

	fts_read_parse_touchdata(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_set_intr(0);
#endif
}

static irqreturn_t fts_irq_handler(int irq, void *data)
{
	struct fts_ts_data *fts_data = data;
	if (!fts_data) {
		pr_err("%s: Invalid fts_data\n", __func__);
		return IRQ_HANDLED;
	}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	if ((fts_data->suspended) && (fts_data->pm_suspend)) {
		if (!wait_for_completion_timeout(&fts_data->pm_completion, msecs_to_jiffies(FTS_TIMEOUT_COMERR_PM))) {
			FTS_ERROR("Bus don't resume from pm(deep),timeout,skip irq");
			return IRQ_HANDLED;
		}
	}
#endif

	fts_irq_read_report();

	return IRQ_HANDLED;
}

static int fts_irq_registration(struct fts_ts_data *ts_data)
{
	int ret = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;

	ts_data->irq = gpio_to_irq(pdata->irq_gpio);
	pdata->irq_gpio_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	FTS_INFO("irq:%d, flag:%x", ts_data->irq, pdata->irq_gpio_flags);
	ret = request_threaded_irq(ts_data->irq, NULL, fts_irq_handler,
				pdata->irq_gpio_flags, FTS_DRIVER_NAME, ts_data);

	return ret;
}

static int fts_input_init(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int key_num = 0;
	struct fts_ts_platform_data *pdata = ts_data->pdata;
	struct input_dev *input_dev;

	FTS_FUNC_ENTER();
	input_dev = input_allocate_device();
	if (!input_dev) {
		FTS_ERROR("Failed to allocate memory for input device");
		return -ENOMEM;
	}

	/* Init and register Input device */
	input_dev->name = FTS_DRIVER_NAME;
	if (ts_data->bus_type == BUS_TYPE_I2C)
		input_dev->id.bustype = BUS_I2C;
	else
		input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = ts_data->dev;

	input_set_drvdata(input_dev, ts_data);

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->have_key) {
		FTS_INFO("set key capabilities");
		for (key_num = 0; key_num < pdata->key_number; key_num++)
			input_set_capability(input_dev, EV_KEY, pdata->keys[key_num]);
	}

	input_mt_init_slots(input_dev, pdata->max_touch_number, INPUT_MT_DIRECT);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, pdata->x_min, pdata->x_max - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR, pdata->x_min, pdata->x_max - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
#if FTS_REPORT_PRESSURE_EN
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
#endif
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_GOTO);
	input_set_capability(input_dev, EV_KEY, BTN_INFO);

	ret = input_register_device(input_dev);
	if (ret) {
		FTS_ERROR("Input device registration failed");
		input_set_drvdata(input_dev, NULL);
		input_free_device(input_dev);
		input_dev = NULL;
		return ret;
	}

	ts_data->input_dev = input_dev;

	FTS_FUNC_EXIT();
	return 0;
}

static int fts_report_buffer_init(struct fts_ts_data *ts_data)
{
	ts_data->touch_buf = kzalloc(FTS_MAX_TOUCH_BUF, GFP_KERNEL);
	if (!ts_data->touch_buf) {
		FTS_ERROR("failed to alloc memory for touch buf");
		return -ENOMEM;
	}

	ts_data->touch_size = FTS_TOUCH_DATA_LEN;

	return 0;
}

#if FTS_POWER_SOURCE_CUST_EN
/*****************************************************************************
* Power Control
*****************************************************************************/
#if FTS_PINCTRL_EN
static int fts_pinctrl_init(struct fts_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(ts->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		FTS_ERROR("Failed to get pinctrl, please check dts");
		ret = PTR_ERR(ts->pinctrl);
		goto err_pinctrl_get;
	}

	ts->pins_active = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(ts->pins_active)) {
		FTS_ERROR("Pin state[active] not found");
		ret = PTR_ERR(ts->pins_active);
		goto err_pinctrl_lookup;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		FTS_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto err_pinctrl_lookup;
	}

	ts->pins_release = pinctrl_lookup_state(ts->pinctrl, "pmx_ts_release");
	if (IS_ERR_OR_NULL(ts->pins_release)) {
		FTS_ERROR("Pin state[release] not found");
		ret = PTR_ERR(ts->pins_release);
	}

	return 0;
err_pinctrl_lookup:
	if (ts->pinctrl) {
		devm_pinctrl_put(ts->pinctrl);
	}
err_pinctrl_get:
	ts->pinctrl = NULL;
	ts->pins_release = NULL;
	ts->pins_suspend = NULL;
	ts->pins_active = NULL;
	return ret;
}

static int fts_pinctrl_select_normal(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_active) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_active);
		if (ret < 0) {
			FTS_ERROR("Set normal pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_suspend(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0) {
			FTS_ERROR("Set suspend pin state error:%d", ret);
		}
	}

	return ret;
}

static int fts_pinctrl_select_release(struct fts_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl) {
		if (IS_ERR_OR_NULL(ts->pins_release)) {
			devm_pinctrl_put(ts->pinctrl);
			ts->pinctrl = NULL;
		} else {
			ret = pinctrl_select_state(ts->pinctrl, ts->pins_release);
			if (ret < 0)
				FTS_ERROR("Set gesture pin state error:%d", ret);
		}
	}

	return ret;
}
#endif /* FTS_PINCTRL_EN */

static int fts_power_source_ctrl(struct fts_ts_data *ts_data, int enable)
{
	int ret = 0;

	if (IS_ERR_OR_NULL(ts_data->avdd)) {
		FTS_ERROR("vdd is invalid");
		return -EINVAL;
	}

	FTS_FUNC_ENTER();
	if (enable) {
		if (ts_data->power_disabled) {
			FTS_DEBUG("regulator enable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);

			if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
				ret = regulator_enable(ts_data->iovdd);
				if (ret)
					FTS_ERROR("enable iovdd regulator failed,ret=%d", ret);
			}

			msleep(1);

			ret = regulator_enable(ts_data->avdd);
			if (ret)
				FTS_ERROR("enable avdd regulator failed,ret=%d", ret);

			ts_data->power_disabled = false;
		}
	} else {
		if (!ts_data->power_disabled) {
			FTS_DEBUG("regulator disable !");
			gpio_direction_output(ts_data->pdata->reset_gpio, 0);
			msleep(1);

			ret = regulator_disable(ts_data->avdd);
			if (ret)
				FTS_ERROR("disable avdd regulator failed,ret=%d", ret);

			if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
				ret = regulator_disable(ts_data->iovdd);
				if (ret)
					FTS_ERROR("disable iovdd regulator failed,ret=%d", ret);

			}

			ts_data->power_disabled = true;
		}
	}

	FTS_FUNC_EXIT();
	return ret;
}

/*****************************************************************************
* Name: fts_power_source_init
* Brief: Init regulator power:vdd/vcc_io(if have), generally, no vcc_io
*        vdd---->vdd-supply in dts, kernel will auto add "-supply" to parse
*        Must be call after fts_gpio_configure() execute,because this function
*        will operate reset-gpio which request gpio in fts_gpio_configure()
* Input:
* Output:
* Return: return 0 if init power successfully, otherwise return error code
*****************************************************************************/
static int fts_power_source_init(struct fts_ts_data *ts_data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ts_data->avdd = regulator_get(ts_data->dev, "avdd");
	if (IS_ERR_OR_NULL(ts_data->avdd)) {
		ret = PTR_ERR(ts_data->avdd);
		FTS_ERROR("get vdd regulator failed,ret=%d", ret);
		return ret;
	}

	if (regulator_count_voltages(ts_data->avdd) > 0) {
		ret = regulator_set_voltage(ts_data->avdd, FTS_VTG_MIN_UV, FTS_VTG_MAX_UV);
		if (ret) {
			FTS_ERROR("avdd regulator set_vtg failed ret=%d", ret);
			regulator_put(ts_data->avdd);
			return ret;
		}
	}

	ts_data->iovdd = regulator_get(ts_data->dev, "iovdd");
	if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
		if (regulator_count_voltages(ts_data->iovdd) > 0) {
			ret = regulator_set_voltage(ts_data->iovdd, FTS_I2C_VTG_MIN_UV, FTS_I2C_VTG_MAX_UV);
			if (ret) {
				FTS_ERROR("iovdd regulator set_vtg failed,ret=%d", ret);
				regulator_put(ts_data->iovdd);
			}
		}
	}

#if FTS_PINCTRL_EN
	fts_pinctrl_init(ts_data);
	fts_pinctrl_select_normal(ts_data);
#endif

	ts_data->power_disabled = true;
	ret = fts_power_source_ctrl(ts_data, ENABLE);
	if (ret) {
		FTS_ERROR("fail to enable power(regulator)");
	}

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_power_source_exit(struct fts_ts_data *ts_data)
{
#if FTS_PINCTRL_EN
	fts_pinctrl_select_release(ts_data);
#endif

	fts_power_source_ctrl(ts_data, DISABLE);

	if (!IS_ERR_OR_NULL(ts_data->avdd)) {
		if (regulator_count_voltages(ts_data->avdd) > 0)
			regulator_set_voltage(ts_data->avdd, 0, FTS_VTG_MAX_UV);
		regulator_put(ts_data->avdd);
	}

	if (!IS_ERR_OR_NULL(ts_data->iovdd)) {
		if (regulator_count_voltages(ts_data->iovdd) > 0)
			regulator_set_voltage(ts_data->iovdd, 0, FTS_I2C_VTG_MAX_UV);
		regulator_put(ts_data->iovdd);
	}

	return 0;
}
#endif /* FTS_POWER_SOURCE_CUST_EN */

static int fts_gpio_configure(struct fts_ts_data *data)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	/* request irq gpio */
	if (gpio_is_valid(data->pdata->irq_gpio)) {
		ret = gpio_request(data->pdata->irq_gpio, "fts_irq_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]irq gpio request failed");
			goto err_irq_gpio_req;
		}

		ret = gpio_direction_input(data->pdata->irq_gpio);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for irq gpio failed");
			goto err_irq_gpio_dir;
		}
	}

	/* request reset gpio */
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		ret = gpio_request(data->pdata->reset_gpio, "fts_reset_gpio");
		if (ret) {
			FTS_ERROR("[GPIO]reset gpio request failed");
			goto err_irq_gpio_dir;
		}

		ret = gpio_direction_output(data->pdata->reset_gpio, 1);
		if (ret) {
			FTS_ERROR("[GPIO]set_direction for reset gpio failed");
			goto err_reset_gpio_dir;
		}
	}

	FTS_FUNC_EXIT();
	return 0;

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	FTS_FUNC_EXIT();
	return ret;
}

static int fts_get_dt_coords(struct device *dev, char *name,
				struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	u32 coords[FTS_COORDS_ARR_SIZE] = { 0 };
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FTS_COORDS_ARR_SIZE) {
		FTS_ERROR("invalid:%s, size:%d", name, coords_size);
		return -EINVAL;
	}

	ret = of_property_read_u32_array(np, name, coords, coords_size);
	if (ret < 0) {
		FTS_ERROR("Unable to read %s, please check dts", name);
		pdata->x_min = FTS_X_MIN_DISPLAY_DEFAULT;
		pdata->y_min = FTS_Y_MIN_DISPLAY_DEFAULT;
		pdata->x_max = FTS_X_MAX_DISPLAY_DEFAULT;
		pdata->y_max = FTS_Y_MAX_DISPLAY_DEFAULT;
		return -ENODATA;
	} else {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}

	FTS_INFO("display x(%d %d) y(%d %d)", pdata->x_min, pdata->x_max,
		pdata->y_min, pdata->y_max);
	return 0;
}

static int fts_parse_dt(struct device *dev, struct fts_ts_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;
	u32 temp_val = 0;

	FTS_FUNC_ENTER();

	ret = fts_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (ret < 0)
		FTS_ERROR("Unable to get display-coords");

	/* key */
	pdata->have_key = of_property_read_bool(np, "focaltech,have-key");
	if (pdata->have_key) {
		ret = of_property_read_u32(np, "focaltech,key-number", &pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key number undefined!");

		ret = of_property_read_u32_array(np, "focaltech,keys",
						pdata->keys, pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Keys undefined!");
		else if (pdata->key_number > FTS_MAX_KEYS)
			pdata->key_number = FTS_MAX_KEYS;

		ret = of_property_read_u32_array(np, "focaltech,key-x-coords",
						pdata->key_x_coords,
						pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key Y Coords undefined!");

		ret = of_property_read_u32_array(np, "focaltech,key-y-coords",
						pdata->key_y_coords,
						pdata->key_number);
		if (ret < 0)
			FTS_ERROR("Key X Coords undefined!");

		FTS_INFO("VK Number:%d, key:(%d,%d,%d), "
			"coords:(%d,%d),(%d,%d),(%d,%d)",
			pdata->key_number,
			pdata->keys[0], pdata->keys[1], pdata->keys[2],
			pdata->key_x_coords[0], pdata->key_y_coords[0],
			pdata->key_x_coords[1], pdata->key_y_coords[1],
			pdata->key_x_coords[2], pdata->key_y_coords[2]);
	}

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
			0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		FTS_ERROR("Unable to get reset_gpio");

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
			0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		FTS_ERROR("Unable to get irq_gpio");

	ret = of_property_read_u32(np, "focaltech,super-resolution-factor", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get super-resolution-factors, please use default");
		pdata->super_resolution_factor = 1;
	} else
		pdata->super_resolution_factor = temp_val;

	ret = of_property_read_u32(np, "focaltech,max-touch-number", &temp_val);
	if (ret < 0) {
		FTS_ERROR("Unable to get max-touch-number, please check dts");
		pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
	} else {
		if (temp_val < 2)
			pdata->max_touch_number = 2; /* max_touch_number must >= 2 */
		else if (temp_val > FTS_MAX_POINTS_SUPPORT)
			pdata->max_touch_number = FTS_MAX_POINTS_SUPPORT;
		else
			pdata->max_touch_number = temp_val;
	}

	FTS_INFO("max touch number:%d, irq gpio:%d, reset gpio:%d",
		pdata->max_touch_number, pdata->irq_gpio, pdata->reset_gpio);

	ret = of_property_read_u32(np, "focaltech,ic-type", &temp_val);
	if (ret < 0)
		pdata->type = _FT3518;
	else
		pdata->type = temp_val;

	FTS_FUNC_EXIT();
	return 0;
}

#if defined(CONFIG_DRM)
static void fts_suspend_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data =
		container_of(work, struct fts_ts_data, suspend_work);

	fts_ts_suspend(ts_data->dev);
}

static void fts_resume_work(struct work_struct *work)
{
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data,
					resume_work);

	fts_ts_resume(ts_data->dev);
}

/**
 * @brief Write 1/0 to Touch IC 0x8B register depending on whether it is in charge state
 */
static void fts_power_supply_work(struct work_struct *work)
{
	int ret = 0;
	int charger_status = 0;
	struct fts_ts_data *ts_data = container_of(work, struct fts_ts_data, power_supply_work);

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	if (ts_data->pm_suspend) {
		FTS_ERROR("TP is in suspend mode, don't set usb status!");
		return;
	}
#endif
	pm_stay_awake(ts_data->dev);

	charger_status = !!power_supply_is_system_supplied();
	if (charger_status != ts_data->charger_status || ts_data->charger_status < 0) {
		ts_data->charger_status = charger_status;
		ret = fts_write_reg(FTS_REG_CHARGER_MODE_EN, charger_status);
		if (ret < 0)
			FTS_ERROR("failed to set power supply status:%d", ts_data->charger_status);
	}

	pm_relax(ts_data->dev);
}

static int fts_power_supply_callback(struct notifier_block *nb, unsigned long event, void *ptr)
{
	struct fts_ts_data *ts_data =
		container_of(nb, struct fts_ts_data, power_supply_notifier);

	queue_work(ts_data->ts_workqueue, &ts_data->power_supply_work);

	return 0;
}

static void fts_ts_panel_notifier_callback(enum panel_event_notifier_tag tag,
		 struct panel_event_notification *notification, void *client_data)
{
	struct fts_ts_data *ts_data = client_data;

	if (!notification) {
		pr_err("Invalid notification\n");
		return;
	}

	FTS_DEBUG("Notification type:%d, early_trigger:%d",
			notification->notif_type,
			notification->notif_data.early_trigger);

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		flush_workqueue(ts_data->ts_workqueue);
		queue_work(ts_data->ts_workqueue, &ts_data->resume_work);
		break;
	case DRM_PANEL_EVENT_BLANK:
	case DRM_PANEL_EVENT_BLANK_LP:
		flush_workqueue(ts_data->ts_workqueue);
		queue_work(ts_data->ts_workqueue, &ts_data->suspend_work);
		break;
	default:
		FTS_DEBUG("notification serviced :%d\n",
				notification->notif_type);
		break;
	}
}
#endif

static int fts_ts_probe_entry(struct fts_ts_data *ts_data)
{
	int ret = 0;
	int pdata_size = sizeof(struct fts_ts_platform_data);

	FTS_FUNC_ENTER();
	FTS_INFO("%s", FTS_DRIVER_VERSION);
	ts_data->pdata = kzalloc(pdata_size, GFP_KERNEL);
	if (!ts_data->pdata) {
		FTS_ERROR("allocate memory for platform_data fail");
		return -ENOMEM;
	}

	if (ts_data->dev->of_node) {
		ret = fts_parse_dt(ts_data->dev, ts_data->pdata);
		if (ret)
			FTS_ERROR("device-tree parse fail");
	} else {
		if (ts_data->dev->platform_data) {
			memcpy(ts_data->pdata, ts_data->dev->platform_data, pdata_size);
		} else {
			FTS_ERROR("platform_data is null");
			return -ENODEV;
		}
	}

	ts_data->ts_workqueue = create_singlethread_workqueue("fts_wq");
	if (!ts_data->ts_workqueue) {
		FTS_ERROR("create fts workqueue fail");
	}

	spin_lock_init(&ts_data->irq_lock);
	mutex_init(&ts_data->report_mutex);
	mutex_init(&ts_data->bus_lock);
	init_waitqueue_head(&ts_data->ts_waitqueue);

	/* Init communication interface */
	ret = fts_bus_init(ts_data);
	if (ret) {
		FTS_ERROR("bus initialize fail");
		goto err_bus_init;
	}

	ret = fts_input_init(ts_data);
	if (ret) {
		FTS_ERROR("input initialize fail");
		goto err_input_init;
	}

	ret = fts_report_buffer_init(ts_data);
	if (ret) {
		FTS_ERROR("report buffer init fail");
		goto err_report_buffer;
	}

	ret = fts_gpio_configure(fts_data);
	if (ret) {
		FTS_ERROR("configure the gpios fail");
		goto err_gpio_config;
	}

#if FTS_POWER_SOURCE_CUST_EN
	ret = fts_power_source_init(fts_data);
	if (ret) {
		FTS_ERROR("fail to get power(regulator)");
		goto err_power_init;
	}
#endif

	if (!FTS_CHIP_IDC(fts_data->pdata->type))
		fts_reset_proc(200);

	ret = fts_get_ic_information(fts_data);
	if (ret) {
		FTS_ERROR("not focal IC, unregister driver");
		goto err_irq_req;
	}

	ret = fts_create_sysfs(ts_data);
	if (ret) {
		FTS_ERROR("create sysfs node fail");
	}

#if FTS_POINT_REPORT_CHECK_EN
	ret = fts_point_report_check_init(ts_data);
	if (ret) {
		FTS_ERROR("init point report check fail");
	}
#endif

	ret = fts_ex_mode_init(ts_data);
	if (ret) {
		FTS_ERROR("init glove/cover/charger fail");
	}

	ret = fts_gesture_init(ts_data);
	if (ret) {
		FTS_ERROR("init gesture fail");
	}


#if FTS_ESDCHECK_EN
	ret = fts_esdcheck_init(ts_data);
	if (ret) {
		FTS_ERROR("init esd check fail");
	}
#endif

	ret = fts_irq_registration(fts_data);
	if (ret) {
		FTS_ERROR("request irq failed");
		goto err_irq_req;
	}

	if (ts_data->ts_workqueue) {
		INIT_WORK(&ts_data->resume_work, fts_resume_work);
		INIT_WORK(&ts_data->suspend_work, fts_suspend_work);
		INIT_WORK(&ts_data->power_supply_work, fts_power_supply_work);
	}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
	init_completion(&ts_data->pm_completion);
	ts_data->pm_suspend = false;
#endif

#ifdef CONFIG_DRM
	fts_ts_register_for_panel_events(ts_data->dev->of_node, ts_data);
#endif

	ts_data->power_supply_notifier.notifier_call = fts_power_supply_callback;
	ret = power_supply_reg_notifier(&ts_data->power_supply_notifier);
	if (ret)
		FTS_ERROR("get battery psy failed, don't register callback for charger mode");

	FTS_FUNC_EXIT();
	return 0;

err_irq_req:
#if FTS_POWER_SOURCE_CUST_EN
err_power_init:
	fts_power_source_exit(ts_data);
#endif
	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);
	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);
err_gpio_config:
	kfree_safe(ts_data->touch_buf);
err_report_buffer:
	input_unregister_device(ts_data->input_dev);
err_input_init:
	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);
err_bus_init:
	kfree_safe(ts_data->bus_tx_buf);
	kfree_safe(ts_data->bus_rx_buf);
	kfree_safe(ts_data->pdata);

	FTS_FUNC_EXIT();
	return ret;
}

static int fts_ts_remove_entry(struct fts_ts_data *ts_data)
{
	FTS_FUNC_ENTER();

	cancel_work_sync(&fts_data->resume_work);
	cancel_work_sync(&fts_data->suspend_work);

#if FTS_POINT_REPORT_CHECK_EN
	fts_point_report_check_exit(ts_data);
#endif

	fts_remove_sysfs(ts_data);
	fts_ex_mode_exit(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_exit(ts_data);
#endif

	fts_gesture_exit(ts_data);
	fts_bus_exit(ts_data);

	free_irq(ts_data->irq, ts_data);
	input_unregister_device(ts_data->input_dev);

	if (ts_data->ts_workqueue)
		destroy_workqueue(ts_data->ts_workqueue);

	if (active_panel && ts_data->notifier_cookie)
		panel_event_notifier_unregister(ts_data->notifier_cookie);

	if (gpio_is_valid(ts_data->pdata->reset_gpio))
		gpio_free(ts_data->pdata->reset_gpio);

	if (gpio_is_valid(ts_data->pdata->irq_gpio))
		gpio_free(ts_data->pdata->irq_gpio);

#if FTS_POWER_SOURCE_CUST_EN
	fts_power_source_exit(ts_data);
#endif

	kfree_safe(ts_data->touch_buf);
	kfree_safe(ts_data->pdata);
	kfree_safe(ts_data);

	FTS_FUNC_EXIT();

	return 0;
}

static int fts_ts_suspend(struct device *dev)
{
	int ret = 0;
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (ts_data->suspended) {
		FTS_INFO("Already in suspend state");
		return 0;
	}

	if (ts_data->fw_loading) {
		FTS_INFO("fw upgrade in process, can't suspend");
		return 0;
	}

#ifdef FTS_TOUCHSCREEN_FOD
	ret = fts_gesture_reg_write(FTS_REG_GESTURE_SUPPORT, FTS_REG_GESTURE_FOD_ON, false);
	if (ret < 0)
		FTS_ERROR("%s fts_fod_reg_write failed\n", __func__);
#endif

#if FTS_ESDCHECK_EN
	fts_esdcheck_suspend();
#endif

	if (ts_data->gesture_mode && !ts_data->poweroff_on_sleep) {
		fts_gesture_suspend(ts_data);
	} else {
		FTS_INFO("make TP enter into sleep mode");
		ret = fts_write_reg(FTS_REG_POWER_MODE, FTS_REG_POWER_MODE_SLEEP);
		if (ret < 0)
			FTS_ERROR("set TP to sleep mode fail, ret=%d", ret);
	}

	fts_release_all_finger();
	ts_data->suspended = true;
	FTS_FUNC_EXIT();
	return 0;
}

static int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = fts_data;

	FTS_FUNC_ENTER();
	if (!ts_data->suspended) {
		FTS_DEBUG("Already in awake state");
		return 0;
	}

	fts_release_all_finger();

	if (!ts_data->ic_info.is_incell)
		fts_reset_proc(200);

	fts_wait_tp_to_valid();
	fts_ex_mode_recovery(ts_data);

#if FTS_ESDCHECK_EN
	fts_esdcheck_resume();
#endif

	if (ts_data->charger_status)
		fts_write_reg(FTS_REG_CHARGER_MODE_EN, true);

	if (ts_data->gesture_mode  && !ts_data->poweroff_on_sleep)
		fts_gesture_resume(ts_data);

#ifdef FTS_TOUCHSCREEN_FOD
	fts_gesture_reg_write(FTS_REG_GESTURE_EN, FTS_REG_GESTURE_DOUBLETAP_ON, false);
#endif

	ts_data->poweroff_on_sleep = false;
	ts_data->suspended = false;
	FTS_FUNC_EXIT();
	return 0;
}

/*****************************************************************************
* TP Driver
*****************************************************************************/
static int fts_ts_check_dt(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "qcom,display-panels", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "qcom,display-panels", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			return 0;
		}
	}

	return PTR_ERR(panel);
}

static int fts_ts_check_default_tp(struct device_node *dt, const char *prop)
{
	const char **active_tp = NULL;
	int count, tmp, score = 0;
	const char *active;
	int ret, i;

	count = of_property_count_strings(dt->parent, prop);
	if (count <= 0 || count > 3)
		return -ENODEV;

	active_tp = kcalloc(count, sizeof(char *),  GFP_KERNEL);
	if (!active_tp) {
		FTS_ERROR("FTS alloc failed\n");
		return -ENOMEM;
	}

	ret = of_property_read_string_array(dt->parent, prop,
			active_tp, count);
	if (ret < 0) {
		FTS_ERROR("fail to read %s %d\n", prop, ret);
		ret = -ENODEV;
		goto out;
	}

	for (i = 0; i < count; i++) {
		active = active_tp[i];
		if (active != NULL) {
			tmp = of_device_is_compatible(dt, active);
			if (tmp > 0)
				score++;
		}
	}

	if (score <= 0) {
		FTS_INFO("not match this driver\n");
		ret = -ENODEV;
		goto out;
	}
	ret = 0;
out:
	kfree(active_tp);
	return ret;
}

#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
static int fts_pm_suspend(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	ts_data->pm_suspend = true;
	reinit_completion(&ts_data->pm_completion);
	return 0;
}

static int fts_pm_resume(struct device *dev)
{
	struct fts_ts_data *ts_data = dev_get_drvdata(dev);

	ts_data->pm_suspend = false;
	complete(&ts_data->pm_completion);
	return 0;
}

static const struct dev_pm_ops fts_dev_pm_ops = {
	.suspend = fts_pm_suspend,
	.resume = fts_pm_resume,
};
#endif

static const struct of_device_id fts_dt_match[] = {
	{.compatible = "focaltech,fts_ts", },
	{},
};
MODULE_DEVICE_TABLE(of, fts_dt_match);

#if FTS_SUPPORT_I2C
static int fts_ts_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;
	struct device_node *dp = client->dev.of_node;

	FTS_INFO("Touch Screen(I2C BUS) driver prboe...");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FTS_ERROR("I2C not supported");
		return -ENODEV;
	}

	ret = fts_ts_check_dt(dp);
	if (ret == -EPROBE_DEFER)
		return ret;

	if (ret) {
		if (!fts_ts_check_default_tp(dp, "qcom,i2c-touch-active"))
			ret = -EPROBE_DEFER;
		else
			ret = -ENODEV;

		return ret;
	}

	/* malloc memory for global struct variable */
	ts_data = (struct fts_ts_data *)kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("allocate memory for fts_data fail");
		return -ENOMEM;
	}

	fts_data = ts_data;
	ts_data->client = client;
	ts_data->dev = &client->dev;
	ts_data->log_level = 1;
	ts_data->fw_is_running = 0;
	ts_data->bus_type = BUS_TYPE_I2C;
	i2c_set_clientdata(client, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen(I2C BUS) driver probe fail");
		kfree_safe(ts_data);
		return ret;
	}

	FTS_INFO("Touch Screen(I2C BUS) driver prboe successfully");
	return 0;
}

static int fts_ts_i2c_remove(struct i2c_client *client)
{
	return fts_ts_remove_entry(i2c_get_clientdata(client));
}

static const struct i2c_device_id fts_ts_i2c_id[] = {
	{FTS_DRIVER_NAME, 0},
	{},
};

static struct i2c_driver fts_ts_i2c_driver = {
	.probe = fts_ts_i2c_probe,
	.remove = fts_ts_i2c_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
		.pm = &fts_dev_pm_ops,
#endif
		.of_match_table = of_match_ptr(fts_dt_match),
	},
	.id_table = fts_ts_i2c_id,
};

static int __init fts_ts_i2c_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = i2c_add_driver(&fts_ts_i2c_driver);
	if (ret != 0)
		FTS_ERROR("Focaltech touch screen driver init failed!");

	FTS_FUNC_EXIT();
	return ret;
}

static void __exit fts_ts_i2c_exit(void)
{
	i2c_del_driver(&fts_ts_i2c_driver);
}
#endif

static int fts_ts_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	struct fts_ts_data *ts_data = NULL;
	struct device_node *dp = spi->dev.of_node;

	FTS_INFO("Touch Screen(SPI BUS) driver prboe...");

	ret = fts_ts_check_dt(dp);
	if (ret == -EPROBE_DEFER)
		return ret;

	if (ret) {
		if (!fts_ts_check_default_tp(dp, "qcom,spi-touch-active"))
			ret = -EPROBE_DEFER;
		else
			ret = -ENODEV;

		return ret;
	}

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret) {
		FTS_ERROR("spi setup fail");
		return ret;
	}

	/* malloc memory for global struct variable */
	ts_data = kzalloc(sizeof(*ts_data), GFP_KERNEL);
	if (!ts_data) {
		FTS_ERROR("allocate memory for fts_data fail");
		return -ENOMEM;
	}

	fts_data = ts_data;
	ts_data->spi = spi;
	ts_data->dev = &spi->dev;
	ts_data->log_level = 1;

	ts_data->bus_type = BUS_TYPE_SPI_V2;
	spi_set_drvdata(spi, ts_data);

	ret = fts_ts_probe_entry(ts_data);
	if (ret) {
		FTS_ERROR("Touch Screen(SPI BUS) driver probe fail");
		kfree_safe(ts_data);
		return ret;
	}

	FTS_INFO("Touch Screen(SPI BUS) driver prboe successfully");
	return 0;
}

static int fts_ts_spi_remove(struct spi_device *spi)
{
	return fts_ts_remove_entry(spi_get_drvdata(spi));
}

static const struct spi_device_id fts_ts_spi_id[] = {
	{ FTS_DRIVER_NAME, 0 },
	{},
};

static struct spi_driver fts_ts_spi_driver = {
	.probe = fts_ts_spi_probe,
	.remove = fts_ts_spi_remove,
	.driver = {
		.name = FTS_DRIVER_NAME,
		.owner = THIS_MODULE,
#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
		.pm = &fts_dev_pm_ops,
#endif
		.of_match_table = of_match_ptr(fts_dt_match),
	},
	.id_table = fts_ts_spi_id,
};

static int __init fts_ts_spi_init(void)
{
	int ret = 0;

	FTS_FUNC_ENTER();
	ret = spi_register_driver(&fts_ts_spi_driver);
	if (ret != 0)
		FTS_ERROR("Focaltech touch screen driver init failed!");

	FTS_FUNC_EXIT();
	return ret;
}

static void __exit fts_ts_spi_exit(void)
{
	spi_unregister_driver(&fts_ts_spi_driver);
}

static int __init fts_ts_init(void)
{
	int ret = 0;

#if FTS_SUPPORT_I2C
	ret = fts_ts_i2c_init();
	if (ret)
		FTS_ERROR("Focaltech I2C driver init failed!");
#endif

	ret = fts_ts_spi_init();
	if (ret)
		FTS_ERROR("Focaltech SPI driver init failed!");

	return ret;
}
late_initcall(fts_ts_init);

static void __exit fts_ts_exit(void)
{
#if FTS_SUPPORT_I2C
	fts_ts_i2c_exit();
#endif
	fts_ts_spi_exit();
}
module_exit(fts_ts_exit);

MODULE_AUTHOR("FocalTech Driver Team");
MODULE_DESCRIPTION("FocalTech Touchscreen Driver");
MODULE_LICENSE("GPL v2");
