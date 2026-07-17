// SPDX-License-Identifier: GPL-2.0-only
/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include "goodix_ts_core.h"

bool debug_log_flag;

u32 goodix_append_checksum(u8 *data, int len, int mode)
{
	u32 checksum = 0;
	int i;

	checksum = 0;
	if (mode == CHECKSUM_MODE_U8_LE) {
		for (i = 0; i < len; i++)
			checksum += data[i];
	} else {
		for (i = 0; i < len; i += 2)
			checksum += (data[i] + (data[i + 1] << 8));
	}

	if (mode == CHECKSUM_MODE_U8_LE) {
		data[len] = checksum & 0xff;
		data[len + 1] = (checksum >> 8) & 0xff;
		return 0xFFFF & checksum;
	}
	data[len] = checksum & 0xff;
	data[len + 1] = (checksum >> 8) & 0xff;
	data[len + 2] = (checksum >> 16) & 0xff;
	data[len + 3] = (checksum >> 24) & 0xff;
	return checksum;
}

int checksum_cmp(const u8 *data, int size, int mode)
{
	u32 cal_checksum = 0;
	u32 r_checksum = 0;
	u32 i;

	if (mode == CHECKSUM_MODE_U8_LE) {
		if (size < 2)
			return 1;
		for (i = 0; i < size - 2; i++)
			cal_checksum += data[i];
		r_checksum = data[size - 2] + (data[size - 1] << 8);
		return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
	}

	if (size < 4)
		return 1;
	for (i = 0; i < size - 4; i += 2)
		cal_checksum += data[i] + (data[i + 1] << 8);
	r_checksum = data[size - 4] + (data[size - 3] << 8) +
		     (data[size - 2] << 16) + (data[size - 1] << 24);
	return cal_checksum == r_checksum ? 0 : 1;
}

/* return 1 if all data is zero or ff
 * else return 0
 */
int is_risk_data(struct goodix_ts_core *cd, const u8 *data, int size)
{
	int i;
	int zero_count = 0;
	int ff_count = 0;
	struct device *dev = cd->bus->dev;

	for (i = 0; i < size; i++) {
		if (data[i] == 0)
			zero_count++;
		else if (data[i] == 0xFF)
			ff_count++;
	}
	if (zero_count == size || ff_count == size) {
		ts_info(dev, "warning data is all %s",
			zero_count == size ? "0x00" : "0xFF");
		return 1;
	}

	return 0;
}

/* get config id form config file */
#define CONFIG_ID_OFFSET 		30
u32 goodix_get_file_config_id(u8 *ic_config)
{
	if (!ic_config)
		return 0;

	return le32_to_cpup((__le32 *)&ic_config[CONFIG_ID_OFFSET]);
}

#if 0
void print_ic_info(struct goodix_ts_core *cd, struct goodix_ic_info *ic_info)
{
	struct device *dev = cd->bus->dev;
	struct goodix_ic_info_version *version = &ic_info->version;
	struct goodix_ic_info_feature *feature = &ic_info->feature;
	struct goodix_ic_info_param *parm = &ic_info->parm;
	struct goodix_ic_info_misc *misc = &ic_info->misc;
	struct goodix_ic_info_other *other = &ic_info->other;

	ts_info(dev, "ic_info_length:                %d", ic_info->length);
	ts_info(dev, "info_customer_id:              0x%01X",
		version->info_customer_id);
	ts_info(dev, "info_version_id:               0x%01X",
		version->info_version_id);
	ts_info(dev, "ic_die_id:                     0x%01X", version->ic_die_id);
	ts_info(dev, "ic_version_id:                 0x%01X",
		version->ic_version_id);
	ts_info(dev, "config_id:                     0x%4X", version->config_id);
	ts_info(dev, "config_version:                0x%01X",
		version->config_version);
	ts_info(dev, "frame_data_customer_id:        0x%01X",
		version->frame_data_customer_id);
	ts_info(dev, "frame_data_version_id:         0x%01X",
		version->frame_data_version_id);
	ts_info(dev, "touch_data_customer_id:        0x%01X",
		version->touch_data_customer_id);
	ts_info(dev, "touch_data_version_id:         0x%01X",
		version->touch_data_version_id);

	ts_info(dev, "freqhop_feature:               0x%04X",
		feature->freqhop_feature);
	ts_info(dev, "calibration_feature:           0x%04X",
		feature->calibration_feature);
	ts_info(dev, "gesture_feature:               0x%04X",
		feature->gesture_feature);
	ts_info(dev, "side_touch_feature:            0x%04X",
		feature->side_touch_feature);
	ts_info(dev, "stylus_feature:                0x%04X",
		feature->stylus_feature);

	ts_info(dev, "Drv*Sen,Button,Force num:      %d * %d, %d, %d", parm->drv_num,
		parm->sen_num, parm->button_num, parm->force_num);

	ts_info(dev, "screen_max_x * screen_max_y:   %d * %d", other->screen_max_x,
		other->screen_max_y);

	ts_info(dev, "Cmd:                           0x%04X, %d", misc->cmd_addr,
		misc->cmd_max_len);
	ts_info(dev, "Cmd-Reply:                     0x%04X, %d",
		misc->cmd_reply_addr, misc->cmd_reply_len);
	ts_info(dev, "FW-State:                      0x%04X, %d",
		misc->fw_state_addr, misc->fw_state_len);
	ts_info(dev, "FW-Buffer:                     0x%04X, %d",
		misc->fw_buffer_addr, misc->fw_buffer_max_len);
	ts_info(dev, "Touch-Data:                    0x%04X, %d",
		misc->touch_data_addr, misc->touch_data_head_len);
	ts_info(dev, "point_struct_len:              %d", misc->point_struct_len);
	ts_info(dev, "frame_data_addr:               0x%04X", misc->frame_data_addr);
	ts_info(dev, "mutual_rawdata_addr:           0x%04X",
		misc->mutual_rawdata_addr);
	ts_info(dev, "mutual_diffdata_addr:          0x%04X",
		misc->mutual_diffdata_addr);
	ts_info(dev, "self_rawdata_addr:             0x%04X",
		misc->self_rawdata_addr);
	ts_info(dev, "self_diffdata_addr:            0x%04X",
		misc->self_diffdata_addr);
	ts_info(dev, "stylus_rawdata_addr:           0x%04X, %d",
		misc->stylus_rawdata_addr, misc->stylus_rawdata_len);
	ts_info(dev, "esd_addr:                      0x%04X", misc->esd_addr);
}

/* matrix transpose */
void goodix_rotate_abcd2cbad(int tx, int rx, s16 *data)
{
	s16 *temp_buf = NULL;
	int size = tx * rx;
	int i;
	int j;
	int col;

	temp_buf = kcalloc(size, sizeof(s16), GFP_KERNEL);
	if (!temp_buf)
		return;

	for (i = 0, j = 0, col = 0; i < size; i++) {
		temp_buf[i] = data[j++ * rx + col];
		if (j == tx) {
			j = 0;
			col++;
		}
	}

	memcpy(data, temp_buf, size * sizeof(s16));
	kfree(temp_buf);
}
#endif

/* get ic type */
int goodix_get_ic_type(struct device *dev,
		       struct goodix_bus_interface *bus_inf)
{
	struct device_node *node = dev->of_node;
	const void *ts_data;

	ts_data = of_device_get_match_data(dev);
	if (!ts_data) {
		ts_err(dev, "unsupported ic type\n");
		return -EINVAL;
	}

	bus_inf->ic_type = (uintptr_t)ts_data;

	if (bus_inf->ic_type == IC_TYPE_BERLIN_A) {
		dev_info(dev, "ic type is brl-a");
	} else if (bus_inf->ic_type == IC_TYPE_BERLIN_B) {
		dev_info(dev, "ic type is brl-b");
		if (of_device_is_compatible(node, "goodix,ga687x")) {
			bus_inf->sub_ic_type = IC_TYPE_SUB_B2;
			dev_info(dev, "sub ic type is brl-b2");
		}
	} else if (bus_inf->ic_type == IC_TYPE_BERLIN_D) {
		dev_info(dev, "ic type is brl-d");
	} else if (bus_inf->ic_type == IC_TYPE_NOTTINGHAM) {
		dev_info(dev, "ic type is nottingham");
	} else if (bus_inf->ic_type == IC_TYPE_MARSEILLE) {
		dev_info(dev, "ic type is marseille");
	} else {
		dev_info(dev, "ic type is Atlanta B");
	}

	return 0;
}

char *find_file_prefix(const char *file_name)
{
    static char result[64];
	char *dot;

	memset(result, 0, sizeof(result));
    dot = strrchr(file_name, '.');
    if (dot) {
		strncpy(result, file_name, dot - file_name);
	}

	return result;
}

#if 0
void ts_info(struct device *dev, const char *fmt, ...)
{
	va_list args;
    char str[256];

    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

	if (dev)
		dev_info(dev, "[GTP-INF] %s\n", str);
	else
		pr_info("gtx8_common:     [GTP-INF] %s\n", str);
}

void ts_err(struct device *dev, const char *fmt, ...)
{
	va_list args;
    char str[256];

    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

	if (dev)
		dev_err(dev, "[GTP-ERR] %s\n", str);
	else
		pr_err("gtx8_common:     [GTP-ERR] %s\n", str);
}

void ts_debug(struct device *dev, const char *fmt, ...)
{
	va_list args;
    char str[256];

	if (!debug_log_flag)
		return;

    va_start(args, fmt);
    vsnprintf(str, sizeof(str), fmt, args);
    va_end(args);

	if (dev)
		dev_dbg(dev, "[GTP_DBG] %s\n", str);
	else
		pr_debug("gtx8_common:     [GTP_DBG] %s\n", str);
}
#endif