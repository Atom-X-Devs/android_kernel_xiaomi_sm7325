/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#ifndef _MI_DSI_PANEL_H_
#define _MI_DSI_PANEL_H_

#include <linux/types.h>

#include <drm/mi_disp.h>

#include "dsi_panel.h"
#include "dsi_defs.h"
#include "mi_disp_feature.h"
#include <linux/pm_wakeup.h>
#include "drm_mipi_dsi.h"

enum bkl_dimming_state {
	STATE_NONE,
	STATE_DIM_BLOCK,
	STATE_DIM_RESTORE,
	STATE_ALL
};

/* Panel flag need update when panel power state changed*/
enum panel_flag_update {
	PANEL_OFF,
	PANEL_ON,
	PANEL_LP1,
	PANEL_LP2,
	PANEL_NOLP,
	PANEL_DOZE_HIGH,
	PANEL_DOZE_LOW,
	PANEL_MAX
};

enum panel_state {
	PANEL_STATE_OFF = 0,
	PANEL_STATE_ON,
	PANEL_STATE_DOZE_HIGH,
	PANEL_STATE_DOZE_LOW,
	PANEL_STATE_MAX,
};

struct mi_dsi_panel_cfg {
	struct dsi_panel *dsi_panel;

	/* xiaomi panel id */
	u64 mi_panel_id;

	/* xiaomi feature values */
	int feature_val[DISP_FEATURE_MAX];

	u32 last_bl_level;
	u32 last_no_zero_bl_level;

	/* indicate esd check gpio and config irq */
	int esd_err_irq_gpio;
	int esd_err_irq;
	int esd_err_irq_flags;
	bool esd_err_enabled;

	u32 doze_brightness;
	bool is_doze_to_off;
	bool bl_enable;

	/* Some panel nolp command is different according to current doze brightness set,
	 * But sometimes doze brightness change to DOZE_TO_NORMAL before nolp. So this
	 * doze_brightness_backup will save doze_brightness and only change to DOZE_TO_NORMAL by nolp.
	 */
	u32 doze_brightness_backup;
	struct wakeup_source *doze_wakelock;

	bool hbm_51_ctl_flag;
	int hbm_on_51_index;
	int hbm_off_51_index;
	int hbm_bl_min_lvl;
	int hbm_bl_max_lvl;

	u32 panel_on_dimming_delay;
	u32 dimming_state;
	u32 dc_type;
	u32 dc_threshold;
	u32 brightness_clone;
	u32 real_brightness_clone;
	u32 max_brightness_clone;
	u32 thermal_max_brightness_clone;
	bool thermal_dimming_enabled;

	int panel_state;
	bool bl_need_update;
	bool gir_enabled;
};

struct dsi_read_config {
	bool is_read;
	struct dsi_panel_cmd_set read_cmd;
	u32 cmds_rlen;
	u8 rbuf[256];
};

extern struct dsi_read_config g_dsi_read_cfg;
extern const char *cmd_set_prop_map[DSI_CMD_SET_MAX];

int mi_dsi_panel_init(struct dsi_panel *panel);
int mi_dsi_panel_deinit(struct dsi_panel *panel);
int mi_dsi_acquire_wakelock(struct dsi_panel *panel);
int mi_dsi_release_wakelock(struct dsi_panel *panel);

bool is_aod_and_panel_initialized(struct dsi_panel *panel);

bool is_backlight_set_skip(struct dsi_panel *panel, u32 bl_lvl);

int mi_dsi_panel_esd_irq_ctrl(struct dsi_panel *panel,
			bool enable);

int mi_dsi_panel_write_cmd_set(struct dsi_panel *panel,
			struct dsi_panel_cmd_set *cmd_sets);

int mi_dsi_panel_read_cmd_set(struct dsi_panel *panel,
			struct dsi_read_config *read_config);

int mi_dsi_panel_write_mipi_reg(struct dsi_panel *panel,
			char *buf);

ssize_t mi_dsi_panel_read_mipi_reg(struct dsi_panel *panel,
			char *buf, size_t size);

bool mi_dsi_panel_is_need_tx_cmd(u32 feature_id);

int mi_dsi_panel_set_disp_param(struct dsi_panel *panel,
			struct disp_feature_ctl *ctl);

ssize_t mi_dsi_panel_get_disp_param(struct dsi_panel *panel,
			char *buf, size_t size);

int mi_dsi_panel_set_doze_brightness(struct dsi_panel *panel,
			u32 doze_brightness);

int mi_dsi_panel_get_doze_brightness(struct dsi_panel *panel,
			u32 *doze_brightness);

int mi_dsi_panel_get_brightness(struct dsi_panel *panel,
			u32 *brightness);

int mi_dsi_panel_write_dsi_cmd(struct dsi_panel *panel,
			struct dsi_cmd_rw_ctl *ctl);

int mi_dsi_panel_write_dsi_cmd_set(struct dsi_panel *panel, int type);

ssize_t mi_dsi_panel_show_dsi_cmd_set_type(struct dsi_panel *panel,
			char *buf, size_t size);

void mi_dsi_panel_update_last_bl_level(struct dsi_panel *panel,
			int brightness);

void mi_dsi_update_micfg_flags(struct dsi_panel *panel,
			int power_mode);

void mi_dsi_dc_mode_enable(struct dsi_panel *panel,
			bool enable);

int mi_dsi_panel_set_brightness_clone(struct dsi_panel *panel,
			u32 brightness_clone);

int mi_dsi_panel_get_brightness_clone(struct dsi_panel *panel,
			u32 *brightness_clone);

#endif /* _MI_DSI_PANEL_H_ */
