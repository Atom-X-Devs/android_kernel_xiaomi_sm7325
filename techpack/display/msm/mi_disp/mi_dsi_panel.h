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
enum panel_state {
	PANEL_STATE_OFF = 0,
	PANEL_STATE_ON,
	PANEL_STATE_DOZE_HIGH,
	PANEL_STATE_DOZE_LOW,
	PANEL_STATE_DOZE_TO_NORMAL,
	PANEL_STATE_MAX,
};

enum brightness_mode {
	BRIGHTNESS_NORMAL_MODE,
	BRIGHTNESS_NORMAL_AOD_MODE,
	BRIGHTNESS_MODE_MAX
};

enum aod_brightness_level {
	AOD_LBM_LEVEL,
	AOD_HBM_LEVEL,
	AOD_LEVEL_MAX
};

struct mi_dsi_panel_cfg {
	struct dsi_panel *dsi_panel;

	/* xiaomi panel id */
	u64 panel_id;

	/* xiaomi feature values */
	int feature_val[DISP_FEATURE_MAX];

	u32 last_bl_level;
	u32 last_no_zero_bl_level;

	/* indicate refresh frequency Fps gpio */
	int disp_rate_gpio;

	/* DC sync TE */
	bool dc_enabled;

	/* indicate esd check gpio and config irq */
	int esd_err_irq_gpio;
	int esd_err_irq;
	int esd_err_irq_flags;
	bool esd_err_enabled;
	bool timming_switch_wait_for_te;

	u32 doze_brightness;
	bool bl_wait_frame;
	bool bl_enable;
	struct wakeup_source *doze_ws;
	bool doze_to_off_command_enabled;

	bool hbm_51_ctl_flag;
	int hbm_on_51_index;
	int hbm_off_51_index;
	int hbm_bl_min_lvl;
	int hbm_bl_max_lvl;
	int hbm_brightness_flag;

	u32 panel_on_dimming_delay;
	u32 panel_status_check_interval;

	u32 dimming_state;

	bool aod_bl_51_ctl;
	bool bl_51_ctl_32_bit;

	u32 dc_type;
	u32 dc_threshold;
	u32 brightness_clone;
	u32 real_brightness_clone;
	u32 max_brightness_clone;
	u32 thermal_max_brightness_clone;
	bool thermal_dimming;

	int doze_hbm_dbv_level;
	int doze_lbm_dbv_level;

	int panel_state;
	bool bl_need_update;

	int aod_hbm_51_index;
	int aod_lbm_51_index;

	u32 aod_exit_delay_time;
	u64 aod_enter_time;
	u32 aod_bl_val[AOD_LEVEL_MAX];

	bool gir_enabled;
	bool aod_to_normal_status;
};

struct dsi_read_config {
	bool is_read;
	struct dsi_panel_cmd_set read_cmd;
	u32 cmds_rlen;
	u8 rbuf[256];
};

extern struct dsi_read_config g_dsi_read_cfg;

int mi_dsi_panel_init(struct dsi_panel *panel);
int mi_dsi_panel_deinit(struct dsi_panel *panel);
int mi_dsi_acquire_wakelock(struct dsi_panel *panel);
int mi_dsi_release_wakelock(struct dsi_panel *panel);

bool is_aod_and_panel_initialized(struct dsi_panel *panel);

bool is_backlight_set_skip(struct dsi_panel *panel);

int mi_dsi_panel_esd_irq_ctrl(struct dsi_panel *panel, bool enable);
int mi_dsi_panel_esd_irq_ctrl_locked(struct dsi_panel *panel, bool enable);

int mi_dsi_panel_write_cmd_set(struct dsi_panel *panel,	struct dsi_panel_cmd_set *cmd_sets);
int mi_dsi_panel_read_cmd_set(struct dsi_panel *panel, struct dsi_read_config *read_config);

int mi_dsi_panel_set_disp_param(struct dsi_panel *panel, struct disp_feature_ctl *ctl);
int mi_dsi_panel_get_disp_param(struct dsi_panel *panel, struct disp_feature_ctl *ctl);
ssize_t mi_dsi_panel_show_disp_param(struct dsi_panel *panel, char *buf, size_t size);

int mi_dsi_panel_set_doze_brightness(struct dsi_panel *panel, u32 doze_brightness);
int mi_dsi_panel_get_doze_brightness(struct dsi_panel *panel, u32 *doze_brightness);
int mi_dsi_panel_get_brightness(struct dsi_panel *panel, u32 *brightness);

int mi_dsi_panel_write_dsi_cmd(struct dsi_panel *panel,	struct dsi_cmd_rw_ctl *ctl);
int mi_dsi_panel_write_dsi_cmd_set(struct dsi_panel *panel, int type);
ssize_t mi_dsi_panel_show_dsi_cmd_set_type(struct dsi_panel *panel,	char *buf, size_t size);

void mi_dsi_panel_update_last_bl_level(struct dsi_panel *panel,	int brightness);
void mi_dsi_dc_mode_enable(struct dsi_panel *panel,	bool enable);

int mi_dsi_panel_set_brightness_clone(struct dsi_panel *panel, u32 brightness_clone);
int mi_dsi_panel_get_brightness_clone(struct dsi_panel *panel, u32 *brightness_clone);
int mi_dsi_panel_get_max_brightness_clone(struct dsi_panel *panel, u32 *max_brightness_clone);

void mi_dsi_update_backlight_in_aod(struct dsi_panel *panel, bool restore_backlight);

#endif /* _MI_DSI_PANEL_H_ */
