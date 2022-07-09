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

#define DEMURA_BL_LEVEL_MAX 10
#define BL_STATISTIC_CNT_MAX 10

enum bkl_dimming_state {
	STATE_NONE,
	STATE_DIM_BLOCK,
	STATE_DIM_RESTORE,
	STATE_ALL
};

/* 90Hz gamma and 144Hz gamma info */
struct gamma_cfg {
	bool read_done;
	/* 144Hz gamma info */
	u8 otp_read_b8[44];
	u8 otp_read_b9[237];
	u8 otp_read_ba[63];

	u32 flash_read_total_param;
	u64 gamma_checksum;
	u8 flash_gamma_read[346];
	/* 90Hz gamma info */
	u8 flash_read_b8[44];
	u8 flash_read_b9[237];
	u8 flash_read_ba[63];
	u8 flash_read_checksum[2];

	int update_b8_index;
	int update_b9_index;
	int update_ba_index;

	bool update_done_90hz;
	bool update_done_144hz;
};

struct flatmode_cfg {
	bool read_done;
	int update_index;
	u8 flatmode_param[4];
};

struct lhbm_rgb_cfg {
	u32 lhbm_1000nit_rgb[3];
	u32 lhbm_750nit_rgb[3];
	u32 lhbm_500nit_rgb[3];
	u32 lhbm_110nit_rgb[3];
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
	u64 panel_id;

	/* xiaomi feature values */
	int feature_val[DISP_FEATURE_MAX];

	/* bl_is_big_endian indicate brightness value
	 * high byte to 1st parameter, low byte to 2nd parameter
	 * eg: 0x51 { 0x03, 0xFF } ->
	 * u8 payload[2] = { brightness >> 8, brightness & 0xff}
	 */
	bool bl_is_big_endian;
	u32 last_bl_level;
	u32 last_no_zero_bl_level;

	/* indicate refresh frequency Fps gpio */
	int disp_rate_gpio;

	/* gamma read */
	bool gamma_update_flag;
	struct gamma_cfg gamma_cfg;
	char demura_data[900];
	struct lhbm_rgb_cfg lhbm_rgb_cfg;

	/* flatmode read */
	bool flatmode_update_flag;
	struct flatmode_cfg flatmode_cfg;

	/* indicate esd check gpio and config irq */
	int esd_err_irq_gpio;
	int esd_err_irq;
	int esd_err_irq_flags;
	bool esd_err_enabled;

	u32 doze_brightness;
	bool doze_to_off_command_enabled;
	bool timming_switch_wait_for_te;
	bool is_doze_to_off;
	/* Some panel nolp command is different according to current doze brightness set,
	 * But sometimes doze brightness change to DOZE_TO_NORMAL before nolp. So this
	 * doze_brightness_backup will save doze_brightness and only change to DOZE_TO_NORMAL by nolp.
	 */
	u32 doze_brightness_backup;
	struct wakeup_source *doze_wakelock;

	bool hbm_51_ctl_flag;
	int hbm_on_51_index;
	int hbm_off_51_index;
	int hbm_fod_on_51_index;
	int hbm_fod_off_51_index;
	int hbm_fod_bl_lvl;
	int hbm_bl_min_lvl;
	int hbm_bl_max_lvl;
	int hbm_brightness_flag;
	int local_hbm_on_87_index;
	int local_hbm_hlpm_on_87_index;

	bool in_fod_calibration;

	u32 panel_on_dimming_delay;

	/* AOD Nolp code customized*/
	bool aod_nolp_command_enabled;

	bool fod_hbm_layer_enabled;
	bool fod_anim_layer_enabled;
	u32 fod_ui_ready;

	bool delay_before_fod_hbm_on;
	bool delay_before_fod_hbm_off;

	u32 dimming_state;

	bool aod_bl_51ctl;
	bool dfps_bl_ctrl;
	u32 dfps_bl_threshold;

	u32 dc_type;
	u32 dc_threshold;
	u32 brightness_clone;
	u32 real_brightness_clone;
	u32 max_brightness_clone;
	u32 thermal_max_brightness_clone;
	bool thermal_dimming;

	bool local_hbm_enabled;
	int local_hbm_on_1000nit_51_index;
	int local_hbm_off_to_hbm_51_index;
	int local_hbm_off_to_normal_51_index;
	u32 fod_low_brightness_clone_threshold;
	u32 fod_low_brightness_lux_threshold;
	int local_hbm_target;

	bool nvt_bic_enabled;
	int nvt_bic_post_on_d0_index;
	bool nvt_bic_reg_transfer_finshed;
	char bic_reg_data[9];

	u32 fod_type;
	bool fp_display_on_optimize;

	bool demura_comp;
	u32 demura_bl_num;
	u32 demura_mask;
	u32 demura_bl[DEMURA_BL_LEVEL_MAX];

	int pending_lhbm_state;

	int doze_hbm_dbv_level;
	int doze_lbm_dbv_level;

	int panel_state;
	bool local_hbm_to_normal;
	int bl_statistic_cnt;
	bool bl_need_update;

	int aod_hbm_51_index;
	int aod_lbm_51_index;

	u32 aod_exit_delay_time;
	u64 aod_enter_time;

	char *bic_data;
	int bic_data_size;

	u32 hbm_backlight_threshold;
	bool gir_enabled;
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

bool is_backlight_set_skip(struct dsi_panel *panel, u32 bl_lvl);

bool is_hbm_fod_on(struct dsi_panel *panel);

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

int mi_dsi_panel_read_gamma_param(struct dsi_panel *panel);

int mi_dsi_panel_update_gamma_param(struct dsi_panel *panel);

ssize_t mi_dsi_panel_print_gamma_param(struct dsi_panel *panel,
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

int mi_dsi_panel_update_dc_status(struct dsi_panel *panel,
			int brightness);

void mi_dsi_panel_update_last_bl_level(struct dsi_panel *panel,
			int brightness);

void mi_dsi_update_micfg_flags(struct dsi_panel *panel,
			int power_mode);

int mi_dsi_panel_nolp(struct dsi_panel *panel);

void mi_dsi_dc_mode_enable(struct dsi_panel *panel,
			bool enable);

int mi_dsi_fps_switch(struct dsi_panel *panel);

int mi_dsi_set_bic_reg(struct dsi_panel *panel);

int mi_dsi_panel_set_brightness_clone(struct dsi_panel *panel,
			u32 brightness_clone);

int mi_dsi_panel_get_brightness_clone(struct dsi_panel *panel,
			u32 *brightness_clone);

void mi_dsi_panel_demura_comp(struct dsi_panel *panel,
			u32 bl_lvl);

int mi_dsi_panel_demura_set(struct dsi_panel *panel);

void mi_dsi_panel_dc_vi_setting(struct dsi_panel *panel,
			u32 bl_lvl);

int mi_disp_set_fod_queue_work(u32 fod_btn, bool from_touch);

void mi_dsi_update_backlight_in_aod(struct dsi_panel *panel, bool restore_backlight);

void mi_dsi_update_dc_backlight(struct dsi_panel *panel, u32 bl_lvl);

void mi_dsi_backlight_logging(struct dsi_panel *panel, u32 bl_lvl);

void mi_disp_handle_lp_event(struct dsi_panel *panel, int power_mode);

int mi_dsi_panel_lhbm_set(struct dsi_panel *panel);

int mi_dsi_panel_read_nvt_bic(struct dsi_panel *panel);

char *mi_dsi_panel_get_bic_data_info(int * bic_len);

char *mi_dsi_panel_get_bic_reg_data_array(struct dsi_panel *panel);

int mi_dsi_panel_read_flatmode_param(struct dsi_panel *panel);

#endif /* _MI_DSI_PANEL_H_ */
