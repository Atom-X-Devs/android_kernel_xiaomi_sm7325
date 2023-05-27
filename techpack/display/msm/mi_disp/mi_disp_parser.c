/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#define pr_fmt(fmt)	"mi-disp-parse:[%s:%d] " fmt, __func__, __LINE__
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "dsi_panel.h"
#include "dsi_parser.h"

#include "mi_disp_print.h"
#include <linux/soc/qcom/smem.h>

#define SMEM_SW_DISPLAY_DEMURA_TABLE 498
#define SMEM_SW_DISPLAY_LHBM_TABLE 499
#define SMEM_SW_DISPLAY_VDC_TABLE 500

#define DEFAULT_HBM_BL_MIN_LEVEL 1
#define DEFAULT_HBM_BL_MAX_LEVEL 2047
#define DEFAULT_MAX_BRIGHTNESS_CLONE 4095

int mi_dsi_panel_parse_esd_gpio_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	mi_cfg->esd_err_irq_gpio = of_get_named_gpio_flags(
			utils->data, "mi,esd-err-irq-gpio",
			0, (enum of_gpio_flags *)&(mi_cfg->esd_err_irq_flags));
	if (gpio_is_valid(mi_cfg->esd_err_irq_gpio)) {
		mi_cfg->esd_err_irq = gpio_to_irq(mi_cfg->esd_err_irq_gpio);
		rc = gpio_request(mi_cfg->esd_err_irq_gpio, "esd_err_irq_gpio");
		if (rc)
			DISP_ERROR("Failed to request esd irq gpio %d, rc=%d\n",
				mi_cfg->esd_err_irq_gpio, rc);
		else
			gpio_direction_input(mi_cfg->esd_err_irq_gpio);
	} else {
		rc = -EINVAL;
	}

	return rc;
}

static int mi_dsi_panel_parse_hbm_51_ctl_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	mi_cfg->hbm_51_ctl_flag = utils->read_bool(utils->data, "mi,hbm-51-ctl-flag");
	if (mi_cfg->hbm_51_ctl_flag) {
		DISP_INFO("mi,hbm-51-ctl-flag is defined\n");
		rc = utils->read_u32(utils->data, "mi,hbm-on-51-index", &mi_cfg->hbm_on_51_index);
		if (rc) {
			mi_cfg->hbm_on_51_index = -1;
			DISP_INFO("mi,hbm-on-51-index not specified\n");
		} else {
			DISP_INFO("mi,hbm-on-51-index is %d\n", mi_cfg->hbm_on_51_index);
		}

		rc = utils->read_u32(utils->data, "mi,hbm-off-51-index", &mi_cfg->hbm_off_51_index);
		if (rc) {
			mi_cfg->hbm_off_51_index = -1;
			DISP_INFO("mi,hbm-off-51-index not specified\n");
		} else {
			DISP_INFO("mi,hbm-off-51-index is %d\n", mi_cfg->hbm_off_51_index);
		}

		rc = utils->read_u32(utils->data, "mi,hbm-bl-min-level", &mi_cfg->hbm_bl_min_lvl);
		if (rc) {
			mi_cfg->hbm_bl_min_lvl = DEFAULT_HBM_BL_MIN_LEVEL;
			DISP_INFO("mi,hbm-bl-min-level not specified, default:%d\n", DEFAULT_HBM_BL_MIN_LEVEL);
		} else {
			DISP_INFO("mi,hbm-bl-min-level is %d\n", mi_cfg->hbm_bl_min_lvl);
		}

		rc = utils->read_u32(utils->data, "mi,hbm-bl-max-level", &mi_cfg->hbm_bl_max_lvl);
		if (rc) {
			mi_cfg->hbm_bl_max_lvl = DEFAULT_HBM_BL_MAX_LEVEL;
			DISP_INFO("mi,hbm-bl-max-level not specified, default:%d\n", DEFAULT_HBM_BL_MAX_LEVEL);
		} else {
			DISP_INFO("mi,hbm-bl-max-level is %d\n", mi_cfg->hbm_bl_max_lvl);
		}
	} else {
		DISP_DEBUG("mi,hbm-51-ctl-flag not defined\n");
	}

	return rc;
}


static int mi_dsi_panel_parse_dc_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	rc = utils->read_u32(utils->data, "mi,mdss-dsi-panel-dc-type", &mi_cfg->dc_type);
	if (rc) {
		mi_cfg->dc_type = 1;
		DISP_INFO("default dc backlight type is %d\n", mi_cfg->dc_type);
	} else {
		DISP_INFO("dc backlight type %d \n", mi_cfg->dc_type);
	}

	return rc;
}

int mi_dsi_panel_parse_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;
	mi_cfg->dsi_panel = panel;
	mi_cfg->bl_wait_frame = false;
	mi_cfg->bl_enable = true;

	rc = utils->read_u64(utils->data, "mi,panel-id", &mi_cfg->panel_id);
	if (rc) {
		mi_cfg->panel_id = 0;
		DISP_INFO("mi,panel-id not specified\n");
	}

	mi_dsi_panel_parse_hbm_51_ctl_config(panel);
	mi_dsi_panel_parse_dc_config(panel);

	rc = utils->read_u32(utils->data, "mi,panel-on-dimming-delay", &mi_cfg->panel_on_dimming_delay);
	if (rc) {
		mi_cfg->panel_on_dimming_delay = 0;
		DISP_INFO("mi,panel-on-dimming-delay not specified\n");
	} else {
		DISP_INFO("mi,panel-on-dimming-delay is %d\n", mi_cfg->panel_on_dimming_delay);
	}

	mi_cfg->timming_switch_wait_for_te = utils->read_bool(utils->data, "mi,timming-switch-wait-for-te-flag");
	if (mi_cfg->timming_switch_wait_for_te) {
		DISP_INFO("mi timming-switch-wait-for-te-flag get\n");
	}

	mi_cfg->aod_bl_51_ctl = utils->read_bool(utils->data, "mi,aod-bl-51ctl-flag");
	rc = utils->read_u32(utils->data, "mi,aod-hbm-51-index", &mi_cfg->aod_hbm_51_index);
	if (rc) {
		mi_cfg->aod_hbm_51_index = -1;
		DISP_INFO("mi,aod-hbm-51-index not specified\n");
	} else {
		DISP_INFO("mi,aod-hbm-51-index is %d\n", mi_cfg->aod_hbm_51_index);
	}

	rc = utils->read_u32(utils->data, "mi,aod-lbm-51-index", &mi_cfg->aod_lbm_51_index);
	if (rc) {
		mi_cfg->aod_lbm_51_index = -1;
		DISP_INFO("mi,aod-lbm-51-index not specified\n");
	} else {
		DISP_INFO("mi,aod-lbm-51-index is %d\n", mi_cfg->aod_lbm_51_index);
	}

	mi_cfg->thermal_dimming = utils->read_bool(utils->data, "mi,thermal-dimming-flag");
	if (mi_cfg->thermal_dimming) {
		DISP_INFO("thermal_dimming enabled\n");
	}

	rc = utils->read_u32(utils->data, "mi,max-brightness-clone", &mi_cfg->max_brightness_clone);
	if (rc)
		mi_cfg->max_brightness_clone = DEFAULT_MAX_BRIGHTNESS_CLONE;

	DISP_INFO("max_brightness_clone=%d\n", mi_cfg->max_brightness_clone);

	rc = utils->read_u32(utils->data, "mi,aod-exit-delay-time", &mi_cfg->aod_exit_delay_time);
	if (rc)
		mi_cfg->aod_exit_delay_time = 0;

	DISP_INFO("aod exit delay %d\n", mi_cfg->aod_exit_delay_time);

	mi_cfg->bl_51_ctl_32_bit = utils->read_bool(utils->data, "mi,bl-51ctl-32bit-flag");
	rc = utils->read_u32_array(utils->data,
				"mi,aod-brightness", mi_cfg->aod_bl_val, AOD_LEVEL_MAX);
	if (rc)
		memset(mi_cfg->aod_bl_val, 0, AOD_LEVEL_MAX*sizeof(u32));

	mi_cfg->doze_to_off_command_enabled = utils->read_bool(utils->data, "mi,panel-aod-to-off-command-need-enabled");
	if (mi_cfg->doze_to_off_command_enabled)
		DISP_INFO("mi,panel-aod-to-off-command-need-enabled\n");

	return rc;
}

