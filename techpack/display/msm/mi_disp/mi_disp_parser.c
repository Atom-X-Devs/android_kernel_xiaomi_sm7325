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

#include "mi_panel_id.h"
#include "mi_disp_print.h"

#include <linux/soc/qcom/smem.h>

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

static int mi_dsi_panel_parse_dc_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	rc = utils->read_u32(utils->data, "mi,mdss-dsi-panel-dc-type", &mi_cfg->dc_type);
	if (rc)
		mi_cfg->dc_type = 1;

	DISP_INFO("default dc backlight type is %d\n", mi_cfg->dc_type);

	return rc;
}

static int mi_dsi_panel_parse_backlight_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	mi_cfg->bl_enable = true;

	rc = utils->read_u32(utils->data, "mi,panel-on-dimming-delay", &mi_cfg->panel_on_dimming_delay);
	if (rc) {
		mi_cfg->panel_on_dimming_delay = 0;
		DISP_INFO("mi,panel-on-dimming-delay not specified\n");
	} else {
		DISP_INFO("mi,panel-on-dimming-delay is %d\n", mi_cfg->panel_on_dimming_delay);
	}

	rc = utils->read_u32(utils->data, "mi,max-brightness-clone", &mi_cfg->max_brightness_clone);
	if (rc)
		mi_cfg->max_brightness_clone = DEFAULT_MAX_BRIGHTNESS_CLONE;

	DISP_INFO("max_brightness_clone=%d\n", mi_cfg->max_brightness_clone);

	mi_cfg->thermal_dimming_enabled = utils->read_bool(utils->data, "mi,thermal-dimming-flag");
	if (mi_cfg->thermal_dimming_enabled)
		DISP_INFO("thermal_dimming enabled\n");

	/* sensor lux init to 50000 to avoid sensor not update value */
	mi_cfg->feature_val[DISP_FEATURE_SENSOR_LUX] = 50000;

	return rc;
}

static int  mi_dsi_panel_parse_hbm_51_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	mi_cfg->hbm_51_ctl_flag = utils->read_bool(utils->data, "mi,hbm-51-ctl-flag");

	if (mi_cfg->hbm_51_ctl_flag) {
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
	}

	return rc;
}

int mi_dsi_panel_parse_config(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_parser_utils *utils = &panel->utils;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	rc = utils->read_u64(utils->data, "mi,panel-id", &mi_cfg->mi_panel_id);
	if (rc) {
		mi_cfg->mi_panel_id = 0;
		DISP_INFO("mi,panel-id not specified\n");
	} else {
		DISP_INFO("mi,panel-id is 0x%llx (%s)\n",
			mi_cfg->mi_panel_id, mi_get_panel_id_name(mi_cfg->mi_panel_id));
	}

	rc = mi_dsi_panel_parse_hbm_51_config(panel);
	rc |= mi_dsi_panel_parse_dc_config(panel);
	rc |= mi_dsi_panel_parse_backlight_config(panel);

	return rc;
}
