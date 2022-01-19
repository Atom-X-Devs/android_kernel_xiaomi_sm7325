// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#define pr_fmt(fmt) "mi_sde_connector:[%s:%d] " fmt, __func__, __LINE__

#include <drm/sde_drm.h>
#include "msm_drv.h"
#include "sde_connector.h"
#include "sde_encoder.h"
#include "sde_trace.h"
#include "dsi_display.h"
#include "dsi_panel.h"

#include "mi_disp_print.h"
#include "mi_disp_feature.h"
#include "mi_dsi_panel.h"
#include "mi_dsi_display.h"

static irqreturn_t mi_esd_err_irq_handle(int irq, void *data)
{
	struct sde_connector *c_conn = (struct sde_connector *)data;
	struct dsi_display *display = c_conn ? (c_conn->display) : NULL;
	struct drm_event event;
	int power_mode;

	if (!display || !display->panel) {
		DISP_ERROR("invalid display/panel\n");
		return IRQ_HANDLED;
	}

	DISP_INFO("%s display esd irq trigging \n", display->display_type);

	mi_dsi_panel_esd_irq_ctrl(display->panel, false);

	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		if (dsi_panel_initialized(display->panel)) {
			if (atomic_read(&(display->panel->esd_recovery_pending))) {
				DISP_ERROR("%s display ESD recovery already pending\n",
					display->display_type);
				return IRQ_HANDLED;
			}
			power_mode = display->panel->power_mode;
			DISP_INFO("%s display, power_mode (%s)\n", display->display_type,
				get_display_power_mode_name(power_mode));
			if (power_mode == SDE_MODE_DPMS_ON ||
				power_mode == SDE_MODE_DPMS_LP1) {
				atomic_set(&display->panel->esd_recovery_pending, 1);
				_sde_connector_report_panel_dead(c_conn, false);
			} else {
				if (!c_conn->panel_dead) {
					atomic_set(&display->panel->esd_recovery_pending, 1);
					c_conn->panel_dead = true;
					event.type = DRM_EVENT_PANEL_DEAD;
					event.length = sizeof(bool);
					msm_mode_object_event_notify(&c_conn->base.base,
						c_conn->base.dev, &event, (u8 *)&c_conn->panel_dead);
					SDE_EVT32(SDE_EVTLOG_ERROR);
					DISP_ERROR("%s display esd irq check failed report PANEL_DEAD"
						" conn_id: %d enc_id: %d\n", display->display_type,
						c_conn->base.base.id, c_conn->encoder->base.id);
				}
			}
		}
	}

	return IRQ_HANDLED;
}

int mi_sde_connector_register_esd_irq(struct sde_connector *c_conn)
{
	struct dsi_display *display = c_conn ? (c_conn->display) : NULL;
	int rc = 0;

	/* register esd irq and enable it after panel enabled */
	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		if (!display || !display->panel) {
			DISP_ERROR("invalid display/panel\n");
			return -EINVAL;
		}
		if (display->panel->mi_cfg.esd_err_irq_gpio > 0) {
			rc = request_threaded_irq(display->panel->mi_cfg.esd_err_irq,
				NULL, mi_esd_err_irq_handle,
				display->panel->mi_cfg.esd_err_irq_flags,
				"esd_err_irq", c_conn);
			if (rc) {
				DISP_ERROR("%s display register esd irq failed\n",
					display->display_type);
			} else {
				DISP_INFO("%s display register esd irq success\n",
					display->display_type);
				disable_irq(display->panel->mi_cfg.esd_err_irq);
			}
		}
	}

	return rc;
}

#if MI_DISP_DEBUGFS_ENABLE
int mi_sde_connector_debugfs_esd_sw_trigger(void *display)
{
	struct dsi_display *dsi_display = (struct dsi_display *)display;
	struct drm_connector *connector = NULL;
	struct sde_connector *c_conn = NULL;
	struct drm_event event;
	int power_mode;

	if (!dsi_display || !dsi_display->panel || !dsi_display->drm_conn) {
		DISP_ERROR("invalid display/panel/drm_conn ptr\n");
		return -EINVAL;
	}

	connector = dsi_display->drm_conn;
	c_conn = to_sde_connector(connector);
	if (!c_conn) {
		DISP_ERROR("invalid sde_connector ptr\n");
		return -EINVAL;
	}

	if (!dsi_panel_initialized(dsi_display->panel)) {
		DISP_ERROR("Panel not initialized\n");
		return -EINVAL;
	}

	if (atomic_read(&(dsi_display->panel->esd_recovery_pending))) {
		DISP_INFO("[esd-test]ESD recovery already pending\n");
		return 0;
	}

	if (c_conn->panel_dead) {
		DISP_INFO("panel_dead is true, return!\n");
		return 0;
	}

	atomic_set(&dsi_display->panel->esd_recovery_pending, 1);
	c_conn->panel_dead = true;
	DISP_ERROR("[esd-test]esd irq check failed report PANEL_DEAD conn_id: %d enc_id: %d\n",
			c_conn->base.base.id, c_conn->encoder->base.id);

	power_mode = dsi_display->panel->power_mode;
	DISP_INFO("[esd-test]%s display, power_mode (%s)\n", dsi_display->display_type,
		get_display_power_mode_name(power_mode));
	if (power_mode == SDE_MODE_DPMS_ON || power_mode == SDE_MODE_DPMS_LP1) {
		sde_encoder_display_failure_notification(c_conn->encoder, false);
	}

	event.type = DRM_EVENT_PANEL_DEAD;
	event.length = sizeof(bool);
	msm_mode_object_event_notify(&c_conn->base.base,
			c_conn->base.dev, &event, (u8 *)&c_conn->panel_dead);

	return 0;
}
#endif

int mi_sde_connector_panel_ctl(struct drm_connector *connector, uint32_t op_code)
{
	int ret = 0;
	struct sde_connector *c_conn = to_sde_connector(connector);
	struct disp_feature_ctl ctl;
	struct dsi_display *dsi_display;
	struct mi_dsi_panel_cfg *mi_cfg;

	dsi_display = (struct dsi_display *) c_conn->display;
	if (!dsi_display || !dsi_display->panel) {
		DISP_ERROR("invalid display/panel ptr\n");
		return -EINVAL;
	}

	if (mi_get_disp_id(dsi_display) != MI_DISP_PRIMARY)
		return -EINVAL;

	mi_cfg = &dsi_display->panel->mi_cfg;

	switch (op_code) {
	case MI_FOD_HBM_ON:
		ctl.feature_id = DISP_FEATURE_HBM_FOD;
		ctl.feature_val = FEATURE_ON;
		break;
	case MI_FOD_HBM_OFF:
		ctl.feature_id = DISP_FEATURE_HBM_FOD;
		ctl.feature_val = FEATURE_OFF;
		break;
	case MI_FOD_AOD_TO_NORMAL:
		ctl.feature_id = DISP_FEATURE_AOD_TO_NORMAL;
		ctl.feature_val = FEATURE_ON;
		break;
	case MI_FOD_NORMAL_TO_AOD:
		ctl.feature_id = DISP_FEATURE_AOD_TO_NORMAL;
		ctl.feature_val = FEATURE_OFF;
		break;
	default:
		break;
	}
	SDE_ATRACE_BEGIN("mi_sde_connector_panel_ctl");
	ret = mi_dsi_display_set_disp_param(c_conn->display, &ctl);
	SDE_ATRACE_END("mi_sde_connector_panel_ctl");
	return ret;
}

int mi_sde_connector_gir_fence(struct drm_connector *connector)
{
	int rc = 0;
	struct sde_connector *c_conn;
	struct dsi_display *dsi_display;
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!connector) {
		DISP_ERROR("invalid connector ptr\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);

	if (c_conn->connector_type != DRM_MODE_CONNECTOR_DSI)
		return 0;

	dsi_display = (struct dsi_display *) c_conn->display;
	if (!dsi_display || !dsi_display->panel) {
		DISP_ERROR("invalid display/panel ptr\n");
		return -EINVAL;
	}

	if (mi_get_disp_id(dsi_display) != MI_DISP_PRIMARY)
		return -EINVAL;

	mi_cfg = &dsi_display->panel->mi_cfg;
	if (mi_cfg->gir_enabled == false
			&& mi_cfg->feature_val[DISP_FEATURE_GIR] == FEATURE_ON) {
		SDE_ATRACE_BEGIN("DISP_FEATURE_GIR_ON");
		dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_FLAT_MODE_ON);
		sde_encoder_wait_for_event(c_conn->encoder,MSM_ENC_VBLANK);
		SDE_ATRACE_END("DISP_FEATURE_GIR_ON");
		mi_cfg->gir_enabled = true;
	} else if (mi_cfg->gir_enabled == true
				&& mi_cfg->feature_val[DISP_FEATURE_GIR] == FEATURE_OFF) {
		SDE_ATRACE_BEGIN("DISP_FEATURE_GIR_OFF");
		dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_FLAT_MODE_OFF);
		sde_encoder_wait_for_event(c_conn->encoder,MSM_ENC_VBLANK);
		SDE_ATRACE_END("DISP_FEATURE_GIR_OFF");
		mi_cfg->gir_enabled = false;
	}

	return rc;
}
