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

	if (c_conn->connector_type == DRM_MODE_CONNECTOR_DSI) {
		dsi_panel_acquire_panel_lock(display->panel);
		mi_dsi_panel_esd_irq_ctrl_locked(display->panel, false);

		if (!dsi_panel_initialized(display->panel)) {
			DISP_ERROR("%s display panel not initialized!\n",
					display->display_type);
			dsi_panel_release_panel_lock(display->panel);
			return IRQ_HANDLED;
		}

		if (atomic_read(&(display->panel->esd_recovery_pending))) {
			DISP_INFO("%s display ESD recovery already pending\n",
					display->display_type);
			dsi_panel_release_panel_lock(display->panel);
			return IRQ_HANDLED;
		}

		if (!c_conn->panel_dead) {
			atomic_set(&display->panel->esd_recovery_pending, 1);
		} else {
			DISP_INFO("%s display already notify PANEL_DEAD\n",
					display->display_type);
			dsi_panel_release_panel_lock(display->panel);
			return IRQ_HANDLED;
		}

		power_mode = display->panel->power_mode;
		DISP_INFO("%s display, power_mode (%s)\n", display->display_type,
			get_display_power_mode_name(power_mode));

		dsi_panel_release_panel_lock(display->panel);

		if (power_mode == SDE_MODE_DPMS_ON ||
			power_mode == SDE_MODE_DPMS_LP1) {
			_sde_connector_report_panel_dead(c_conn, false);
		} else {
			c_conn->panel_dead = true;
			event.type = DRM_EVENT_PANEL_DEAD;
			event.length = sizeof(bool);
			msm_mode_object_event_notify(&c_conn->base.base,
				c_conn->base.dev, &event, (u8 *)&c_conn->panel_dead);
			SDE_EVT32(SDE_EVTLOG_ERROR);
			DISP_ERROR("%s display esd irq check failed report"
				" PANEL_DEAD conn_id: %d enc_id: %d\n",
				display->display_type,
				c_conn->base.base.id, c_conn->encoder->base.id);
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
		mutex_lock(&dsi_display->panel->panel_lock);
		if (!is_aod_and_panel_initialized(dsi_display->panel)) {
			dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_FLAT_MODE_ON);
			sde_encoder_wait_for_event(c_conn->encoder,MSM_ENC_VBLANK);
			mi_cfg->gir_enabled = true;
		}
		mutex_unlock(&dsi_display->panel->panel_lock);
	} else if (mi_cfg->gir_enabled == true
			&& mi_cfg->feature_val[DISP_FEATURE_GIR] == FEATURE_OFF) {
		mutex_lock(&dsi_display->panel->panel_lock);
		dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_FLAT_MODE_OFF);
		sde_encoder_wait_for_event(c_conn->encoder,MSM_ENC_VBLANK);
		mi_cfg->gir_enabled = false;
		mutex_unlock(&dsi_display->panel->panel_lock);
	}

	return rc;
}

int mi_sde_connector_dc_fence(struct drm_connector *connector)
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
	if (mi_cfg->dc_enabled == false
			&& mi_cfg->feature_val[DISP_FEATURE_DC] == FEATURE_ON) {
		mutex_lock(&dsi_display->panel->panel_lock);
		if (!is_aod_and_panel_initialized(dsi_display->panel)) {
			dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_DC_ON);
			sde_encoder_wait_for_event(c_conn->encoder, MSM_ENC_VBLANK);
			mi_cfg->dc_enabled = true;
		}
		mutex_unlock(&dsi_display->panel->panel_lock);
	} else if (mi_cfg->dc_enabled == true
			&& mi_cfg->feature_val[DISP_FEATURE_DC] == FEATURE_OFF) {
		mutex_lock(&dsi_display->panel->panel_lock);
		dsi_panel_tx_cmd_set(dsi_display->panel, DSI_CMD_SET_MI_DC_OFF);
		sde_encoder_wait_for_event(c_conn->encoder, MSM_ENC_VBLANK);
		mi_cfg->dc_enabled = false;
		mutex_unlock(&dsi_display->panel->panel_lock);
	}

	return rc;
}

int mi_sde_connector_hbm_fence(struct drm_connector *connector)
{
	int rc = 0;
	struct sde_connector *c_conn;
	struct dsi_display *dsi_display;
	/*bool target_overlay;*/
	struct mi_dsi_panel_cfg *mi_cfg;
	struct sde_encoder_virt *sde_enc;

	if (!connector) {
		DISP_ERROR("invalid connector ptr\n");
		return -EINVAL;
	}

	c_conn = to_sde_connector(connector);
	sde_enc = to_sde_encoder_virt(connector->encoder);

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

	if (c_conn->lp_mode == SDE_MODE_DPMS_ON)
		mi_cfg->bl_enable = true;

	if (!mi_cfg->bl_wait_frame && c_conn->lp_mode == SDE_MODE_DPMS_ON) {
		sde_enc->ready_kickoff = true;
		if (sde_enc->prepare_kickoff) {
			SDE_ATRACE_BEGIN("set_backlight_after_aod");
			if (mi_cfg->panel_id == 0x4D323000360200) {
				if (mi_cfg->last_bl_level > 0)
					dsi_display_set_backlight(connector, dsi_display, mi_cfg->last_bl_level);
			} else {
				dsi_display_set_backlight(connector, dsi_display, mi_cfg->last_bl_level);
			}
			SDE_ATRACE_END("set_backlight_after_aod");
			DISP_INFO("backlight %d set after aod layer\n", mi_cfg->last_bl_level);
			mi_cfg->bl_wait_frame = true;
			sde_enc->ready_kickoff = false;
			sde_enc->prepare_kickoff = false;
		}
	} else {
		sde_enc->ready_kickoff = false;
	}

	return rc;
}
