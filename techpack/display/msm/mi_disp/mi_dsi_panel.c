/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#define pr_fmt(fmt)	"mi-dsi-panel:[%s:%d] " fmt, __func__, __LINE__
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/pm_wakeup.h>
#include <video/mipi_display.h>

#include <drm/mi_disp.h>
#include <drm/sde_drm.h>
#include "sde_connector.h"
#include "sde_encoder.h"
#include "sde_crtc.h"
#include "sde_trace.h"

#include "dsi_panel.h"
#include "dsi_display.h"
#include "dsi_ctrl_hw.h"
#include "dsi_parser.h"
#include "../../../../kernel/irq/internals.h"
#include "mi_disp_feature.h"
#include "mi_disp_print.h"
#include "mi_dsi_display.h"

#define to_dsi_display(x) container_of(x, struct dsi_display, host)

struct dsi_read_config g_dsi_read_cfg;

static int mi_dsi_update_hbm_cmd_51reg(struct dsi_panel *panel,
			enum dsi_cmd_set_type type, int bl_lvl);

int mi_dsi_panel_init(struct dsi_panel *panel)
{
	struct mi_dsi_panel_cfg *mi_cfg = NULL;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mi_cfg = &panel->mi_cfg;

	mi_cfg->doze_wakelock = wakeup_source_register(NULL, "doze_wakelock");
	if (!mi_cfg->doze_wakelock) {
		DISP_ERROR("doze_wakelock wake_source register failed");
		return -ENOMEM;
	}
	return 0;
}

int mi_dsi_panel_deinit(struct dsi_panel *panel)
{
	struct mi_dsi_panel_cfg *mi_cfg = NULL;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mi_cfg = &panel->mi_cfg;

	if (mi_cfg->doze_wakelock)
		wakeup_source_unregister(mi_cfg->doze_wakelock);

	return 0;
}

int mi_dsi_acquire_wakelock(struct dsi_panel *panel)
{
	struct mi_dsi_panel_cfg *mi_cfg = NULL;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mi_cfg = &panel->mi_cfg;

	if (mi_cfg->doze_wakelock)
		__pm_stay_awake(mi_cfg->doze_wakelock);

	return 0;
}

int mi_dsi_release_wakelock(struct dsi_panel *panel)
{
	struct mi_dsi_panel_cfg *mi_cfg = NULL;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mi_cfg = &panel->mi_cfg;

	if (mi_cfg->doze_wakelock)
		__pm_relax(mi_cfg->doze_wakelock);

	return 0;

}

bool is_aod_and_panel_initialized(struct dsi_panel *panel)
{
	if ((panel->power_mode == SDE_MODE_DPMS_LP1 ||
		panel->power_mode == SDE_MODE_DPMS_LP2) &&
		dsi_panel_initialized(panel)){
		return true;
	} else {
		return false;
	}
}

bool is_backlight_set_skip(struct dsi_panel *panel, u32 bl_lvl)
{
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;

	if (panel->power_mode == SDE_MODE_DPMS_LP1 && bl_lvl == 0) {
		DSI_INFO("%s panel skip set backlight 0 due to LP1 on\n", panel->type);
		return true;
	} else if (!panel->mi_cfg.bl_enable) {
		DSI_INFO("%s panel skip set backlight %d due to aod on\n", panel->type, bl_lvl);
		return true;
	} else {
		return false;
	}
}

int mi_dsi_panel_esd_irq_ctrl(struct dsi_panel *panel,
				bool enable)
{
	struct mi_dsi_panel_cfg *mi_cfg;
	struct irq_desc *desc;

	if (!panel || !panel->panel_initialized) {
		DISP_ERROR("Panel not ready!\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;
	if (gpio_is_valid(mi_cfg->esd_err_irq_gpio)) {
		if (mi_cfg->esd_err_irq) {
			if (enable) {
				if (!mi_cfg->esd_err_enabled) {
					desc = irq_to_desc(mi_cfg->esd_err_irq);
					if (!irq_settings_is_level(desc))
						desc->istate &= ~IRQS_PENDING;
					enable_irq_wake(mi_cfg->esd_err_irq);
					enable_irq(mi_cfg->esd_err_irq);
					mi_cfg->esd_err_enabled = true;
					DISP_INFO("%s panel esd irq is enable\n", panel->type);
				}
			} else {
				if (mi_cfg->esd_err_enabled) {
					disable_irq_wake(mi_cfg->esd_err_irq);
					disable_irq_nosync(mi_cfg->esd_err_irq);
					mi_cfg->esd_err_enabled = false;
					DISP_INFO("%s panel esd irq is disable\n", panel->type);
				}
			}
		}
	} else {
		DISP_INFO("%s panel esd irq gpio invalid\n", panel->type);
	}

	mutex_unlock(&panel->panel_lock);
	return 0;
}

int mi_dsi_panel_write_cmd_set(struct dsi_panel *panel,
				struct dsi_panel_cmd_set *cmd_sets)
{
	int rc = 0, i = 0;
	ssize_t len;
	struct dsi_cmd_desc *cmds;
	u32 count;
	enum dsi_cmd_set_state state;
	struct dsi_display_mode *mode;
	const struct mipi_dsi_host_ops *ops = panel->host->ops;

	if (!panel || !panel->cur_mode) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mode = panel->cur_mode;

	cmds = cmd_sets->cmds;
	count = cmd_sets->count;
	state = cmd_sets->state;

	if (count == 0) {
		DISP_DEBUG("[%s] No commands to be sent for state\n", panel->type);
		goto error;
	}

	for (i = 0; i < count; i++) {
		if (state == DSI_CMD_SET_STATE_LP)
			cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;

		if (cmds->last_command)
			cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;

		len = ops->transfer(panel->host, &cmds->msg);
		if (len < 0) {
			rc = len;
			DISP_ERROR("failed to set cmds, rc=%d\n", rc);
			goto error;
		}
		if (cmds->post_wait_ms)
			usleep_range(cmds->post_wait_ms * 1000,
					((cmds->post_wait_ms * 1000) + 10));
		cmds++;
	}
error:
	return rc;
}

int mi_dsi_panel_read_cmd_set(struct dsi_panel *panel,
				struct dsi_read_config *read_config)
{
	struct dsi_display *display;
	struct dsi_display_ctrl *ctrl;
	struct dsi_cmd_desc *cmds;
	enum dsi_cmd_set_state state;
	int i, rc = 0, count = 0;
	u32 flags = 0;

	if (!panel || !panel->host || !read_config) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	display = to_dsi_display(panel->host);

	/* Avoid sending DCS commands when ESD recovery is pending */
	if (atomic_read(&display->panel->esd_recovery_pending)) {
		DISP_ERROR("[%s] ESD recovery pending\n", panel->type);
		return 0;
	}

	if (!panel->panel_initialized) {
		DISP_INFO("[%s] Panel not initialized\n", panel->type);
		return -EINVAL;
	}

	if (!read_config->is_read) {
		DISP_INFO("[%s] read operation was not permitted\n", panel->type);
		return -EPERM;
	}

	dsi_display_clk_ctrl(display->dsi_clk_handle,
		DSI_ALL_CLKS, DSI_CLK_ON);

	ctrl = &display->ctrl[display->cmd_master_idx];

	rc = dsi_display_cmd_engine_enable(display);
	if (rc) {
		DISP_ERROR("[%s] cmd engine enable failed\n", panel->type);
		rc = -EPERM;
		goto error_disable_clks;
	}

	if (display->tx_cmd_buf == NULL) {
		rc = dsi_host_alloc_cmd_tx_buffer(display);
		if (rc) {
			DISP_ERROR("[%s] failed to allocate cmd tx buffer\n", panel->type);
			goto error_disable_cmd_engine;
		}
	}

	count = read_config->read_cmd.count;
	cmds = read_config->read_cmd.cmds;
	state = read_config->read_cmd.state;
	if (count == 0) {
		DISP_ERROR("[%s] No commands to be sent\n", panel->type);
		goto error_disable_cmd_engine;
	}
	if (cmds->last_command) {
		cmds->msg.flags |= MIPI_DSI_MSG_LASTCOMMAND;
		flags |= DSI_CTRL_CMD_LAST_COMMAND;
	}
	if (state == DSI_CMD_SET_STATE_LP)
		cmds->msg.flags |= MIPI_DSI_MSG_USE_LPM;
	flags |= (DSI_CTRL_CMD_FETCH_MEMORY | DSI_CTRL_CMD_READ |
		  DSI_CTRL_CMD_CUSTOM_DMA_SCHED);

	memset(read_config->rbuf, 0x0, sizeof(read_config->rbuf));
	cmds->msg.rx_buf = read_config->rbuf;
	cmds->msg.rx_len = read_config->cmds_rlen;

	rc = dsi_ctrl_cmd_transfer(ctrl->ctrl, &(cmds->msg), &flags);
	if (rc <= 0) {
		DISP_ERROR("[%s] rx cmd transfer failed rc=%d\n", panel->type, rc);
		goto error_disable_cmd_engine;
	}

	/* for debug log */
	for (i = 0; i < read_config->cmds_rlen; i++)
		DISP_DEBUG("[%d] = 0x%02X\n", i, read_config->rbuf[i]);

error_disable_cmd_engine:
	dsi_display_cmd_engine_disable(display);
error_disable_clks:
	dsi_display_clk_ctrl(display->dsi_clk_handle,
		DSI_ALL_CLKS, DSI_CLK_OFF);

	return rc;
}

int mi_dsi_panel_write_mipi_reg(struct dsi_panel *panel,
				char *buf)
{
	struct dsi_panel_cmd_set cmd_sets = {0};
	int rc = 0, dlen = 0;
	u32 packet_count = 0;
	char *token, *input_copy, *input_dup = NULL;
	const char *delim = " ";
	char *buffer = NULL;
	u32 buf_size = 0;
	u32 tmp_data = 0;

	mutex_lock(&panel->panel_lock);

	if (!panel || !panel->panel_initialized) {
		DISP_ERROR("Panel not initialized!\n");
		rc = -EAGAIN;
		goto exit_unlock;
	}

	DISP_DEBUG("[%s] input buffer:{%s}\n", panel->type, buf);

	input_copy = kstrdup(buf, GFP_KERNEL);
	if (!input_copy) {
		rc = -ENOMEM;
		goto exit_unlock;
	}

	input_dup = input_copy;
	/* removes leading and trailing whitespace from input_copy */
	input_copy = strim(input_copy);

	/* Split a string into token */
	token = strsep(&input_copy, delim);
	if (token) {
		rc = kstrtoint(token, 10, &tmp_data);
		if (rc) {
			DISP_ERROR("input buffer conversion failed\n");
			goto exit_free0;
		}
		g_dsi_read_cfg.is_read= !!tmp_data;
	}

	/* Removes leading whitespace from input_copy */
	if (input_copy)
		input_copy = skip_spaces(input_copy);
	else
		goto exit_free0;

	token = strsep(&input_copy, delim);
	if (token) {
		rc = kstrtoint(token, 10, &tmp_data);
		if (rc) {
			DISP_ERROR("input buffer conversion failed\n");
			goto exit_free0;
		}
		if (tmp_data > sizeof(g_dsi_read_cfg.rbuf)) {
			DISP_ERROR("read size exceeding the limit %d\n",
					sizeof(g_dsi_read_cfg.rbuf));
			goto exit_free0;
		}
		g_dsi_read_cfg.cmds_rlen = tmp_data;
	}

	/* Removes leading whitespace from input_copy */
	if (input_copy)
		input_copy = skip_spaces(input_copy);
	else
		goto exit_free0;

	buffer = kzalloc(strlen(input_copy), GFP_KERNEL);
	if (!buffer) {
		rc = -ENOMEM;
		goto exit_free0;
	}

	token = strsep(&input_copy, delim);
	while (token) {
		rc = kstrtoint(token, 16, &tmp_data);
		if (rc) {
			DISP_ERROR("input buffer conversion failed\n");
			goto exit_free1;
		}
		DISP_DEBUG("buffer[%d] = 0x%02x\n", buf_size, tmp_data);
		buffer[buf_size++] = (tmp_data & 0xff);
		/* Removes leading whitespace from input_copy */
		if (input_copy) {
			input_copy = skip_spaces(input_copy);
			token = strsep(&input_copy, delim);
		} else {
			token = NULL;
		}
	}

	rc = dsi_panel_get_cmd_pkt_count(buffer, buf_size, &packet_count);
	if (!packet_count) {
		DISP_ERROR("get pkt count failed!\n");
		goto exit_free1;
	}

	rc = dsi_panel_alloc_cmd_packets(&cmd_sets, packet_count);
	if (rc) {
		DISP_ERROR("failed to allocate cmd packets, ret=%d\n", rc);
		goto exit_free1;
	}

	rc = dsi_panel_create_cmd_packets(buffer, dlen, packet_count,
						  cmd_sets.cmds);
	if (rc) {
		DISP_ERROR("failed to create cmd packets, ret=%d\n", rc);
		goto exit_free2;
	}

	if (g_dsi_read_cfg.is_read) {
		g_dsi_read_cfg.read_cmd = cmd_sets;
		rc = mi_dsi_panel_read_cmd_set(panel, &g_dsi_read_cfg);
		if (rc <= 0) {
			DISP_ERROR("[%s]failed to read cmds, rc=%d\n", panel->name, rc);
			goto exit_free3;
		}
	} else {
		g_dsi_read_cfg.read_cmd = cmd_sets;
		rc = mi_dsi_panel_write_cmd_set(panel, &cmd_sets);
		if (rc) {
			DISP_ERROR("[%s] failed to send cmds, rc=%d\n", panel->name, rc);
			goto exit_free3;
		}
	}

	DISP_DEBUG("[%s]: done!\n", panel->name);
	rc = 0;

exit_free3:
	dsi_panel_destroy_cmd_packets(&cmd_sets);
exit_free2:
	dsi_panel_dealloc_cmd_packets(&cmd_sets);
exit_free1:
	kfree(buffer);
exit_free0:
	kfree(input_dup);
exit_unlock:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

ssize_t mi_dsi_panel_read_mipi_reg(struct dsi_panel *panel,
			char *buf, size_t size)
{
	int i = 0;
	ssize_t count = 0;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	if (g_dsi_read_cfg.is_read) {
		for (i = 0; i < g_dsi_read_cfg.cmds_rlen; i++) {
			if (i == g_dsi_read_cfg.cmds_rlen - 1) {
				count += snprintf(buf + count, size - count, "0x%02X\n",
				     g_dsi_read_cfg.rbuf[i]);
			} else {
				count += snprintf(buf + count, size - count, "0x%02X,",
				     g_dsi_read_cfg.rbuf[i]);
			}
		}
	}

	mutex_unlock(&panel->panel_lock);

	return count;
}

int mi_dsi_panel_write_dsi_cmd_set(struct dsi_panel *panel,
			int type)
{
	int rc = 0;
	int i = 0, j = 0;
	u8 *tx_buf = NULL;
	u8 *buffer = NULL;
	int buf_size = 1024;
	u32 cmd_count = 0;
	int buf_count = 1024;
	struct dsi_cmd_desc *cmds;
	enum dsi_cmd_set_state state;
	struct dsi_display_mode *mode;

	if (!panel || !panel->cur_mode || type < 0 || type >= DSI_CMD_SET_MAX) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	buffer = kzalloc(buf_size, GFP_KERNEL);
	if (!buffer) {
		return -ENOMEM;
	}

	mutex_lock(&panel->panel_lock);

	mode = panel->cur_mode;
	cmds = mode->priv_info->cmd_sets[type].cmds;
	cmd_count = mode->priv_info->cmd_sets[type].count;
	state = mode->priv_info->cmd_sets[type].state;

	if (cmd_count == 0) {
		DISP_ERROR("[%s] No commands to be sent\n", cmd_set_prop_map[type]);
		rc = -EAGAIN;
		goto error;
	}

	DISP_INFO("set cmds [%s], count (%d), state(%s)\n",
		cmd_set_prop_map[type], cmd_count,
		(state == DSI_CMD_SET_STATE_LP) ? "dsi_lp_mode" : "dsi_hs_mode");

	for (i = 0; i < cmd_count; i++) {
		memset(buffer, 0, buf_size);
		buf_count = snprintf(buffer, buf_size, "%02X", cmds->msg.tx_len);
		tx_buf = (u8 *)cmds->msg.tx_buf;
		for (j = 0; j < cmds->msg.tx_len ; j++) {
			buf_count += snprintf(buffer + buf_count, buf_size - buf_count, " %02X", tx_buf[j]);
		}
		DISP_DEBUG("[%d] %s\n", i, buffer);
		cmds++;
	}

	rc = dsi_panel_tx_cmd_set(panel, type);

error:
	mutex_unlock(&panel->panel_lock);
	kfree(buffer);
	return rc;
}

ssize_t mi_dsi_panel_show_dsi_cmd_set_type(struct dsi_panel *panel,
			char *buf, size_t size)
{
	ssize_t count = 0;
	int type = 0;

	if (!panel || !buf) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	count = snprintf(buf, size, "%s: dsi cmd_set name\n", "id");

	for (type = DSI_CMD_SET_PRE_ON; type < DSI_CMD_SET_MAX; type++) {
		count += snprintf(buf + count, size - count, "%02d: %s\n",
				     type, cmd_set_prop_map[type]);
	}

	return count;
}

int mi_dsi_panel_set_doze_brightness(struct dsi_panel *panel,
			u32 doze_brightness)
{
	int rc = 0;
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;

	if (doze_brightness != DOZE_TO_NORMAL && !is_aod_and_panel_initialized(panel)) {
		DISP_ERROR("Skip! %s panel set doze brightness %d, power mode(%s) initialized(%d)\n",
			panel->type, doze_brightness,
			get_display_power_mode_name(panel->power_mode), panel->panel_initialized);
		mi_cfg->doze_brightness = DOZE_TO_NORMAL;
		goto exit;
	}

	if (mi_cfg->doze_brightness != doze_brightness) {
		if (doze_brightness == DOZE_BRIGHTNESS_HBM) {
			mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_HBM, mi_cfg->last_no_zero_bl_level);
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_HBM);
			if (rc) {
				DISP_ERROR("[%s] failed to send DSI_CMD_SET_MI_DOZE_HBM cmd, rc=%d\n",
					panel->name, rc);
			}
			mi_dsi_update_micfg_flags(panel, PANEL_DOZE_HIGH);
			mi_cfg->doze_brightness_backup = DOZE_BRIGHTNESS_HBM;
		} else if (doze_brightness == DOZE_BRIGHTNESS_LBM) {
			mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_LBM, mi_cfg->last_no_zero_bl_level);
			rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_LBM);
			if (rc) {
				DISP_ERROR("[%s] failed to send DSI_CMD_SET_MI_DOZE_LBM cmd, rc=%d\n",
					panel->name, rc);
			}
			mi_dsi_update_micfg_flags(panel, PANEL_DOZE_LOW);
			mi_cfg->doze_brightness_backup = DOZE_BRIGHTNESS_LBM;
		}

		mi_cfg->doze_brightness = doze_brightness;
		mi_cfg->is_doze_to_off = !doze_brightness;

		DISP_UTC_INFO("%s panel set doze brightness to %s\n",
			panel->type, get_doze_brightness_name(doze_brightness));
	} else {
		DISP_INFO("%s panel %s has been set, skip\n", panel->type,
			get_doze_brightness_name(doze_brightness));
	}

exit:
	mutex_unlock(&panel->panel_lock);

	return rc;
}

int mi_dsi_panel_get_doze_brightness(struct dsi_panel *panel,
			u32 *doze_brightness)
{
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;
	*doze_brightness =  mi_cfg->doze_brightness;

	mutex_unlock(&panel->panel_lock);

	return 0;
}

int mi_dsi_panel_get_brightness(struct dsi_panel *panel,
			u32 *brightness)
{
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;
	*brightness =  mi_cfg->last_bl_level;

	mutex_unlock(&panel->panel_lock);

	return 0;
}

int mi_dsi_panel_write_dsi_cmd(struct dsi_panel *panel,
			struct dsi_cmd_rw_ctl *ctl)
{
	struct dsi_panel_cmd_set cmd_sets = {0};
	u32 packet_count = 0;
	u32 dlen = 0;
	int rc = 0;

	mutex_lock(&panel->panel_lock);

	if (!panel || !panel->panel_initialized) {
		DISP_ERROR("Panel not initialized!\n");
		rc = -EAGAIN;
		goto exit_unlock;
	}

	if (!ctl->tx_len || !ctl->tx_ptr) {
		DISP_ERROR("%s panel invalid params\n", panel->type);
		rc = -EINVAL;
		goto exit_unlock;
	}

	rc = dsi_panel_get_cmd_pkt_count(ctl->tx_ptr, ctl->tx_len, &packet_count);
	if (rc) {
		DISP_ERROR("%s panel write dsi commands failed, rc=%d\n",
			panel->type, rc);
		goto exit_unlock;
	}

	DISP_DEBUG("%s panel packet-count=%d\n", panel->type, packet_count);

	rc = dsi_panel_alloc_cmd_packets(&cmd_sets, packet_count);
	if (rc) {
		DISP_ERROR("%s panel failed to allocate cmd packets, rc=%d\n",
			panel->type, rc);
		goto exit_unlock;
	}

	rc = dsi_panel_create_cmd_packets(ctl->tx_ptr, dlen, packet_count,
						cmd_sets.cmds);
	if (rc) {
		DISP_ERROR("%s panel failed to create cmd packets, rc=%d\n",
			panel->type, rc);
		goto exit_free1;
	}

	if (ctl->tx_state == MI_DSI_CMD_LP_STATE) {
		cmd_sets.state = DSI_CMD_SET_STATE_LP;
	} else if (ctl->tx_state == MI_DSI_CMD_HS_STATE) {
		cmd_sets.state = DSI_CMD_SET_STATE_HS;
	} else {
		DISP_ERROR("%s panel command state unrecognized-%s\n",
			panel->type, cmd_sets.state);
		goto exit_free1;
	}

	rc = mi_dsi_panel_write_cmd_set(panel, &cmd_sets);
	if (rc) {
		DISP_ERROR("%s panel [%s] failed to send cmds, rc=%d\n",
			panel->type, panel->name, rc);
		goto exit_free2;
	}

exit_free2:
	if (ctl->tx_len && ctl->tx_ptr)
		dsi_panel_destroy_cmd_packets(&cmd_sets);
exit_free1:
	if (ctl->tx_len && ctl->tx_ptr)
		dsi_panel_dealloc_cmd_packets(&cmd_sets);
exit_unlock:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

int mi_dsi_panel_set_brightness_clone(struct dsi_panel *panel,
			u32 brightness_clone)
{
	int rc = 0;
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;
	mi_cfg->brightness_clone = brightness_clone;
	DISP_UTC_INFO("%s panel set brightness clone to %d\n",
			panel->type, brightness_clone);

	mutex_unlock(&panel->panel_lock);

	return rc;
}

int mi_dsi_panel_get_brightness_clone(struct dsi_panel *panel,
			u32 *brightness_clone)
{
	struct mi_dsi_panel_cfg *mi_cfg;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	mi_cfg = &panel->mi_cfg;
	*brightness_clone =  mi_cfg->brightness_clone;

	mutex_unlock(&panel->panel_lock);

	return 0;
}

static void mi_disp_set_dimming_delayed_work_handler(struct kthread_work *work)
{
	struct disp_work *cur_work = container_of(work,
					struct disp_work, delayed_work.work);
	struct dsi_panel *panel = (struct dsi_panel *)(cur_work->data);
	struct disp_feature_ctl ctl;

	ctl.feature_id = DISP_FEATURE_DIMMING;
	ctl.feature_val = FEATURE_ON;

	mi_dsi_panel_set_disp_param(panel, &ctl);

	kfree(cur_work);
}

static int mi_disp_set_dimming_queue_delayed_work(struct disp_display *dd_ptr,
			struct dsi_panel *panel)
{
	struct disp_work *cur_work;

	cur_work = kzalloc(sizeof(*cur_work), GFP_ATOMIC);
	if (!cur_work)
		return -ENOMEM;

	kthread_init_delayed_work(&cur_work->delayed_work, mi_disp_set_dimming_delayed_work_handler);
	cur_work->dd_ptr = dd_ptr;
	cur_work->wq = &dd_ptr->pending_wq;
	cur_work->data = panel;

	kthread_queue_delayed_work(&dd_ptr->feature_thread.worker,
			&cur_work->delayed_work,
			msecs_to_jiffies(panel->mi_cfg.panel_on_dimming_delay));

	return 0;
}

void mi_dsi_panel_update_last_bl_level(struct dsi_panel *panel, int brightness)
{
	struct mi_dsi_panel_cfg *mi_cfg;
	struct disp_feature *df = mi_get_disp_feature();
	struct dsi_display *display;
	int disp_id = 0;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return;
	}

	display = to_dsi_display(panel->host);
	mi_cfg = &panel->mi_cfg;

	if ((mi_cfg->last_bl_level == 0 || mi_cfg->dimming_state == STATE_DIM_RESTORE) &&
		brightness > 0) {
		disp_id = mi_get_disp_id(display);
		mi_disp_set_dimming_queue_delayed_work(&df->d_display[disp_id], panel);

		if (mi_cfg->dimming_state == STATE_DIM_RESTORE)
			mi_cfg->dimming_state = STATE_NONE;
	}

	mi_cfg->last_bl_level = brightness;
	if (brightness != 0) {
		mi_cfg->last_no_zero_bl_level = brightness;
		DISP_INFO("mi_cfg->last_no_zero_bl_level = %d\n", brightness);
	}

	return;
}

void mi_dsi_update_micfg_flags(struct dsi_panel *panel, int power_mode)
{
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return;
	}

	mi_cfg = &panel->mi_cfg;

	switch (power_mode) {
	case PANEL_OFF:
		mi_cfg->dimming_state = STATE_NONE;
		mi_cfg->panel_state = PANEL_STATE_OFF;
		mi_cfg->feature_val[DISP_FEATURE_COLOR_INVERT] = FEATURE_OFF;
		mi_cfg->feature_val[DISP_FEATURE_HBM] = FEATURE_OFF;
		break;
	case PANEL_ON:
		mi_cfg->dimming_state = STATE_NONE;
		mi_cfg->panel_state = PANEL_STATE_ON;
		break;
	case PANEL_NOLP:
		mi_cfg->dimming_state = STATE_DIM_RESTORE;
		mi_cfg->panel_state = PANEL_STATE_ON;
		break;
	case PANEL_DOZE_HIGH:
		mi_cfg->dimming_state = STATE_DIM_BLOCK;
		mi_cfg->panel_state = PANEL_STATE_DOZE_HIGH;
		break;
	case PANEL_DOZE_LOW:
		mi_cfg->dimming_state = STATE_DIM_BLOCK;
		mi_cfg->panel_state = PANEL_STATE_DOZE_LOW;
		break;
	case PANEL_LP1:
	case PANEL_LP2:
	default:
		break;
	}

	return;
}

void mi_dsi_dc_mode_enable(struct dsi_panel *panel,
			bool enable)
{
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;

	if (!panel && !panel->cur_mode) {
		DISP_ERROR("invalid params\n");
		return;
	}

	mi_cfg = &panel->mi_cfg;

	if (mi_cfg->dc_type == 0) {
		if (enable)
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DC_ON);
		else
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DC_OFF);
	}
}

static int mi_dsi_update_hbm_cmd_51reg(struct dsi_panel *panel,
			enum dsi_cmd_set_type type, int bl_lvl)
{
	struct dsi_display_mode_priv_info *priv_info;
	struct dsi_cmd_desc *cmds = NULL;
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;
	u32 count;
	int index;
	u8 *tx_buf;
	int rc = 0;

	if (!panel || !panel->cur_mode || !panel->cur_mode->priv_info) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mi_cfg = &panel->mi_cfg;
	priv_info = panel->cur_mode->priv_info;

	switch (type) {
	case DSI_CMD_SET_MI_HBM_ON:
		index = mi_cfg->hbm_on_51_index;
		break;
	case DSI_CMD_SET_MI_HBM_OFF:
		index = mi_cfg->hbm_off_51_index;
		break;
	case DSI_CMD_SET_MI_DOZE_HBM:
		index = -1;
		break;
	case DSI_CMD_SET_MI_DOZE_LBM:
		index = -1;
		break;
	default:
		DISP_ERROR("%s panel wrong cmd type!\n", panel->type);
		return -EINVAL;
	}

	if (index < 0) {
		DISP_DEBUG("%s panel cmd[%s] 0x51 update not supported\n",
				panel->type, cmd_set_prop_map[type]);
		return 0;
	}

	DISP_INFO("cmd[%s], bl_lvl=%d\n", cmd_set_prop_map[type], bl_lvl);

	/* restore last backlight value when hbm off */
	cmds = priv_info->cmd_sets[type].cmds;
	count = priv_info->cmd_sets[type].count;
	if (cmds && count >= index) {
		tx_buf = (u8 *)cmds[index].msg.tx_buf;
		if (tx_buf && tx_buf[0] == 0x51) {
			tx_buf[1] = (bl_lvl >> 8) & 0x0f;
			tx_buf[2] = bl_lvl & 0xff;
		} else {
			if (tx_buf) {
				DISP_ERROR("%s panel tx_buf[0] = 0x%02X, check cmd[%s] 0x51 index\n",
					panel->type, tx_buf[0], cmd_set_prop_map[type]);
			} else {
				DISP_ERROR("%s panel tx_buf is NULL pointer\n", panel->type);
			}
			rc = -EINVAL;
		}
	} else {
		DISP_ERROR("%s panel cmd[%s] 0x51 index(%d) error\n",
			panel->type, cmd_set_prop_map[type], index);
		rc = -EINVAL;
	}

	return rc;
}

static void mi_dsi_update_backlight_in_aod(struct dsi_panel *panel)
{
	int bl_lvl = 0;
	struct mi_dsi_panel_cfg *mi_cfg = &panel->mi_cfg;
	struct mipi_dsi_device *dsi = &panel->mipi_device;

	switch (mi_cfg->doze_brightness_backup) {
	case DOZE_BRIGHTNESS_HBM:
		bl_lvl = 0;
		break;
	case DOZE_BRIGHTNESS_LBM:
		bl_lvl = 0;
		break;
	default:
		return;
	}

	DISP_INFO("mi_dsi_update_backlight_in_aod %d\n", bl_lvl);
	if (panel->bl_config.bl_inverted_dbv)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));
	mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);

	return;
}

static void mi_dsi_update_dc_backlight(struct dsi_panel *panel, u32 bl_lvl)
{
	struct mipi_dsi_device *dsi = &panel->mipi_device;

	DISP_INFO("mi_dsi_update_dc_backlight bl_lvl=%d\n", bl_lvl);

	if (panel->bl_config.bl_inverted_dbv)
		bl_lvl = (((bl_lvl & 0xff) << 8) | (bl_lvl >> 8));

	mipi_dsi_dcs_set_display_brightness(dsi, bl_lvl);

	return;
}

bool mi_dsi_panel_is_need_tx_cmd(u32 feature_id)
{
	switch (feature_id) {
	case DISP_FEATURE_SENSOR_LUX:
	case DISP_FEATURE_LOW_BRIGHTNESS_FOD:
	case DISP_FEATURE_FP_STATUS:
	case DISP_FEATURE_FOLD_STATUS:
		return false;
	default:
		return true;
	}
}

int mi_dsi_panel_set_disp_param(struct dsi_panel *panel, struct disp_feature_ctl *ctl)
{
	int rc = 0;
	struct mi_dsi_panel_cfg *mi_cfg  = NULL;

	if (!panel || !ctl) {
		DISP_ERROR("invalid params\n");
		return -EINVAL;
	}

	mutex_lock(&panel->panel_lock);

	DISP_UTC_INFO("%s panel feature: %s, value: %d\n", panel->type,
		get_disp_feature_id_name(ctl->feature_id), ctl->feature_val);

	if (!panel->panel_initialized &&
		mi_dsi_panel_is_need_tx_cmd(ctl->feature_id)) {
		DISP_WARN("[%s] panel not initialized!\n", panel->type);
		rc = -ENODEV;
		goto exit;
	}

	mi_cfg = &panel->mi_cfg;

	switch (ctl->feature_id) {
	case DISP_FEATURE_DIMMING:
		if (mi_cfg->dimming_state != STATE_DIM_BLOCK) {
			if (ctl->feature_val == FEATURE_ON)
				dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DIMMINGON);
			else
				dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DIMMINGOFF);
			mi_cfg->feature_val[DISP_FEATURE_DIMMING] = ctl->feature_val;
		} else
			DISP_INFO("skip dimming %s\n", ctl->feature_val ? "on" : "off");
		break;
	case DISP_FEATURE_HBM:
		if (ctl->feature_val == FEATURE_ON) {
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_HBM_ON);
			mi_cfg->dimming_state = STATE_DIM_BLOCK;
		} else {
			mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_HBM_OFF,
					mi_cfg->last_bl_level);
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_HBM_OFF);
			mi_cfg->dimming_state = STATE_DIM_RESTORE;
		}
		mi_cfg->feature_val[DISP_FEATURE_HBM] = ctl->feature_val;
		break;
	case DISP_FEATURE_HBM_FOD:
		DISP_DEBUG("DISP_FEATURE_HBM_FOD disabled\n");
		break;
	case DISP_FEATURE_DOZE_BRIGHTNESS:
		if (is_aod_and_panel_initialized(panel) &&
			is_aod_brightness(ctl->feature_val)) {
			if (ctl->feature_val == DOZE_BRIGHTNESS_HBM) {
				mi_cfg->doze_brightness = DOZE_BRIGHTNESS_HBM;
				mi_cfg->doze_brightness_backup = DOZE_BRIGHTNESS_HBM;
				mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_HBM, 0);
				dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_HBM);
			} else {
				mi_cfg->doze_brightness = DOZE_BRIGHTNESS_LBM;
				mi_cfg->doze_brightness_backup = DOZE_BRIGHTNESS_LBM;
				mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_LBM, 0);
				dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_LBM);
			}
			mi_cfg->dimming_state = STATE_DIM_BLOCK;
		} else {
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_NOLP);
			mi_cfg->doze_brightness = DOZE_TO_NORMAL;
			mi_cfg->doze_brightness_backup = DOZE_TO_NORMAL;
			mi_cfg->dimming_state = STATE_DIM_RESTORE;
		}
		mi_cfg->feature_val[DISP_FEATURE_DOZE_BRIGHTNESS] = ctl->feature_val;
		break;
	case DISP_FEATURE_FOD_CALIBRATION_BRIGHTNESS:
		DISP_DEBUG("DISP_FEATURE_FOD_CALIBRATION_BRIGHTNESS disabled\n");
		break;
	case DISP_FEATURE_FOD_CALIBRATION_HBM:
		DISP_DEBUG("DISP_FEATURE_FOD_CALIBRATION_HBM disabled\n");
		break;
	case DISP_FEATURE_FLAT_MODE:
		if (ctl->feature_val == FEATURE_ON) {
			DISP_INFO("flat mode on\n");
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_FLAT_MODE_ON);
		} else {
			DISP_INFO("flat mode off\n");
			dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_FLAT_MODE_OFF);
		}
		mi_cfg->feature_val[DISP_FEATURE_FLAT_MODE] = ctl->feature_val;
		break;
	case DISP_FEATURE_NATURE_FLAT_MODE:
		DISP_DEBUG("DISP_FEATURE_NATURE_FLAT_MODE disabled\n");
		break;
	case DISP_FEATURE_DC:
		DISP_INFO("DC mode state:%d\n", ctl->feature_val);
		mi_cfg->feature_val[DISP_FEATURE_DC] = ctl->feature_val;
		mi_dsi_dc_mode_enable(panel, ctl->feature_val == FEATURE_ON);
		break;
	case DISP_FEATURE_CRC:
		DISP_DEBUG("DISP_FEATURE_CRC disabled\n");
		break;
	case DISP_FEATURE_LOCAL_HBM:
		DISP_DEBUG("DISP_FEATURE_LOCAL_HBM disabled\n");
		break;
	case DISP_FEATURE_SENSOR_LUX:
		DISP_DEBUG("DISP_FEATURE_SENSOR_LUX=%d\n", ctl->feature_val);
		mi_cfg->feature_val[DISP_FEATURE_SENSOR_LUX] = ctl->feature_val;
		break;
	case DISP_FEATURE_LOW_BRIGHTNESS_FOD:
		DISP_DEBUG("DISP_FEATURE_LOW_BRIGHTNESS_FOD disabled\n");
		break;
	case DISP_FEATURE_FP_STATUS:
		DISP_DEBUG("DISP_FEATURE_FP_STATUS disabled\n");
		break;
	case DISP_FEATURE_FOLD_STATUS:
		DISP_DEBUG("DISP_FEATURE_FOLD_STATUS disabled\n");
		break;
	case DISP_FEATURE_SPR_RENDER:
		DISP_DEBUG("DISP_FEATURE_SPR_RENDER disabled\n");
		break;
	case DISP_FEATURE_AOD_TO_NORMAL:
		if (ctl->feature_val == FEATURE_ON) {
			mi_dsi_update_backlight_in_aod(panel);
			mi_dsi_update_micfg_flags(panel, PANEL_NOLP);
			panel->mi_cfg.is_doze_to_off = false;
		} else if (ctl->feature_val == FEATURE_OFF) {
			switch (mi_cfg->doze_brightness_backup) {
			case DOZE_BRIGHTNESS_HBM:
				DISP_INFO("enter DOZE HBM\n");
				mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_HBM, 0);
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_HBM);
				if (rc) {
					DISP_ERROR("[%s] failed to send DSI_CMD_SET_MI_DOZE_HBM cmd, rc=%d\n",
						panel->name, rc);
				}
				mi_dsi_update_micfg_flags(panel, PANEL_DOZE_HIGH);
				break;
			case DOZE_BRIGHTNESS_LBM:
				DISP_INFO("enter DOZE LBM\n");
				mi_dsi_update_hbm_cmd_51reg(panel, DSI_CMD_SET_MI_DOZE_LBM, 0);
				rc = dsi_panel_tx_cmd_set(panel, DSI_CMD_SET_MI_DOZE_LBM);
				if (rc) {
					DISP_ERROR("[%s] failed to send DSI_CMD_SET_MI_DOZE_LBM cmd, rc=%d\n",
						panel->name, rc);
				}
				mi_dsi_update_micfg_flags(panel, PANEL_DOZE_LOW);
				break;
			default:
				break;
			}
			panel->mi_cfg.is_doze_to_off = true;
		}
		break;
	case DISP_FEATURE_COLOR_INVERT:
		DISP_DEBUG("DISP_FEATURE_COLOR_INVERT disabled\n");
		break;
	case DISP_FEATURE_DC_BACKLIGHT:
		DISP_INFO("DC backlight:%d\n", ctl->feature_val);
		mi_dsi_update_dc_backlight(panel, ctl->feature_val);
		break;
	case DISP_FEATURE_BIC:
		DISP_DEBUG("DISP_FEATURE_BIC disabled\n");
		break;
	case DISP_FEATURE_GIR:
		DISP_INFO("DISP_FEATURE_GIR ON:%d\n", ctl->feature_val);
		mi_cfg->feature_val[DISP_FEATURE_GIR] = ctl->feature_val;
        break;
	default:
		DISP_ERROR("invalid feature id\n");
		break;
	}

exit:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

ssize_t mi_dsi_panel_get_disp_param(struct dsi_panel *panel,
			char *buf, size_t size)
{
	struct mi_dsi_panel_cfg *mi_cfg;
	ssize_t count = 0;
	int i = 0;

	if (!panel) {
		DISP_ERROR("invalid params\n");
		return -EAGAIN;
	}

	mi_cfg = &panel->mi_cfg;

	count = snprintf(buf, size, "%040s: feature vaule\n", "feature name[feature id]");

	mutex_lock(&panel->panel_lock);
	for (i = DISP_FEATURE_DIMMING; i < DISP_FEATURE_MAX; i++) {
		count += snprintf(buf + count, size - count, "%036s[%02d]: %d\n",
				     get_disp_feature_id_name(i), i, mi_cfg->feature_val[i]);
	}
	mutex_unlock(&panel->panel_lock);

	return count;
}
