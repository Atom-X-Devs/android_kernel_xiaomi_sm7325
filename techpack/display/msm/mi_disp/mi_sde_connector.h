/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#ifndef _MI_SDE_CONNECTOR_H_
#define _MI_SDE_CONNECTOR_H_

#include "drm_connector.h"

#include "mi_disp_config.h"

struct sde_connector;

enum mi_panel_op_code {
	MI_FOD_HBM_ON = 0,
	MI_FOD_HBM_OFF = 1,
	MI_FOD_AOD_TO_NORMAL = 2,
	MI_FOD_NORMAL_TO_AOD = 4,
};

int mi_sde_connector_register_esd_irq(struct sde_connector *c_conn);

#if MI_DISP_DEBUGFS_ENABLE
int mi_sde_connector_debugfs_esd_sw_trigger(void *display);
#else
static inline int mi_sde_connector_debugfs_esd_sw_trigger(void *display) { return 0; }
#endif

int mi_sde_connector_panel_ctl(struct drm_connector *connector, uint32_t op_code);

int mi_sde_connector_gir_fence(struct drm_connector *connector);

#endif /* _MI_SDE_CONNECTOR_H_ */
