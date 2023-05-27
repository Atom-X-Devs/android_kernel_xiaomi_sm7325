/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#ifndef _MI_SDE_CONNECTOR_H_
#define _MI_SDE_CONNECTOR_H_

#include "drm_connector.h"

struct sde_connector;

int mi_sde_connector_register_esd_irq(struct sde_connector *c_conn);

int mi_sde_connector_gir_fence(struct drm_connector *connector);
int mi_sde_connector_hbm_fence(struct drm_connector *connector);
int mi_sde_connector_dc_fence(struct drm_connector *connector);

#endif /* _MI_SDE_CONNECTOR_H_ */
