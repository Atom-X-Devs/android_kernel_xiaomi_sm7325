/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 * Copyright (C) 2021 XiaoMi, Inc.
 */

#ifndef _MI_PANEL_ID_H_
#define _MI_PANEL_ID_H_

#include <linux/types.h>
#include "dsi_display.h"
#include "dsi_panel.h"

#define K9D_36_02_0A_PANEL_ID   0x4b394400360200
#define K9D_42_0D_0B_PANEL_ID   0x4b394400420d00

/*
 * PA: Primary display, First selection screen
 * PB: Primary display, Second selection screen
 * SA: Secondary display, First selection screen
 * SB: Secondary display, Second selection screen
 */
enum mi_project_panel_id {
	PANEL_ID_INVALID = 0,
	K9D_TIAN_PA,
	K9D_CSOT_PA,
	PANEL_ID_MAX
};

static inline enum mi_project_panel_id mi_get_panel_id(u64 mi_panel_id)
{
	switch(mi_panel_id) {
		case K9D_36_02_0A_PANEL_ID:
			return K9D_TIAN_PA;
		case K9D_42_0D_0B_PANEL_ID:
			return K9D_CSOT_PA;
		default:
			return PANEL_ID_INVALID;
	}
}

static inline const char *mi_get_panel_id_name(u64 mi_panel_id)
{
	switch (mi_get_panel_id(mi_panel_id)) {
		case K9D_TIAN_PA:
			return "K9D_TIAN_PA";
		case K9D_CSOT_PA:
			return "K9D_CSOT_PA";
		default:
			return "unknown";
	}
}

static inline bool is_use_nvt_dsc_config(u64 mi_panel_id)
{
	switch(mi_panel_id) {
	case K9D_36_02_0A_PANEL_ID:
		return true;
	default:
		return false;
	}
}

#endif /* _MI_PANEL_ID_H_ */
