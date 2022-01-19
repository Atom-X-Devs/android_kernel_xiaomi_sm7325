

// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2019, The Linux Foundation. All rights reserved.
// Copyright (C) 2021 XiaoMi, Inc.

#ifndef _MI_DRM_NOTIFIER_H_
#define _MI_DRM_NOTIFIER_H_

#include <linux/notifier.h>

/* A hardware display power mode state change occurred */
#define MI_DISP_DPMS_EVENT             0x01
/* A hardware display power mode state early change occurred */
#define MI_DISP_DPMS_EARLY_EVENT       0x02

enum {
	/* panel: power on */
	MI_DISP_DPMS_ON			= 0,
	MI_DISP_DPMS_LP1		= 1,
	MI_DISP_DPMS_LP2		= 2,
	MI_DISP_DPMS_STANDBY	= 3,
	MI_DISP_DPMS_SUSPEND	= 4,
	/* panel: power off */
	MI_DISP_DPMS_POWERDOWN	= 5,
};

enum mi_disp_id {
	MI_DISPLAY_PRIMARY = 0,
	MI_DISPLAY_SECONDARY,
	MI_DISPLAY_MAX,
};

struct mi_disp_notifier {
	int disp_id;
	void *data;
};

int mi_disp_register_client(struct notifier_block *nb);
int mi_disp_unregister_client(struct notifier_block *nb);
int mi_disp_notifier_call_chain(unsigned long val, void *v);

#endif /* _MI_DRM_NOTIFIER_H_ */
