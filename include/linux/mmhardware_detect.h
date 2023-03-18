// SPDX-License-Identifier: GPL-2.0-only

#ifndef __MMHARDWARE_OTHERS_H__
#define __MMHARDWARE_OTHERS_H__

#define MM_HARDWARE_SYSFS_OTHERS_FOLDER           "others"
#define MM_HARDWARE_SYSFS_AUDIOSWITCH_FOLDER      "audioswitch"
#define MM_HARDWARE_SYSFS_HAPTIC_1_FOLDER         "haptic1"
#define MM_HARDWARE_SYSFS_HAPTIC_2_FOLDER         "haptic2"

enum hardware_id {
	MM_HW_AS            = 0x800,
	MM_HW_HAPTIC_1      = 0x1000,
	MM_HW_HAPTIC_2      = 0x2000
};

struct mm_info {
	struct kobj_attribute k_attr;
	enum hardware_id mm_id;
	int on_register;
};

int register_otherkobj_under_mmsysfs(enum hardware_id mm_id, const char *name);

#endif
