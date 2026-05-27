// SPDX-License-Identifier: GPL-2.0-only
/*
 * Goodix Touchscreen Driver
 * Copyright (C) 2020 - 2021 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/input/mt.h>

#include "goodix_ts_core.h"

#define GOODIX_GESTURE_DOUBLE_TAP		0xCC
#define GOODIX_GESTURE_SINGLE_TAP		0x4C
#define GOODIX_GESTURE_FOD_DOWN			0x46
#define GOODIX_GESTURE_FOD_UP			0x55
#define GOODIX_GESTURE_SLIDE_UP			0xBA
#define GOODIX_GESTURE_SLIDE_DOWN		0xAB
#define GOODIX_GESTURE_SLIDE_LEFT		0xAA
#define GOODIX_GESTURE_SLIDE_RIGHT		0xBB

static ssize_t double_type_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
		       (type & GESTURE_DOUBLE_TAP) ? "enable" : "disable");
}

static ssize_t double_type_store(struct kobject *kobj,
				 struct kobj_attribute *attr, const char *buf,
				 size_t count)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	struct device *dev = cd->bus->dev;

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info(dev, "enable double tap");
		cd->gesture_type |= GESTURE_DOUBLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info(dev, "disable double tap");
		cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
	} else
		ts_err(dev, "invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t single_type_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
		       (type & GESTURE_SINGLE_TAP) ? "enable" : "disable");
}

static ssize_t single_type_store(struct kobject *kobj,
				 struct kobj_attribute *attr, const char *buf,
				 size_t count)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	struct device *dev = cd->bus->dev;

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info(dev, "enable single tap");
		cd->gesture_type |= GESTURE_SINGLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info(dev, "disable single tap");
		cd->gesture_type &= ~GESTURE_SINGLE_TAP;
	} else
		ts_err(dev, "invalid cmd[%d]", buf[0]);

	return count;
}

#if GOODIX_ENABLE_GESTURE_FOD
static ssize_t fod_type_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
		       (type & GESTURE_FOD_PRESS) ? "enable" : "disable");
}

static ssize_t fod_type_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	struct device *dev = cd->bus->dev;

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info(dev, "enable fod");
		cd->gesture_type |= GESTURE_FOD_PRESS;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info(dev, "disable fod");
		cd->gesture_type &= ~GESTURE_FOD_PRESS;
	} else
		ts_err(dev, "invalid cmd[%d]", buf[0]);

	return count;
}
#endif

#if GOODIX_ENABLE_GESTURE_SLIDE
static ssize_t slide_type_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
		       (type & (GESTURE_SLIDE_UP | GESTURE_SLIDE_DOWN | GESTURE_SLIDE_LEFT | GESTURE_SLIDE_RIGHT)) ? "enable" : "disable");
}

static ssize_t slide_type_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t count)
{
	struct device *device = container_of(kobj->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	struct device *dev = cd->bus->dev;

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info(dev, "enable slide");
		cd->gesture_type |= (GESTURE_SLIDE_UP | GESTURE_SLIDE_DOWN | GESTURE_SLIDE_LEFT | GESTURE_SLIDE_RIGHT);
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info(dev, "disable slide");
		cd->gesture_type &= ~(GESTURE_SLIDE_UP | GESTURE_SLIDE_DOWN | GESTURE_SLIDE_LEFT | GESTURE_SLIDE_RIGHT);
	} else {
		ts_err(dev, "invalid cmd[%d]", buf[0]);
	}

	return count;
}
#endif

int goodix_ts_report_gesture(struct goodix_ts_core *cd,
			     struct goodix_ts_event *event)
{
	struct device *dev = cd->bus->dev;
#if GOODIX_ENABLE_GESTURE_FOD
	int fodx, fody, overlay_area;
#endif

	switch (event->gesture_type) {
	case GOODIX_GESTURE_SINGLE_TAP:
		if (cd->gesture_type & GESTURE_SINGLE_TAP) {
			ts_info(dev, "get SINGLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			// input_report_key(cd->input_dev, KEY_GOTO, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			// input_report_key(cd->input_dev, KEY_GOTO, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug(dev, "not enable SINGLE-TAP");
		}
		break;
	case GOODIX_GESTURE_DOUBLE_TAP:
		if (cd->gesture_type & GESTURE_DOUBLE_TAP) {
			ts_info(dev, "get DOUBLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug(dev, "not enable DOUBLE-TAP");
		}
		break;
#if GOODIX_ENABLE_GESTURE_FOD
	case GOODIX_GESTURE_FOD_DOWN:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info(dev, "get FOD-DOWN gesture");
			fodx = le16_to_cpup((__le16 *)event->gesture_data);
			fody = le16_to_cpup(
				(__le16 *)(event->gesture_data + 2));
			overlay_area = event->gesture_data[4];
			ts_debug(dev, "fodx:%d fody:%d overlay_area:%d", fodx, fody,
				 overlay_area);
			input_report_key(cd->input_dev, BTN_TOUCH, 1);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev,
						   MT_TOOL_FINGER, 1);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_X,
					 fodx);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_Y,
					 fody);
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MAJOR,
					 overlay_area);
			input_sync(cd->input_dev);
		} else {
			ts_debug(dev, "not enable FOD-DOWN");
		}
		break;
	case GOODIX_GESTURE_FOD_UP:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info(dev, "get FOD-UP gesture");
			// fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			// fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			// overlay_area = gs_event.gesture_data[4];
			input_report_key(cd->input_dev, BTN_TOUCH, 0);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev,
						   MT_TOOL_FINGER, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug(dev, "not enable FOD-UP");
		}
		break;
#endif
#if GOODIX_ENABLE_GESTURE_SLIDE
	case GOODIX_GESTURE_SLIDE_UP:
		if (cd->gesture_type & GESTURE_SLIDE_UP) {
			ts_info(dev, "get SLIDE-UP gesture");
			//TODO
		} else {
			ts_debug(dev, "not enable SLIDE-UP");
		}
		break;
	case GOODIX_GESTURE_SLIDE_DOWN:
		if (cd->gesture_type & GESTURE_SLIDE_DOWN) {
			ts_info(dev, "get SLIDE-DOWN gesture");
			//TODO
		} else {
			ts_debug(dev, "not enable SLIDE-DOWN");
		}
		break;
	case GOODIX_GESTURE_SLIDE_LEFT:
		if (cd->gesture_type & GESTURE_SLIDE_LEFT) {
			//TODO
		} else {
			ts_debug(dev, "not enable SLIDE-LEFT");
		}
		break;
	case GOODIX_GESTURE_SLIDE_RIGHT:
		if (cd->gesture_type & GESTURE_SLIDE_RIGHT) {
			ts_info(dev, "get SLIDE-RIGHT gesture");
			//TODO
		} else {
			ts_debug(dev, "not enable SLIDE-RIGHT");
		}
		break;
#endif
	default:
		ts_err(dev, "not support gesture type[%02X]", event->gesture_type);
		break;
	}

	return 0;
}

static struct kobj_attribute double_type = __ATTR_RW(double_type);
static struct kobj_attribute single_type = __ATTR_RW(single_type);
#if GOODIX_ENABLE_GESTURE_FOD
static struct kobj_attribute fod_type = __ATTR_RW(fod_type);
#endif
#if GOODIX_ENABLE_GESTURE_SLIDE
static struct kobj_attribute slide_type = __ATTR_RW(slide_type);
#endif

static struct attribute *gesture_attrs[] = {
	&double_type.attr,
	&single_type.attr,
#if GOODIX_ENABLE_GESTURE_FOD
	&fod_type.attr,
#endif
#if GOODIX_ENABLE_GESTURE_SLIDE
	&slide_type.attr,
#endif
	NULL,
};

static struct attribute_group gesture_sysfs_group = {
	.attrs = gesture_attrs,
};

int gesture_module_init(struct goodix_ts_core *cd)
{
	int ret = -EINVAL;
	struct kobject *parent = &cd->pdev->dev.kobj;
	struct device *dev = cd->bus->dev;

	/* gesture sysfs init */
	cd->gesture_kobj = kobject_create_and_add("gesture", parent);
	if (!cd->gesture_kobj) {
		ts_err(dev, "failed create gesture sysfs node!");
		goto err_out;
	}

	ret = sysfs_create_group(cd->gesture_kobj, &gesture_sysfs_group);
	if (ret) {
		ts_err(dev, "failed create gesture sysfs files");
		kobject_put(cd->gesture_kobj);
		goto err_out;
	}

	cd->gesture_type = GESTURE_SINGLE_TAP | GESTURE_DOUBLE_TAP;

	ts_info(dev, "gesture module init success");
	return 0;

err_out:
	ts_err(dev, "gesture module init failed!");
	return ret;
}

void gesture_module_exit(struct goodix_ts_core *cd)
{
	struct device *dev = cd->bus->dev;

	ts_info(dev, "gesture module exit");

	sysfs_remove_group(cd->gesture_kobj, &gesture_sysfs_group);
	kobject_put(cd->gesture_kobj);
}
