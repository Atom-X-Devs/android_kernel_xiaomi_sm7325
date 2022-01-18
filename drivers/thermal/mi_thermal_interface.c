// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/kdev_t.h>
#include <linux/kernfs.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/thermal.h>
#include <linux/soc/qcom/panel_event_notifier.h>

#include <net/netlink.h>
#include <net/genetlink.h>

#include "thermal_core.h"
#include "../base/base.h"

#if defined(CONFIG_DRM_PANEL)
static struct drm_panel *active_panel;
#endif

struct mi_thermal_device  {
	struct device *dev;
	struct class *class;
	struct attribute_group attrs;
};

static atomic_t switch_mode = ATOMIC_INIT(-1);
static atomic_t temp_state = ATOMIC_INIT(0);
const char *board_sensor;
static char boost_buf[128];
static char board_sensor_temp[128];
static int screen_state = 0;
static int screen_last_status = 0;
static void *cookie = NULL;
static struct mi_thermal_device mi_thermal_dev;
static struct workqueue_struct *screen_state_wq;
static struct delayed_work screen_state_dw;

static ssize_t thermal_board_sensor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	if (!board_sensor)
		board_sensor = "invalid";

	return snprintf(buf, PAGE_SIZE, "%s", board_sensor);
}
static DEVICE_ATTR(board_sensor, 0664, thermal_board_sensor_show, NULL);

static ssize_t thermal_board_sensor_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, board_sensor_temp);
}

static ssize_t thermal_board_sensor_temp_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	snprintf(board_sensor_temp, PAGE_SIZE, buf);

	return len;
}
static DEVICE_ATTR(board_sensor_temp, 0664,	thermal_board_sensor_temp_show, thermal_board_sensor_temp_store);

static ssize_t thermal_boost_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, boost_buf);
}

static ssize_t thermal_boost_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	snprintf(boost_buf, PAGE_SIZE, buf);

	return len;
}
static DEVICE_ATTR(boost, 0644, thermal_boost_show, thermal_boost_store);

static ssize_t cpu_limits_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t cpu_limits_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned int cpu;
	unsigned int max;

	if (sscanf(buf, "cpu%u %u", &cpu, &max) != 2) {
		pr_err("input param error, can not prase param\n");
		return -EINVAL;
	}

	cpu_limits_set_level(cpu, max);

	return len;
}
static DEVICE_ATTR(cpu_limits, 0664, cpu_limits_show, cpu_limits_store);

static ssize_t thermal_sconfig_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&switch_mode));
}

static ssize_t thermal_sconfig_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int val = -1;

	val = simple_strtol(buf, NULL, 10);

	atomic_set(&switch_mode, val);

	return len;
}
static DEVICE_ATTR(sconfig, 0664, thermal_sconfig_show, thermal_sconfig_store);

static ssize_t thermal_screen_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", screen_state);
}
static DEVICE_ATTR(screen_state, 0664, thermal_screen_state_show, NULL);

static ssize_t thermal_temp_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&temp_state));
}

static ssize_t thermal_temp_state_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int val = -1;

	val = simple_strtol(buf, NULL, 10);

	atomic_set(&temp_state, val);

	return len;
}
static DEVICE_ATTR(temp_state, 0664, thermal_temp_state_show, thermal_temp_state_store);

static struct attribute *mi_thermal_dev_attr_group[] = {
	&dev_attr_board_sensor.attr,
	&dev_attr_board_sensor_temp.attr,
	&dev_attr_boost.attr,
	&dev_attr_cpu_limits.attr,
	&dev_attr_sconfig.attr,
	&dev_attr_screen_state.attr,
	&dev_attr_temp_state.attr,
	NULL,
};

static void create_thermal_message_node(void)
{
	int ret = 0;
	struct class *cls = NULL;
	struct kernfs_node *class_sd = NULL;
	struct kernfs_node *thermal_sd = NULL;
	struct kernfs_node *sysfs_sd = NULL;
	struct kobject *kobj_tmp = NULL;
	struct subsys_private *cp = NULL;

	sysfs_sd = kernel_kobj->sd->parent;
	if (sysfs_sd) {
		class_sd = kernfs_find_and_get(sysfs_sd, "class");
		if (class_sd) {
			thermal_sd = kernfs_find_and_get(class_sd, "thermal");
			if (thermal_sd) {
				kobj_tmp = (struct kobject *)thermal_sd->priv;
				if (kobj_tmp) {
					cp = to_subsys_private(kobj_tmp);
					cls = cp->class;
				} else
					pr_err("%s: can not find thermal kobj\n", __func__);
			} else
				pr_err("%s: can not find thermal_sd\n", __func__);
		} else
			pr_err("%s: can not find class_sd\n", __func__);
	} else
		pr_err("%s: sysfs_sd is NULL\n", __func__);

	if (!mi_thermal_dev.class && cls) {
		mi_thermal_dev.class = cls;
		mi_thermal_dev.dev = device_create(mi_thermal_dev.class, NULL, 'H', NULL, "thermal_message");
		if (!mi_thermal_dev.dev) {
			pr_err("%s create device dev err\n", __func__);
			return;
		}

		mi_thermal_dev.attrs.attrs = mi_thermal_dev_attr_group;
		ret = sysfs_create_group(&mi_thermal_dev.dev->kobj, &mi_thermal_dev.attrs);
		if (ret) {
			pr_err("%s ERROR: Cannot create sysfs structure!:%d\n", __func__, ret);
			return;
		}
	}
}

static void destroy_thermal_message_node(void)
{
	sysfs_remove_group(&mi_thermal_dev.dev->kobj, &mi_thermal_dev.attrs);
	if (mi_thermal_dev.class != NULL) {
		device_destroy(mi_thermal_dev.class,'H');
		mi_thermal_dev.class = NULL;
	}
}

#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
static const char *get_screen_state_name(int mode)
{
	switch (mode) {
	case DRM_PANEL_EVENT_UNBLANK:
		return "On";
	case DRM_PANEL_EVENT_BLANK_LP:
		return "Doze";
	case DRM_PANEL_EVENT_BLANK:
		return "Off";
	default:
		return "Unknown";
	}
}

static void screen_state_for_thermal_callback(enum panel_event_notifier_tag tag,
		struct panel_event_notification *notification, void *client_data)
{
	if (!notification) {
		printk(KERN_ERR "%s:Invalid notification\n", __func__);
		return;
	}

	if (notification->notif_data.early_trigger)
		return;

	switch (notification->notif_type) {
	case DRM_PANEL_EVENT_UNBLANK:
		screen_state = 1;
		break;
	case DRM_PANEL_EVENT_BLANK:
	case DRM_PANEL_EVENT_BLANK_LP:
		screen_state = 0;
		break;
	case DRM_PANEL_EVENT_FPS_CHANGE:
		return;
	default:
		return;
	}

	pr_debug("%s: mode: %s, screen_state = %d\n", __func__,
			get_screen_state_name(notification->notif_type),
			screen_state);

	if (screen_last_status != screen_state) {
		sysfs_notify(&mi_thermal_dev.dev->kobj, NULL, "screen_state");
		screen_last_status = screen_state;
	}
}

static int thermal_check_panel(struct device_node *np)
{
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "qcom,display-panels", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "qcom,display-panels", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);
		if (!IS_ERR(panel)) {
			active_panel = panel;
			return 0;
		} else
			active_panel = NULL;
	}

	return PTR_ERR(panel);
}

static void screen_state_check(struct work_struct *work)
{
	struct device_node *node;
	void *pvt_data = NULL;
	int error = 0;
	static int retry_count = 10;

	node = of_find_node_by_name(NULL, "thermal-screen");
	if (!node) {
		pr_err("%s ERROR: Cannot find node with panel!", __func__);
		return;
	}

	error = thermal_check_panel(node);
	if (error == -EPROBE_DEFER)
		pr_err("%s ERROR: Cannot fine panel of node!", __func__);

	if (active_panel) {
		cookie = panel_event_notifier_register(
				PANEL_EVENT_NOTIFICATION_PRIMARY,
				PANEL_EVENT_NOTIFIER_CLIENT_MI_THERMAL,
				active_panel,
				screen_state_for_thermal_callback,
				pvt_data);
		if (IS_ERR(cookie))
			pr_err("%s:Failed to register for panel events\n", __func__);
		else
			pr_info("%s: panel_event_notifier_register succeed\n", __func__);
	} else if (retry_count > 0) {
		pr_err("%s:active_panel is NULL Failed to register for panel events\n", __func__);
		retry_count--;
		queue_delayed_work(screen_state_wq, &screen_state_dw, 5 * HZ);
	}
}
#endif

static int of_parse_thermal_message(void)
{
	struct device_node *np;

	np = of_find_node_by_name(NULL, "thermal-message");
	if (!np)
		return -EINVAL;

	if (of_property_read_string(np, "board-sensor", &board_sensor))
		return -EINVAL;

	pr_info("%s board sensor: %s\n", __func__, board_sensor);

	return 0;
}

static int __init mi_thermal_interface_init(void)
{
	int result;

#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	screen_state_wq = create_singlethread_workqueue("screen_state_wq");
	if (screen_state_wq) {
		INIT_DELAYED_WORK(&screen_state_dw, screen_state_check);
		queue_delayed_work(screen_state_wq, &screen_state_dw, 5 * HZ);
	}
#endif

	result = of_parse_thermal_message();
	if (result)
		pr_err("%s: Can not parse thermal message node: %d\n", __func__, result);

	create_thermal_message_node();

	return 0;
}
module_init(mi_thermal_interface_init);

static void __exit mi_thermal_interface_exit(void)
{
#if defined(CONFIG_OF) && defined(CONFIG_DRM_PANEL)
	if (screen_state_wq) {
		cancel_delayed_work_sync(&screen_state_dw);
		destroy_workqueue(screen_state_wq);
	}

	if (active_panel && !IS_ERR(cookie))
		panel_event_notifier_unregister(cookie);
	else
		pr_err("%s:panel_event_notifier_unregister failed\n", __func__);
#endif

	destroy_thermal_message_node();
}
module_exit(mi_thermal_interface_exit);

MODULE_AUTHOR("Xiaomi thermal team");
MODULE_DESCRIPTION("Xiaomi thermal control interface");
MODULE_LICENSE("GPL v2");