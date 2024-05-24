// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "VendorHooks: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <trace/hooks/timer.h>

static void timer_recalc_index(void *unused,
			unsigned int lvl, unsigned long *expires)
{
	*expires -= 1;
}

static int __init qcom_vendor_hook_driver_init(void)
{
	int ret;

	ret = register_trace_android_vh_timer_calc_index(timer_recalc_index, NULL);
	if (ret)
		pr_err("Failed to android_vh_timer_calc_index hook\n");

	return ret;
}

static void __exit qcom_vendor_hook_driver_exit(void)
{
	/* Reset all initialized global variables and unregister callbacks. */
	unregister_trace_android_vh_timer_calc_index(timer_recalc_index, NULL);
}

#if IS_MODULE(CONFIG_QCOM_CPU_VENDOR_HOOKS)
module_init(qcom_vendor_hook_driver_init);
#else
pure_initcall(qcom_vendor_hook_driver_init);
#endif
module_exit(qcom_vendor_hook_driver_exit);
MODULE_DESCRIPTION("QCOM CPU Vendor Hooks Driver");
MODULE_LICENSE("GPL v2");
