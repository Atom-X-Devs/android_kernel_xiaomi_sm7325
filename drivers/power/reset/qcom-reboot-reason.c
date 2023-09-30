// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2019 The Linux Foundation. All rights reserved.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/pm.h>
#include <linux/of_address.h>
#include <linux/nvmem-consumer.h>

struct qcom_reboot_reason {
	struct device *dev;
	struct notifier_block reboot_nb;
#ifdef CONFIG_MACH_XIAOMI
	struct notifier_block panic_nb;
#endif
	struct nvmem_cell *nvmem_cell;
};

struct poweroff_reason {
	const char *cmd;
	unsigned char pon_reason;
};

static struct poweroff_reason reasons[] = {
	{ "recovery",			0x01 },
	{ "bootloader",			0x02 },
	{ "rtc",			0x03 },
	{ "dm-verity device corrupted",	0x04 },
	{ "dm-verity enforcing",	0x05 },
	{ "keys clear",			0x06 },
#ifdef CONFIG_MACH_XIAOMI
	{ "panic",			0x21 },
	{ NULL,				0x20 },
#endif
	{}
};

#ifdef CONFIG_MACH_XIAOMI
#define RESTART_REASON_PANIC  6
#define RESTART_REASON_NORMAL 7
#endif

static int qcom_reboot_reason_reboot(struct notifier_block *this,
				     unsigned long event, void *ptr)
{
	char *cmd = ptr;
	struct qcom_reboot_reason *reboot = container_of(this,
		struct qcom_reboot_reason, reboot_nb);
	struct poweroff_reason *reason;

	if (!cmd) {
#ifdef CONFIG_MACH_XIAOMI
		nvmem_cell_write(reboot->nvmem_cell,
				 &reasons[RESTART_REASON_NORMAL].pon_reason,
				 sizeof(reasons[RESTART_REASON_NORMAL].pon_reason));
#endif
		return NOTIFY_OK;
	}
	for (reason = reasons; reason->cmd; reason++) {
		if (!strcmp(cmd, reason->cmd)) {
			nvmem_cell_write(reboot->nvmem_cell,
					 &reason->pon_reason,
					 sizeof(reason->pon_reason));
#ifndef CONFIG_MACH_XIAOMI
			break;
#else
			return NOTIFY_OK;
#endif
		}
	}

#ifdef CONFIG_MACH_XIAOMI
	nvmem_cell_write(reboot->nvmem_cell, &reason->pon_reason,
			sizeof(reason->pon_reason));
#endif

	return NOTIFY_OK;
}

#ifdef CONFIG_MACH_XIAOMI
static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	struct qcom_reboot_reason *reboot = container_of(this,
	struct qcom_reboot_reason, panic_nb);

	nvmem_cell_write(reboot->nvmem_cell,
			&reasons[RESTART_REASON_PANIC].pon_reason,
			sizeof(reasons[RESTART_REASON_PANIC].pon_reason));

	return NOTIFY_DONE;
}
#endif

static int qcom_reboot_reason_probe(struct platform_device *pdev)
{
	struct qcom_reboot_reason *reboot;

	reboot = devm_kzalloc(&pdev->dev, sizeof(*reboot), GFP_KERNEL);
	if (!reboot)
		return -ENOMEM;

	reboot->dev = &pdev->dev;

	reboot->nvmem_cell = nvmem_cell_get(reboot->dev, "restart_reason");

	if (IS_ERR(reboot->nvmem_cell))
		return PTR_ERR(reboot->nvmem_cell);

	reboot->reboot_nb.notifier_call = qcom_reboot_reason_reboot;
	reboot->reboot_nb.priority = 255;
	register_reboot_notifier(&reboot->reboot_nb);

	platform_set_drvdata(pdev, reboot);

#ifdef CONFIG_MACH_XIAOMI
	reboot->panic_nb.notifier_call = panic_prep_restart;
	reboot->panic_nb.priority = INT_MAX;
	atomic_notifier_chain_register(&panic_notifier_list, &reboot->panic_nb);
#endif

	return 0;
}

static int qcom_reboot_reason_remove(struct platform_device *pdev)
{
	struct qcom_reboot_reason *reboot = platform_get_drvdata(pdev);

#ifdef CONFIG_MACH_XIAOMI
	atomic_notifier_chain_unregister(&panic_notifier_list, &reboot->panic_nb);
#endif
	unregister_reboot_notifier(&reboot->reboot_nb);

	return 0;
}

static const struct of_device_id of_qcom_reboot_reason_match[] = {
	{ .compatible = "qcom,reboot-reason", },
	{},
};
MODULE_DEVICE_TABLE(of, of_qcom_reboot_reason_match);

static struct platform_driver qcom_reboot_reason_driver = {
	.probe = qcom_reboot_reason_probe,
	.remove = qcom_reboot_reason_remove,
	.driver = {
		.name = "qcom-reboot-reason",
		.of_match_table = of_match_ptr(of_qcom_reboot_reason_match),
	},
};

module_platform_driver(qcom_reboot_reason_driver);

MODULE_DESCRIPTION("MSM Reboot Reason Driver");
MODULE_LICENSE("GPL v2");
