// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <trace/hooks/signal.h>

static void reap_eligible(void *data, struct task_struct *task, bool *reap)
{
	/* TODO: Can this logic be moved to module params approach? */
	if (!strcmp(task->comm, "lmkd") || !strcmp(task->comm, "PreKillActionT"))
		*reap = true;
}

static int __init init_mem_hooks(void)
{
	int ret;

	ret = register_trace_android_vh_process_killed(reap_eligible, NULL);
	if (ret) {
		pr_err("Failed to register process_killed hooks\n");
		return ret;
	}

	return 0;
}
module_init(init_mem_hooks);

MODULE_DESCRIPTION("Qualcomm Technologies, Inc. Memory Trace Hook Call-Back Registration");
MODULE_LICENSE("GPL v2");
