// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 XiaoMi, Inc.
 */

#ifndef _MI_DISP_PRINT_H_
#define _MI_DISP_PRINT_H_

#include <linux/compiler.h>
#include <linux/printk.h>

#define DISP_WARN(fmt, ...) no_printk(KERN_WARNING fmt, ##__VA_ARGS__)
#define DISP_INFO(fmt, ...) no_printk(KERN_INFO fmt, ##__VA_ARGS__)
#define DISP_ERROR(fmt, ...) no_printk(KERN_ERR fmt, ##__VA_ARGS__)
#define DISP_DEBUG(fmt, ...) no_printk(fmt, ##__VA_ARGS__)

#define DISP_UTC_WARN(fmt, ...) no_printk(KERN_WARNING fmt, ##__VA_ARGS__)
#define DISP_UTC_INFO(fmt, ...) no_printk(KERN_INFO fmt, ##__VA_ARGS__)
#define DISP_UTC_ERROR(fmt, ...) no_printk(KERN_ERR fmt, ##__VA_ARGS__)
#define DISP_UTC_DEBUG(fmt, ...) no_printk(fmt, ##__VA_ARGS__)

#endif /* _MI_DISP_PRINT_H_ */
