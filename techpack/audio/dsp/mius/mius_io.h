/*
 * Copyright (C) 2021 XiaoMi, Inc.
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef MIUS_IO_H
#define MIUS_IO_H

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/wait.h>

#include <asm/atomic.h>
#include <asm/uaccess.h>

#include <mius/mius_data_io.h>
#include <mius/mius_device.h>

#endif //MIUS_IO_H
