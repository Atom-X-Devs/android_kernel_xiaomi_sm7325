// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2017 Goodix

#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>

struct gf_ioc_chip_info {
	unsigned char vendor_id;
	unsigned char mode;
	unsigned char operation;
	unsigned char reserved[5];
};

struct gf_dev {
	dev_t devt;
	struct list_head device_entry;
	struct platform_device *spi;
	struct input_dev *input;

	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	int irq;
	int irq_enabled;
	char device_available;
};

#define GF_IOC_MAGIC	'g'			/*define magic number*/
#define GF_IOC_INIT					_IOR(GF_IOC_MAGIC, 0, uint8_t)
#define GF_IOC_RESET				_IO(GF_IOC_MAGIC, 2)
#define GF_IOC_ENABLE_IRQ			_IO(GF_IOC_MAGIC, 3)
#define GF_IOC_DISABLE_IRQ			_IO(GF_IOC_MAGIC, 4)
#define GF_IOC_ENABLE_POWER			_IO(GF_IOC_MAGIC, 7)
#define GF_IOC_DISABLE_POWER		_IO(GF_IOC_MAGIC, 8)
#define GF_IOC_CHIP_INFO			_IOW(GF_IOC_MAGIC, 13, struct gf_ioc_chip_info)
#define GF_IOC_MAXNR			14  /* THIS MACRO IS NOT USED NOW... */

#endif /*__GF_SPI_H*/
