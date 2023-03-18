// SPDX-License-Identifier: GPL-2.0-only
/*
 * TEE driver for goodix fingerprint sensor
 * Copyright (C) 2016 Goodix
 */

 #define pr_fmt(fmt)     KBUILD_MODNAME ": " fmt

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <net/netlink.h>
#include <net/sock.h>

#include "goodix_tee.h"

#define VER_MAJOR   1
#define VER_MINOR   2
#define PATCH_LEVEL 1

/*device name after register in charater*/
#define GF_DEV_NAME				"goodix_fp"

#define N_SPI_MINORS			32  /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static DEFINE_MUTEX(regulator_ocp_lock);

static struct regulator *p_3v3_vreg = NULL;
static struct gf_dev gf;
static int SPIDEV_MAJOR;

/*********************************
 *       NETLINK INTERFACE       *
 *********************************/
static int pid = -1;
struct sock *nl_sk;

#define MAX_MSGSIZE 2
static inline void sendnlmsg(void)
{
	struct sk_buff *skb_1 = alloc_skb(NLMSG_SPACE(MAX_MSGSIZE), GFP_KERNEL);
	struct nlmsghdr *nlh = nlmsg_put(skb_1, 0, 0, 0, MAX_MSGSIZE, 0);
	char msg[MAX_MSGSIZE] = { 1, 0 };

	NETLINK_CB(skb_1).portid = 0;
	NETLINK_CB(skb_1).dst_group = 0;
	memcpy(NLMSG_DATA(nlh), msg, MAX_MSGSIZE);
	netlink_unicast(nl_sk, skb_1, pid, MSG_DONTWAIT);
}

static inline void nl_data_ready(struct sk_buff *__skb)
{
	struct sk_buff *skb = skb_get(__skb);
	struct nlmsghdr *nlh;

	if (skb->len >= NLMSG_SPACE(0)) {
		nlh = nlmsg_hdr(skb);
		pid = nlh->nlmsg_pid;
		kfree_skb(skb);
	}
}

#define NETLINK_TEST 25
static void netlink_init(void)
{
	struct netlink_kernel_cfg netlink_cfg;

	memset(&netlink_cfg, 0, sizeof(struct netlink_kernel_cfg));
	netlink_cfg.groups = 0;
	netlink_cfg.flags = 0;
	netlink_cfg.input = nl_data_ready;
	netlink_cfg.cb_mutex = NULL;

	nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST, &netlink_cfg);
	if (!nl_sk)
		pr_err("create netlink socket error\n");
}

static void netlink_exit(void)
{
	if (nl_sk) {
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}

/*********************************
 *        REGULATOR SETUP        *
 *********************************/
static int enable_regulator_3V3(struct device *dev)
{
	int rc = 0;

	//struct regulator *vreg;
	mutex_lock(&regulator_ocp_lock);

	p_3v3_vreg = devm_regulator_get(dev, "l3c_vdd");
	if (IS_ERR(p_3v3_vreg)) {
		pr_err("fp %s: no of vreg found\n", __func__);
		rc = PTR_ERR(p_3v3_vreg);
		goto exit;
	}

	if (regulator_is_enabled(p_3v3_vreg)) {
		dev_err(dev, "%s: regulator_is_enabled!\n", __func__);
		goto exit;
	}

	rc = regulator_set_load(p_3v3_vreg, 200000);
	if (rc) {
		dev_err(dev, "fp %s: set load faild\n", __func__);
		goto exit;
	}

	rc = regulator_enable(p_3v3_vreg);
	if (rc) {
		dev_err(dev, "fp %s: enable voltage failed\n", __func__);
		goto exit;
	}

exit:
	mutex_unlock(&regulator_ocp_lock);
	return rc;
}

static int disable_regulator_3V3(void)
{
	int rc = 0;

	mutex_lock(&regulator_ocp_lock);
	if (p_3v3_vreg == NULL) {
		pr_err("p_3v3_vreg is null!");
		goto exit;
	}

	if (regulator_is_enabled(p_3v3_vreg)) {
		rc = regulator_disable(p_3v3_vreg);
		if (rc) {
			pr_err("disable voltage failed\n");
			goto exit;
		}
	}
	devm_regulator_put(p_3v3_vreg);
	p_3v3_vreg = NULL;

exit:
	mutex_unlock(&regulator_ocp_lock);

	return 0;
}


static void gf_irq_setup(struct gf_dev *gf_dev, int status)
{
	if (!gf_dev->irq_enabled && status) {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	} else if (gf_dev->irq_enabled && !status) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	}

	gf_dev->irq_enabled = status;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	int retval = 0;
	u8 netlink_route = NETLINK_TEST;
	struct gf_ioc_chip_info info;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if ((_IOC_DIR(cmd) & _IOC_READ) || (_IOC_DIR(cmd) & _IOC_WRITE)) {
		retval = !access_ok((void __user *)arg, _IOC_SIZE(cmd));
		if (retval)
			return -EFAULT;
	}

	if (gf_dev->device_available == 0) {
		if ((cmd == GF_IOC_ENABLE_POWER) || (cmd == GF_IOC_DISABLE_POWER)) {
			pr_debug("power cmd\n");
		} else {
			pr_debug("get cmd %d, but sensor is power off currently.\n", _IOC_NR(cmd));
			return -ENODEV;
		}
	}

	switch (cmd) {
	case GF_IOC_INIT:
		if (copy_to_user((void __user *)arg, (void *)&netlink_route, sizeof(u8)))
			retval = -EFAULT;

		break;
	case GF_IOC_DISABLE_IRQ:
		gf_irq_setup(gf_dev, 0);
		break;
	case GF_IOC_ENABLE_IRQ:
		gf_irq_setup(gf_dev, 1);
		break;
	case GF_IOC_RESET:
		gpio_direction_output(gf_dev->reset_gpio, 0);
		mdelay(3);
		gpio_set_value(gf_dev->reset_gpio, 1);
		mdelay(3);
		break;
	case GF_IOC_ENABLE_POWER:
		if (!gf_dev->device_available) {
			enable_regulator_3V3(&gf_dev->spi->dev);
			msleep(10); //Do we really need this?
		}
		gf_dev->device_available = 1;
		break;
	case GF_IOC_DISABLE_POWER:
		if (gf_dev->device_available)
			disable_regulator_3V3();

		gf_dev->device_available = 0;
		break;
	case GF_IOC_CHIP_INFO:
		if (copy_from_user(&info, (struct gf_ioc_chip_info *)arg,
					sizeof(struct gf_ioc_chip_info))) {
			retval = -EFAULT;
			break;
		}
		pr_debug("vendor_id : 0x%x\n", info.vendor_id);
		pr_debug("mode : 0x%x\n", info.mode);
		pr_debug("operation: 0x%x\n", info.operation);
		break;
	default:
		pr_debug("unsupport cmd:0x%x\n", cmd);
		break;
	}

	return retval;
}

#ifdef CONFIG_COMPAT
static long gf_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
	sendnlmsg();

	return IRQ_HANDLED;
}

static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;
	int rc = 0;
	int err = 0;

	mutex_lock(&device_list_lock);
	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			pr_debug("Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		rc = gpio_request(gf_dev->reset_gpio, "gpio-reset");
		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request RESET GPIO. rc = %d\n", rc);
			err = -EPERM;
			goto open_error1;
		}
		gpio_direction_output(gf_dev->reset_gpio, 0);

		rc = gpio_request(gf_dev->irq_gpio, "gpio-irq");
		if (rc) {
			dev_err(&gf_dev->spi->dev, "Failed to request IRQ GPIO. rc = %d\n", rc);
			err = -EPERM;
			goto open_error2;
		}
		gpio_direction_input(gf_dev->irq_gpio);

		rc = request_threaded_irq(gf_dev->irq, NULL, gf_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);
		if (rc) {
			err = -EPERM;
			goto open_error3;
		}

		enable_irq_wake(gf_dev->irq);
		gf_dev->irq_enabled = 1;
		gf_irq_setup(gf_dev, 0);

		gf_dev->users++;
		filp->private_data = gf_dev;
		nonseekable_open(inode, filp);
	} else
		pr_debug("No device for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	return status;

open_error3:
	gpio_free(gf_dev->irq_gpio);
open_error2:
	gpio_free(gf_dev->reset_gpio);
open_error1:
	mutex_unlock(&device_list_lock);
	return err;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;
	gf_dev->users--;
	if (!gf_dev->users) {
		gf_irq_setup(gf_dev, 0);
		gf_dev->device_available = 0;
		free_irq(gf_dev->irq, gf_dev);
		gpio_free(gf_dev->irq_gpio);
		gpio_free(gf_dev->reset_gpio);
		disable_regulator_3V3();
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
};

static inline int gf_parse_dts(struct gf_dev *gf_dev)
{
	/*get reset resource*/
	gf_dev->reset_gpio =
		of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio-reset", 0);
	if (!gpio_is_valid(gf_dev->reset_gpio)) {
		pr_info("RESET GPIO is invalid.\n");
		return -EPERM;
	}

	/*get irq resourece*/
	gf_dev->irq_gpio =
		of_get_named_gpio(gf_dev->spi->dev.of_node, "goodix,gpio-irq", 0);
	if (!gpio_is_valid(gf_dev->irq_gpio)) {
		pr_info("IRQ GPIO is invalid.\n");
		return -EPERM;
	}

	return 0;
}

static inline void gf_cleanup(struct gf_dev *gf_dev)
{
	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

static inline int gf_irq_num(struct gf_dev *gf_dev)
{
	if (gf_dev == NULL) {
		pr_info("Input buff is NULL.\n");
		return -EPERM;
	}

	return gpio_to_irq(gf_dev->irq_gpio);
}

static struct class *gf_class;
static int gf_probe(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;
	unsigned long minor;
	int status = 0;

	/* Initialize the driver data */
	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = pdev;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->devt = 0;

	if (gf_parse_dts(gf_dev))
		goto error_hw;

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev,
				gf_dev->devt, gf_dev, GF_DEV_NAME);
		if (!IS_ERR(dev)) {
			set_bit(minor, minors);
			list_add(&gf_dev->device_entry, &device_list);
		} else
			status = PTR_ERR(dev);
	} else {
		dev_dbg(&gf_dev->spi->dev, "no minor number available!\n");
		status = -ENODEV;
		mutex_unlock(&device_list_lock);
		goto error_hw;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			pr_err("%s, failed to allocate input device\n", __func__);
			status = -ENOMEM;
			goto error_dev;
		}

		gf_dev->input->name = "uinput-goodix";
		gf_dev->input->id.vendor  = 0x0666;
		gf_dev->input->id.product = 0x0888;

		status = input_register_device(gf_dev->input);
		if (status) {
			pr_err("failed to register input device\n");
			goto error_input;
		}
	}

	gf_dev->irq = gf_irq_num(gf_dev);
	dev_info(&gf_dev->spi->dev, "version: V%d.%d.%02d\n",
		VER_MAJOR, VER_MINOR, PATCH_LEVEL);

	return status;

error_input:
	if (gf_dev->input != NULL)
		input_free_device(gf_dev->input);
error_dev:
	if (gf_dev->devt != 0) {
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
	}
error_hw:
	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	return status;
}

static int gf_remove(struct platform_device *pdev)
{
	struct gf_dev *gf_dev = &gf;

	disable_regulator_3V3();

	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->irq)
		free_irq(gf_dev->irq, gf_dev);

	if (gf_dev->input != NULL) {
		input_unregister_device(gf_dev->input);
		input_free_device(gf_dev->input);
	}

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	mutex_unlock(&device_list_lock);

	if (gf_dev->users == 0)
		gf_cleanup(gf_dev);

	return 0;
}

static struct of_device_id gx_match_table[] = {
	{ .compatible = "goodix,fingerprint" },
	{},
};

static struct platform_driver gf_driver = {
	.driver = {
		.name = GF_DEV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gx_match_table,
	},
	.probe = gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status;
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);

	status = register_chrdev(SPIDEV_MAJOR, "goodix_fp_spi", &gf_fops);
	if (status < 0) {
		pr_warn("Failed to register char device!\n");
		return status;
	}

	SPIDEV_MAJOR = status;

	gf_class = class_create(THIS_MODULE, "goodix_fp");
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to create class.\n");
		return PTR_ERR(gf_class);
	}

	status = platform_driver_register(&gf_driver);
	if (status < 0) {
		class_destroy(gf_class);
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		pr_warn("Failed to register SPI driver.\n");
	}
	netlink_init();

	return 0;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	netlink_exit();
	platform_driver_unregister(&gf_driver);
	class_destroy(gf_class);
	unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
}
module_exit(gf_exit);

MODULE_DESCRIPTION("fingerprint sensor device driver");
MODULE_LICENSE("GPL v2");
