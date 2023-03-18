// SPDX-License-Identifier: GPL-2.0-only

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

struct xiaomi_fp_data {
	struct device *dev;
	struct mutex lock;
	int fingerdown;
	int actpower;
};

static ssize_t get_fingerdown_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xiaomi_fp_data *xiaomi_fp = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", xiaomi_fp->fingerdown);
}

static ssize_t set_fingerdown_event(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct xiaomi_fp_data *xiaomi_fp = dev_get_drvdata(dev);

	if (!strncmp(buf, "1", strlen("1"))) {
		xiaomi_fp->fingerdown = 1;
		sysfs_notify(&xiaomi_fp->dev->kobj, NULL, "fingerdown");
	} else if (!strncmp(buf, "0", strlen("0"))) {
		xiaomi_fp->fingerdown = 0;
	} else
		return -EINVAL;

	return count;
}

static ssize_t get_actpower_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct xiaomi_fp_data *xiaomi_fp = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", xiaomi_fp->actpower);
}

static ssize_t set_actpower_event(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct xiaomi_fp_data *xiaomi_fp = dev_get_drvdata(dev);

	dev_info(xiaomi_fp->dev, "%s -> %s\n", __func__, buf);
	if (!strncmp(buf, "0", strlen("0")))
		xiaomi_fp->actpower = 0;
	else if (!strncmp(buf, "1", strlen("1"))) {
		xiaomi_fp->actpower = 1;
		sysfs_notify(&xiaomi_fp->dev->kobj, NULL, "actpower");
	} else if (!strncmp(buf, "2", strlen("2"))) {
		xiaomi_fp->actpower = 2;
		sysfs_notify(&xiaomi_fp->dev->kobj, NULL, "actpower");
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(actpower, 0644, get_actpower_event, set_actpower_event);
static DEVICE_ATTR(fingerdown, 0644, get_fingerdown_event, set_fingerdown_event);

static struct attribute *xm_fp[] = {
	&dev_attr_fingerdown.attr,
	&dev_attr_actpower.attr,
	NULL,
};

static const struct attribute_group xm_fp_grp = {
	.attrs = xm_fp,
};

static int xiaomi_fp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xiaomi_fp_data *xiaomi_fp = NULL;
	int rc = 0;

	xiaomi_fp = devm_kzalloc(dev, sizeof(*xiaomi_fp), GFP_KERNEL);
	if (!xiaomi_fp) {
		dev_err(dev,"failed to allocate memory for struct xiaomi_fp\n");
		return -ENOMEM;
	}

	xiaomi_fp->dev = dev;
	xiaomi_fp->fingerdown = 0;
	platform_set_drvdata(pdev, xiaomi_fp);
	mutex_init(&xiaomi_fp->lock);

	rc = sysfs_create_group(&dev->kobj, &xm_fp_grp);
	if (rc)
		dev_err(dev, "xiaomi_fp could not create sysfs\n");

	return 0;
}

static int xiaomi_fp_remove(struct platform_device *pdev)
{
	struct xiaomi_fp_data  *xiaomi_fp = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &xm_fp_grp);
	mutex_destroy(&xiaomi_fp->lock);

	return 0;
}

static struct of_device_id xiaomi_fp_of_match[] = {
	{.compatible = "xiaomi-fingerprint",},
	{}
};
MODULE_DEVICE_TABLE(of, xiaomi_fp_of_match);

static struct platform_driver xiaomi_fp_driver = {
	.probe = xiaomi_fp_probe,
	.remove = xiaomi_fp_remove,
	.driver = {
		.name = "xiaomi-fp",
		.of_match_table = xiaomi_fp_of_match,
	},
};
module_platform_driver(xiaomi_fp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Xiaomi Fingerprint Class");
