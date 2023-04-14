// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2014, 2017-2018, 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <sound/audio_slimslave.h>
#include <linux/slimbus/slimbus.h>
#include <linux/pm_runtime.h>

static struct slim_device *slim;
static int vote_count;

static int audio_slim_open(struct inode *inode, struct file *file)
{
	pr_debug("%s:\n", __func__);

	if (vote_count) {
		pr_debug("%s: unvote: vote_count=%d\n", __func__, vote_count);
		pm_runtime_mark_last_busy(slim->dev.parent);
		pm_runtime_put(slim->dev.parent);
		vote_count--;
	}
	return 0;
};

static int audio_slim_release(struct inode *inode, struct file *file)
{
	pr_debug("%s:\n", __func__);

	if (vote_count) {
		pr_debug("%s: unvote: vote_count=%d\n", __func__, vote_count);
		pm_runtime_mark_last_busy(slim->dev.parent);
		pm_runtime_put(slim->dev.parent);
		vote_count--;
	} else {
		pr_debug("%s: vote: vote_count=%d\n", __func__, vote_count);
		pm_runtime_get_sync(slim->dev.parent);
		vote_count++;
	}
	return 0;
};

static long audio_slim_ioctl(struct file *file, unsigned int cmd,
			     unsigned long u_arg)
{
	switch (cmd) {
	case AUDIO_SLIMSLAVE_VOTE:
		if (!vote_count) {
			pr_debug("%s:AUDIO_SLIMSLAVE_VOTE\n", __func__);
			pm_runtime_get_sync(slim->dev.parent);
			vote_count++;
		} else {
			pr_err("%s:Invalid vote: vote_count=%d\n",
				 __func__, vote_count);
		}
		break;
	case AUDIO_SLIMSLAVE_UNVOTE:
		if (vote_count) {
			pr_debug("%s:AUDIO_SLIMSLAVE_UNVOTE\n", __func__);
			pm_runtime_mark_last_busy(slim->dev.parent);
			pm_runtime_put(slim->dev.parent);
			vote_count--;
		} else {
			pr_err("%s:Invalid unvote: vote_count=%d\n",
				 __func__, vote_count);
		}
		break;
	default:
		pr_debug("%s: Invalid ioctl cmd: %d\n", __func__, cmd);
		break;
	}
	return 0;
}

static const struct file_operations audio_slimslave_fops = {
	.open =                 audio_slim_open,
	.unlocked_ioctl =       audio_slim_ioctl,
	.release =              audio_slim_release,
};

struct miscdevice audio_slimslave_misc = {
	.minor  =       MISC_DYNAMIC_MINOR,
	.name   =       AUDIO_SLIMSLAVE_IOCTL_NAME,
	.fops   =       &audio_slimslave_fops,
};

static int audio_slimslave_probe(struct slim_device *audio_slim)
{
	pr_debug("%s:\n", __func__);

	slim = audio_slim;
	misc_register(&audio_slimslave_misc);
	return 0;
}

static void audio_slimslave_remove(struct slim_device *audio_slim)
{
	pr_debug("%s:\n", __func__);
	misc_deregister(&audio_slimslave_misc);
}

static const struct slim_device_id audio_slimslave_dt_match[] = {
	{}
};

static struct slim_driver audio_slimslave_driver = {
	.driver = {
		.name = "audio-slimslave",
		.owner = THIS_MODULE,
	},
	.probe = audio_slimslave_probe,
	.remove = audio_slimslave_remove,
	.id_table = audio_slimslave_dt_match,
};

int __init audio_slimslave_init(void)
{
	return slim_driver_register(&audio_slimslave_driver);
}

void audio_slimslave_exit(void)
{
	slim_driver_unregister(&audio_slimslave_driver);
}

/* Module information */
MODULE_DESCRIPTION("Audio side Slimbus slave driver");
MODULE_LICENSE("GPL v2");
