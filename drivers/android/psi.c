// SPDX-License-Identifier: GPL-2.0

#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/poll.h>

static bool psi_enable = true;

static int __init setup_psi(char *str)
{
	return kstrtobool(str, &psi_enable) == 0;
}
__setup("psi=", setup_psi);

static int psi_io_show(struct seq_file *m, void *v)
{
	return 0;
}

static int psi_memory_show(struct seq_file *m, void *v)
{
	return 0;
}

static int psi_cpu_show(struct seq_file *m, void *v)
{
	return 0;
}

static int psi_io_open(struct inode *inode, struct file *file)
{
	return single_open(file, psi_io_show, NULL);
}

static int psi_memory_open(struct inode *inode, struct file *file)
{
	return single_open(file, psi_memory_show, NULL);
}

static int psi_cpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, psi_cpu_show, NULL);
}

static ssize_t psi_io_write(struct file *file, const char __user *user_buf,
			    size_t nbytes, loff_t *ppos)
{
	return 0;
}

static ssize_t psi_memory_write(struct file *file, const char __user *user_buf,
				size_t nbytes, loff_t *ppos)
{
	return 0;
}

static ssize_t psi_cpu_write(struct file *file, const char __user *user_buf,
			     size_t nbytes, loff_t *ppos)
{
	return 0;
}

static __poll_t psi_fop_poll(struct file *file, poll_table *wait)
{
	return 0;
}

static int psi_fop_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct proc_ops psi_io_proc_ops = {
	.proc_open	= psi_io_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= psi_io_write,
	.proc_poll	= psi_fop_poll,
	.proc_release	= psi_fop_release,
};

static const struct proc_ops psi_memory_proc_ops = {
	.proc_open	= psi_memory_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= psi_memory_write,
	.proc_poll	= psi_fop_poll,
	.proc_release	= psi_fop_release,
};

static const struct proc_ops psi_cpu_proc_ops = {
	.proc_open	= psi_cpu_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_write	= psi_cpu_write,
	.proc_poll	= psi_fop_poll,
	.proc_release	= psi_fop_release,
};

static int __init psi_proc_init(void)
{
	proc_mkdir("pressure", NULL);
	proc_create("pressure/io", 0, NULL, &psi_io_proc_ops);
	proc_create("pressure/memory", 0, NULL, &psi_memory_proc_ops);
	proc_create("pressure/cpu", 0, NULL, &psi_cpu_proc_ops);

	return 0;
}
module_init(psi_proc_init);
