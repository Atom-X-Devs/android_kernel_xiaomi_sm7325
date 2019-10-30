/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/include/linux/cpu_cooling.h
 *
 *  Copyright (C) 2012	Samsung Electronics Co., Ltd(http://www.samsung.com)
 *  Copyright (C) 2012  Amit Daniel <amit.kachhap@linaro.org>
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef __CPU_COOLING_H__
#define __CPU_COOLING_H__

#include <linux/of.h>
#include <linux/thermal.h>
#include <linux/cpumask.h>

struct cpufreq_policy;

#ifdef CONFIG_MACH_XIAOMI
void cpu_limits_set_level(unsigned int cpu, unsigned int max_freq);
#endif

#ifdef CONFIG_CPU_THERMAL
/**
 * cpufreq_cooling_register - function to create cpufreq cooling device.
 * @policy: cpufreq policy.
 */
struct thermal_cooling_device *
cpufreq_cooling_register(struct cpufreq_policy *policy);

/**
 * cpufreq_cooling_unregister - function to remove cpufreq cooling device.
 * @cdev: thermal cooling device pointer.
 */
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev);

/**
 * of_cpufreq_cooling_register - create cpufreq cooling device based on DT.
 * @policy: cpufreq policy.
 */
struct thermal_cooling_device *
of_cpufreq_cooling_register(struct cpufreq_policy *policy);

#else /* !CONFIG_CPU_THERMAL */
static inline struct thermal_cooling_device *
cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	return ERR_PTR(-ENOSYS);
}

static inline
void cpufreq_cooling_unregister(struct thermal_cooling_device *cdev)
{
	return;
}

static inline struct thermal_cooling_device *
of_cpufreq_cooling_register(struct cpufreq_policy *policy)
{
	return NULL;
}
#endif /* CONFIG_CPU_THERMAL */

#ifdef CONFIG_QTI_CPU_ISOLATE_COOLING_DEVICE
extern void cpu_cooling_max_level_notifier_register(struct notifier_block *n);
extern void cpu_cooling_max_level_notifier_unregister(struct notifier_block *n);
extern const struct cpumask *cpu_cooling_get_max_level_cpumask(void);
#else
static inline
void cpu_cooling_max_level_notifier_register(struct notifier_block *n)
{
}

static inline
void cpu_cooling_max_level_notifier_unregister(struct notifier_block *n)
{
}

static inline const struct cpumask *cpu_cooling_get_max_level_cpumask(void)
{
	return cpu_none_mask;
}
#endif /* CONFIG_QTI_CPU_ISOLATE_COOLING_DEVICE */
#endif /* __CPU_COOLING_H__ */
