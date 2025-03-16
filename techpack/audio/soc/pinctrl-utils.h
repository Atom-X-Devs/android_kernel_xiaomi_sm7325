/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * Utils functions to implement the pincontrol driver.
 *
 * Copyright (c) 2013, NVIDIA Corporation.
 *
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */
#ifndef __PINCTRL_UTILS_H__
#define __PINCTRL_UTILS_H__

int pinctrl_utils_reserve_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map **map, unsigned int *reserved_maps,
		unsigned int *num_maps, unsigned int reserve);
int pinctrl_utils_add_map_mux(struct pinctrl_dev *pctldev,
		struct pinctrl_map **map, unsigned int *reserved_maps,
		unsigned int *num_maps, const char *group,
		const char *function);
int pinctrl_utils_add_map_configs(struct pinctrl_dev *pctldev,
		struct pinctrl_map **map, unsigned int *reserved_maps,
		unsigned int *num_maps, const char *group,
		unsigned long *configs, unsigned int num_configs,
		enum pinctrl_map_type type);
int pinctrl_utils_add_config(struct pinctrl_dev *pctldev,
		unsigned long **configs, unsigned int *num_configs,
		unsigned long config);
void pinctrl_utils_free_map(struct pinctrl_dev *pctldev,
		struct pinctrl_map *map, unsigned int num_maps);

#endif /* __PINCTRL_UTILS_H__ */
