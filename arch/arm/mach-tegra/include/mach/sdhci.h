/*
 * include/asm-arm/arch-tegra/include/mach/sdhci.h
 *
 * Copyright (C) 2009 Palm, Inc.
 * Author: Yvonne Yip <y@palm.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __ASM_ARM_ARCH_TEGRA_SDHCI_H
#define __ASM_ARM_ARCH_TEGRA_SDHCI_H

#include <linux/mmc/host.h>
#include <asm/mach/mmc.h>

struct tegra_sdhci_platform_data {
	int cd_gpio;
	int wp_gpio;
	int power_gpio;
	int is_8bit;
	unsigned int tap_delay;
#if defined(CONFIG_ARCH_ACER_T20) || defined(CONFIG_ARCH_ACER_T30)
	int cd_gpio_polarity;
#endif
#if defined(CONFIG_ARCH_ACER_T30)
	bool is_voltage_switch_supported;
	const char *vdd_rail_name;
	const char *slot_rail_name;
	int vdd_max_uv;
	int vdd_min_uv;
#endif
	unsigned int max_clk_limit;
	struct mmc_platform_data mmc_data;
};

#endif
