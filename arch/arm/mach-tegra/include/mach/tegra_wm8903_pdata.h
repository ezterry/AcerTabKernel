/*
 * arch/arm/mach-tegra/include/mach/tegra_wm8903_pdata.h
 *
 * Copyright 2011 NVIDIA, Inc.
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

struct tegra_wm8903_platform_data {
	int gpio_spkr_en;
	int gpio_hp_det;
	int gpio_hp_mute;
	int gpio_int_mic_en;
	int gpio_ext_mic_en;
#if defined(CONFIG_ARCH_ACER_T20) || defined(CONFIG_ARCH_ACER_T30)
	int gpio_debug_switch_en;
#if defined(CONFIG_ARCH_ACER_T30)
	int gpio_bypass_switch_en;
#endif
#if defined(CONFIG_ARCH_ACER_T20)
	int gpio_spkr_mute;
#endif
#endif
};
