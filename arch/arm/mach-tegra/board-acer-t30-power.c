/*
 * arch/arm/mach-tegra/board-acer-t30-power.c
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/max77663-core.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/regulator/gpio-switch-regulator.h>
#include <linux/regulator/tps6591x-regulator.h>
#include <linux/regulator/tps6236x-regulator.h>
#include <linux/power/gpio-charger.h>

#include <asm/mach-types.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/edp.h>
#include <mach/tsensor.h>

#ifdef CONFIG_BATTERY_BQ27541
#include <linux/bq27541.h>
#endif

#include "gpio-names.h"
#include "board.h"
#include "board-acer-t30.h"
#include "pm.h"
#include "wakeups-t3.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

static struct regulator_consumer_supply tps6591x_vdd2_supply_0[] = {
	REGULATOR_SUPPLY("en_vddio_ddr_1v2", NULL),
	REGULATOR_SUPPLY("vdd_gen1v5", NULL),
	REGULATOR_SUPPLY("vcore_lcd", NULL),
	REGULATOR_SUPPLY("track_ldo1", NULL),
	REGULATOR_SUPPLY("external_ldo_1v2", NULL),
	REGULATOR_SUPPLY("vcore_cam1", NULL),
	REGULATOR_SUPPLY("vcore_cam2", NULL),
};

static struct regulator_consumer_supply tps6591x_vddctrl_supply_0[] = {
	REGULATOR_SUPPLY("vdd_cpu_pmu", NULL),
	REGULATOR_SUPPLY("vdd_cpu", NULL),
	REGULATOR_SUPPLY("vdd_sys", NULL),
};

static struct regulator_consumer_supply tps6591x_vio_supply_0[] = {
	REGULATOR_SUPPLY("vdd_gen1v8", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.3"),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vdd1v8_satelite", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_lcd_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_vi", NULL),
	REGULATOR_SUPPLY("pwrdet_vi", NULL),
	REGULATOR_SUPPLY("ldo6", NULL),
	REGULATOR_SUPPLY("ldo7", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
	REGULATOR_SUPPLY("vcore_audio", NULL),
	REGULATOR_SUPPLY("avcore_audio", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "sdhci-tegra.2"),
	REGULATOR_SUPPLY("pwrdet_sdmmc3", NULL),
	REGULATOR_SUPPLY("vcore1_lpddr2", NULL),
	REGULATOR_SUPPLY("vcom_1v8", NULL),
	REGULATOR_SUPPLY("pmuio_1v8", NULL),
	REGULATOR_SUPPLY("avdd_ic_usb", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo1_supply_0[] = {
	REGULATOR_SUPPLY("avdd_pexb", NULL),
	REGULATOR_SUPPLY("vdd_pexb", NULL),
	REGULATOR_SUPPLY("avdd_pex_pll", NULL),
	REGULATOR_SUPPLY("avdd_pexa", NULL),
	REGULATOR_SUPPLY("vdd_pexa", NULL),
	REGULATOR_SUPPLY("vmmc", "sdhci-tegra.3"),
};

static struct regulator_consumer_supply tps6591x_ldo2_supply_0[] = {
	REGULATOR_SUPPLY("avdd_sata", NULL),
	REGULATOR_SUPPLY("vdd_sata", NULL),
	REGULATOR_SUPPLY("avdd_sata_pll", NULL),
	REGULATOR_SUPPLY("avdd_plle", NULL),
	REGULATOR_SUPPLY("vddio_sd_slot", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo3_supply_0[] = {
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo4_supply_0[] = {
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo5_supply_0[] = {
	REGULATOR_SUPPLY("avdd_vdac", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo6_supply_0[] = {
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("pwrdet_mipi", NULL),
};
static struct regulator_consumer_supply tps6591x_ldo7_supply_0[] = {
	REGULATOR_SUPPLY("avdd_plla_p_c_s", NULL),
	REGULATOR_SUPPLY("avdd_pllm", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d", NULL),
	REGULATOR_SUPPLY("avdd_pllu_d2", NULL),
	REGULATOR_SUPPLY("avdd_pllx", NULL),
};

static struct regulator_consumer_supply tps6591x_ldo8_supply_0[] = {
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

#define TPS_PDATA_INIT(_name, _sname, _minmv, _maxmv, _supply_reg, _always_on, \
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply, _ectrl, _flags) \
	static struct tps6591x_regulator_platform_data pdata_##_name##_##_sname = \
	{								\
		.regulator = {						\
			.constraints = {				\
				.min_uV = (_minmv)*1000,		\
				.max_uV = (_maxmv)*1000,		\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
						     REGULATOR_MODE_STANDBY), \
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
						   REGULATOR_CHANGE_STATUS |  \
						   REGULATOR_CHANGE_VOLTAGE), \
				.always_on = _always_on,		\
				.boot_on = _boot_on,			\
				.apply_uV = _apply_uv,			\
			},						\
			.num_consumer_supplies =			\
				ARRAY_SIZE(tps6591x_##_name##_supply_##_sname),	\
			.consumer_supplies = tps6591x_##_name##_supply_##_sname,	\
			.supply_regulator = _supply_reg,		\
		},							\
		.init_uV =  _init_uV * 1000,				\
		.init_enable = _init_enable,				\
		.init_apply = _init_apply,				\
		.ectrl = _ectrl,					\
		.flags = _flags,					\
	}

TPS_PDATA_INIT(vdd2, 0,         600,  1500, 0, 1, 1, 0, -1, 0, 0, 0, 0);
TPS_PDATA_INIT(vddctrl, 0,      600,  1400, 0, 1, 1, 0, 1000, 0, 1, EXT_CTRL_EN1, 0);
TPS_PDATA_INIT(vio,  0,         1500, 3300, 0, 1, 1, 0, -1, 0, 0, 0, 0);

TPS_PDATA_INIT(ldo1, 0,         1000, 3300, 0, 1, 0, 0, 2850, 1, 1, 0, 0);
TPS_PDATA_INIT(ldo2, 0,         1000, 3300, 0, 0, 0, 1, 3300, 0, 1, 0, 0);

TPS_PDATA_INIT(ldo3, 0,         1000, 3300, 0, 0, 0, 1, 3300, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo4, 0,         1000, 3300, 0, 1, 0, 0, -1, 0, 0, 0, LDO_LOW_POWER_ON_SUSPEND);
TPS_PDATA_INIT(ldo5, 0,         1000, 3300, 0, 1, 0, 0, 2800, 0, 1, 0, 0);

TPS_PDATA_INIT(ldo6, 0,         1000, 3300, tps6591x_rails(VIO), 0, 0, 0, 1200, 0, 1, 0, 0);
TPS_PDATA_INIT(ldo7, 0,         1200, 1200, tps6591x_rails(VIO), 1, 1, 1, -1, 0, 0, 0, LDO_LOW_POWER_ON_SUSPEND);
TPS_PDATA_INIT(ldo8, 0,         1000, 3300, tps6591x_rails(VIO), 1, 0, 0, -1, 0, 0, 0, LDO_LOW_POWER_ON_SUSPEND);

#if defined(CONFIG_RTC_DRV_TPS6591x)
static struct tps6591x_rtc_platform_data rtc_data = {
	.irq = TEGRA_NR_IRQS + TPS6591X_INT_RTC_ALARM,
	.time = {
		.tm_year = 2000,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 0,
		.tm_min = 0,
		.tm_sec = 0,
	},
};

#define TPS_RTC_REG()					\
	{						\
		.id	= 0,				\
		.name	= "rtc_tps6591x",		\
		.platform_data = &rtc_data,		\
	}
#endif

#define TPS_REG(_id, _name, _sname)				\
	{							\
		.id	= TPS6591X_ID_##_id,			\
		.name	= "tps6591x-regulator",			\
		.platform_data	= &pdata_##_name##_##_sname,	\
	}

static struct tps6591x_subdev_info tps_devs_picasso2[] = {
	TPS_REG(VIO, vio, 0),
	TPS_REG(VDD_2, vdd2, 0),
	TPS_REG(VDDCTRL, vddctrl, 0),
	TPS_REG(LDO_1, ldo1, 0),
	TPS_REG(LDO_2, ldo2, 0),
	TPS_REG(LDO_3, ldo3, 0),
	TPS_REG(LDO_4, ldo4, 0),
	TPS_REG(LDO_5, ldo5, 0),
	TPS_REG(LDO_6, ldo6, 0),
	TPS_REG(LDO_7, ldo7, 0),
	TPS_REG(LDO_8, ldo8, 0),
#if defined(CONFIG_RTC_DRV_TPS6591x)
	TPS_RTC_REG(),
#endif
};

#define TPS_GPIO_INIT_PDATA(gpio_nr, _init_apply, _sleep_en, _pulldn_en, _output_en, _output_val)	\
	[gpio_nr] = {					\
			.sleep_en	= _sleep_en,	\
			.pulldn_en	= _pulldn_en,	\
			.output_mode_en	= _output_en,	\
			.output_val	= _output_val,	\
			.init_apply	= _init_apply,	\
		     }
static struct tps6591x_gpio_init_data tps_gpio_pdata_e1291_a04[] =  {
	TPS_GPIO_INIT_PDATA(0, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(1, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(2, 1, 1, 0, 1, 1),
	TPS_GPIO_INIT_PDATA(3, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(4, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(5, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(6, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(7, 0, 0, 0, 0, 0),
	TPS_GPIO_INIT_PDATA(8, 0, 0, 0, 0, 0),
};

static struct tps6591x_sleep_keepon_data tps_slp_keepon = {
	.clkout32k_keepon = 1,
};

static struct tps6591x_platform_data tps_platform = {
	.irq_base	= TPS6591X_IRQ_BASE,
	.gpio_base	= TPS6591X_GPIO_BASE,
	.dev_slp_en	= true,
	.slp_keepon	= &tps_slp_keepon,
};

static struct i2c_board_info __initdata cardhu_regulators[] = {
	{
		I2C_BOARD_INFO("tps6591x", 0x2D),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/* TPS62361B DC-DC converter */
static struct regulator_consumer_supply tps6236x_dcdc_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct tps6236x_regulator_platform_data tps6236x_pdata = {
	.reg_init_data = {					\
		.constraints = {				\
			.min_uV = 500000,			\
			.max_uV = 1770000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |  \
					     REGULATOR_MODE_STANDBY), \
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |    \
					   REGULATOR_CHANGE_STATUS |  \
					   REGULATOR_CHANGE_VOLTAGE), \
			.always_on = 1,				\
			.boot_on =  1,				\
			.apply_uV = 0,				\
		},						\
		.num_consumer_supplies = ARRAY_SIZE(tps6236x_dcdc_supply), \
		.consumer_supplies = tps6236x_dcdc_supply,		\
		},							\
	.internal_pd_enable = 0,					\
	.enable_discharge = true,					\
	.vsel = 3,							\
	.init_uV = 1200 * 1000,							\
	.init_apply = 1,						\
};

static struct i2c_board_info __initdata tps6236x_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps62361B", 0x60),
		.platform_data	= &tps6236x_pdata,
	},
};

#ifdef CONFIG_BATTERY_BQ27541
#define AC_DETECT_GPIO TEGRA_GPIO_PO4

static void bq27541_gpio_init(void)
{
	tegra_gpio_enable(AC_DETECT_GPIO);
	gpio_request(AC_DETECT_GPIO, "ac_present");
	gpio_direction_input(AC_DETECT_GPIO);
}

static struct bq27541_platform_data bq27541_pdata = {
	.ac_present_gpio = AC_DETECT_GPIO,
};

static struct i2c_board_info __initdata bq27541_boardinfo[] = {
	{
		I2C_BOARD_INFO("bq27541-battery", 0x55),
		.irq = TEGRA_GPIO_TO_IRQ(AC_DETECT_GPIO),
		.platform_data = &bq27541_pdata,
	},
};
#endif

int __init cardhu_regulator_init(void)
{
	struct board_info board_info;
	struct board_info pmu_board_info;
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	tegra_get_board_info(&board_info);
	tegra_get_pmu_board_info(&pmu_board_info);

	/* The regulator details have complete constraints */
	regulator_has_full_constraints();

	tps_platform.num_subdevs = ARRAY_SIZE(tps_devs_picasso2);
	tps_platform.subdevs = tps_devs_picasso2;

	i2c_register_board_info(4, cardhu_regulators, 1);

		pr_info("Registering the device TPS62361B\n");
		i2c_register_board_info(4, tps6236x_boardinfo, 1);

        tps_platform.dev_slp_en = true;
	tps_platform.gpio_init_data = tps_gpio_pdata_e1291_a04;
	tps_platform.num_gpioinit_data = ARRAY_SIZE(tps_gpio_pdata_e1291_a04);

#ifdef CONFIG_BATTERY_BQ27541
	bq27541_gpio_init();
	i2c_register_board_info(4, bq27541_boardinfo, 1);
#endif

	return 0;
}

/* EN_5V_CP from PMU GP0 */
static struct regulator_consumer_supply gpio_switch_en_5v_cp_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sby", NULL),
	REGULATOR_SUPPLY("vdd_hall", NULL),
	REGULATOR_SUPPLY("vterm_ddr", NULL),
	REGULATOR_SUPPLY("v2ref_ddr", NULL),
};
static int gpio_switch_en_5v_cp_voltages[] = { 5000};

/* EN_5V0 From PMU GP2 */
static struct regulator_consumer_supply gpio_switch_en_5v0_supply[] = {
	REGULATOR_SUPPLY("vdd_5v0_sys", NULL),
};
static int gpio_switch_en_5v0_voltages[] = { 5000};

/* EN_DDR From PMU GP6 */
static struct regulator_consumer_supply gpio_switch_en_ddr_supply[] = {
	REGULATOR_SUPPLY("mem_vddio_ddr", NULL),
	REGULATOR_SUPPLY("t30_vddio_ddr", NULL),
};
static int gpio_switch_en_ddr_voltages[] = { 1500};

/* EN_3V3_SYS From PMU GP7 */
static struct regulator_consumer_supply gpio_switch_en_3v3_sys_supply[] = {
	REGULATOR_SUPPLY("vdd_lvds", NULL),
	REGULATOR_SUPPLY("vdd_pnl", NULL),
	REGULATOR_SUPPLY("vcom_3v3", NULL),
	REGULATOR_SUPPLY("vdd_3v3", NULL),
	REGULATOR_SUPPLY("vcore_mmc", NULL),
	REGULATOR_SUPPLY("vddio_pex_ctl", NULL),
	REGULATOR_SUPPLY("pwrdet_pex_ctl", NULL),
	REGULATOR_SUPPLY("hvdd_pex_pmu", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
	REGULATOR_SUPPLY("vcore_nand", NULL),
	REGULATOR_SUPPLY("hvdd_sata", NULL),
	REGULATOR_SUPPLY("vddio_gmi_pmu", NULL),
	REGULATOR_SUPPLY("pwrdet_nand", NULL),
	REGULATOR_SUPPLY("avdd_cam1", NULL),
	REGULATOR_SUPPLY("vdd_af", NULL),
	REGULATOR_SUPPLY("avdd_cam2", NULL),
	REGULATOR_SUPPLY("vdd_acc", NULL),
	REGULATOR_SUPPLY("vdd_phtl", NULL),
	REGULATOR_SUPPLY("vddio_tp", NULL),
	REGULATOR_SUPPLY("vdd_led", NULL),
	REGULATOR_SUPPLY("vddio_cec", NULL),
	REGULATOR_SUPPLY("vdd_cmps", NULL),
	REGULATOR_SUPPLY("vdd_temp", NULL),
	REGULATOR_SUPPLY("vpp_kfuse", NULL),
	REGULATOR_SUPPLY("vddio_ts", NULL),
	REGULATOR_SUPPLY("vdd_ir_led", NULL),
	REGULATOR_SUPPLY("vddio_1wire", NULL),
	REGULATOR_SUPPLY("avddio_audio", NULL),
	REGULATOR_SUPPLY("vdd_ec", NULL),
	REGULATOR_SUPPLY("vcom_pa", NULL),
	REGULATOR_SUPPLY("vdd_3v3_devices", NULL),
	REGULATOR_SUPPLY("vdd_3v3_dock", NULL),
	REGULATOR_SUPPLY("vdd_3v3_edid", NULL),
	REGULATOR_SUPPLY("vdd_3v3_hdmi_cec", NULL),
	REGULATOR_SUPPLY("vdd_3v3_gmi", NULL),
	REGULATOR_SUPPLY("vdd_spk_amp", "tegra-snd-wm8903"),
	REGULATOR_SUPPLY("vdd_3v3_sensor", NULL),
	REGULATOR_SUPPLY("vdd_3v3_cam", NULL),
	REGULATOR_SUPPLY("vdd_3v3_als", NULL),
	REGULATOR_SUPPLY("debug_cons", NULL),
        REGULATOR_SUPPLY("vdd_nct1008", NULL),
	REGULATOR_SUPPLY("vdd", "4-004c"),
};
static int gpio_switch_en_3v3_sys_voltages[] = { 3300};

/* EN_VDD_PNL1 from AP GPIO VI_D6 L04*/
static struct regulator_consumer_supply gpio_switch_en_vdd_pnl1_supply[] = {
	REGULATOR_SUPPLY("vdd_lcd_panel", NULL),
};
static int gpio_switch_en_vdd_pnl1_voltages[] = { 3300};

/* EN_VDD_BL */
static struct regulator_consumer_supply gpio_switch_en_vdd_bl_supply[] = {
	REGULATOR_SUPPLY("vdd_backlight", NULL),
	REGULATOR_SUPPLY("vdd_backlight1", NULL),
};
static int gpio_switch_en_vdd_bl_voltages[] = { 5000};

/* EN_VDD_GPS */
static struct regulator_consumer_supply gpio_switch_en_vdd_gps_supply[] = {
	REGULATOR_SUPPLY("vdd_gps", NULL),
};
static int gpio_switch_en_vdd_gps_voltages[] = { 3000};

/* EN_3V3_MODEM */
static struct regulator_consumer_supply gpio_switch_en_3v3_modem_supply[] = {
	REGULATOR_SUPPLY("vdd_3v3_mini_card", NULL),
	REGULATOR_SUPPLY("vdd_mini_card", NULL),
};
static int gpio_switch_en_3v3_modem_voltages[] = { 3300};

/* EN_SENSOR_1V8 */
static struct regulator_consumer_supply gpio_switch_en_sensor_1v8_supply[] = {
	REGULATOR_SUPPLY("sensor_1v8", NULL),
};
static int gpio_switch_en_sensor_1v8_voltages[] = { 1800};

/* EN_VDD_WIFI */
static struct regulator_consumer_supply gpio_switch_en_wifi_vdd_supply[] = {
	REGULATOR_SUPPLY("wifi_vdd", NULL),
};
static int gpio_switch_en_wifi_vdd_voltages[] = { 3300};

/* EN_SENSOR_3V3 */
static struct regulator_consumer_supply gpio_switch_en_sensor_3v3_supply[] = {
	REGULATOR_SUPPLY("sensor_3v3", NULL),
};
static int gpio_switch_en_sensor_3v3_voltages[] = { 3300};

/* EN_3v3_FUSE */
static struct regulator_consumer_supply gpio_switch_en_3v3_fuse_supply[] = {
	REGULATOR_SUPPLY("vdd_fuse", NULL),
};
static int gpio_switch_en_3v3_fuse_voltages[] = { 3300};

/* EN_HDMI_5V0 */
static struct regulator_consumer_supply gpio_switch_en_hdmi_5v0_supply[] = {
	REGULATOR_SUPPLY("hdmi_5v0", NULL),
};
static int gpio_switch_en_hdmi_5v0_voltages[] = { 5000};

static struct regulator_consumer_supply gpio_switch_en_cam_1v8_supply[] = {
	REGULATOR_SUPPLY("cam_1v8", NULL),
};
static int gpio_switch_en_cam_1v8_voltages[] = { 1800};

static struct regulator_consumer_supply gpio_switch_en_cam_2v8_supply[] = {
	REGULATOR_SUPPLY("cam_2v8", NULL),
};
static int gpio_switch_en_cam_2v8_voltages[] = { 2800};

/* EN_USB1_VBUS_OC*/
static struct regulator_consumer_supply gpio_switch_en_usb1_vbus_oc_supply[] = {
	REGULATOR_SUPPLY("vdd_vbus_micro_usb", NULL),
};
static int gpio_switch_en_usb1_vbus_oc_voltages[] = { 5000};

static int enable_load_switch_rail(
		struct gpio_switch_regulator_subdev_data *psubdev_data)
{
	int ret;

	if (psubdev_data->pin_group <= 0)
		return -EINVAL;

	/* Tristate and make pin as input*/
	ret = tegra_pinmux_set_tristate(psubdev_data->pin_group,
						TEGRA_TRI_TRISTATE);
	if (ret < 0)
		return ret;
	return gpio_direction_input(psubdev_data->gpio_nr);
}

static int disable_load_switch_rail(
		struct gpio_switch_regulator_subdev_data *psubdev_data)
{
	int ret;

	if (psubdev_data->pin_group <= 0)
		return -EINVAL;

	/* Un-tristate and driver low */
	ret = tegra_pinmux_set_tristate(psubdev_data->pin_group,
						TEGRA_TRI_NORMAL);
	if (ret < 0)
		return ret;
	return gpio_direction_output(psubdev_data->gpio_nr, 0);
}


/* Macro for defining gpio switch regulator sub device data */
#define GREG_INIT(_id, _var, _name, _input_supply, _always_on, _boot_on, \
	_gpio_nr, _active_low, _init_state, _pg, _enable, _disable)	 \
	static struct gpio_switch_regulator_subdev_data gpio_pdata_##_var =  \
	{								\
		.regulator_name	= "gpio-switch-"#_name,			\
		.input_supply	= _input_supply,			\
		.id		= _id,					\
		.gpio_nr	= _gpio_nr,				\
		.pin_group	= _pg,					\
		.active_low	= _active_low,				\
		.init_state	= _init_state,				\
		.voltages	= gpio_switch_##_name##_voltages,	\
		.n_voltages	= ARRAY_SIZE(gpio_switch_##_name##_voltages), \
		.num_consumer_supplies =				\
				ARRAY_SIZE(gpio_switch_##_name##_supply), \
		.consumer_supplies = gpio_switch_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					     REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					   REGULATOR_CHANGE_STATUS |	\
					   REGULATOR_CHANGE_VOLTAGE),	\
			.always_on = _always_on,			\
			.boot_on = _boot_on,				\
		},							\
		.enable_rail = _enable,					\
		.disable_rail = _disable,				\
	}

GREG_INIT(0, en_5v_cp,		en_5v_cp,	NULL,			1,	0,	TPS6591X_GPIO_0,	false,	1,	0,	0,	0);
GREG_INIT(1, en_5v0,		en_5v0,		NULL,			0,      0,      TPS6591X_GPIO_8,	false,	1,	0,	0,	0);
GREG_INIT(2, en_ddr,		en_ddr,		NULL,			0,      0,      TPS6591X_GPIO_7,	false,	1,	0,	0,	0);
GREG_INIT(3, en_3v3_sys,	en_3v3_sys,	NULL,			0,      0,      TPS6591X_GPIO_6,	false,	1,	0,	0,	0);
GREG_INIT(4, en_vdd_gps,	en_vdd_gps,	NULL,			0,      0,      TEGRA_GPIO_PY2,		false,	0,	0,	0,	0);
GREG_INIT(5, en_3v3_modem,	en_3v3_modem,	NULL,			1,      0,      TEGRA_GPIO_PR7,		false,	1,	0,	0,	0);
GREG_INIT(6, en_sensor_1v8,	en_sensor_1v8,	NULL,	0,      0,      TEGRA_GPIO_PY3,		false,	1,	0,	0,	0);
GREG_INIT(7, en_vdd_pnl1,	en_vdd_pnl1,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PB1,		false,	1,	0,	0,	0);
GREG_INIT(8, en_wifi_vdd,	en_wifi_vdd, NULL,	0,      0,      TEGRA_GPIO_PK7,		false,	0,	0,	0,	0);
GREG_INIT(9, en_sensor_3v3,	en_sensor_3v3,	"vdd_3v3_devices",	1,      0,      TEGRA_GPIO_PH7,		false,	1,	0,	0,	0);
GREG_INIT(10, en_3v3_fuse,	en_3v3_fuse,	"vdd_3v3_devices",	0,      0,      TEGRA_GPIO_PH2,		false,	0,	0,	0,	0);
GREG_INIT(11, en_hdmi_5v0,	en_hdmi_5v0, NULL,		0,      0,      TEGRA_GPIO_PI4,	false,	0,	0,	0,	0);
GREG_INIT(12, en_cam_1v8,	en_cam_1v8,	NULL,	0,      0,      TEGRA_GPIO_PQ3,		false,	1,	0,	0,	0);
GREG_INIT(13, en_cam_2v8,	en_cam_2v8,	NULL,	0,      0,      TEGRA_GPIO_PR2,		false,	1,	0,	0,	0);
GREG_INIT(14, en_vdd_bl,	en_vdd_bl,	"vdd_5v0_sys",			0,      0,      TEGRA_GPIO_PH1,		false,	1,	0,	0,	0);
GREG_INIT(15, en_usb1_vbus_oc,		en_usb1_vbus_oc,	"vdd_5v0_sys",
		0,      0,      TEGRA_GPIO_PN1,		false,	0,	TEGRA_PINGROUP_PEX_L1_CLKREQ_N,
		enable_load_switch_rail, disable_load_switch_rail);

#define ADD_GPIO_REG(_name) &gpio_pdata_##_name

#define ACER_T30_GPIO_REG	\
	ADD_GPIO_REG(en_5v_cp),			\
	ADD_GPIO_REG(en_5v0),			\
	ADD_GPIO_REG(en_ddr),			\
	ADD_GPIO_REG(en_3v3_sys),		\
	ADD_GPIO_REG(en_vdd_pnl1),		\
	ADD_GPIO_REG(en_vdd_bl),

/* Gpio switch regulator platform data  for ACER T30*/
static struct gpio_switch_regulator_subdev_data *gswitch_subdevs_acer_t30[] = {
	ACER_T30_GPIO_REG
};

static struct gpio_switch_regulator_platform_data  gswitch_pdata;
static struct platform_device gswitch_regulator_pdata = {
	.name = "gpio-switch-regulator",
	.id   = -1,
	.dev  = {
	     .platform_data = &gswitch_pdata,
	},
};

int __init cardhu_gpio_switch_regulator_init(void)
{
	int i;

	gswitch_pdata.num_subdevs = ARRAY_SIZE(gswitch_subdevs_acer_t30);
	gswitch_pdata.subdevs = gswitch_subdevs_acer_t30;

	for (i = 0; i < gswitch_pdata.num_subdevs; ++i) {
		struct gpio_switch_regulator_subdev_data *gswitch_data = gswitch_pdata.subdevs[i];
		if (gswitch_data->gpio_nr < TEGRA_NR_GPIOS)
			tegra_gpio_enable(gswitch_data->gpio_nr);
	}

	return platform_device_register(&gswitch_regulator_pdata);
}

static void cardhu_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void cardhu_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data cardhu_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.cpu_lp2_min_residency = 2000,
	.board_suspend = cardhu_board_suspend,
	.board_resume = cardhu_board_resume,
};

int __init cardhu_suspend_init(void)
{
	struct board_info board_info;
	struct board_info pmu_board_info;

	tegra_get_board_info(&board_info);
	tegra_get_pmu_board_info(&pmu_board_info);

	/* For PMU Fab A03, A04 and A05 make core_pwr_req to high */
	if ((pmu_board_info.fab == BOARD_FAB_A03) ||
		(pmu_board_info.fab == BOARD_FAB_A04) ||
		 (pmu_board_info.fab == BOARD_FAB_A05))
		cardhu_suspend_data.corereq_high = true;

	/* CORE_PWR_REQ to be high for all processor/pmu board whose sku bit 0
	 * is set. This is require to enable the dc-dc converter tps62361x */
	if ((board_info.sku & SKU_DCDC_TPS62361_SUPPORT) || (pmu_board_info.sku & SKU_DCDC_TPS62361_SUPPORT))
		cardhu_suspend_data.corereq_high = true;

	switch (board_info.board_id) {
	case BOARD_E1291:
		/* CORE_PWR_REQ to be high for E1291-A03 */
		if (board_info.fab == BOARD_FAB_A03)
			cardhu_suspend_data.corereq_high = true;
		break;
	case BOARD_E1198:
	case BOARD_PM305:
	case BOARD_PM311:
		break;
	case BOARD_E1187:
	case BOARD_E1186:
	case BOARD_E1256:
	case BOARD_E1257:
		cardhu_suspend_data.cpu_timer = 5000;
		cardhu_suspend_data.cpu_off_timer = 5000;
		break;
	default:
		break;
	}

	tegra_init_suspend(&cardhu_suspend_data);
	return 0;
}

static void cardhu_power_off(void)
{
	int ret;
	pr_err("cardhu: Powering off the device\n");
	ret = tps6591x_power_off();
	if (ret)
		pr_err("cardhu: failed to power off\n");

	while (1);
}

int __init cardhu_power_off_init(void)
{
	struct board_info pmu_board_info;

	tegra_get_pmu_board_info(&pmu_board_info);

	pm_power_off = cardhu_power_off;

	return 0;
}

static struct tegra_tsensor_pmu_data  tpdata = {
	.poweroff_reg_addr = 0x3F,
	.poweroff_reg_data = 0x80,
	.reset_tegra = 1,
	.controller_type = 0,
	.i2c_controller_id = 4,
	.pinmux = 0,
	.pmu_16bit_ops = 0,
	.pmu_i2c_addr = 0x2D,
};

void __init cardhu_tsensor_init(void)
{
	tegra3_tsensor_init(&tpdata);
}

#ifdef CONFIG_TEGRA_EDP_LIMITS

int __init cardhu_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 6000; /* regular T30/s */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	return 0;
}
#endif

static char *cardhu_battery[] = {
	"bq27510-0",
};

static struct gpio_charger_platform_data cardhu_charger_pdata = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.gpio = AC_PRESENT_GPIO,
	.gpio_active_low = 0,
	.supplied_to = cardhu_battery,
	.num_supplicants = ARRAY_SIZE(cardhu_battery),
};

static struct platform_device cardhu_charger_device = {
	.name = "gpio-charger",
	.dev = {
		.platform_data = &cardhu_charger_pdata,
	},
};

static int __init cardhu_charger_late_init(void)
{
	if (!machine_is_cardhu())
		return 0;

	platform_device_register(&cardhu_charger_device);
	return 0;
}

late_initcall(cardhu_charger_late_init);

