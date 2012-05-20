/*
 * arch/arm/mach-tegra/board-acer-t30.c
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>

#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8903_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/thermal.h>
#ifdef CONFIG_ROTATELOCK
#include <linux/switch.h>
#endif
#ifdef CONFIG_EEPROM_AT24C02C
#include <linux/i2c/at24.h>
#endif

#include "board.h"
#include "clock.h"
#include "board-acer-t30.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "wdt-recovery.h"

#if defined(CONFIG_DOCK_V2)
#include <linux/switch.h>
#endif

#if defined(CONFIG_ACER_VIBRATOR)
#include <../../../drivers/staging/android/timed_output.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#endif

#if defined(CONFIG_ACER_LEDS)
#include <linux/leds-gpio-p2.h>
#endif


#if defined(CONFIG_ACER_ES305)
#include "../../../sound/soc/tegra/acer_a1026.h"
#endif

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
static void bt_shutdown_pin_init(void);
#endif
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)  // GPIO init
void gpio_unused_init(void);
#endif

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
	.temp_throttle = 85000,
	.temp_shutdown = 90000,
	.temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
	.edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
	.hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
	.tc1 = 0,
	.tc2 = 1,
	.passive_delay = 2000,
#else
	.hysteresis_throttle = 1000,
#endif
};

extern int acer_board_type;
extern int acer_board_id;
extern int acer_sku;
extern int acer_wifi_module;

/* !!!TODO: Change for cardhu (Taken from Ventana) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 4,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};


#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)

static int bt_uart_gpio[] = {
	TEGRA_GPIO_PW7,
	TEGRA_GPIO_PW6,
	TEGRA_GPIO_PA1,
	TEGRA_GPIO_PC0,
};

static int disable_bt_uart_func(void)
{
	unsigned int rc = 0;
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(bt_uart_gpio); i++) {
		rc = gpio_request(bt_uart_gpio[i], NULL);
		if (rc) {
			printk(KERN_INFO "%s, request gpio %d failed !!!\n", __func__, bt_uart_gpio[i]);
			return rc;
		}

		tegra_gpio_enable(bt_uart_gpio[i]);

		rc = gpio_direction_output(bt_uart_gpio[i], 0);
		if (rc) {
			printk(KERN_INFO "%s, direction gpio %d failed !!!\n", __func__, bt_uart_gpio[i]);
			return rc;
		}
	}
	return 0;
}

static int enable_bt_uart_func(void)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(bt_uart_gpio); i++) {
		tegra_gpio_disable(bt_uart_gpio[i]);
		gpio_free(bt_uart_gpio[i]);
	}
	return 0;
}

static void bt_ext_gpio_init(void)
{
	int ret;

	pr_info("%s: \n", __func__);

	tegra_gpio_enable(TEGRA_GPIO_PP0);

	ret = gpio_request(TEGRA_GPIO_PP0, "bt_ext_wake");
	if (ret)
		pr_warn("%s : can't find bt_ext_wake gpio.\n", __func__);

	/* configure ext_wake as output mode*/
	ret = gpio_direction_output(TEGRA_GPIO_PP0, 0);
	if (ret < 0) {
		pr_warn("gpio-keys: failed to configure output"
			" direction for GPIO %d, error %d\n",
			  TEGRA_GPIO_PP0, ret);
		gpio_free(TEGRA_GPIO_PP0);
	}
	gpio_set_value(TEGRA_GPIO_PP0, 0);
	gpio_free(TEGRA_GPIO_PP0);
}
#endif

#ifdef CONFIG_BCM4329_RFKILL

static struct resource cardhu_bcm4329_rfkill_resources[] = {
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU6,
		.end    = TEGRA_GPIO_PU6,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_nreset_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_vdd_gpio",
		.start  = TEGRA_GPIO_PK7,
		.end    = TEGRA_GPIO_PK7,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_wifi_reset_gpio",
		.start  = TEGRA_GPIO_PP2,
		.end    = TEGRA_GPIO_PP2,
		.flags  = IORESOURCE_IO,
	},
#else
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
#endif
};

static struct platform_device cardhu_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bcm4329_rfkill_resources),
	.resource       = cardhu_bcm4329_rfkill_resources,
};

static noinline void __init cardhu_bcm4329_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", cardhu_bcm4329_rfkill_device.name, \
				"blink", NULL);
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
	disable_bt_uart_func();
	bt_shutdown_pin_init();
#endif

	return;
}
#else
static inline void cardhu_bcm4329_bt_rfkill(void) { }
#endif

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
static void bt_shutdown_pin_init(void) {
        if(acer_board_type == BOARD_PICASSO_2) {
            if(acer_board_id == BOARD_DVT1) {
                cardhu_bcm4329_rfkill_device.resource[0].start = TEGRA_GPIO_PS3;
                cardhu_bcm4329_rfkill_device.resource[0].end   = TEGRA_GPIO_PS3;
                pr_info("bt_shutdown_pin_init: TEGRA_GPIO_PS3\n");
            }
        }
}
#endif


static struct resource cardhu_bluesleep_resources[] = {
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PS7,
			.end    = TEGRA_GPIO_PS7,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PP0,
			.end    = TEGRA_GPIO_PP0,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
#else
	[0] = {
		.name = "gpio_host_wake",
			.start  = TEGRA_GPIO_PU6,
			.end    = TEGRA_GPIO_PU6,
			.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "gpio_ext_wake",
			.start  = TEGRA_GPIO_PU1,
			.end    = TEGRA_GPIO_PU1,
			.flags  = IORESOURCE_IO,
	},
	[2] = {
		.name = "host_wake",
			.start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
			.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
#endif
};

static struct platform_device cardhu_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(cardhu_bluesleep_resources),
	.resource       = cardhu_bluesleep_resources,
};

static noinline void __init cardhu_setup_bluesleep(void)
{
	platform_device_register(&cardhu_bluesleep_device);

#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)
	bt_ext_gpio_init();
	tegra_gpio_enable(TEGRA_GPIO_PS7);
	tegra_gpio_enable(TEGRA_GPIO_PP0);
#else
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
#endif
	return;
}

static __initdata struct tegra_clk_init_table cardhu_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	0,		false},
	{ "dam0",	"pll_a_out0",	0,		false},
	{ "dam1",	"pll_a_out0",	0,		false},
	{ "dam2",	"pll_a_out0",	0,		false},
	{ "audio1",     "i2s1_sync",    0,              false},
	{ "audio3",     "i2s3_sync",    0,              false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

#ifdef CONFIG_EEPROM_AT24C02C
static struct at24_platform_data at24c02c = {
	.byte_len = SZ_2K/8,
	.page_size = 8,
};
static struct i2c_board_info __initdata cardhu_i2c_eeprom_board_info[] = {
	{
		I2C_BOARD_INFO("at24",0x50),
		.platform_data = &at24c02c,
	},
};
#endif

static struct tegra_i2c_platform_data cardhu_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data cardhu_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

static struct wm8903_platform_data cardhu_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = CARDHU_GPIO_WM8903(0),
	.gpio_cfg = {
		0,
		0,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &cardhu_wm8903_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

#if defined(CONFIG_ACER_ES305)
static struct a1026_platform_data a1026_pdata = {
	.gpio_a1026_clk = TEGRA_GPIO_PX0,
	.gpio_a1026_reset = TEGRA_GPIO_PN0,
	.gpio_a1026_wakeup = TEGRA_GPIO_PY0,
};

static struct i2c_board_info __initdata a1026_board_info = {
	I2C_BOARD_INFO("audience_a1026", 0x3e),
	.platform_data = &a1026_pdata,
};

static void a1026_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PX0);
	tegra_gpio_enable(TEGRA_GPIO_PN0);

	i2c_register_board_info(4, &a1026_board_info, 1);
}
#endif

static void cardhu_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &cardhu_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &cardhu_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &cardhu_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &cardhu_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &cardhu_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(4, &wm8903_board_info, 1);
#ifdef CONFIG_EEPROM_AT24C02C
	i2c_register_board_info(4, cardhu_i2c_eeprom_board_info, 1);
#endif
}

static struct platform_device *cardhu_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data cardhu_uart_pdata;

static void __init uart_debug_init(void)
{
	struct board_info board_info;
	int debug_port_id;

	tegra_get_board_info(&board_info);

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
			/* UARTB is debug port
			 *       for SLT - E1186/E1187/PM269
			 *       for E1256/E1257
			 */
		if (((board_info.sku & SKU_SLT_ULPI_SUPPORT) &&
			((board_info.board_id == BOARD_E1186) ||
			(board_info.board_id == BOARD_E1187) ||
			(board_info.board_id == BOARD_PM269))) ||
			(board_info.board_id == BOARD_E1256) ||
			(board_info.board_id == BOARD_E1257))
				debug_port_id = 1;
	}
	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		cardhu_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		cardhu_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		cardhu_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		cardhu_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA", debug_port_id);
		cardhu_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
	return;
}

static void __init cardhu_uart_init(void)
{
	struct clk *c;
	int i;
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PU5);
	ret = gpio_request(TEGRA_GPIO_PU5, "dock_int_gpio");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %d\n",__func__, TEGRA_GPIO_PU5);
	else
		gpio_direction_input(TEGRA_GPIO_PU5);

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_uart_pdata.parent_clk_list = uart_parent_clk;
	cardhu_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uarta_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartb_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartc_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uartd_device.dev.platform_data = &cardhu_uart_pdata;
	tegra_uarte_device.dev.platform_data = &cardhu_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(cardhu_uart_devices,
				ARRAY_SIZE(cardhu_uart_devices));
}

#if defined(CONFIG_ACER_VIBRATOR)
static struct timed_gpio vib_timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = TEGRA_GPIO_PJ7,
		.max_timeout = 10000,
		.active_low = 0,
	},
};

static struct timed_gpio_platform_data vib_timed_gpio_platform_data = {
	.num_gpios      = ARRAY_SIZE(vib_timed_gpios),
	.gpios          = vib_timed_gpios,
};

static struct platform_device vib_timed_gpio_device = {
	.name   = TIMED_GPIO_NAME,
	.id     = 0,
	.dev    = {
		.platform_data  = &vib_timed_gpio_platform_data,
	},
};

static void vib_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PJ7);
}
#endif

#ifdef CONFIG_ROTATELOCK
static struct gpio_switch_platform_data rotationlock_switch_platform_data = {
	.gpio = TEGRA_GPIO_PQ0,
};

static struct platform_device rotationlock_switch = {
	.name   = "rotationlock",
	.id     = -1,
	.dev    = {
		.platform_data = &rotationlock_switch_platform_data,
	},
};

static void rotationlock_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PQ0);
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct platform_device *cardhu_spi_devices[] __initdata = {
	&tegra_spi_device4,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data cardhu_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static void __init cardhu_spi_init(void)
{
	int i;
	struct clk *c;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	cardhu_spi_pdata.parent_clk_list = spi_parent_clk;
	cardhu_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device4.dev.platform_data = &cardhu_spi_pdata;
	platform_add_devices(cardhu_spi_devices,
				ARRAY_SIZE(cardhu_spi_devices));

	if (board_info.board_id == BOARD_E1198) {
		tegra_spi_device2.dev.platform_data = &cardhu_spi_pdata;
		platform_device_register(&tegra_spi_device2);
		tegra_spi_slave_device1.dev.platform_data = &cardhu_spi_pdata;
		platform_device_register(&tegra_spi_slave_device1);
	}
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_wm8903_platform_data cardhu_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_bypass_switch_en	= TEGRA_GPIO_BYPASS_SWITCH_EN,
	.gpio_debug_switch_en   = TEGRA_GPIO_DEBUG_SWITCH_EN,
};

static struct platform_device cardhu_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &cardhu_audio_pdata,
	},
};

static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name 		= "ram_console",
	.id 		= -1,
	.num_resources	= ARRAY_SIZE(ram_console_resources),
	.resource	= ram_console_resources,
};

#ifdef CONFIG_PSENSOR3G
static struct gpio_switch_platform_data psensor_3g_switch_platform_data[] = {
	{
		.name = "p-sensor_3g",
		.gpio = TEGRA_GPIO_PI5,
	},
	{
		.name = "p-sensor_3g_1",
		.gpio = TEGRA_GPIO_PK2,
	},
};

static struct psensor_switch_platform_data psensor_switch_platform_data = {
	.psormap =  psensor_3g_switch_platform_data,
	.psormap_size = ARRAY_SIZE(psensor_3g_switch_platform_data),
};

static struct platform_device psensor_switch_3g = {
	.name   = "psensor",
	.id     = -1,
	.dev    = {
		.platform_data  = &psensor_switch_platform_data,
	},
};

static void picasso2_psensor_3g_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PI5);
	tegra_gpio_enable(TEGRA_GPIO_PK2);
}
#endif

static struct platform_device *cardhu_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_ACER_VIBRATOR)
	&vib_timed_gpio_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#ifdef CONFIG_ROTATELOCK
	&rotationlock_switch,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&cardhu_bcm4329_rfkill_device,
	&tegra_pcm_device,
	&cardhu_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&ram_console_device,
#ifdef CONFIG_PSENSOR3G
	&psensor_switch_3g,
#endif
};

#define MXT_CONFIG_CRC  0xD62DE8
static const u8 config[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3C, 0x04, 0x05, 0x00,
	0x02, 0x01, 0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x64, 0x02, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0xFF, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define MXT_CONFIG_CRC_SKU2000  0xA24D9A
static const u8 config_sku2000[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0x32, 0x0A, 0x00, 0x14, 0x14, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x00, 0x00,
	0x1B, 0x2A, 0x00, 0x20, 0x3A, 0x04, 0x05, 0x00,  //23=thr  2 di
	0x04, 0x04, 0x41, 0x0A, 0x0A, 0x0A, 0x0A, 0xFF,
	0x02, 0x55, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,  //0A=limit
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x23,
	0x00, 0x00, 0x00, 0x05, 0x0A, 0x15, 0x1E, 0x00,
	0x00, 0x04, 0x00, 0x03, 0x3F, 0x64, 0x64, 0x01,
	0x0A, 0x14, 0x28, 0x4B, 0x00, 0x02, 0x00, 0x64,
	0x00, 0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x08, 0x10, 0x3C, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static struct i2c_board_info __initdata atmel_i2c_info[] = {
	{
		I2C_BOARD_INFO("maXTouch", 0X4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PJ0),
	},
};

static int __init cardhu_touch_init(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PB0); /* LDO_ENABLE */
	tegra_gpio_enable(TEGRA_GPIO_PJ0); /* INTERRUPT */
	tegra_gpio_enable(TEGRA_GPIO_PI2); /* RESET */

	ret = gpio_request(TEGRA_GPIO_PB0, "Atmel_mXT1386_ENABLE");
	if (ret < 0)
		pr_err("[Touch] gpio_request: TEGRA_GPIO_PB0 fail\n");

	ret = gpio_request(TEGRA_GPIO_PJ0, "Atmel_mXT1386_INT");
	if (ret < 0)
		pr_err("[Touch] gpio_request: TEGRA_GPIO_PJ0 fail\n");

	ret = gpio_request(TEGRA_GPIO_PI2, "Atmel_mXT1386_RESET");
	if (ret < 0)
		pr_err("[Touch] gpio_request: TEGRA_GPIO_PI2 fail\n");

	ret = gpio_direction_output(TEGRA_GPIO_PB0, 1);
	if (ret < 0)
		pr_err("[Touch] gpio_direction_output: TEGRA_GPIO_PB0 fail\n");

	ret = gpio_direction_input(TEGRA_GPIO_PJ0);
	if (ret < 0)
		pr_err("[Touch] gpio_direction_input: TEGRA_GPIO_PJ0 fail\n");

	ret = gpio_direction_output(TEGRA_GPIO_PI2, 0);
	if (ret < 0)
		pr_err("[Touch] gpio_direction_output: TEGRA_GPIO_PI2 fail\n");
	gpio_set_value(TEGRA_GPIO_PI2, 0);
	msleep(2);
	gpio_set_value(TEGRA_GPIO_PI2, 1);

	i2c_register_board_info(1, atmel_i2c_info, 1);

	return 0;
}

static struct tegra_uhsic_config uhsic_phy_config = {
	.enable_gpio = EN_HSIC_GPIO,
	.reset_gpio = -1,
	.sync_start_delay = 9,
	.idle_wait_delay = 17,
	.term_range_adj = 0,
	.elastic_underrun_limit = 16,
	.elastic_overrun_limit = 16,
};

static struct tegra_ehci_platform_data tegra_ehci_uhsic_pdata = {
	.phy_type = TEGRA_USB_PHY_TYPE_HSIC,
	.phy_config = &uhsic_phy_config,
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.hotplug = 1,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

#ifdef CONFIG_USB_SUPPORT
static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = TEGRA_GPIO_PN1,
			.vbus_reg_supply = NULL,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = TEGRA_GPIO_PN1,
			.vbus_reg_supply = NULL,
	},
};

static int cardhu_usb_hsic_postsupend(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2);
#endif
	return 0;
}

static int cardhu_usb_hsic_preresume(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L2TOL0);
#endif
	return 0;
}

static int cardhu_usb_hsic_phy_ready(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L0);
#endif
	return 0;
}

static int cardhu_usb_hsic_phy_off(void)
{
#ifdef CONFIG_TEGRA_BB_XMM_POWER
	baseband_xmm_set_power_status(BBXMM_PS_L3);
#endif
	return 0;
}

static void cardhu_usb_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	tegra_usb_phy_init(tegra_usb_phy_pdata,
			ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	if ( acer_sku != BOARD_SKU_WIFI ) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
		platform_device_register(&tegra_ehci2_device);
	}

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
}
#else
static void cardhu_usb_init(void) { }
#endif

static void cardhu_gps_uart_init(void)
{
            tegra_gpio_disable(TEGRA_GPIO_PC3);//RX
            tegra_gpio_disable(TEGRA_GPIO_PC2);//TX
            tegra_gpio_disable(TEGRA_GPIO_PJ5);//CTS
            tegra_gpio_disable(TEGRA_GPIO_PJ6);//RTS
}

static void cardhu_gps_init(void)
{
	int rc;
	rc = gpio_request(TEGRA_GPIO_PY2, "EN_VDD_GPS");
	if (rc)
		pr_err("EN_VDD_GPS request failed:%d\n", rc);
	tegra_gpio_enable(TEGRA_GPIO_PU2);
	tegra_gpio_enable(TEGRA_GPIO_PU3);
	tegra_gpio_enable(TEGRA_GPIO_PY2);
	rc = gpio_direction_output(TEGRA_GPIO_PY2, 1);
	if (rc)
		pr_err("EN_VDD_GPS direction configuration failed:%d\n", rc);
}

#ifdef CONFIG_ACER_LEDS
static struct gpio_led_data led_pdata = {
	.gpio = TEGRA_GPIO_PR0,
};

static struct platform_device gpio_led_device = {
	.name   = "gpio-leds",
	.id     = -1,
	.dev    = {
		.platform_data  = &led_pdata,
	},
};

static void picasso2_led_init(void)
{
	platform_device_register(&gpio_led_device);
}
#endif

static struct baseband_power_platform_data tegra_baseband_power_data = {
	.baseband_type = BASEBAND_XMM,
	.modem = {
	.xmm = {
			.bb_rst = XMM_GPIO_BB_RST,
			.bb_on = XMM_GPIO_BB_ON,
			.ipc_bb_wake = XMM_GPIO_IPC_BB_WAKE,
			.ipc_ap_wake = XMM_GPIO_IPC_AP_WAKE,
			.ipc_hsic_active = XMM_GPIO_IPC_HSIC_ACTIVE,
			.ipc_hsic_sus_req = XMM_GPIO_IPC_HSIC_SUS_REQ,
			.hsic_device = &tegra_ehci2_device,
		},
	},
};

static struct platform_device tegra_baseband_power_device = {
	.name = "baseband_xmm_power",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static struct platform_device tegra_baseband_power2_device = {
	.name = "baseband_xmm_power2",
	.id = -1,
	.dev = {
		.platform_data = &tegra_baseband_power_data,
	},
};

static void cardhu_modem_init(void)
{
	struct board_info board_info;
	int w_disable_gpio, ret;

	tegra_get_board_info(&board_info);
	switch (board_info.board_id) {
	case BOARD_E1291:
	case BOARD_E1198:
		if (((board_info.board_id == BOARD_E1291) &&
				(board_info.fab < BOARD_FAB_A03)) ||
			((board_info.board_id == BOARD_E1198) &&
					(board_info.fab < BOARD_FAB_A02))) {
			w_disable_gpio = TEGRA_GPIO_PH5;
		} else {
			w_disable_gpio = TEGRA_GPIO_PDD5;
		}
		tegra_gpio_enable(w_disable_gpio);
		ret = gpio_request(w_disable_gpio, "w_disable_gpio");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %d\n",
				__func__, w_disable_gpio);
		else
			gpio_direction_input(w_disable_gpio);

		/* E1291-A04 & E1198:A02: Set PERST signal to high */
		if (((board_info.board_id == BOARD_E1291) &&
				(board_info.fab >= BOARD_FAB_A04)) ||
			((board_info.board_id == BOARD_E1198) &&
					(board_info.fab >= BOARD_FAB_A02))) {
			ret = gpio_request(TEGRA_GPIO_PH7, "modem_perst");
			if (ret < 0) {
				pr_err("%s(): Error in allocating gpio "
					"TEGRA_GPIO_PH7\n", __func__);
				break;
			}
			gpio_direction_output(TEGRA_GPIO_PH7, 1);
			tegra_gpio_enable(TEGRA_GPIO_PH7);
		}
		break;
	case BOARD_E1186:
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.bb_rst);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.bb_on);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_bb_wake);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_ap_wake);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_hsic_active);
		tegra_gpio_enable(
			tegra_baseband_power_data.modem.xmm.ipc_hsic_sus_req);
		platform_device_register(&tegra_baseband_power_device);
		platform_device_register(&tegra_baseband_power2_device);
		break;
	default:
		break;
	}

}

#ifdef CONFIG_SATA_AHCI_TEGRA
static void cardhu_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void cardhu_sata_init(void) { }
#endif

static void acer_board_info(void) {
	if (acer_board_type == BOARD_PICASSO_2)
		pr_info("Board Type: Picasso 2");
	else if (acer_board_type == BOARD_PICASSO_M)
		pr_info("Board Type: Picasso M");

	switch (acer_board_id) {
		case BOARD_EVT:
			pr_info("Board Type: EVT\n");
			break;
		case BOARD_DVT1:
			pr_info("Board Type: DVT1\n");
			break;
		case BOARD_DVT2:
			pr_info("Board Type: DVT2\n");
			break;
		case BOARD_PVT:
			pr_info("Board Type: PVT\n");
			break;
		default:
			pr_info("Board Type: none\n");
			break;
	}

	switch (acer_sku) {
		case BOARD_SKU_WIFI:
			pr_info("SKU Type: Wifi\n");
			break;
		case BOARD_SKU_3G:
			pr_info("SKU Type: 3G\n");
			break;
		case BOARD_SKU_LTE:
			pr_info("SKU Type: LTE\n");
			break;
		default:
			pr_info("SKU: none");
			break;
	}

	if (acer_wifi_module == BOARD_WIFI_AH663)
		pr_info("Wifi module: AH663");
	else if (acer_wifi_module == BOARD_WIFI_NH660)
		pr_info("Wifi module: NH660");

}

#ifdef CONFIG_DOCK_V2
static struct gpio_switch_platform_data dock_switch_platform_dvt1_data = {
	.gpio = TEGRA_GPIO_PBB0,
};

static struct platform_device dock_switch_dvt1 = {
	.name   = "acer-dock",
	.id     = -1,
	.dev    = {
		.platform_data  = &dock_switch_platform_dvt1_data,
	},
};

static struct gpio_switch_platform_data dock_switch_platform_dvt2_data = {
	.gpio = TEGRA_GPIO_PBB6,
};

static struct platform_device dock_switch_dvt2 = {
	.name   = "acer-dock",
	.id     = -1,
	.dev    = {
		.platform_data  = &dock_switch_platform_dvt2_data,
	},
};

static void acer_dock_init(void)
{
	if (acer_board_type == BOARD_PICASSO_2 && (acer_board_id == BOARD_EVT || acer_board_id == BOARD_DVT1)) {
		tegra_gpio_enable(TEGRA_GPIO_PBB0);
		tegra_gpio_enable(TEGRA_GPIO_PS6);
		if (is_tegra_debug_uartport_hs())
			platform_device_register(&dock_switch_dvt1);
		else
			pr_info("[ACER-DOCK-DVT1] Enable the debug message, close the dock!\n");
	} else {
		tegra_gpio_enable(TEGRA_GPIO_PBB6);
		tegra_gpio_enable(TEGRA_GPIO_PS6);
		if (is_tegra_debug_uartport_hs())
			platform_device_register(&dock_switch_dvt2);
		else
			pr_info("[ACER-DOCK-DVT2] Enable the debug message, close the dock!\n");
	}
}
#endif

#ifdef CONFIG_SIMDETECT
static struct gpio_switch_platform_data simdetect_switch_platform_data = {
	.gpio = TEGRA_GPIO_PO5,
};

static struct platform_device picasso_simdetect_switch = {
	.name = "simdetect",
	.id   = -1,
	.dev  = {
		.platform_data = &simdetect_switch_platform_data,
	},
};

static void simdet_init(void)
{
	if (acer_sku != BOARD_SKU_WIFI) {
		platform_device_register(&picasso_simdetect_switch);
	}
}
#endif

static void __init tegra_cardhu_init(void)
{
	tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(cardhu_clk_init_table);
	acer_t30_pinmux_init();
#if defined(CONFIG_MACH_PICASSO2) || defined(CONFIG_MACH_PICASSO_M)  // GPIO init
	gpio_unused_init();
#endif
	cardhu_i2c_init();
	cardhu_spi_init();
	cardhu_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	cardhu_edp_init();
#endif
	cardhu_uart_init();
	cardhu_tsensor_init();
	platform_add_devices(cardhu_devices, ARRAY_SIZE(cardhu_devices));
	cardhu_sdhci_init();
	cardhu_regulator_init();
	cardhu_gpio_switch_regulator_init();
	cardhu_suspend_init();
	cardhu_power_off_init();
	cardhu_touch_init();
	cardhu_gps_uart_init();
	cardhu_gps_init();
	cardhu_modem_init();
	cardhu_scroll_init();
#ifdef CONFIG_DOCK_V2
	acer_dock_init();
#endif
	acer_keys_init();
	acer_panel_init();
	cardhu_sensors_init();
#ifdef CONFIG_BCM4329_RFKILL
        cardhu_bcm4329_bt_rfkill();
#endif
	cardhu_setup_bluesleep();
	cardhu_sata_init();
#ifdef CONFIG_SIMDETECT
	simdet_init();
#endif
	//audio_wired_jack_init();
#if defined(CONFIG_ACER_ES305)
	a1026_init();
#endif
	cardhu_pins_state_init();
#ifdef CONFIG_ROTATELOCK
	rotationlock_init();
#endif
	cardhu_emc_init();
#if defined(CONFIG_ACER_VIBRATOR)
	vib_init();
#endif
#ifdef CONFIG_ACER_LEDS
	picasso2_led_init();
#endif
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	acer_board_info();
#ifdef CONFIG_PSENSOR3G
	picasso2_psensor_3g_init();
#endif
}

static void __init cardhu_ramconsole_reserve(unsigned long size)
{
	struct resource *res;
	long ret;

	res = platform_get_resource(&ram_console_device, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("Failed to find memory resource for ram console\n");
		return;
	}
	res->start = memblock_end_of_DRAM() - size;
	res->end = res->start + size - 1;
	ret = memblock_remove(res->start, size);
	if (ret) {
		ram_console_device.resource = NULL;
		ram_console_device.num_resources = 0;
		pr_err("Failed to reserve memory block for ram console\n");
	}
}

static void __init tegra_cardhu_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* support 1920X1200 with 24bpp */
	tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	cardhu_ramconsole_reserve(SZ_1M);
}

MACHINE_START(PICASSO2, "picasso2")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_cardhu_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_cardhu_init,
MACHINE_END

MACHINE_START(PICASSO_M, "picasso_m")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_cardhu_reserve,
	.init_early     = tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_cardhu_init,
MACHINE_END
