/*
 * arch/arm/mach-tegra/board-ventana.c
 *
 * Copyright (c) 2010-2011, NVIDIA Corporation.
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
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/mfd/tps6586x.h>
#include <linux/memblock.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/tegra_uart.h>

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
#ifdef CONFIG_ROTATELOCK
#include <linux/switch.h>
#endif

#include "board.h"
#include "clock.h"
#include "board-acer-t20.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "wakeups-t2.h"
#include "pm.h"

#if defined(CONFIG_ACER_VIBRATOR)
#include <../../../drivers/staging/android/timed_output.h>
#include <../../../drivers/staging/android/timed_gpio.h>
#endif

extern void SysShutdown(void);

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
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
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 10,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PG2,
	.clk = "cdev2",
};

#ifdef CONFIG_BCM4329_RFKILL

static struct resource ventana_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
#if defined(CONFIG_MACH_PICASSO_E)
	{
		.name   = "bcm4329_wifi_reset_gpio",
		.start  = TEGRA_GPIO_PK6,
		.end    = TEGRA_GPIO_PK6,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_vdd_gpio",
		.start  = TEGRA_GPIO_PD1,
		.end    = TEGRA_GPIO_PD1,
		.flags  = IORESOURCE_IO,
	},
#endif
};

static struct platform_device ventana_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ventana_bcm4329_rfkill_resources),
	.resource       = ventana_bcm4329_rfkill_resources,
};

static void __init ventana_bt_rfkill(void)
{
	/*Add Clock Resource*/
	clk_add_alias("bcm4329_32k_clk", ventana_bcm4329_rfkill_device.name, \
				"blink", NULL);
	return;
}
#else
static inline void ventana_bt_rfkill(void) { }
#endif

static struct resource ventana_bluesleep_resources[] = {
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
};

static struct platform_device ventana_bluesleep_device = {
	.name           = "bluesleep",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ventana_bluesleep_resources),
	.resource       = ventana_bluesleep_resources,
};

static void __init ventana_setup_bluesleep(void)
{
	platform_device_register(&ventana_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);
	return;
}

static __initdata struct tegra_clk_init_table ventana_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "blink",	"clk_32k",	32768,		false},
	{ "pll_p_out4",	"pll_p",	24000000,	true },
	{ "pwm",	"clk_32k",	32768,		false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s2",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_ulpi_config ventana_ehci2_ulpi_phy_config = {
	.reset_gpio = TEGRA_GPIO_PV1,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data ventana_ehci2_ulpi_platform_data = {
	.operating_mode = TEGRA_USB_HOST,
	.power_down_on_bus_suspend = 1,
	.phy_config = &ventana_ehci2_ulpi_phy_config,
	.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
};

static struct tegra_i2c_platform_data ventana_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data ventana_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate   = { 50000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr = 0x00FC,
};

static struct tegra_i2c_platform_data ventana_i2c3_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr = 0x00FC,
};

static struct tegra_i2c_platform_data ventana_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
};

static struct wm8903_platform_data ventana_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = VENTANA_GPIO_WM8903(0),
	.gpio_cfg = {
		0, /* Fn is GPIO */
		0, /* Fn is GPIO */
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &ventana_wm8903_pdata,
	.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_CDC_IRQ),
};

static void ventana_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &ventana_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &ventana_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &ventana_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &ventana_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device4);

	i2c_register_board_info(0, &wm8903_board_info, 1);
}
static struct platform_device *ventana_uart_devices[] __initdata = {
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data ventana_uart_pdata;

static void __init uart_debug_init(void)
{
	unsigned long rate;
	struct clk *c;

	/* UARTD is the debug port. */
	pr_info("Selecting UARTD as the debug console\n");
	ventana_uart_devices[2] = &debug_uartd_device;
	debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
	debug_uart_clk = clk_get_sys("serial8250.0", "uartd");

	/* Clock enable for the debug channel */
	if (!IS_ERR_OR_NULL(debug_uart_clk)) {
		rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;
		pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
		c = tegra_get_clock_by_name("pll_p");
		if (IS_ERR_OR_NULL(c))
			pr_err("Not getting the parent clock pll_p\n");
		else
			clk_set_parent(debug_uart_clk, c);

		clk_enable(debug_uart_clk);
		clk_set_rate(debug_uart_clk, rate);
	} else {
		pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
	}
}

static void __init ventana_uart_init(void)
{
	int i;
	struct clk *c;

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
	ventana_uart_pdata.parent_clk_list = uart_parent_clk;
	ventana_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	tegra_uartb_device.dev.platform_data = &ventana_uart_pdata;
	tegra_uartc_device.dev.platform_data = &ventana_uart_pdata;
	tegra_uartd_device.dev.platform_data = &ventana_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(ventana_uart_devices,
				ARRAY_SIZE(ventana_uart_devices));
}

#ifdef CONFIG_ROTATELOCK
static struct gpio_switch_platform_data rotationlock_switch_platform_data = {
	.gpio = TEGRA_GPIO_PQ2,
};

static struct platform_device rotationlock_switch = {
	.name	= "rotationlock",
	.id	= -1,
	.dev	= {
		.platform_data = &rotationlock_switch_platform_data,
	},
};

static void rotationlock_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PQ2);
}
#endif

#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEY(_id, _gpio, _isactivelow, _iswake, _debounce_msec)            \
	{                                       \
		.code = _id,                    \
		.gpio = TEGRA_GPIO_##_gpio,     \
		.active_low = _isactivelow,     \
		.desc = #_id,                   \
		.type = EV_KEY,                 \
		.wakeup = _iswake,              \
		.debounce_interval = _debounce_msec,        \
	}

#ifdef CONFIG_MACH_VANGOGH
static struct gpio_keys_button acer_keys[] = {
	[0] = GPIO_KEY(KEY_VOLUMEUP, PQ4, 1, 0, 10),
	[1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ5, 1, 0, 10),
	[2] = GPIO_KEY(KEY_POWER, PC7, 0, 1, 0),
	[3] = GPIO_KEY(KEY_POWER, PI3, 0, 0, 0),
	[4] = GPIO_KEY(KEY_HOME, PQ1, 0, 0,0),
};
#else
static struct gpio_keys_button acer_keys[] = {
       [0] = GPIO_KEY(KEY_VOLUMEUP, PQ4, 1, 0, 10),
       [1] = GPIO_KEY(KEY_VOLUMEDOWN, PQ5, 1, 0, 10),
       [2] = GPIO_KEY(KEY_POWER, PC7, 0, 1, 0),
       [3] = GPIO_KEY(KEY_POWER, PI3, 0, 0, 0),
};
#endif

#define PMC_WAKE_STATUS 0x14

static int ventana_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

#if defined(CONFIG_ARCH_ACER_T20)
	int clr_key_pwr = 0x100;

	writel(clr_key_pwr, IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
#endif
	return status & TEGRA_WAKE_GPIO_PC7 ? KEY_POWER : KEY_RESERVED;
}

static struct gpio_keys_platform_data acer_keys_platform_data = {
	.buttons	= acer_keys,
	.nbuttons	= ARRAY_SIZE(acer_keys),
	.wakeup_key	= ventana_wakeup_key,
};

static struct platform_device acer_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev	= {
		.platform_data	= &acer_keys_platform_data,
	},
};

static void acer_keys_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(acer_keys); i++)
		tegra_gpio_enable(acer_keys[i].gpio);
}
#endif

#if defined(CONFIG_ACER_VIBRATOR)
static struct timed_gpio vib_timed_gpios[] = {
	{
		.name = "vibrator",
		.gpio = TEGRA_GPIO_PV5,
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
	tegra_gpio_enable(TEGRA_GPIO_PV5);
}
#endif

#ifdef CONFIG_DOCK_V1
static struct gpio_switch_platform_data dock_switch_platform_data = {
	.gpio = TEGRA_GPIO_PR0,
};

static struct platform_device dock_switch = {
	.name	= "acer-dock",
	.id	= -1,
	.dev	= {
		.platform_data  = &dock_switch_platform_data,
	},
};

static void acer_dock_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PR0);
	tegra_gpio_enable(TEGRA_GPIO_PR1);
	tegra_gpio_enable(TEGRA_GPIO_PX6);

	if (is_tegra_debug_uartport_hs()) {
		platform_device_register(&dock_switch);
	} else {
		pr_info("UART DEBUG MESSAGE ON!!!\n");
	}
}
#endif

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

static struct tegra_wm8903_platform_data ventana_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= -1,
#ifdef CONFIG_MACH_PICASSO_E
	.gpio_debug_switch_en   = TEGRA_GPIO_DEBUG_SWITCH_EN,
#else
	.gpio_debug_switch_en   = -1,
#endif
#ifdef CONFIG_MACH_VANGOGH
	.gpio_spkr_mute      = TEGRA_GPIO_SPKR_MUTE,
#else
	.gpio_spkr_mute      = -1,
#endif
};

static struct platform_device ventana_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &ventana_audio_pdata,
	},
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
	{
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};
#endif

#ifdef CONFIG_PSENSOR
static struct gpio_switch_platform_data psensor_switch_platform_data = {
	.gpio = TEGRA_GPIO_PC1,
};

static struct platform_device psensor_switch = {
	.name   = "psensor",
	.id     = -1,
	.dev    = {
		.platform_data  = &psensor_switch_platform_data,
	},
};

static void p_sensor_init(void)
{
	// enable gpio for psensor
	tegra_gpio_enable(TEGRA_GPIO_PC1);
}
#endif

static struct platform_device *ventana_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_gart_device,
	&tegra_aes_device,
#ifdef CONFIG_KEYBOARD_GPIO
	&acer_keys_device,
#endif
#if defined(CONFIG_ACER_VIBRATOR)
	&vib_timed_gpio_device,
#endif
	&tegra_wdt_device,
	&tegra_avp_device,
#ifdef CONFIG_ROTATELOCK
	&rotationlock_switch,
#endif
	&tegra_camera,
	&tegra_i2s_device1,
	&tegra_i2s_device2,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&ventana_bcm4329_rfkill_device,
	&tegra_pcm_device,
	&ventana_audio_device,
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
#ifdef CONFIG_PSENSOR
	&psensor_switch,
#endif

};

#ifdef CONFIG_TOUCHSCREEN_CYPRESS
#include <linux/input/cyttsp.h>
#define CY_I2C_IRQ_GPIO        TEGRA_GPIO_PV6
#define CY_I2C_ADR             CY_TCH_I2C_ADDR  /* LSTS Operational mode I2C address */
#define CY_I2C_VKEY_NAME       "virtualkeys.cyttsp-i2c" /* must match I2C name */
#define CY_MAXX 1024
#define CY_MAXY 600
#define CY_VK_SZ_X             60
#define CY_VK_SZ_Y             80
#define CY_VK_CNTR_X1          (CY_VK_SZ_X*0)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X2          (CY_VK_SZ_X*1)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X3          (CY_VK_SZ_X*2)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_X4          (CY_VK_SZ_X*3)+(CY_VK_SZ_X/2)
#define CY_VK_CNTR_Y1          CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y2          CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y3          CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK_CNTR_Y4          CY_MAXY+(CY_VK_SZ_Y/2)
#define CY_VK1_POS             ":95:770:190:60"
#define CY_VK2_POS             ":285:770:190:60"
#define CY_VK3_POS             ":475:770:190:60"
#define CY_VK4_POS             ":665:770:190:60"

enum cyttsp_gest {
	CY_GEST_GRP_NONE = 0x0F,
	CY_GEST_GRP1 = 0x10,
	CY_GEST_GRP2 = 0x20,
	CY_GEST_GRP3 = 0x40,
	CY_GEST_GRP4 = 0x80,
};

/* default bootloader keys */
u8 dflt_bl_keys[] = {
	0, 1, 2, 3, 4, 5, 6, 7
};

static int cyttsp_i2c_init(void)
{
	int ret;

	ret = gpio_request(CY_I2C_IRQ_GPIO, "CYTTSP I2C IRQ GPIO");
	if (ret) {
		pr_err("%s: Failed to request GPIO %d\n", __func__, CY_I2C_IRQ_GPIO);
		return ret;
	}
	gpio_direction_input(CY_I2C_IRQ_GPIO);
	return 0;
}

static void cyttsp_i2c_exit(void)
{
	gpio_free(CY_I2C_IRQ_GPIO);
}

static int cyttsp_i2c_wakeup(void)
{
	return 0;
}

static struct cyttsp_platform_data cypress_i2c_ttsp_platform_data = {
	.wakeup = cyttsp_i2c_wakeup,
	.init = cyttsp_i2c_init,
	.exit = cyttsp_i2c_exit,
	.maxx = CY_MAXX,
	.maxy = CY_MAXY,
	.use_hndshk = false,
	.use_sleep = true,

	/* activate up to 4 groups and set active distance */
	.act_dist = CY_GEST_GRP_NONE & CY_ACT_DIST_DFLT,

	/*change act_intrvl to customize the Active power state
	 *scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.name = CY_I2C_NAME,
	.irq_gpio = CY_I2C_IRQ_GPIO,
	.bl_keys = dflt_bl_keys,
};

static const struct i2c_board_info ventana_i2c_bus1_touch_info[] = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CY_I2C_ADR),
		.irq = TEGRA_GPIO_TO_IRQ(CY_I2C_IRQ_GPIO),
		.platform_data = &cypress_i2c_ttsp_platform_data,
	},
};

static int __init ventana_touch_init_cypress(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PV6);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);
	i2c_register_board_info(0, ventana_i2c_bus1_touch_info, 1);
	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT1386)
static struct i2c_board_info __initdata atmel_mXT1386_i2c_info[] = {
	{
		I2C_BOARD_INFO("maXTouch", 0X4C),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	},
};

static int __init touch_init_atmel_mXT1386(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PV6);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);

	ret = gpio_request(TEGRA_GPIO_PV6, "atmel_maXTouch1386_irq_gpio");
	if (ret < 0)
		printk("atmel_maXTouch1386: gpio_request TEGRA_GPIO_PQ6 fail\n");

	ret = gpio_request(TEGRA_GPIO_PQ7, "atmel_maXTouch1386");
	if (ret < 0)
		printk("atmel_maXTouch1386: gpio_request fail\n");

	ret = gpio_direction_output(TEGRA_GPIO_PQ7, 0);
	if (ret < 0)
		printk("atmel_maXTouch1386: gpio_direction_output fail\n");
	gpio_set_value(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, atmel_mXT1386_i2c_info, 1);

	return 0;
}
#endif

#if defined(CONFIG_TOUCHSCREEN_ATMEL_768E)
static struct i2c_board_info __initdata atmel_mXT768e_i2c_info[] = {
	{
		I2C_BOARD_INFO("maXTouch", 0X4D),
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PV6),
	},
};

static int __init touch_init_atmel_mXT768e(void)
{
	int ret;

	tegra_gpio_enable(TEGRA_GPIO_PV6); /* TP INTERRUPT PIN */
	tegra_gpio_enable(TEGRA_GPIO_PQ7); /* TP RESET PIN */

	ret = gpio_request(TEGRA_GPIO_PV6, "atmel_maXTouch768e_irq_gpio");
	if (ret < 0)
		printk("atmel_maXTouch768e: gpio_request TEGRA_GPIO_PQ6 fail\n");

	ret = gpio_request(TEGRA_GPIO_PQ7, "atmel_maXTouch768e_rst_gpio");
	if (ret < 0)
		printk("atmel_maXTouch768e: gpio_request fail\n");

	ret = gpio_direction_output(TEGRA_GPIO_PQ7, 0);
	if (ret < 0)
		printk("atmel_maXTouch768e: gpio_direction_output fail\n");

	gpio_set_value(TEGRA_GPIO_PQ7, 0);
	msleep(1);
	gpio_set_value(TEGRA_GPIO_PQ7, 1);
	msleep(100);

	i2c_register_board_info(0, atmel_mXT768e_i2c_info, 1);

	return 0;
}
#endif

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_irq = TPS6586X_INT_BASE + TPS6586X_INT_USB_DET,
			.vbus_gpio = TEGRA_GPIO_PD0,
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = TEGRA_GPIO_PD3,
	},
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
	},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 0,
			.hotplug = 1,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

static int __init ventana_gps_init(void)
{
	struct clk *clk32 = clk_get_sys(NULL, "blink");
	if (!IS_ERR(clk32)) {
		clk_set_rate(clk32,clk32->parent->rate);
		clk_enable(clk32);
	}

	tegra_gpio_enable(TEGRA_GPIO_PZ3);
	return 0;
}

static void __init ventana_power_off_init(void)
{
	pm_power_off = SysShutdown;
}

static void ventana_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
	/* OTG should be the first to be registered */
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	platform_device_register(&tegra_udc_device);
#if !defined(CONFIG_MACH_PICASSO_E)
	if(get_sku_id() == BOARD_PICASSO_3G || get_sku_id() == BOARD_VANGOGH_3G)
		platform_device_register(&tegra_ehci2_device);
#endif

#if !defined(CONFIG_MACH_VANGOGH)
	tegra_ehci3_device.dev.platform_data=&tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
#endif
}

int get_pin_value(unsigned int gpio, char *name)
{
	int pin_value = 0;

	tegra_gpio_enable(gpio);
	gpio_request(gpio, name);
	gpio_direction_input(gpio);
	pin_value = gpio_get_value(gpio);
	gpio_free(gpio);
	tegra_gpio_disable(gpio);
	return pin_value;
}

int get_sku_id(void)
{
#ifdef CONFIG_MACH_PICASSO
	/* Wifi=5, 3G=3, DVT2=7 */
	return (get_pin_value(TEGRA_GPIO_PQ0, "PIN0") << 2) + \
		 (get_pin_value(TEGRA_GPIO_PQ3, "PIN1") << 1) + \
		 get_pin_value(TEGRA_GPIO_PQ6, "PIN2");
#endif
#ifdef CONFIG_MACH_VANGOGH
	/* Wifi=0, 3G=1 */
	return (get_pin_value(TEGRA_GPIO_PQ0,"PIN0"));
#endif
}

void acer_board_info(void)
{
#ifdef CONFIG_MACH_PICASSO_E
	switch(get_sku_id()){
		case BOARD_PICASSO_WIFI:
			pr_info("Board Type: Picasso E Wifi\n");
			break;
		default:
			pr_info("Board Type: Unknown\n");
			break;
	}
#elif CONFIG_MACH_PICASSO
	switch(get_sku_id()){
		case BOARD_PICASSO_3G:
			pr_info("Board Type: Picasso 3G\n");
			break;
		case BOARD_PICASSO_WIFI:
			pr_info("Board Type: Picasso Wifi\n");
			break;
		case BOARD_PICASSO_DVT2:
			pr_info("Board Type: Picasso DVT2\n");
			break;
		default:
			pr_info("Board Type: Unknown\n");
			break;
	}
#endif
#ifdef CONFIG_MACH_VANGOGH
	switch(get_sku_id()){
		case BOARD_VANGOGH_3G:
			pr_info("Board Type: VanGogh 3G\n");
			break;
		case BOARD_VANGOGH_WIFI:
			pr_info("Board Type: VanGogh Wifi\n");
			break;
		default:
			pr_info("Board Type: Unknown\n");
			break;
	}
#endif
}

static void __init acer_t20_init(void)
{

	tegra_clk_init_from_table(ventana_clk_init_table);
	ventana_pinmux_init();
	ventana_i2c_init();
	ventana_uart_init();
	tegra_ehci2_device.dev.platform_data
		= &ventana_ehci2_ulpi_platform_data;
	platform_add_devices(ventana_devices, ARRAY_SIZE(ventana_devices));

	ventana_sdhci_init();
	acer_t20_charge_init();
	acer_t20_regulator_init();

#ifdef CONFIG_TOUCHSCREEN_CYPRESS
	ventana_touch_init_cypress();
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1386
	touch_init_atmel_mXT1386();
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_768E
	touch_init_atmel_mXT768e();
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	acer_keys_init();
#endif
#ifdef CONFIG_ROTATELOCK
        rotationlock_init();
#endif
#ifdef CONFIG_DOCK_V1
	acer_dock_init();
#endif
	acer_board_info();

#ifdef CONFIG_PSENSOR
	p_sensor_init();
#endif
	ventana_usb_init();
	ventana_gps_init();
	ventana_panel_init();
	ventana_sensors_init();
	ventana_bt_rfkill();
	ventana_power_off_init();
	acer_t20_emc_init();

	ventana_setup_bluesleep();
#if defined(CONFIG_ACER_VIBRATOR)
	vib_init();
#endif
	tegra_release_bootloader_fb();
}

int __init tegra_ventana_protected_aperture_init(void)
{
	if (!machine_is_ventana())
		return 0;

	tegra_protected_aperture_init(tegra_grhost_aperture);
	return 0;
}
late_initcall(tegra_ventana_protected_aperture_init);

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static void __init ventana_ramconsole_reserve(unsigned long size)
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
#endif

void __init tegra_ventana_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_256M, SZ_8M, SZ_16M);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ventana_ramconsole_reserve(SZ_1M);
#endif
}

MACHINE_START(VENTANA, "picasso")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ventana_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= acer_t20_init,
MACHINE_END

MACHINE_START(PICASSO, "picasso")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ventana_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= acer_t20_init,
MACHINE_END

MACHINE_START(VANGOGH, "vangogh")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ventana_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= acer_t20_init,
MACHINE_END

MACHINE_START(PICASSO_E, "picasso_e")
	.boot_params    = 0x00000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ventana_reserve,
	.init_early	= tegra_init_early,
	.init_irq	= tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine	= acer_t20_init,
MACHINE_END
