/*
 * acer_audio_common.c - for acer audio common function.
 *
 * Copyright (C) 2011 Acer, Inc.
 * Author: Andyl Liu <Andyl_Liu@acer.com.tw>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dai.h>
#include <sound/control.h>
#include <sound/tlv.h>

#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include "../codecs/wm8903.h"
#include "acer_audio_common.h"

#define AUDIO_COMMON_DRIVER_NAME "acer-audio-common"

/* Enable log or not */
#if 1
#define ACER_DBG(fmt, arg...) printk(KERN_INFO "[AudioCommon](%d): " fmt "\n", __LINE__, ## arg)
#else
#define ACER_DBG(fmt, arg...) do {} while (0)
#endif

#if defined(CONFIG_ARCH_ACER_T20) && !defined(CONFIG_MACH_PICASSO_E)
extern void start_stop_psensor(bool);
#endif

extern struct acer_audio_data audio_data;

struct notifier_block notifier;

static int acer_audio_notifier(struct notifier_block *this,
				unsigned long code, void *dev)
{
	gpio_set_value_cansleep(audio_data.gpio.spkr_en, 0);
	return NOTIFY_DONE;
}

bool is_hp_plugged(void)
{
	if (audio_data.gpio.hp_det == GPIO_INVALID)
		return false;

#if defined(CONFIG_ARCH_ACER_T30)
	if (gpio_get_value(audio_data.gpio.hp_det))
#else
	if (!gpio_get_value(audio_data.gpio.hp_det))
#endif
		return true;
	else
		return false;
}

bool is_debug_on(void)
{
	if (audio_data.gpio.debug_en == GPIO_INVALID)
		return false;

	if (gpio_get_value(audio_data.gpio.debug_en))
		return true;
	else
		return false;
}

void wm8903_event_printf(const char* func, int event)
{
#if defined(CONFIG_ARCH_ACER_T30)
	pr_info("[Audio]:%s = %d, bypass = %d\n", func, SND_SOC_DAPM_EVENT_ON(event),
			gpio_get_value(audio_data.gpio.bypass_en));
#else
	pr_info("[Audio]:%s = %d\n", func, SND_SOC_DAPM_EVENT_ON(event));
#endif
}

bool handset_mic_detect(struct snd_soc_codec *codec)
{
	int i = 0;
	int withMic= 0;
	int withoutMic= 0;
	int MICDET_EINT_14= 0;
	int MICSHRT_EINT_15= 0;
	int irqStatus= 0;
	int irq_mask;
	int irq_val;
	int CtrlReg = 0;
	int is_recording = 0;

	/* delay for avoiding pop noise when user plug in/out headset quickly */
	msleep(1000);
	if (!is_hp_plugged()) {
		ACER_DBG("Do not enalbe mic bias when user plug in/out headset quickly");
		return false;
	}

#if defined(CONFIG_ARCH_ACER_T20) && !defined(CONFIG_MACH_PICASSO_E)
	start_stop_psensor(false);
#endif

	snd_soc_update_bits(codec, WM8903_CLOCK_RATES_2,
				WM8903_CLK_SYS_ENA_MASK, WM8903_CLK_SYS_ENA);
	snd_soc_update_bits(codec, WM8903_WRITE_SEQUENCER_0,
				WM8903_WSEQ_ENA_MASK, WM8903_WSEQ_ENA);

	irq_mask = WM8903_MICDET_EINT_MASK | WM8903_MICSHRT_EINT_MASK;
	irq_val = WM8903_MICDET_EINT | WM8903_MICSHRT_EINT;
	snd_soc_update_bits(codec, WM8903_INTERRUPT_STATUS_1_MASK, irq_mask, 0);

	CtrlReg = snd_soc_read(codec, WM8903_MIC_BIAS_CONTROL_0);
	is_recording = CtrlReg & WM8903_MICBIAS_ENA;

	CtrlReg = WM8903_MICDET_ENA | WM8903_MICBIAS_ENA;
	snd_soc_update_bits(codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg, CtrlReg);

	/* add delay for mic bias stable */
	msleep(100);

	for (i = 0; i <= 5; i++) {
		msleep(1);
		irqStatus = snd_soc_read(codec, WM8903_INTERRUPT_STATUS_1);
		MICDET_EINT_14 = (irqStatus >> 14) & 0x1;
		MICSHRT_EINT_15 = (irqStatus >> 15) & 0x1;

		if (MICDET_EINT_14 == MICSHRT_EINT_15)
			withoutMic++;
		else
			withMic++;

		if (i%2 == 0)
			snd_soc_update_bits(codec, WM8903_INTERRUPT_POLARITY_1, irq_mask, 0);
		else
			snd_soc_update_bits(codec, WM8903_INTERRUPT_POLARITY_1, irq_mask, irq_val);
	}

	snd_soc_update_bits(codec, WM8903_MIC_BIAS_CONTROL_0, WM8903_MICDET_ENA, 0);
	snd_soc_update_bits(codec, WM8903_MIC_BIAS_CONTROL_0, WM8903_MICBIAS_ENA, is_recording);

#if defined(CONFIG_ARCH_ACER_T20) && !defined(CONFIG_MACH_PICASSO_E)
	start_stop_psensor(true);
#endif

	if (withMic > withoutMic) {
		ACER_DBG("%s HEADSET_WITH_MIC !\n", __func__);
		return true;
	} else {
		ACER_DBG("%s HEADSET_WITHOUT_MIC !\n", __func__);
		return false;
	}
}

static int acer_audio_common_probe(struct platform_device *pdev)
{
	pr_info("[AudioCommon] probe done.\n");
	return 0;
}

static int acer_audio_common_remove(struct platform_device *pdev)
{
	ACER_DBG("%s", __func__);
	return 0;
}

/* platform driver register */
static struct platform_driver acer_audio_common_driver = {
	.probe  = acer_audio_common_probe,
	.remove = acer_audio_common_remove,
	.driver = {
		.name = AUDIO_COMMON_DRIVER_NAME
	},
};

/* platform device register */
static struct platform_device acer_audio_common_device = {
	.name = AUDIO_COMMON_DRIVER_NAME,
};

static int __init acer_audio_common_init(void)
{
	int ret;
	ACER_DBG("%s", __func__);
	ret = platform_driver_register(&acer_audio_common_driver);
	if (ret) {
		pr_err("[acer_audio_control_driver] failed to register!!\n");
		return ret;
	}

	notifier.notifier_call = acer_audio_notifier;
	register_reboot_notifier(&notifier);

	return platform_device_register(&acer_audio_common_device);
}

static void __exit acer_audio_common_exit(void)
{
	platform_device_unregister(&acer_audio_common_device);
	platform_driver_unregister(&acer_audio_common_driver);
	unregister_reboot_notifier(&notifier);
}

module_init(acer_audio_common_init);
module_exit(acer_audio_common_exit);

MODULE_DESCRIPTION("ACER AUDIO COMMON DRIVER");
MODULE_LICENSE("GPL");
