#ifndef __ACER_AUDIO_CONTROL_T20_H__
#define __ACER_AUDIO_CONTROL_T20_H__

#include "acer_audio_common.h"

/* settings */
#define R44_CTS_LRIN_VOL 0x05
#define R44_ASR_LRIN_VOL 0x16
#define R44_SPK_LRIN_VOL 0x10
#define R44_HP_LRIN_VOL 0x1C
#define R59_SPK_LINEOUT_VOL 0x3F
#define R24_DAC_BOOST_0dB 0x00
#define R24_DAC_BOOST_6dB 0x01
#define R41_DRC_MAXGAIN_38dB 0x03
#define R42_COMPRESSOR_SLOP_R0 0x05
#define R42_COMPRESSOR_SLOP_DEFAULT_R0 0x04
#define R43_COMPRESSOR_THRESSHOLD_T 0x14
#define R43_COMPRESSOR_THRESSHOLD_DEFAULT_T 0
#define R43_COMPRESSOR_THRESSHOLD_YT 0x01
#define R43_COMPRESSOR_THRESSHOLD_DEFAULT_YT 0
#if defined(CONFIG_MACH_PICASSO_E)
#define HPOUT_VOL 0xB0
#define LINEOUT_VOL 0xB7
#elif defined(CONFIG_MACH_PICASSO)
#define HPOUT_VOL 0xB8
#define LINEOUT_VOL 0xB7
#else
#define HPOUT_VOL 0xB8
#define LINEOUT_VOL 0xB7
#endif

/* Acoustic table */
#ifdef CONFIG_ACER_FM_SINGLE_MIC
#define ACOUSTIC_DEVICE_MIC_VOIP_TABLE                  0x02
#define ACOUSTIC_HEADSET_MIC_VOIP_TABLE                 0x04
#define ACOUSTIC_DEVICE_MIC_RECORDING_TABLE             0x02
#define ACOUSTIC_HEADSET_MIC_RECORDING_TABLE            0x04
#define ACOUSTIC_CAMCORDER_RECORDER_TABLE               0x06
#define ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE     0x03
#define ACOUSTIC_HEADSET_MIC_MUSIC_RECOGNITION_TABLE    0x05
#define ACOUSTIC_SPEECH_RECOGNITION_TABLE               0x08
#define ACOUSTIC_CTS_VERIFIER_TABLE                     0x07
#else
#define ACOUSTIC_DEVICE_MIC_VOIP_TABLE                  0x02
#define ACOUSTIC_DEVICE_MIC_RECORDING_TABLE             0x02
#define ACOUSTIC_CAMCORDER_TABLE                        0x04
#define ACOUSTIC_FRONT_CAMCORDER_TABLE                  0x02
#define ACOUSTIC_REAR_CAMCORDER_TABLE                   0x06
#define ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE     0x04
#define ACOUSTIC_SPEECH_RECOGNITION_TABLE               0x07
#define ACOUSTIC_CTS_VERIFIER_TABLE                     0x07
#endif

enum ap_id {
    aud_default = 0,
    G_TALK,
    CTS,
    ASR,
    CAMCORDER_FRONT,
    CAMCORDER_REAR,
    VOICE_RECORDER,
    SOUNDHOUND_MUSIC,
    SOUNDHOUND_VOICE,
    MUSICA,
    EVERNOTE,
    ACOUSTIC_NUM
};

extern void mic_switch(struct tegra_wm8903_platform_data *pdata);
extern void fm2018_switch(struct tegra_wm8903_platform_data *pdata);
extern void set_int_mic_state(bool state);
extern void set_ext_mic_state(bool state);
#ifdef CONFIG_ACER_FM_SINGLE_MIC
#ifdef CONFIG_MACH_PICASSO_E
extern void setAudioCABC(int enable);
#endif
#endif
extern int switch_audio_table(int control_mode, bool fromAP);
extern void acer_volume_setting(struct snd_soc_codec *codec, struct snd_pcm_substream *substream);

int tune_codec_setting(int control_mode);
void set_voip_hp_gain(struct snd_soc_codec* codec);
void set_voip_spk_gain(struct snd_soc_codec* codec);
void set_cts_spk_gain(struct snd_soc_codec* codec);
void set_asr_spk_gain(struct snd_soc_codec* codec);

enum mic_mode{
	SINGLE_MIC = 1,
	DUAL_MIC,
};

#define SOC_DAPM_SET_PARAM(xname) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_dapm_info_iconia_param, \
	.get = snd_soc_dapm_get_iconia_param, \
	.put = snd_soc_dapm_put_iconia_param, \
	.private_value = (unsigned long)xname }

int snd_soc_dapm_info_iconia_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_dapm_get_iconia_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *uncontrol);
int snd_soc_dapm_put_iconia_param(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *uncontrol);

#endif
