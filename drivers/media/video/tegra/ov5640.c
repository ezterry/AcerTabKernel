/*
 * ov5640.c - OV5640 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 1280x960. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 1280x960
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/tegra_camera.h>
#include <media/ov5640.h>
#include "ov5640_setting.h"

#if defined(CONFIG_TORCH_TPS61050YZGR)
extern void tps61050_turn_on_torch(void);
extern void tps61050_turn_off_torch(void);
extern void tps61050_turn_on_flash(void);
extern void tps61050_turn_off_flash(void);
#endif

struct ov5640_info {
	int mode;
	struct i2c_client *i2c_client;
	struct ov5640_platform_data *pdata;
	int af_initialized;
	int focus_mode;
#if defined(CONFIG_TORCH_TPS61050YZGR)
	int flash_mode;
	int flash_status;
	int lowlight_mode;
#endif
	int white_balance_mode;
	int color_effect_mode;
	int exposure_mode;
};

#define OV5640_MAX_RETRIES  3

static int ov5640_read_reg(struct i2c_client *client, u16 addr, u8 *val)
{
	int err, retry = 0;
	struct i2c_msg msg[2];
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	do {
		err = i2c_transfer(client->adapter, msg, 2);
		if (err == 2) {
			*val = data[2];
			return 0;
		}

		retry++;
		pr_err("%s: i2c transfer failed, retrying addr=0x%04X\n", __func__, addr);
		msleep(10);
	} while (retry <= OV5640_MAX_RETRIES);

	return err;
}

static int ov5640_write_reg(struct i2c_client *client, u16 addr, u8 val)
{
	int err, retry = 0;
	struct i2c_msg msg;
	unsigned char data[3];

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) val;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;

		retry++;
		pr_err("%s: i2c transfer failed, retrying addr=0x%04X, val=0x%02X\n",
			__func__, addr, val);
		msleep(10);
	} while (retry <= OV5640_MAX_RETRIES);

	return err;
}

#if defined(CONFIG_TORCH_TPS61050YZGR)
static int ov5640_is_lowlight_mode(struct ov5640_info *info)
{
	u8 reg_350C, reg_350D;
	u16 AEC_PK_VTS;

	ov5640_read_reg(info->i2c_client, 0x350C, &reg_350C);
	ov5640_read_reg(info->i2c_client, 0x350D, &reg_350D);
	AEC_PK_VTS = (u16)reg_350C << 8 | (u16)reg_350D;
	pr_info("%s: AEC_PK_VTS = 0x%04X\n", __func__, AEC_PK_VTS);

	if (AEC_PK_VTS > 0)
		return 1;
	else
		return 0;
}
#endif

static int ov5640_write_table(struct ov5640_info *info, const struct ov5640_reg table[])
{
	int err;
	const struct ov5640_reg *next;

	pr_info("%s ++\n", __func__);

	for (next=table; next->op!=OV5640_TABLE_END; next++) {
		switch(next->op) {
			case OV5640_WRITE_REG:
				err = ov5640_write_reg(info->i2c_client, next->addr, next->val);
				if (err)
					return err;
				break;

			case OV5640_WRITE_REG_MASK:
				// when encountering WRITE_REG_MASK, a write operation with mask is introduced
				// the written value is stored in this entry
				// and the data mask is stored in the next entry with op type DATA_MASK
				// the original value is read out and only the bits under the mask will be modified
				{
					u8 temp, data, mask;

					data = next->val;
					mask = (next+1)->val;
					err = ov5640_read_reg(info->i2c_client, next->addr, &temp);
					if (err)
						return err;
					// clear and modify the bits under the mask
					temp = (temp & (~mask)) | (data & mask);
					err = ov5640_write_reg(info->i2c_client, next->addr, temp);
					if (err)
						return err;
				}
				break;

			case OV5640_DATA_MASK:
				break;

			case OV5640_WAIT_MS:
				msleep(next->val);
				break;

			default:
				pr_err("%s: invalid op %d\n", __func__, next->op);
				return -EINVAL;
		}
	}

	pr_info("%s --\n", __func__);
	return 0;
}

static inline void ov5640_writegroup_enter(struct ov5640_info *info, u8 reg_group)
{
	ov5640_write_reg(info->i2c_client, OV5640_REG_SRM_GROUP_ACCESS, reg_group);
}

static inline void ov5640_writegroup_exit(struct ov5640_info *info, u8 reg_group)
{
	ov5640_write_reg(info->i2c_client, OV5640_REG_SRM_GROUP_ACCESS,
			(reg_group | OV5640_SRM_GROUP_END_HOLD));
}

static inline void ov5640_writegroup_launch(struct ov5640_info *info, u8 reg_group)
{
	ov5640_write_reg(info->i2c_client, OV5640_REG_SRM_GROUP_ACCESS,
			(reg_group | OV5640_SRM_GROUP_LAUNCH_ENABLE | OV5640_SRM_GROUP_LAUNCH));
}

static int ov5640_set_mode(struct ov5640_info *info, struct ov5640_mode *mode)
{
	int sensor_mode;
	int err;

	pr_info("%s ++: xres=%d, yres=%d\n", __func__, mode->xres, mode->yres);

	if (mode->xres==1280 && mode->yres==960)
		sensor_mode = OV5640_MODE_1280x960;
	else if (mode->xres==2592 && mode->yres==1944)
		sensor_mode = OV5640_MODE_2592x1944;
	else if (mode->xres==1920 && mode->yres==1080)
		sensor_mode = OV5640_MODE_1920x1080;
	else {
		pr_err("%s: invalid resolution supplied to set mode xres=%d, yres=%d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	if (info->mode == sensor_mode) {
		pr_info("%s --\n", __func__);
		return 0;
	}

	err = ov5640_write_table(info, mode_table[sensor_mode]);
	if (err)
		return err;

	// If need to do auto focus based on different sensor's internal resolution configuration,
	// this command is needed.
	err = ov5640_write_reg(info->i2c_client, OV5640_AF_CMD_MAIN, OV5640_RELAUNCH_FOCUS_ZONES);
	if (err)
		return err;

	info->mode = sensor_mode;

	pr_info("%s --\n", __func__);
	return 0;
}

static int ov5640_get_status(struct ov5640_info *info)
{
	pr_info("%s\n", __func__);

	return 0;
}

static int ov5640_af_trigger(struct ov5640_info *info, int trigger)
{
	int err;

	pr_info("%s: trigger = %d\n", __func__, trigger);

	if (trigger == OV5640_AF_TRIGGER) {
#if defined(CONFIG_TORCH_TPS61050YZGR)
		if (info->flash_mode == OV5640_FlashMode_Auto) {
			info->lowlight_mode = ov5640_is_lowlight_mode(info);
			if (info->lowlight_mode)
				tps61050_turn_on_torch();
		} else if (info->flash_mode == OV5640_FlashMode_On)
			tps61050_turn_on_torch();
#endif

		err = ov5640_write_reg(info->i2c_client, OV5640_AF_CMD_MAIN, OV5640_TRIG_AUTO_FOCUS);
	} else {
#if defined(CONFIG_TORCH_TPS61050YZGR)
		// the IOCTL command AF_ABORT is always running when finishing taking a picture,
		// we can use this call flow to control flash
		tps61050_turn_off_flash();
		info->flash_status = 0;
		info->lowlight_mode = 0;
#endif

		err = ov5640_write_reg(info->i2c_client, OV5640_AF_CMD_MAIN, OV5640_RELEASE_FOCUS);
	}

	info->focus_mode = trigger;

	return err;
}

static int ov5640_get_af_status(struct ov5640_info *info)
{
	int af_status;
	u8 fw_status, s_zone;

	ov5640_read_reg(info->i2c_client, OV5640_AF_FW_STATUS, &fw_status);
	if (fw_status == OV5640_FW_S_FOCUSING || fw_status == OV5640_FW_S_IDLE)
		af_status = 0;  // AF is running
	else if (fw_status == OV5640_FW_S_FOCUSED) {
#if defined(CONFIG_TORCH_TPS61050YZGR)
		tps61050_turn_off_torch();
#endif

		ov5640_read_reg(info->i2c_client, OV5640_AF_CMD_PARA4, &s_zone);
		if (s_zone)
			af_status = 1;  // AF is completed and success
		else
			af_status = 2;  // AF is completed and failed
	} else {
		pr_err("%s: fw_status = %02X\n", __func__, fw_status);
		af_status = -1;
	}

	return af_status;
}

#if defined(CONFIG_TORCH_TPS61050YZGR)
static int ov5640_set_flash_mode(struct ov5640_info *info, int flash_mode)
{
	pr_info("%s: flash_mode = %d\n", __func__, flash_mode);

	info->flash_mode = flash_mode;
	switch (flash_mode) {
		case OV5640_FlashMode_Auto:
			break;

		case OV5640_FlashMode_On:
			break;

		case OV5640_FlashMode_Off:
			tps61050_turn_off_flash();
			break;

		case OV5640_FlashMode_Torch:
			tps61050_turn_on_torch();
			break;

		default:
			break;
	}

	return 0;
}

static int ov5640_get_flash_status(struct ov5640_info *info)
{
	pr_info("%s: flash_status = %d\n", __func__, info->flash_status);

	return info->flash_status;
}
#endif

static int ov5640_set_white_balance(struct ov5640_info *info, int white_balance)
{
	int err = 0;

	pr_info("%s: white_balance = %d\n", __func__, white_balance);

	ov5640_writegroup_enter(info, 0);

	switch (white_balance) {
		case OV5640_WhiteBalance_Auto:
			err = ov5640_write_table(info, WhiteBalance_Auto);
			break;

		case OV5640_WhiteBalance_Incandescent:
			err = ov5640_write_table(info, WhiteBalance_Incandescent);
			break;

		case OV5640_WhiteBalance_Fluorescent:
			err = ov5640_write_table(info, WhiteBalance_Fluorescent);
			break;

		case OV5640_WhiteBalance_Daylight:
			err = ov5640_write_table(info, WhiteBalance_Daylight);
			break;

		case OV5640_WhiteBalance_CloudyDaylight:
			err = ov5640_write_table(info, WhiteBalance_CloudyDaylight);
			break;

		default:
			break;
	}

	ov5640_writegroup_exit(info, 0);
	ov5640_writegroup_launch(info, 0);

	info->white_balance_mode = white_balance;

	return err;
}

static int ov5640_set_color_effect(struct ov5640_info *info, int color_effect)
{
	int err;

	pr_info("%s: color_effect = %d\n", __func__, color_effect);

	ov5640_writegroup_enter(info, 0);

	//  white balance and color effect are both written in group write mode
	//  but only one group can be launched at the frame boundary
	//  a prior group would be overridden by its following one
	//  Both white balance and color effect are written in the same write group,
	//  so white balance needs writing again in color effect function
	switch (info->white_balance_mode) {
		case OV5640_WhiteBalance_Auto:
			err = ov5640_write_table(info, WhiteBalance_Auto);
			break;

		case OV5640_WhiteBalance_Incandescent:
			err = ov5640_write_table(info, WhiteBalance_Incandescent);
			break;

		case OV5640_WhiteBalance_Fluorescent:
			err = ov5640_write_table(info, WhiteBalance_Fluorescent);
			break;

		case OV5640_WhiteBalance_Daylight:
			err = ov5640_write_table(info, WhiteBalance_Daylight);
			break;

		case OV5640_WhiteBalance_CloudyDaylight:
			err = ov5640_write_table(info, WhiteBalance_CloudyDaylight);
			break;

		default:
			break;
	}

	switch (color_effect) {
		case OV5640_ColorEffect_Aqua:
			err = ov5640_write_table(info, ColorEffect_Aqua);
			break;

		case OV5640_ColorEffect_Mono:
			err = ov5640_write_table(info, ColorEffect_Mono);
			break;

		case OV5640_ColorEffect_Negative:
			err = ov5640_write_table(info, ColorEffect_Negative);
			break;

		case OV5640_ColorEffect_None:
			err = ov5640_write_table(info, ColorEffect_None);
			break;

		case OV5640_ColorEffect_Sepia:
			err = ov5640_write_table(info, ColorEffect_Sepia);
			break;

		case OV5640_ColorEffect_Solarize:
			err = ov5640_write_table(info, ColorEffect_Solarize);
			break;

		default:
			break;
        }

	ov5640_writegroup_exit(info, 0);
	ov5640_writegroup_launch(info, 0);

	info->color_effect_mode = color_effect;

	return err;
}

static int ov5640_set_exposure(struct ov5640_info *info, int exposure)
{
	int err;

	pr_info("%s: exposure = %d\n", __func__, exposure);

	switch (exposure) {
		case OV5640_Exposure_Minus_Two:
			err = ov5640_write_table(info, Exposure_Minus_Two);
			break;

		case OV5640_Exposure_Minus_One:
			err = ov5640_write_table(info, Exposure_Minus_One);
			break;

		case OV5640_Exposure_Zero:
			err = ov5640_write_table(info, Exposure_Zero);
			break;

		case OV5640_Exposure_Plus_One:
			err = ov5640_write_table(info, Exposure_Plus_One);
			break;

		case OV5640_Exposure_Plus_Two:
			err = ov5640_write_table(info, Exposure_Plus_Two);
			break;

		default:
			break;
	}

	info->exposure_mode = exposure;

	return err;
}

static int ov5640_capture_cmd(struct ov5640_info *info)
{
	pr_info("%s\n", __func__);

#if defined(CONFIG_TORCH_TPS61050YZGR)
	if (info->flash_mode == OV5640_FlashMode_Auto) {
		if (info->lowlight_mode ||
			(info->focus_mode==OV5640_AF_ABORT && ov5640_is_lowlight_mode(info))) {
			tps61050_turn_on_flash();
			info->flash_status = 1;
		}
	} else if (info->flash_mode == OV5640_FlashMode_On) {
		tps61050_turn_on_flash();
		info->flash_status = 1;
	}
#endif

	return 0;
}

static int ov5640_get_exposure_time(struct ov5640_info *info, struct ov5640_rational *exposure_time)
{
	u8 reg_3500, reg_3501, reg_3502;

	ov5640_read_reg(info->i2c_client, 0x3500, &reg_3500);
	ov5640_read_reg(info->i2c_client, 0x3501, &reg_3501);
	ov5640_read_reg(info->i2c_client, 0x3502, &reg_3502);

	// exposure_time = sec_per_line * exposure_lines
	// sec_per_line = 1 sec / (30 frames * 984 VTS per frame)
	// exposure_lines = 0x3500 bit[3:0], 0x3501 bit[7:0], 0x3502 bit[7:4]
	exposure_time->numerator = (u32)reg_3500<<12 | (u32)reg_3501<<4 | (u32)reg_3502>>4;
	exposure_time->denominator = 29520;
	pr_info("%s: exposure_lines = %lu\n", __func__, exposure_time->numerator);

	return 0;
}

static long ov5640_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ov5640_info *info = file->private_data;

	switch (cmd) {
		case OV5640_IOCTL_SET_MODE:
		{
			struct ov5640_mode mode;
			if (copy_from_user(&mode,
					(const void __user *)arg,
					sizeof(struct ov5640_mode)))
				return -EFAULT;

			return ov5640_set_mode(info, &mode);
		}

		case OV5640_IOCTL_GET_STATUS:
			return ov5640_get_status(info);

		case OV5640_IOCTL_AF_TRIGGER:
			if (info->af_initialized)
				return ov5640_af_trigger(info, arg);
			else
				return -1;

		case OV5640_IOCTL_GET_AF_STATUS:
			if (info->af_initialized)
				return ov5640_get_af_status(info);
			else
				return -1;

#if defined(CONFIG_TORCH_TPS61050YZGR)
		case OV5640_IOCTL_SET_FLASH_MODE:
			return ov5640_set_flash_mode(info, arg);

		case OV5640_IOCTL_GET_FLASH_STATUS:
			return ov5640_get_flash_status(info);
#endif

		case OV5640_IOCTL_SET_WHITE_BALANCE:
			return ov5640_set_white_balance(info, arg);

		case OV5640_IOCTL_SET_COLOR_EFFECT:
			return ov5640_set_color_effect(info, arg);

		case OV5640_IOCTL_SET_EXPOSURE:
			return ov5640_set_exposure(info, arg);

		case OV5640_IOCTL_CAPTURE_CMD:
			return ov5640_capture_cmd(info);

		case OV5640_IOCTL_GET_EXPOSURE_TIME:
		{
			struct ov5640_rational exposure_time;
			ov5640_get_exposure_time(info, &exposure_time);
			if (copy_to_user((void __user *)arg,
					&exposure_time,
					sizeof(struct ov5640_rational)))
				return -EFAULT;

			return 0;
		}

		default:
			pr_err("%s: invalid cmd %u\n", __func__, cmd);
			return -EINVAL;
	}

	return 0;
}

static int ov5640_initialize(struct ov5640_info *info)
{
	int err;
	u8 reg_300E;
	u8 high_byte = 0, low_byte = 0;
	u16 chip_id = 0;
	u8 fw_status;
	struct tegra_camera_clk_info clk_info;

	extern void extern_tegra_camera_enable_vi(void);
	extern void extern_tegra_camera_disable_vi(void);
	extern void extern_tegra_camera_clk_set_rate(struct tegra_camera_clk_info *);

	pr_info("%s ++\n", __func__);

	// set MCLK to 24MHz
	clk_info.id = TEGRA_CAMERA_MODULE_VI;
	clk_info.clk_id = TEGRA_CAMERA_VI_SENSOR_CLK;
	clk_info.rate = 24000000;
	extern_tegra_camera_clk_set_rate(&clk_info);

	// turn on MCLK and pull down PWDN pin
	extern_tegra_camera_enable_vi();
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	// read chip_id
	ov5640_read_reg(info->i2c_client, 0x300A, &high_byte);
	ov5640_read_reg(info->i2c_client, 0x300B, &low_byte);

	chip_id = (u16)high_byte << 8 | (u16)low_byte;
	if (chip_id == 0x5640)
		pr_info("%s: chip_id = 0x%04X", __func__, chip_id);
	else {
		pr_err("%s: wrong chip_id = 0x%04X\n", __func__, chip_id);
		return -1;
	}

	// write initial setting
	err = ov5640_write_table(info, mode_initial);
	if (err)
		return err;

	// write 1280x960 key setting
	err = ov5640_write_table(info, mode_table[OV5640_MODE_1280x960]);
	if (err)
		return err;
	info->mode = OV5640_MODE_1280x960;

	// write AF firmware code
	ov5640_write_table(info, af_firmware_code);
	ov5640_read_reg(info->i2c_client, OV5640_AF_FW_STATUS, &fw_status);
	if (fw_status == OV5640_FW_S_IDLE)
		info->af_initialized = 1;
	else {
		pr_err("%s: failed to initialize AF firmware, fw_status = %02X\n",
			__func__, fw_status);
		info->af_initialized = 0;
	}

	// 0x300E[4:3] stands for MIPI TX/RX PHY power down
	// while in MIPI mode, set register 0x300E[4:3] to 2'b11
	// before the PWDN pin is set to high
	ov5640_read_reg(info->i2c_client, 0x300E, &reg_300E);
	reg_300E |= 0x18;
	ov5640_write_reg(info->i2c_client, 0x300E, reg_300E);

	// pull high PWDN pin and turn off MCLK
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	extern_tegra_camera_disable_vi();

	pr_info("%s --\n", __func__);
	return 0;
}

static struct ov5640_info *info;

static int ov5640_open(struct inode *inode, struct file *file)
{
	u8 reg_300E;

	pr_info("%s\n", __func__);

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	// 0x300E[4:3] stands for MIPI TX/RX PHY power down
	// set register 0x300E[4:3] to 2'b00
	ov5640_read_reg(info->i2c_client, 0x300E, &reg_300E);
	reg_300E &= ~0x18;
	ov5640_write_reg(info->i2c_client, 0x300E, reg_300E);

	return 0;
}

static int ov5640_release(struct inode *inode, struct file *file)
{
	u8 reg_300E;

	pr_info("%s\n", __func__);

	// release VCM to reduce power consumption
	ov5640_write_reg(info->i2c_client, OV5640_AF_CMD_MAIN, OV5640_RELEASE_FOCUS);

#if defined(CONFIG_TORCH_TPS61050YZGR)
	tps61050_turn_off_flash();
#endif

	// 0x300E[4:3] stands for MIPI TX/RX PHY power down
	// while in MIPI mode, set register 0x300E[4:3] to 2'b11
	// before the PWDN pin is set to high
	ov5640_read_reg(info->i2c_client, 0x300E, &reg_300E);
	reg_300E |= 0x18;
	ov5640_write_reg(info->i2c_client, 0x300E, reg_300E);

	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	file->private_data = NULL;

	return 0;
}

static const struct file_operations ov5640_fileops = {
	.owner          = THIS_MODULE,
	.open           = ov5640_open,
	.unlocked_ioctl = ov5640_ioctl,
	.release        = ov5640_release,
};

static struct miscdevice ov5640_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ov5640",
	.fops = &ov5640_fileops,
};

static int ov5640_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

	pr_info("%s ++\n", __func__);

	info = kzalloc(sizeof(struct ov5640_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	if (ov5640_initialize(info) != 0) {
		kfree(info);
		return -ENODEV;
	}

	err = misc_register(&ov5640_device);
	if (err) {
		pr_err("%s: unable to register misc device\n", __func__);
		kfree(info);
		return err;
	}

	i2c_set_clientdata(client, info);

	pr_info("%s --\n", __func__);
	return 0;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640_info *info;

	pr_info("%s\n", __func__);

	info = i2c_get_clientdata(client);
	misc_deregister(&ov5640_device);
	kfree(info);

	return 0;
}

static const struct i2c_device_id ov5640_id[] = {
	{ "ov5640", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver   = {
		.name  = "ov5640",
		.owner = THIS_MODULE,
	},
	.probe    = ov5640_probe,
	.remove   = ov5640_remove,
	.id_table = ov5640_id,
};

static int __init ov5640_init(void)
{
	pr_info("%s\n", __func__);

	return i2c_add_driver(&ov5640_i2c_driver);
}

static void __exit ov5640_exit(void)
{
	pr_info("%s\n", __func__);

	i2c_del_driver(&ov5640_i2c_driver);
}

module_init(ov5640_init);
module_exit(ov5640_exit);
