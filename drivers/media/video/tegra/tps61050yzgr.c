/*
 * tps61050yzgr.c - TPS61050YZGR flash/torch kernel driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

struct tps61050_info {
	struct i2c_client *i2c_client;
};

static struct tps61050_info *info = NULL;

#define TPS61050_MAX_RETRIES  3

static int tps61050_write_reg(struct i2c_client *client, u8 addr, u8 val)
{
	int err, retry = 0;
	struct i2c_msg msg;
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) addr;
	data[1] = (u8) val;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;

		retry++;
		pr_err("%s: i2c transfer failed, retrying addr=0x%02X, val=0x%02X\n",
			__func__, addr, val);
		msleep(10);
	} while (retry <= TPS61050_MAX_RETRIES);

	return err;
}

static const struct file_operations tps61050_fileops = {
	.owner = THIS_MODULE,
};

static struct miscdevice tps61050_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "tps61050",
	.fops  = &tps61050_fileops,
};

static int tps61050_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err;

	pr_info("%s ++\n", __func__);

	info = kzalloc(sizeof(struct tps61050_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: unable to allocate memory\n", __func__);
		return -ENOMEM;
	}

	info->i2c_client = client;

	// set each register to reset state
	tps61050_write_reg(info->i2c_client, 0x00, 0x12);
	tps61050_write_reg(info->i2c_client, 0x01, 0x04);
	tps61050_write_reg(info->i2c_client, 0x02, 0x00);
	tps61050_write_reg(info->i2c_client, 0x03, 0xD1);

	err = misc_register(&tps61050_device);
	if (err) {
		pr_err("%s: unable to register misc device\n", __func__);
		kfree(info);
		return err;
	}

	i2c_set_clientdata(client, info);

	pr_info("%s --\n", __func__);
	return 0;
}

static int tps61050_remove(struct i2c_client *client)
{
	struct tps61050_info *info;

	pr_info("%s\n", __func__);

	info = i2c_get_clientdata(client);
	misc_deregister(&tps61050_device);
	kfree(info);

	return 0;
}

static const struct i2c_device_id tps61050_id[] = {
	{ "tps61050", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tps61050_id);

static struct i2c_driver tps61050_i2c_driver = {
	.driver = {
		.name  = "tps61050",
		.owner = THIS_MODULE,
	},
	.probe    = tps61050_probe,
	.remove   = tps61050_remove,
	.id_table = tps61050_id,
};

static int __init tps61050_init(void)
{
	pr_info("%s\n", __func__);

	return i2c_add_driver(&tps61050_i2c_driver);
}

static void __exit tps61050_exit(void)
{
	pr_info("%s\n", __func__);

	i2c_del_driver(&tps61050_i2c_driver);
}

void tps61050_turn_on_torch(void)
{
	pr_info("%s\n", __func__);

	// torch only mode, 100mA
	tps61050_write_reg(info->i2c_client, 0x00, 0x43);
}

void tps61050_turn_off_torch(void)
{
	pr_info("%s\n", __func__);
	tps61050_write_reg(info->i2c_client, 0x00, 0x00);
}


void tps61050_turn_on_flash(void)
{
	pr_info("%s\n", __func__);

	// set STIM to 11111b, Timer = STIM x 32.8ms = 1.02s
	tps61050_write_reg(info->i2c_client, 0x03, 0xDF);

	// torch and flash modes, START FLASH, 900mA
	tps61050_write_reg(info->i2c_client, 0x01, 0x8E);
}

void tps61050_turn_off_flash(void)
{
	pr_info("%s\n", __func__);
	tps61050_write_reg(info->i2c_client, 0x01, 0x00);
}

module_init(tps61050_init);
module_exit(tps61050_exit);
