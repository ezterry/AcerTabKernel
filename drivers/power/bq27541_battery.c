/*
 * bq27541 battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/bq27541.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include "../../arch/arm/mach-tegra/gpio-names.h"
#if defined(CONFIG_ARCH_ACER_T30)
#include "../../arch/arm/mach-tegra/board-acer-t30.h"
extern int acer_board_id;
extern int acer_board_type;
#endif

#define DRIVER_VERSION			"1.4.7"

#ifdef CONFIG_BATTERY_BQ27541

#define BQ27541_REG_TEMP		0x06
#define BQ27541_REG_VOLT		0x08
#define BQ27541_REG_AI			0x14
#define BQ27541_REG_FLAGS		0x0A
#define BQ27541_REG_TTE		0x16
#define BQ27541_REG_TTF		0x18
#define BQ27541_REG_TTECP		0x26
#define BQ27541_REG_SOC		0x2c
#define BQ27541_REG_RSOC		0x0B

#define BQ27541_CNTL			0x00
#define BQ27541_ATRATE			0x02
#define BQ27541_ENERGY_AVAIL		0x22
#define BQ27541_POWER_AVG		0x24
#define BQ27541_CYCLE_COUNT		0x2a
#define BQ27541_DESIGN_CAPACITY	0x3c

#define BQ27541_HIBERNATE_I	0x4b
#define BQ27541_HIBERNATE_V	0x4d

/* Control status bit for sealed/full access sealed */
#define BQ27541_CNTL_SS		BIT(13)
#define BQ27541_CNTL_FAS		BIT(14)

/* FLAGS register bit definition*/
#define BQ27541_FLAG_DSG		BIT(0)
#define BQ27541_FLAG_SOCF		BIT(1)
#define BQ27541_FLAG_FC		BIT(9)
#define BQ27541_FLAG_CHGS		BIT(8)
#define BQ27541_FLAG_OTC		BIT(15)

/* Control register sub-commands */
#define BQ27541_CNTL_DEVICE_TYPE	0x0001
#define BQ27541_CNTL_SET_SLEEP		0x0013
#define BQ27541_CNTL_CLEAR_SLEEP	0x0014
#define BQ27541_CNTL_SET_SEALED	0x0020
#define BQ27541_CNTL_FULL_RESET	0x0041

/* Define battery poll /irq period */
#define BATTERY_POLL_PERIOD		30000
#define BATTERY_FAST_POLL_PERIOD	500

/* Define AC_OUT delay time */
#define DEBOUNCE	80
#define DVT_DELAY	3500
#define PVT_DELAY	1000

/* Define low temperature threshold */
#define TEMP_EQUAL_ZERO		0
#define TEMP_UNDER_ZERO		1
#define TEMP_UNDER_NAT_TEN		2

/* Define charger report signal */
#define ADAPTER_PLUG_IN		1
#define ADAPTER_PULL_OUT		0

#define BATT_LEARN_GPIO	TEGRA_GPIO_PX6
#define CHARGING_FULL	TEGRA_GPIO_PW5

struct i2c_client *bat_client;
bool AC_IN = false;
int design_capacity = 0;
int bat_temp = 0;
int Capacity = 0;
int old_rsoc = 0;
int counter = 0;
int gpio = 0;


struct bq27541_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27541_device_info *di);
};

struct bq27541_device_info {
	struct device			*dev;
	struct bq27541_access_methods	*bus;
	struct power_supply		bat;
	struct power_supply		ac;
	struct timer_list		battery_poll_timer;
	struct i2c_client		*client;
	struct bq27541_platform_data	*plat_data;
	int				id;
	int				irq;
};

static enum power_supply_property bq27541_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};


static int bat_i2c_read(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	return di->bus->read(reg, rt_value, b_single, di);
}


/*
 * Create Sysfs debug console.
 * File path: /sys/BatControl
 */
static struct kobject *bq27541_dbg_kobj;

#define debug_attr(_name) \
	static struct kobj_attribute _name##_attr = { \
	.attr = { \
	.name = __stringify(_name), \
	.mode = 0644, \
	}, \
	.show = _name##_show, \
	}


static ssize_t BatHealth_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_FLAGS);
	msleep(10);

	if (ret & BQ27541_FLAG_SOCF)
		s += sprintf(s, "DEAD\n");
	else if (ret & BQ27541_FLAG_OTC)
		s += sprintf(s, "OVERHEAT\n");
	else
		s += sprintf(s, "GOOD\n");

	msleep(100);

	return (s - buf);

}

static ssize_t BatTemperature_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_TEMP);
	msleep(10);
	s += sprintf(s, "%d\n", ret - 2731 );

	return (s - buf);
}

static ssize_t BatVoltage_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_VOLT);
	msleep(10);
	s += sprintf(s, "%dmV\n", ret);

	return (s - buf);
}

static ssize_t BatCurrent_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_AI);
	msleep(10);
	s += sprintf(s, "%dmA\n", ret * 1000);

	return (s - buf);
}
static ssize_t BatCapacity_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_SOC);
	msleep(10);
	s += sprintf(s, "%d\n", ret);

	return (s - buf);
}

static ssize_t BatStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_REG_FLAGS);
	msleep(10);

	if (ret & BQ27541_FLAG_DSG)
		s += sprintf(s, "DISCHARGING\n");
	else
		s += sprintf(s, "CHARGING\n");

	if (ret & BQ27541_FLAG_FC)
		s += sprintf(s, "FULL\n");

	return (s - buf);
}

static ssize_t BatPowerAVG_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_POWER_AVG);
	msleep(10);
	s += sprintf(s, "%dmW\n", ret);

	return (s - buf);
}

static ssize_t BatEnergyNow_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_ENERGY_AVAIL);
	msleep(10);
	s += sprintf(s, "%dmWh\n", ret * (10 * 1000));

	return (s - buf);
}

static ssize_t SealStatus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	int ret;

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL);
	msleep(10);
	s += sprintf(s, "FAS = %x , SS = %x\n",
		(int)(ret & BQ27541_CNTL_FAS), (int)(ret & BQ27541_CNTL_SS));
	msleep(10);
	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "Reset status = %d\n", ret);
	msleep(10);

	return (s - buf);
}

static ssize_t GaugeReset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret;

	/* write PASSWARD registry procedure : 04143672 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
	s += sprintf(s, "write 1st pw = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
	s += sprintf(s, "write 2nd pw = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "write resest registry = %d\n", ret);
	msleep(10);

	ret = i2c_smbus_read_word_data(bat_client, BQ27541_CNTL_FULL_RESET);
	s += sprintf(s, "Reset status = %d\n", ret);
	msleep(10);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(10);

	return (s - buf);
}

static ssize_t ChargerReset_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;

	tegra_gpio_enable(BATT_LEARN_GPIO);
	gpio_request(BATT_LEARN_GPIO, "batt_learn");

	s += sprintf(s, "BATT_LEARN_GPIO = %d\n", gpio_get_value(BATT_LEARN_GPIO));
	gpio_direction_output(BATT_LEARN_GPIO, 1);
	msleep(500);
	s += sprintf(s, "BATT_LEARN_GPIO = %d\n", gpio_get_value(BATT_LEARN_GPIO));
	gpio_direction_output(BATT_LEARN_GPIO, 0);
	s += sprintf(s, "BATT_LEARN_GPIO = %d\n", gpio_get_value(BATT_LEARN_GPIO));

	return (s - buf);
}

static ssize_t CheckSYHI_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, hi, hv;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	s += sprintf(s, "write SY pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	s += sprintf(s, "write SY pw2 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x53);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	ret = i2c_smbus_read_byte_data(bat_client, 0x40);
	s += sprintf(s, "Chem ID = %x\n", ret);
	msleep(50);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(50);

	return (s - buf);
}

static ssize_t CheckLGHI_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, hi, hv;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
	s += sprintf(s, "write LG pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
	s += sprintf(s, "write LG pw2 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x38);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	ret = i2c_smbus_read_byte_data(bat_client, 0x44);
	s += sprintf(s, "FIRMWARE = %x\n", ret);
	msleep(50);

	ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", ret);
	msleep(200);
	ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", ret);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
	s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
	msleep(50);

	return (s - buf);
}


static ssize_t DisableHibernate_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s32 ret, BDC, DFC, DFB, hi, hv;
	int sum = 0;
	int cheksum = 0;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	s += sprintf(s, "write SY pw1 = %d\n", ret);
	msleep(1000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	s += sprintf(s, "write SY pw2 = %d\n", ret);
	msleep(1000);

	BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
	s += sprintf(s, "0x61 = %d\n", BDC);
	msleep(200);
	DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
	s += sprintf(s, "0x3e = %d\n", DFC);
	msleep(200);
	DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
	s += sprintf(s, "0x3f = %d\n", DFB);

	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
	msleep(50);

	if(BDC < 0 || DFC < 0){
		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		s += sprintf(s, "clean SY pw1 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		s += sprintf(s, "clean SY pw2 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
		s += sprintf(s, "write LG pw1 = %d\n", ret);
		msleep(1000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
		s += sprintf(s, "write LG pw2 = %d\n", ret);
		msleep(1000);

		BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", BDC);
		msleep(200);
		DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", DFC);
		msleep(200);
		DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", DFB);

		hi = i2c_smbus_read_word_data(bat_client, 0x4b);
		s += sprintf(s, "Hibernate I (0x4b) = %x\n", hi);
		msleep(50);
		hv = i2c_smbus_read_word_data(bat_client, 0x4d);
		s += sprintf(s, "Hibernate V (0x4d) = %x\n", hv);
		msleep(50);
	}

	if ((hi == 0) && (hv == 0))
	{
		s += sprintf(s, "Hibernate has been set!\n");
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
		msleep(50);
	}
	else
	{
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", ret);

		ret = i2c_smbus_read_byte_data(bat_client, 0x40);
		s += sprintf(s, "=========== 0x40 = %x =========\n", ret);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x41);
		s += sprintf(s, "=========== 0x41 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x42);
		s += sprintf(s, "=========== 0x42 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x43);
		s += sprintf(s, "=========== 0x43 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x44);
		s += sprintf(s, "=========== 0x44 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x45);
		s += sprintf(s, "=========== 0x45 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x46);
		s += sprintf(s, "=========== 0x46 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x47);
		s += sprintf(s, "=========== 0x47 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x48);
		s += sprintf(s, "=========== 0x48 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x49);
		s += sprintf(s, "=========== 0x49 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4a);
		s += sprintf(s, "=========== 0x4a = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4b);
		s += sprintf(s, "=========== 0x4b = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4c);
		s += sprintf(s, "=========== 0x4c = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4d);
		s += sprintf(s, "=========== 0x4d = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4e);
		s += sprintf(s, "=========== 0x4e = %x =========\n", ret);
		msleep(50);
		//sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4f);
		s += sprintf(s, "=========== 0x4f = %x =========\n", ret);
		msleep(50);
		sum += ret;

		ret = i2c_smbus_read_byte_data(bat_client, 0x50);
		s += sprintf(s, "=========== 0x50 = %x =========\n", ret);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x51);
		s += sprintf(s, "=========== 0x51 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x52);
		s += sprintf(s, "=========== 0x52 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x53);
		s += sprintf(s, "=========== 0x53 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x54);
		s += sprintf(s, "=========== 0x54 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x55);
		s += sprintf(s, "=========== 0x55 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x56);
		s += sprintf(s, "=========== 0x56 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x57);
		s += sprintf(s, "=========== 0x57 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x58);
		s += sprintf(s, "=========== 0x58 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x59);
		s += sprintf(s, "=========== 0x59 = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5a);
		s += sprintf(s, "=========== 0x5a = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5b);
		s += sprintf(s, "=========== 0x5b = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5c);
		s += sprintf(s, "=========== 0x5c = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5d);
		s += sprintf(s, "=========== 0x5d = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5e);
		s += sprintf(s, "=========== 0x5e = %x =========\n", ret);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5f);
		s += sprintf(s, "=========== 0x5f = %x =========\n", ret);
		msleep(50);
		sum += ret;
		s += sprintf(s, "=========== sum = %x =========\n", sum);
		ret = 255 - (sum % 256);
		cheksum = ret;
		s += sprintf(s, "=========== CHECKSUM = %x =========\n", ret);

		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);	//BlockDataControl(): 0x61
		s += sprintf(s, "0x61 = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);	//DataFlashClass(): 0x3e
		s += sprintf(s, "0x3e = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);	//DataFlashBlock(): 0x3f
		s += sprintf(s, "0x3f = %d\n", ret);

		ret = i2c_smbus_write_byte_data(bat_client, 0x4b, 0x00);
		s += sprintf(s, "Clear Hibernate I (0x4b) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4c, 0x00);
		s += sprintf(s, "Clear Hibernate I (0x4c) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4d, 0x00);
		s += sprintf(s, "Clear Hibernate V (0x4d) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4e, 0x00);
		s += sprintf(s, "Clear Hibernate V (0x4e) = %d\n", ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x60, cheksum);
		s += sprintf(s, "Checksum (0x60) = %x,\n", cheksum);
		msleep(200);

		ret = i2c_smbus_read_word_data(bat_client, 0x4b);
		s += sprintf(s, "Hibernate I (0x4b) = %x\n", ret);
		msleep(50);
		ret = i2c_smbus_read_word_data(bat_client, 0x4d);
		s += sprintf(s, "Hibernate V (0x4d) = %x\n", ret);
		msleep(50);

		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		s += sprintf(s, "Change to Sealed Mode = %d\n", ret);
		msleep(50);

	}
	return (s - buf);
}


debug_attr(BatHealth);
debug_attr(BatTemperature);
debug_attr(BatVoltage);
debug_attr(BatCurrent);
debug_attr(BatCapacity);
debug_attr(BatStatus);
debug_attr(BatPowerAVG);
debug_attr(BatEnergyNow);
debug_attr(SealStatus);
debug_attr(GaugeReset);
debug_attr(ChargerReset);
debug_attr(DisableHibernate);
debug_attr(CheckSYHI);
debug_attr(CheckLGHI);



static struct attribute * g[] = {
	&BatHealth_attr.attr,
	&BatTemperature_attr.attr,
	&BatVoltage_attr.attr,
	&BatCurrent_attr.attr,
	&BatCapacity_attr.attr,
	&BatStatus_attr.attr,
	&BatPowerAVG_attr.attr,
	&BatEnergyNow_attr.attr,
	&SealStatus_attr.attr,
	&GaugeReset_attr.attr,
	&ChargerReset_attr.attr,
	&DisableHibernate_attr.attr,
	&CheckSYHI_attr.attr,
	&CheckLGHI_attr.attr,
	NULL,
};

static struct attribute_group dbg_attr_group =
{
	.attrs = g,
};

static void bq27541_disable_hibernate(void)
{
	s32 ret, BDC, DFC, DFB, hi, hv;
	int sum = 0;
	int cheksum = 0;

	/* write PASSWARD registry procedure : LG:04143672 / SY:16406303 */
	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x40);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x16);
	printk(KERN_INFO "%s -- write SY pw1 = %d\n",__func__ ,ret);
	msleep(2000);

	ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x03);
	ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x63);
	printk(KERN_INFO "%s -- write SY pw2 = %d\n",__func__ ,ret);
	msleep(2000);

	/* write data lash block procedure */
	BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
	printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
	msleep(500);
	DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
	printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
	msleep(500);
	DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
	printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
	msleep(500);

	/* read Hibernate I&V status */
	hi = i2c_smbus_read_word_data(bat_client, 0x4b);
	printk(KERN_INFO "%s -- Hibernate I (0x4b) = %d\n",__func__ ,hi);
	msleep(50);
	hv = i2c_smbus_read_word_data(bat_client, 0x4d);
	printk(KERN_INFO "%s -- Hibernate V (0x4d) = %d\n",__func__ ,hv);
	msleep(50);

	if(BDC < 0 || DFC < 0){
		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		printk(KERN_INFO "%s -- clean SY pw1 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x00);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x00);
		printk(KERN_INFO "%s -- clean SY pw2 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x14);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x04);
		printk(KERN_INFO "%s -- write LG pw1 = %d\n",__func__ ,ret);
		msleep(2000);

		ret = i2c_smbus_write_byte_data(bat_client, 0x00, 0x72);
		ret = i2c_smbus_write_byte_data(bat_client, 0x01, 0x36);
		printk(KERN_INFO "%s -- write LG pw2 = %d\n",__func__ ,ret);
		msleep(2000);

		/* write data lash block procedure */
		BDC = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,BDC);
		msleep(500);
		DFC = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,DFC);
		msleep(500);
		DFB = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,DFB);
		msleep(500);

		/* read Hibernate I&V status */
		hi = i2c_smbus_read_word_data(bat_client, 0x4b);
		printk(KERN_INFO "%s -- Hibernate I (0x4b) = %x\n",__func__ ,hi);
		msleep(50);
		hv = i2c_smbus_read_word_data(bat_client, 0x4d);
		printk(KERN_INFO "%s -- Hibernate V (0x4d) = %x\n",__func__ ,hv);
		msleep(50);
	}

	if ((hi == 0) && (hv == 0))
	{
		printk(KERN_INFO "%s -- Hibernate has been set!\n",__func__ );
		/* set bq27541 to SEALED mode */
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		printk(KERN_INFO "%s -- Change to Sealed Mode = %d\n",__func__ ,ret);
		msleep(50);
	}
	else
	{
		/* write data lash block procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
		msleep(200);

		/* calculate data block checksum */
		ret = i2c_smbus_read_byte_data(bat_client, 0x40);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x41);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x42);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x43);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x44);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x45);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x46);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x47);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x48);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x49);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4a);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x4b);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4c);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4d);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4e);
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x4f);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x50);
		sum += ret;
		msleep(50);
		ret = i2c_smbus_read_byte_data(bat_client, 0x51);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x52);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x53);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x54);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x55);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x56);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x57);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x58);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x59);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5a);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5b);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5c);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5d);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5e);
		msleep(50);
		sum += ret;
		ret = i2c_smbus_read_byte_data(bat_client, 0x5f);
		msleep(50);
		sum += ret;
		cheksum = 255 - (sum % 256);
		printk(KERN_INFO "%s -- CHECKSUM = %x\n",__func__ ,ret);

		/* write data lash block procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x61, 0x00);
		printk(KERN_INFO "%s -- write BlockDataControl() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3e, 0x44);
		printk(KERN_INFO "%s -- write DataFlashClass() = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x3f, 0x00);
		printk(KERN_INFO "%s -- write DataFlashBlock() = %d\n",__func__ ,ret);
		msleep(200);

		/* clear Hibernate I&V procedure */
		ret = i2c_smbus_write_byte_data(bat_client, 0x4b, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate I (0x4b) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4c, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate I (0x4c) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4d, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate V (0x4d) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x4e, 0x00);
		printk(KERN_INFO "%s -- Clear Hibernate V (0x4e) = %d\n",__func__ ,ret);
		msleep(200);
		ret = i2c_smbus_write_byte_data(bat_client, 0x60, cheksum);
		printk(KERN_INFO "%s -- Checksum (0x60) = %x\n",__func__ ,cheksum);
		msleep(200);

		/* confirm Hibernate I&V status : 0 */
		ret = i2c_smbus_read_word_data(bat_client, 0x4b);
		printk(KERN_INFO "%s -- Hibernate I (0x4b) = %x\n",__func__ ,ret);
		msleep(50);
		ret = i2c_smbus_read_word_data(bat_client, 0x4d);
		printk(KERN_INFO "%s -- Hibernate V (0x4d) = %x\n",__func__ ,ret);
		msleep(50);

		/* set bq27541 to SEALED mode */
		ret = i2c_smbus_write_word_data(bat_client, BQ27541_CNTL, BQ27541_CNTL_SET_SEALED);
		printk(KERN_INFO "%s -- Change to Sealed Mode = %d\n",__func__ ,ret);
		msleep(50);
	}
}

static void bq27541_charger_reset(void)
{
	tegra_gpio_enable(BATT_LEARN_GPIO);
	gpio_request(BATT_LEARN_GPIO, "batt_learn");
	gpio_direction_output(BATT_LEARN_GPIO, 1);
	msleep(100);
	gpio_direction_output(BATT_LEARN_GPIO, 0);
}

int bq27541_battery_check(int arg)
{
	int ret;
	int rsoc = 0;

	struct bq27541_device_info *di = i2c_get_clientdata(bat_client);
	if (!bat_client)
		dev_err(di->dev, "error getting bat_client\n");

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(50);

	switch (arg) {
	case 1:
		return rsoc;
		break;
	case 2:
		if(rsoc < 100)
		{
			if(gpio_get_value(gpio))
				return POWER_SUPPLY_STATUS_CHARGING;
			else
				return POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else
		{
			if(gpio_get_value(gpio))
				return POWER_SUPPLY_STATUS_FULL;
			else
				return POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		break;
	default:
		return -EINVAL;
	}

}
EXPORT_SYMBOL(bq27541_battery_check);

int bq27541_low_temp_check(void)
{
	int ret, rsoc, temp;
	struct bq27541_device_info *di = i2c_get_clientdata(bat_client);

	if (!bat_client)
		dev_err(di->dev, "error getting bat_client\n");

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(50);
	ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
	msleep(50);

	if((rsoc < 60) && (temp < 2744) && (temp > 2647))
		return TEMP_UNDER_ZERO;
	else if((rsoc < 40) && (temp < 2647) && (temp > 2560))
		return TEMP_UNDER_NAT_TEN;
	else
		return TEMP_EQUAL_ZERO;
}
EXPORT_SYMBOL(bq27541_low_temp_check);

static int bq27541_battery_health(struct bq27541_device_info *di)
{
	int ret;
	int status = 0;

	ret = bat_i2c_read(BQ27541_REG_FLAGS, &status, 0, di);
	msleep(50);

	if (ret  < 0) {
		dev_err(di->dev, "read failure\n");
		return ret;
	}

	if (ret & BQ27541_FLAG_SOCF)
		status = POWER_SUPPLY_HEALTH_DEAD;
	else if (ret & BQ27541_FLAG_OTC)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;

}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int bq27541_battery_temperature(struct bq27541_device_info *di)
{
	int ret;
	int temp = 0;
	int report = temp - 2731;

	ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
	msleep(50);

	report = temp - 2731;
	bat_temp = report;
	if (ret < 0) {
		dev_err(di->dev, "error reading temperature\n");
		return ret;
	}

	if((report == 0) || (report > 680)){
		dev_err(di->dev, "1st error temperature value\n");
		ret = bat_i2c_read(BQ27541_REG_TEMP, &temp, 0, di);
		msleep(200);
		if((report == 0) || (report > 680)){
			dev_err(di->dev, "2nd error temperature value\n");
			bat_temp = 273;
		}
		else
			bat_temp = temp - 2731;
	}
	else
		bat_temp = temp - 2731;

	return bat_temp;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27541_battery_voltage(struct bq27541_device_info *di)
{
	int ret;
	int volt = 0;

	ret = bat_i2c_read(BQ27541_REG_VOLT, &volt, 0, di);
	msleep(50);

	if (ret < 0) {
		dev_err(di->dev, "error reading voltage\n");
		return ret;
	}

	return volt * 1000;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27541_battery_current(struct bq27541_device_info *di)
{
	int ret;
	int curr = 0;

	ret = bat_i2c_read(BQ27541_REG_AI, &curr, 0, di);
	msleep(50);

	if (ret < 0) {
		dev_err(di->dev, "error reading current\n");
		return 0;
	}
	curr = (int)(s16)curr;

	return curr * 1000;
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27541_battery_rsoc(struct bq27541_device_info *di)
{
	int ret;
	int rsoc = 0;

	ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
	msleep(50);

	if (ret < 0) {
		dev_err(di->dev, "error reading relative State-of-Charge\n");
		return ret;
	}

	Capacity = rsoc;
	if((rsoc == 0) || (rsoc > 100)){
		dev_err(di->dev, "1st error capacity value\n");
		ret = bat_i2c_read(BQ27541_REG_SOC, &rsoc, 0, di);
		msleep(200);
		if((rsoc == 0) || (rsoc > 100)){
			dev_err(di->dev, "2nd error capacity value\n");
			Capacity = old_rsoc;
		}
		else
			Capacity = rsoc;
	}
	else
		Capacity = rsoc;

	if(bq27541_low_temp_check() > TEMP_EQUAL_ZERO)
		return NULL;
	else
	{
		/* Capacity mapping for 5% preserved engrgy */
		if (rsoc <= 23 && rsoc > 15)
			return (Capacity - 2);
		else if (rsoc <= 15 && rsoc > 11)
			return (Capacity - 3);
		else if (rsoc <= 11 && rsoc > 4)
			return (Capacity - 4);
		else if (rsoc <= 4){
			Capacity = 0;
			return 0;
		}
		else
			return Capacity;
	}

}

static int bq27541_battery_status(struct bq27541_device_info *di)
{
	int ret;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;

	ret = bat_i2c_read(BQ27541_REG_FLAGS, &status, 0, di);
	msleep(50);

	if (ret < 0) {
		dev_err(di->dev, "error reading flags\n");
		return ret;
	}

	ret = bat_i2c_read(BQ27541_REG_SOC, &Capacity, 0, di);
	msleep(50);

	if (ret < 0) {
		dev_err(di->dev, "error reading capacity\n");
		return ret;
	}

	if(bq27541_low_temp_check() > TEMP_EQUAL_ZERO)
		status = POWER_SUPPLY_STATUS_UNKNOWN;
	else
	{
		if(Capacity < 100)
		{
			if(gpio_get_value(gpio)){
				gpio_direction_output(CHARGING_FULL, 1);
				status = POWER_SUPPLY_STATUS_CHARGING;
			}
			else
				status = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		else
		{
			if(gpio_get_value(gpio)){
				gpio_direction_output(CHARGING_FULL, 1);
				status = POWER_SUPPLY_STATUS_FULL;
			}
			else
				status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}
	return status;

}

static int bq27541_battery_present(struct bq27541_device_info *di)
{
	int ret;
	int present = 0;

	/* Calculate counter for limitation of 10.5hr charger reset */
	if(counter <= 120){
		if(gpio_get_value(gpio))
			counter += 1;
		else
			counter = 0;
	}
	else{
		bq27541_charger_reset();
		counter = 0;
	}

	ret = bat_i2c_read(BQ27541_DESIGN_CAPACITY, &present, 0, di);
	msleep(50);
	design_capacity = present;

	if (ret >= 0){
		return 1;
	}
	else {
		dev_err(di->dev, "No Battery due to error reading design capacity! \n");
		return 0;
	}
}


#define bat_to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, bat);

static int bq27541_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	int ret = 0;
	struct bq27541_device_info *di = bat_to_bq27541_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq27541_battery_status(di);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq27541_battery_present(di);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq27541_battery_voltage(di);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq27541_battery_current(di);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq27541_battery_rsoc(di);
		old_rsoc = val->intval;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq27541_battery_temperature(di);
		printk("AC_IN = %d, Capacity = %d, ROSC = %d, Temperature = %d\n",
			gpio_get_value(gpio), old_rsoc, bq27541_battery_check(1), val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq27541_battery_health(di);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = design_capacity;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

#define ac_to_bq27541_device_info(x) container_of((x), \
				struct bq27541_device_info, ac);

static enum power_supply_property bq27541_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *ac_power_supplied_to[] = {
	"bq27541-bat",
};

static int bq27541_get_ac_status(void)
{
	if(gpio_get_value(gpio))
		return ADAPTER_PLUG_IN;
	else
		return ADAPTER_PULL_OUT;
}

static int bq27541_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct bq27541_device_info *di = ac_to_bq27541_device_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq27541_get_ac_status();
		break;
	default:
		dev_err(&di->client->dev, "%s: INVALID property\n", __func__);
		return -EINVAL;
	}
	return 0;
}

static void bq27541_powersupply_init(struct bq27541_device_info *di)
{
	di->bat.name = "bq27541-bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27541_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27541_battery_props);
	di->bat.get_property = bq27541_battery_get_property;
	di->bat.external_power_changed = NULL;

	di->ac.name = "bq27541-ac";
	di->ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->ac.supplied_to = ac_power_supplied_to;
	di->ac.num_supplicants = ARRAY_SIZE(ac_power_supplied_to);
	di->ac.properties = bq27541_ac_props;
	di->ac.num_properties = ARRAY_SIZE(bq27541_ac_props);
	di->ac.get_property = bq27541_ac_get_property;
}

/*
 * i2c specific code
 */
static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single,
			struct bq27541_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[2];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;

	data[0] = reg;
	msg[0].buf = data;

	msg[1].addr = client->addr;
	msg[1].buf = data;

		if (!b_single)
			msg[1].len = 2;
		else
			msg[1].len = 1;

		msg[1].flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 2);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}

	return err;
}


static irqreturn_t ac_present_irq(int irq, void *data)
{
	struct bq27541_device_info *di = data;
	/* Debounce with 80ms to block abnormal interrupt when AC plug out */
	mdelay(DEBOUNCE);
	power_supply_changed(&di->ac);
	return IRQ_HANDLED;
}

static void bq27541_poll_timer_func(unsigned long pdi)
{
	struct bq27541_device_info *di = (void *)pdi;
	power_supply_changed(&di->bat);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));
}

static int bq27541_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;
	struct bq27541_access_methods *bus;
	int retval = 0;
	int err;

	printk(KERN_INFO "%s ++ \n",__func__);
	bat_client = client;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		dev_err(&client->dev, "failed to allocate device info data\n");
		return -ENOMEM;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		dev_err(&client->dev, "failed to allocate access method data\n");
		retval = -ENOMEM;
		goto failed_device;
	}

	gpio = irq_to_gpio(client->irq);
	AC_IN = gpio_get_value(gpio);
	di->irq = client->irq;

	tegra_gpio_enable(CHARGING_FULL);
	gpio_request(CHARGING_FULL, "full_half_chg");

	if (client->dev.platform_data) {
		di->plat_data = kzalloc(sizeof(struct bq27541_platform_data), GFP_KERNEL);
		if (!di->plat_data) {
			dev_err(&client->dev, "failed to allocate platform data\n");
			retval = -ENOMEM;
			goto failed_bus;
		}
		memcpy(di->plat_data, client->dev.platform_data, sizeof(struct bq27541_platform_data));
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	bus->read = &bq27541_read_i2c;
	di->bus = bus;
	di->client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "insufficient functionality!\n");
		retval = -ENODEV;
		goto failed_pdata;
	}

	bq27541_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		dev_err(&client->dev, "failed to register battery\n");
		goto failed_pdata;
	}

	setup_timer(&di->battery_poll_timer, bq27541_poll_timer_func, (unsigned long) di);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_POLL_PERIOD));

	retval = power_supply_register(&client->dev, &di->ac);
	if (retval) {
		dev_err(&client->dev, "failed to register ac power supply\n");
		goto failed_reg_bat;
	}

	retval = request_threaded_irq(di->irq, NULL, ac_present_irq, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, "ac_present", di);
	if (retval < 0) {
		dev_err(&di->client->dev, "%s: request_irq failed(%d)\n", __func__, retval);
		goto failed_reg_ac;
	}

	bq27541_dbg_kobj = kobject_create_and_add("BatControl", NULL);
	if (bq27541_dbg_kobj == NULL)
	{
		dev_err(&di->client->dev,"%s: subsystem_register failed\n", __FUNCTION__);
	}
	err = sysfs_create_group(bq27541_dbg_kobj, &dbg_attr_group);
	if(err)
	{
		dev_err(&di->client->dev,"%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

#if defined(CONFIG_ARCH_ACER_T30)
	if ( (acer_board_type == BOARD_PICASSO_2 || acer_board_type == BOARD_PICASSO_M ) &&
		(acer_board_id == BOARD_EVT || acer_board_id == BOARD_DVT1 || acer_board_id == BOARD_DVT2)) {
		bq27541_disable_hibernate();
	}
#endif

	return 0;

failed_reg_ac:
	power_supply_unregister(&di->ac);
failed_reg_bat:
	power_supply_unregister(&di->bat);
	del_timer_sync(&di->battery_poll_timer);
failed_pdata:
	kfree(di->plat_data);
failed_bus:
	kfree(bus);
failed_device:
	kfree(di);

	return retval;
}

static int bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	free_irq(di->irq, di);
	power_supply_unregister(&di->ac);
	power_supply_unregister(&di->bat);
	del_timer_sync(&di->battery_poll_timer);

	kfree(di->plat_data);
	kfree(di->bus);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27541_battery_suspend(struct i2c_client *client,
	pm_message_t state)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	if(gpio_get_value(gpio))
		gpio_direction_output(CHARGING_FULL, 0);
	del_timer_sync(&di->battery_poll_timer);
	enable_irq_wake(client->irq);
	return 0;
}

static int bq27541_battery_resume(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);
	setup_timer(&di->battery_poll_timer, bq27541_poll_timer_func, (unsigned long) di);
	mod_timer(&di->battery_poll_timer, jiffies + msecs_to_jiffies(BATTERY_FAST_POLL_PERIOD));
	disable_irq_wake(client->irq);
	return 0;
}
#endif


static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
#if defined(CONFIG_PM)
	.suspend	= bq27541_battery_suspend,
	.resume		= bq27541_battery_resume,
#endif
	.id_table = bq27541_id,
	.driver = {
		.name = "bq27541-battery",
	},
};

static int __init bq27541_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		printk(KERN_ERR "Fail to register bq27541 driver\n");

	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("StanleyTW Chang <StanleyTW_Chang@acer.com.tw>");
MODULE_DESCRIPTION("bq27541 battery driver");
MODULE_LICENSE("GPL");

#endif /* CONFIG_BATTERY_BQ27541 */

