#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"
#include "../../../arch/arm/mach-tegra/board-acer-t30.h"

extern int acer_board_type;
extern int acer_board_id;
extern int acer_sku;

static struct kobject *modem_control_kobj;
static int en_modem_3v3_gpio = TEGRA_GPIO_PU1;

static ssize_t power_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	int i;
	char *s = buf;

	i = gpio_get_value(en_modem_3v3_gpio);
	s += sprintf(s, "%d", i);
	return (s - buf);
}

static ssize_t power_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
	int i;

	if(n == 2 && (*buf == 0x30 || *buf == 0x31)){
		i = (int)(*buf) - 0x30;
		gpio_set_value(en_modem_3v3_gpio, i);
		printk(KERN_INFO "modem_power: power enable = %d\n ", i);
	}else{
		printk(KERN_INFO "modem_power: invalid args, n = %d, *buf = %d\n", n, *buf);
	}
	return n;
}

static struct kobj_attribute power_attr = {
	.attr = {
		.name = "power",
		.mode = 0644,
	},
	.show = power_show,
	.store = power_store,
};

static struct attribute * g[] = {
	&power_attr.attr,
	NULL,
};

static struct attribute_group attr_group =
{
	.attrs = g,
};

static int __init init_modem_power(void)
{
	int err, power_enable;

	if(acer_sku != BOARD_SKU_WIFI){
		power_enable = 1;
	}else{
		power_enable = 0;
	}

	if((acer_board_type == BOARD_PICASSO_2) && (acer_board_id <= BOARD_DVT1)){
		en_modem_3v3_gpio = TEGRA_GPIO_PR7;
	}

	modem_control_kobj = kobject_create_and_add("ModemControl", NULL);
	if(modem_control_kobj == NULL){
		printk(KERN_INFO "modem_power: create kobject failed\n");
		goto create_kobject_fail;
	}
	err = sysfs_create_group(modem_control_kobj, &attr_group);
	if(err){
		printk(KERN_INFO "modem_power: sysfs_create_group failed\n");
		goto init_fail;
	}

	tegra_gpio_enable(en_modem_3v3_gpio);
	err = gpio_request(en_modem_3v3_gpio, NULL);
	if(err){
		printk(KERN_INFO "modem_power: gpio %d request failed\n", en_modem_3v3_gpio);
		goto init_fail;
	}

	err = gpio_direction_output(en_modem_3v3_gpio, power_enable);
	if(err){
		printk(KERN_INFO "modem_power: gpio %d set output high failed\n", en_modem_3v3_gpio);
		goto init_fail;
	}
	printk(KERN_INFO "modem_power: gpio %d = %d, init OK\n", en_modem_3v3_gpio, gpio_get_value(en_modem_3v3_gpio));
	return 0;
init_fail:
	kobject_put(modem_control_kobj);
create_kobject_fail:
	return -1;
}

static void __exit cleanup_modem_power(void)
{
	printk(KERN_INFO "modem_power: cleanup modem power\n");
	kobject_put(modem_control_kobj);
}

module_init(init_modem_power);
module_exit(cleanup_modem_power);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luke Lai (luke_lai@acer.com.tw)");
MODULE_DESCRIPTION("modem power control");
