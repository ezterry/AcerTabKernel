#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include "../../../arch/arm/mach-tegra/gpio-names.h"

#define DRIVER_NAME             "acer-dock"

#define DOCK_HS_GPIO    TEGRA_GPIO_PS6  //Dock headset
#define DOCK_IR_INT     TEGRA_GPIO_PU5  //Dock IR
#define USB_HOST_EN     TEGRA_GPIO_PN1  //USB Vbus

struct dock_switch_data *switch_data;
struct input_dev *input_dock;
static bool is_ir_wake = false;
static bool is_ir_irq_enable = false;

enum {
	NO_DEVICE = 0,
	DOCK_IN = 1,
	DOCK_HEADSET_IN = 2,
	DOCK_IR_WAKE = 3
};

enum {
	DOCK_KEY_ACERRING = 1,
	DOCK_KEY_UNLOCK,
	DOCK_KEY_VOLUMEUP,
	DOCK_KEY_VOLUMEDOWN,
	DOCK_KEY_PLAYPAUSE,
	DOCK_KEY_NEXTSONG,
	DOCK_KEY_PREVIOUSSONG,
	DOCK_KEY_BACK,
	DOCK_KEY_MENU,
	DOCK_KEY_HOME,
	DOCK_KEY_UP,
	DOCK_KEY_DOWN,
	DOCK_KEY_LEFT,
	DOCK_KEY_RIGHT,
	DOCK_KEY_ENTER,
};

static ssize_t dock_show(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf);
static ssize_t dock_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n);
static ssize_t dockin_show(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf);
static ssize_t dockin_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n);
static ssize_t vbus_show(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf);
static ssize_t vbus_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n);

static struct kobject *dock_kobj;

#define SYS_ATTR(_name) \
	static struct kobj_attribute _name##_attr = { \
	.attr = { \
	.name = __stringify(_name), \
	.mode = S_IRUGO|S_IWUSR, \
	}, \
	.show = _name##_show, \
	.store = _name##_store, \
	}

SYS_ATTR(dock);
SYS_ATTR(dockin);
SYS_ATTR(vbus);

static struct attribute * attributes[] = {
	&dock_attr.attr,
	&dockin_attr.attr,
	&vbus_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
        .attrs = attributes,
};

struct dock_switch_data {
	struct switch_dev sdev;
	unsigned det_gpio;
	unsigned det_irq;
	struct work_struct work;
	struct hrtimer timer;
	ktime_t debounce_time;
	unsigned ir_gpio;
	unsigned ir_irq;
	unsigned hs_gpio;
	unsigned hs_irq;
	struct work_struct hs_work;
	struct work_struct ir_work;
	struct hrtimer hs_timer;
	ktime_t hs_debounce_time;
};

static int atoi(const char *a)
{
	int s = 0;
	while(*a >= '0' && *a <= '9')
		s = (s << 3) + (s << 1) + *a++ - '0';
	return s;
}

static int usb3_vbus_enabled = 0;
static void host_vbus_enable(int enable)
{
	int gpio_status;

	if (enable) {
		gpio_status = gpio_request(USB_HOST_EN, "VBUS_USB");
		if (gpio_status < 0) {
			printk("request usb vbus enable gpio failed\n");
			return;
		}
		tegra_gpio_enable(USB_HOST_EN);
		gpio_status = gpio_direction_output(USB_HOST_EN, 1);
		if (gpio_status < 0) {
			printk("set usb vbus enable gpio direction failed\n");
			return;
		}
		gpio_set_value(USB_HOST_EN, 1);
		usb3_vbus_enabled = 1;
		printk(KERN_INFO "dock attached, enable vbus\n");
	} else if (usb3_vbus_enabled) {
		gpio_set_value(USB_HOST_EN, 0);
		gpio_free(USB_HOST_EN);
		usb3_vbus_enabled = 0;
		printk(KERN_INFO "dock detached, disable vbus\n");
	} else {
		printk(KERN_INFO "skip, dock is not attached\n");
	}
}

static ssize_t dock_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s += sprintf(s, "dock_keyevent\n");
	return (s - buf);
}

static ssize_t dock_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n)
{
	int buffer;
	buffer = atoi(buf);

	switch (buffer) {
	case DOCK_KEY_ACERRING:
		input_report_key(input_dock, KEY_F1, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_F1, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_UNLOCK:
		input_report_key(input_dock, KEY_F2, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_F2, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_VOLUMEUP:
		input_report_key(input_dock, KEY_VOLUMEUP, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_VOLUMEUP, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_VOLUMEDOWN:
		input_report_key(input_dock, KEY_VOLUMEDOWN, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_VOLUMEDOWN, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_PLAYPAUSE:
		input_report_key(input_dock, KEY_PLAYPAUSE, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_PLAYPAUSE, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_NEXTSONG:
		input_report_key(input_dock, KEY_NEXTSONG, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_NEXTSONG, 0);
		input_sync(input_dock );
		break;
	case DOCK_KEY_PREVIOUSSONG:
		input_report_key(input_dock, KEY_PREVIOUSSONG, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_PREVIOUSSONG, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_BACK:
		input_report_key(input_dock, KEY_BACK, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_BACK, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_MENU:
		input_report_key(input_dock, KEY_MENU, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_MENU, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_HOME:
		input_report_key(input_dock, KEY_HOME, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_HOME, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_LEFT:
		input_report_key(input_dock, KEY_LEFT, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_LEFT, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_RIGHT:
		input_report_key(input_dock, KEY_RIGHT, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_RIGHT, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_UP:
		input_report_key(input_dock, KEY_UP, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_UP, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_DOWN:
		input_report_key(input_dock, KEY_DOWN, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_DOWN, 0);
		input_sync(input_dock);
		break;
	case DOCK_KEY_ENTER:
		input_report_key(input_dock, KEY_ENTER, 1);
		input_sync(input_dock);
		input_report_key(input_dock, KEY_ENTER, 0);
		input_sync(input_dock);
		break;
	}

	return n;
}

static ssize_t dockin_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s += sprintf(s, "%d\n",gpio_get_value(switch_data->det_gpio));
	return (s - buf);
}

static ssize_t dockin_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n)
{
	return n;
}

static ssize_t vbus_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
	char * s = buf;
	s += sprintf(s, "%d\n",gpio_get_value(USB_HOST_EN));
	return (s - buf);
}

static ssize_t vbus_store(struct kobject* kobj, struct kobj_attribute* kobj_attr, char* buf, size_t n)
{
	int enable;

	if (sscanf(buf, "%d", &enable) > 0) {
		host_vbus_enable(enable);
	} else {
		pr_debug("Get value failed!!\n");
	}

	return 0;
}

static void dock_switch_early_suspend(struct early_suspend *h)
{

}

static void dock_switch_late_resume(struct early_suspend *h)
{

}

static struct early_suspend dock_switch_early_suspend_handler = {
        .suspend = dock_switch_early_suspend,
        .resume = dock_switch_late_resume,
};

static ssize_t switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t switch_print_state(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
		case NO_DEVICE:
			return sprintf(buf, "%s\n", "0");
		case DOCK_IN:
			return sprintf(buf, "%s\n", "1");
		case DOCK_HEADSET_IN:
			return sprintf(buf, "%s\n", "2");
		case DOCK_IR_WAKE:
			return sprintf(buf, "%s\n", "3");
	}
	return -EINVAL;
}

/* Dock Headset */
static void dock_hs_switch_work(struct work_struct *work)
{
	int state,hs_state;
	struct dock_switch_data *pSwitch = switch_data;

	state = gpio_get_value(pSwitch->det_gpio);
	if (!state) {
		hs_state = gpio_get_value(pSwitch->hs_gpio);
		printk("dock_switch hs_gpio=%d\n",hs_state);
		if (!hs_state) {
			switch_set_state(&pSwitch->sdev, DOCK_HEADSET_IN);
		} else {
			switch_set_state(&pSwitch->sdev, DOCK_IN);
		}
	}
}

static enum hrtimer_restart hs_detect_event_timer_func(struct hrtimer *data)
{
	struct dock_switch_data *pSwitch = switch_data;

	schedule_work(&pSwitch->hs_work);
	return HRTIMER_NORESTART;
}

static irqreturn_t dock_hs_interrupt(int irq, void *dev_id)
{
	struct dock_switch_data *pSwitch = switch_data;

	pr_info("HS Interrupt!!!\n");
	hrtimer_start(&pSwitch->hs_timer, pSwitch->hs_debounce_time, HRTIMER_MODE_REL);
	return IRQ_HANDLED;
}
/* Dock Headset */

/* Dock IR */
static void dock_ir_switch_work(struct work_struct *work)
{
	int state,hs_state;
	struct dock_switch_data *pSwitch = switch_data;

	state = gpio_get_value(pSwitch->det_gpio);
	if (!state) {
		switch_set_state(&pSwitch->sdev, DOCK_IR_WAKE);
		hrtimer_start(&pSwitch->hs_timer, pSwitch->hs_debounce_time, HRTIMER_MODE_REL);
	}
}

static irqreturn_t dock_ir_interrupt(int irq, void *dev_id)
{
	struct dock_switch_data *pSwitch = switch_data;

	if (is_ir_wake == false) {
		is_ir_wake = true;
		pr_info("IR Wake Interrupt!!!\n");
		schedule_work(&pSwitch->ir_work);
	}
	return IRQ_HANDLED;
}
/* Dock IR*/

/* Dock Detection */
static void dock_switch_work(struct work_struct *work)
{
	int state;
	struct dock_switch_data *pSwitch = switch_data;

	state = gpio_get_value(pSwitch->det_gpio);
	printk("dock_switch det_gpio=%d\n",state);

	/* vbus enable if dock is detected , */
	host_vbus_enable(state^1);

	switch_set_state(&pSwitch->sdev, !state);
	if (state) {
		pr_info("Disable HS IRQ\n");
		disable_irq(switch_data->hs_irq);
	} else {
		pr_info("Enable HS IRQ\n");
		enable_irq(switch_data->hs_irq);
	}
	hrtimer_start(&pSwitch->hs_timer, pSwitch->hs_debounce_time, HRTIMER_MODE_REL);
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	struct dock_switch_data *pSwitch = switch_data;

	schedule_work(&pSwitch->work);
	return HRTIMER_NORESTART;
}

static irqreturn_t dock_interrupt(int irq, void *dev_id)
{
	struct dock_switch_data *pSwitch = switch_data;

	hrtimer_start(&pSwitch->timer, pSwitch->debounce_time, HRTIMER_MODE_REL);

	return IRQ_HANDLED;
}
/* Dock Detection */

static int __init dock_register_input(struct input_dev *input)
{
	input->name = DRIVER_NAME;
	input->evbit[0] = BIT_MASK(EV_SYN)|BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(KEY_F1)] |= BIT_MASK(KEY_F1);
	input->keybit[BIT_WORD(KEY_F2)] |= BIT_MASK(KEY_F2);
	input->keybit[BIT_WORD(KEY_VOLUMEUP)] |= BIT_MASK(KEY_VOLUMEUP);
	input->keybit[BIT_WORD(KEY_VOLUMEDOWN)] |= BIT_MASK(KEY_VOLUMEDOWN);
	input->keybit[BIT_WORD(KEY_PLAYPAUSE)] |= BIT_MASK(KEY_PLAYPAUSE);
	input->keybit[BIT_WORD(KEY_NEXTSONG)] |= BIT_MASK(KEY_NEXTSONG);
	input->keybit[BIT_WORD(KEY_PREVIOUSSONG)] |= BIT_MASK(KEY_PREVIOUSSONG);
	input->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);
	input->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
	input->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
	input->keybit[BIT_WORD(KEY_UP)] |= BIT_MASK(KEY_UP);
	input->keybit[BIT_WORD(KEY_DOWN)] |= BIT_MASK(KEY_DOWN);
	input->keybit[BIT_WORD(KEY_LEFT)] |= BIT_MASK(KEY_LEFT);
	input->keybit[BIT_WORD(KEY_RIGHT)] |= BIT_MASK(KEY_RIGHT);
	input->keybit[BIT_WORD(KEY_ENTER)] |= BIT_MASK(KEY_ENTER);

	return input_register_device(input);
}

static int dock_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;

	int ret = -EBUSY;
	int err;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct dock_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->det_gpio = pdata->gpio;
	switch_data->ir_gpio = DOCK_IR_INT;
	switch_data->hs_gpio = DOCK_HS_GPIO;
	switch_data->det_irq = gpio_to_irq(pdata->gpio);
	switch_data->ir_irq = gpio_to_irq(DOCK_IR_INT);
	switch_data->hs_irq = gpio_to_irq(DOCK_HS_GPIO);
	switch_data->sdev.print_state = switch_print_state;
	switch_data->sdev.name = DRIVER_NAME;
	switch_data->sdev.print_name = switch_print_name;
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_register_switch;

	/* Dock Detection */
	INIT_WORK(&switch_data->work, dock_switch_work);
	switch_data->debounce_time = ktime_set(0, 300000000); //300 ms
	hrtimer_init(&switch_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	switch_data->timer.function = detect_event_timer_func;

	ret = gpio_request(switch_data->det_gpio, "dock_det");

	if (ret) {
		pr_err("dock_switch request det_gpio failed\n");
		goto err_request_det_gpio;
	}

	gpio_direction_input(switch_data->det_gpio);

	ret = request_irq(switch_data->det_irq, dock_interrupt,
		IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		DRIVER_NAME, switch_data);
	if (ret) {
		pr_err("dock_switch request det_irq failed\n");
		goto err_request_det_irq;
	}

	/* Dock IR */
	INIT_WORK(&switch_data->ir_work, dock_ir_switch_work);
	ret = request_irq(switch_data->ir_irq, dock_ir_interrupt,
	IRQF_DISABLED | IRQF_TRIGGER_FALLING,
	DRIVER_NAME, switch_data);
	if (ret) {
	        pr_err("dock_switch request ir_irq failed\n");
	goto err_request_ir_irq;
	}

	disable_irq(switch_data->ir_irq);

	input_dock = input_allocate_device();
	if (input_dock == NULL) {
		pr_err("dock input_allocate_device error!\n");
		ret = -ENOMEM;
		goto err_alloc_input_dev;
	}

	err = dock_register_input(input_dock);
	if (err < 0) {
		pr_err("dock register_input error\n");
		goto err_register_input_dev;
	}

	/* Dock Headset */
	INIT_WORK(&switch_data->hs_work, dock_hs_switch_work);
	switch_data->hs_debounce_time = ktime_set(0, 250000000);
	hrtimer_init(&switch_data->hs_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	switch_data->hs_timer.function = hs_detect_event_timer_func;

	ret = gpio_request(switch_data->hs_gpio, "dock_hs_det");
	if (ret) {
		pr_err("dock_switch request hs_det_gpio failed\n");
		goto err_request_hs_det_gpio;
	}

	gpio_direction_input(switch_data->hs_gpio);

	ret = request_irq(switch_data->hs_irq, dock_hs_interrupt,
		IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		"dock_hs", switch_data);
	if (ret) {
		pr_err("dock_switch request hs_irq failed\n");
		goto err_request_hs_det_irq;
	}

	dock_kobj = kobject_create_and_add("dock", NULL);
	if (dock_kobj == NULL)
		pr_err("%s: subsystem_register failed\n", __FUNCTION__);
	err = sysfs_create_group(dock_kobj, &attr_group);
	if (err)
		pr_err("%s: sysfs_create_group failed, %d\n", __FUNCTION__, __LINE__);

	register_early_suspend(&dock_switch_early_suspend_handler);

	// set current status
	dock_switch_work(&switch_data->work);

	printk(KERN_INFO "Dock switch driver probe done!\n");
	return 0;

err_request_hs_det_irq:
	gpio_free(switch_data->hs_gpio);
err_request_hs_det_gpio:
	input_unregister_device(input_dock);
err_register_input_dev:
	input_free_device(input_dock);
err_alloc_input_dev:
	free_irq(switch_data->ir_irq, switch_data);
err_request_ir_irq:
	free_irq(switch_data->det_irq, switch_data);
err_request_det_irq:
	gpio_free(switch_data->det_gpio);
err_request_det_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_register_switch:
	kfree(switch_data);
	return ret;
}

static int __devexit dock_switch_remove(struct platform_device *pdev)
{
	unregister_early_suspend(&dock_switch_early_suspend_handler);
	cancel_work_sync(&switch_data->work);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static int dock_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	int det;
	det = gpio_get_value(switch_data->det_gpio);
	if (!det) {
		pr_info("[ACER-DOCK] enable ir irq wake\n");
		enable_irq(switch_data->ir_irq);
		enable_irq_wake(switch_data->ir_irq);
		is_ir_wake = false;
		is_ir_irq_enable = true;
	}
	enable_irq_wake(switch_data->det_irq);
	return 0;
}

static int dock_switch_resume(struct platform_device *pdev)
{
	if (is_ir_irq_enable) {
		pr_info("[ACER-DOCK] disable ir irq wake\n");
		disable_irq_wake(switch_data->ir_irq);
		disable_irq(switch_data->ir_irq);
		is_ir_irq_enable = false;
	}
	disable_irq_wake(switch_data->det_irq);
	return 0;
}

static struct platform_driver dock_switch_driver = {
	.probe      = dock_switch_probe,
	.remove     = __devexit_p(dock_switch_remove),
	.suspend    = dock_switch_suspend,
	.resume     = dock_switch_resume,
	.driver     = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init dock_switch_init(void)
{
	int err;

	err = platform_driver_register(&dock_switch_driver);
	if (err)
		goto err_exit;

	return 0;

err_exit:
	printk(KERN_INFO "Dock Switch register Failed! ----->>>\n");
	return err;
}

static void __exit dock_switch_exit(void)
{
	platform_driver_unregister(&dock_switch_driver);
}

module_init(dock_switch_init);
module_exit(dock_switch_exit);

MODULE_DESCRIPTION("Dock Switch Driver");
MODULE_LICENSE("GPL");
