#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#define DRIVER_NAME "rotationlock"
#define ROTATELOCK_DEBOUNCE_TIME_MS 1

struct rotationlock_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	unsigned irq;
	struct work_struct work;
};

static struct rotationlock_switch_data *switch_data;

static ssize_t switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t switch_print_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (switch_get_state(sdev) ? "1" : "0"));
}


static void rotationlock_switch_work(struct work_struct *work)
{
	int state;
	struct rotationlock_switch_data *pSwitch =
		container_of(work, struct rotationlock_switch_data, work);

	state = gpio_get_value(pSwitch->gpio);
	switch_set_state(&pSwitch->sdev, state);
}

static void rotationlock_switch_early_suspend(struct early_suspend *h)
{
	pr_debug("rotationlock_switch early_suspend\n");
}

static void rotationlock_switch_late_resume(struct early_suspend *h)
{
	pr_debug("rotationlock_switch late_resume\n");

	// set current status
	rotationlock_switch_work(&switch_data->work);
}

static struct early_suspend rotationlock_switch_early_suspend_handler = {
	.suspend = rotationlock_switch_early_suspend,
	.resume = rotationlock_switch_late_resume,
};

static irqreturn_t rotationlock_interrupt(int irq, void *dev_id)
{
	struct rotationlock_switch_data *pSwitch =
		(struct rotationlock_switch_data *)dev_id;

	mdelay(ROTATELOCK_DEBOUNCE_TIME_MS);
	schedule_work(&pSwitch->work);

	return IRQ_HANDLED;
}

static int rotationlock_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;

	int ret = -EBUSY;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct rotationlock_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->gpio = pdata->gpio;
	switch_data->irq = gpio_to_irq(pdata->gpio);
	switch_data->sdev.name = DRIVER_NAME;
	switch_data->sdev.print_name = switch_print_name;
	switch_data->sdev.print_state = switch_print_state;
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_register_switch;

	INIT_WORK(&switch_data->work, rotationlock_switch_work);

	ret = gpio_request(pdata->gpio, "RotationSwitch_EN");
	if (ret) {
		pr_err("[RotationSwitch] gpio_request failed on pin %d (rc=%d)\n", pdata->gpio, ret);
		goto err_gpio_request;
	}

	ret = request_irq(switch_data->irq, rotationlock_interrupt,
		IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		DRIVER_NAME, switch_data);

	if (ret) {
		pr_err("rotationlock_switch request irq failed!\n");
		goto err_request_irq;
	}

	register_early_suspend(&rotationlock_switch_early_suspend_handler);

	// set current status
	rotationlock_switch_work(&switch_data->work);

	pr_info("rotationlock driver probe done!\n");
	return 0;

err_request_irq:
	gpio_free(pdata->gpio);
err_gpio_request:
	kfree(switch_data);
err_register_switch:
	return ret;
}

static int __devexit rotationlock_switch_remove(struct platform_device *pdev)
{
	struct rotationlock_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static int rotationlock_switch_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int rotationlock_switch_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rotationlock_switch_driver = {
	.probe      = rotationlock_switch_probe,
	.remove     = __devexit_p(rotationlock_switch_remove),
	.suspend    = rotationlock_switch_suspend,
	.resume     = rotationlock_switch_resume,
	.driver     = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init rotationlock_switch_init(void)
{
	int err;

	err = platform_driver_register(&rotationlock_switch_driver);
	if (err)
		goto err_exit;

	return 0;

err_exit:
	pr_err("Rotationlock Switch register Failed! ----->>>\n");
	return err;
}

static void __exit rotationlock_switch_exit(void)
{
	platform_driver_unregister(&rotationlock_switch_driver);
}

module_init(rotationlock_switch_init);
module_exit(rotationlock_switch_exit);

MODULE_DESCRIPTION("Rotationlock Switch Driver");
MODULE_LICENSE("GPL");
