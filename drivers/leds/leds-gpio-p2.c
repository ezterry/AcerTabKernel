#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/earlysuspend.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/leds.h>
#include <linux/leds-gpio-p2.h>

static int __init gpio_led_init(void);
static int gpio_led_probe(struct platform_device *pdev);
static int gpio_led_remove(struct platform_device *pdev);
static void gpio_led_work_func(struct work_struct *work);
static struct delayed_work gpio_led_wq;
static struct gpio_led_data *pdata;

#define LED_DELAY_TIME 5000

static void gpio_led_early_suspend(struct early_suspend *h)
{
	gpio_direction_output(pdata->gpio,0);
}

static void gpio_led_early_resume(struct early_suspend *h)
{
	cancel_delayed_work(&gpio_led_wq);
	gpio_direction_output(pdata->gpio,1);
	schedule_delayed_work(&gpio_led_wq, msecs_to_jiffies(LED_DELAY_TIME));
}

static void gpio_led_work_func(struct work_struct *work)
{
	gpio_direction_output(pdata->gpio,0);
}

static struct early_suspend gpio_led_early_suspend_handler = {
	.suspend = gpio_led_early_suspend,
	.resume = gpio_led_early_resume,
};

static void gpio_whiteled_set(struct led_classdev *led_dev,
                              enum led_brightness value)
{
	if (value) {
		gpio_direction_output(pdata->gpio,1);
	} else {
		gpio_direction_output(pdata->gpio,0);
	}
}

static struct led_classdev whiteled = {
	.name = "acer-leds",
	.brightness_set = gpio_whiteled_set,
};

static int gpio_led_probe(struct platform_device *pdev)
{
	int rc;

	pdata = pdev->dev.platform_data;

	rc = gpio_request(pdata->gpio, "LED_EN");

	if (rc) {
		pr_err("[LED] gpio_request failed on pin %d (rc=%d)\n", pdata->gpio, rc);
		goto err_gpio_request;
	}

	tegra_gpio_enable(pdata->gpio);

	rc = gpio_direction_output(pdata->gpio,0);

	if (rc) {
		pr_err("[LED] direction configuration failed %d\n", rc);
		goto err_gpio_direction_cfg;
	}

	rc = led_classdev_register(&pdev->dev, &whiteled);

	if (rc) {
		pr_err("[LED] led_classdev_register failed %d\n", rc);
		goto err_led_classdev_reg;
	}

	register_early_suspend(&gpio_led_early_suspend_handler);

	INIT_DELAYED_WORK(&gpio_led_wq, gpio_led_work_func);

	pr_info("[LED] driver init done.\n");

	return 0;

err_led_classdev_reg:
err_gpio_direction_cfg:
	gpio_free(pdata->gpio);
err_gpio_request:
	return rc;
}

static int gpio_led_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&gpio_led_wq);
	unregister_early_suspend(&gpio_led_early_suspend_handler);
	led_classdev_unregister(&whiteled);
	gpio_free(pdata->gpio);
	kfree(pdata);

	return 0;
}

static struct platform_driver gpio_led_driver = {
	.probe		= gpio_led_probe,
	.remove		= gpio_led_remove,
	.driver		= {
		.name	= "gpio-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init gpio_led_init(void)
{
	return platform_driver_register(&gpio_led_driver);
}

static void __exit gpio_led_exit(void)
{
	platform_driver_unregister(&gpio_led_driver);
}

module_init(gpio_led_init);
module_exit(gpio_led_exit);

MODULE_AUTHOR("Shawn Tu <Shawn_Tu@acer.com.tw>");
MODULE_DESCRIPTION("ACER LED driver");
MODULE_LICENSE("GPL");
