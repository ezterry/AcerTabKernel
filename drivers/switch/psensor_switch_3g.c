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

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define DRIVER_NAME     "psensor"

#ifdef CONFIG_HAS_EARLYSUSPEND
static void func_psensor_late_resume(struct early_suspend *);
static void func_psensor_early_suspend(struct early_suspend *);
#endif

struct psensor_data {
	struct switch_dev sdev;
	unsigned gpio[2];
	unsigned irq[2];
	unsigned count;
	struct work_struct work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

};

enum Version
{
	PWIFISKU = 0,
	P3GSKU,
};

static enum Version Boardversion;
extern int acer_sku;

static void Checkversion_3g(void)
{

#if defined(CONFIG_ARCH_ACER_T30)
	if(acer_sku == 1)   // WIFI SKU
	{
		pr_info("%s: this is 3G SKU\n",DRIVER_NAME);
		Boardversion = P3GSKU;
	} else if(acer_sku == 0)
	{
		pr_info("%s: this is WIFI SKU\n",DRIVER_NAME);
		Boardversion = PWIFISKU;
	}
#endif

	pr_info("%s: check device sku \n",DRIVER_NAME);

}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void func_psensor_early_suspend(struct early_suspend *es)
{
	int i=0;
	struct psensor_data *psensor = container_of(es, struct psensor_data, early_suspend);

	//pr_info("%s: early suspend\n", DRIVER_NAME);
	for (i=0;i<psensor->count;i++)
		disable_irq(psensor->irq[i]);
	cancel_work_sync(&psensor->work);

}

static void func_psensor_late_resume(struct early_suspend *es)
{
	struct psensor_data *psensor = container_of(es, struct psensor_data, early_suspend);
	int i=0;

	//pr_info("%s: late resume\n", DRIVER_NAME);
	for (i=0;i<psensor->count;i++)
		enable_irq(psensor->irq[i]);
}
#endif

static ssize_t switch_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t switch_print_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", (switch_get_state(sdev) ? "1" : "0"));
}

static void psensor_work(struct work_struct *work)
{
	struct psensor_data *psensor =
	    container_of(work, struct psensor_data, work);

	int temp_status0 = -1,temp_status1 = -1;
	int psensor_Status = -1;

	temp_status0 = gpio_get_value(psensor->gpio[0]);
	temp_status1 = gpio_get_value(psensor->gpio[1]);

#if defined CONFIG_MACH_ACER_VANGOGH || CONFIG_ARCH_ACER_T30
	if ((temp_status0 && temp_status1) == 1)
		psensor_Status = 0;
	else
		psensor_Status = 1;
#endif
	if(!psensor_Status)
		pr_info("%s: Nothing approach\n", DRIVER_NAME);
	else
		pr_info("%s: Something approaches\n", DRIVER_NAME);

	switch_set_state(&psensor->sdev,psensor_Status);
}

static irqreturn_t psensor_interrupt(int irq, void *dev_id)
{
	struct psensor_data *psensor = (struct psensor_data *)dev_id;

	schedule_work(&psensor->work);

	return IRQ_HANDLED;
}

static int psensor_probe(struct platform_device *pdev)
{
	struct psensor_switch_platform_data *pdata = pdev->dev.platform_data;
	struct psensor_data *psensor;
	int ret = -EBUSY;
	int i=0, temp_irq = 0;

	Checkversion_3g();
	if (Boardversion == PWIFISKU)
		return -EBUSY;

	if (!pdata)
		return -EBUSY;

	psensor = kzalloc(sizeof(struct psensor_data), GFP_KERNEL);
	if (!psensor)
		return -ENOMEM;
	memset(psensor,0,sizeof(struct psensor_data));

	psensor->count = pdata->psormap_size;
	psensor->sdev.print_state = switch_print_state;
	psensor->sdev.name = DRIVER_NAME;
	psensor->sdev.print_name = switch_print_name;

	ret = switch_dev_register(&psensor->sdev);
	if (ret < 0)
		goto err_register_switch;

	for (i=0;i<pdata->psormap_size;i++)
	{
		ret = gpio_request(pdata->psormap[i].gpio, pdev->name);
		if (ret < 0)
			goto err_request_gpio;
		psensor->gpio[i] = pdata->psormap[i].gpio;
		ret = gpio_direction_input(pdata->psormap[i].gpio);
		if (ret < 0)
			goto err_set_gpio_input;
	}

	INIT_WORK(&psensor->work, psensor_work);

	for (i=0;i<pdata->psormap_size;i++)
	{
		temp_irq = gpio_to_irq(pdata->psormap[i].gpio);
		if (temp_irq < 0) {
			ret = temp_irq;
			goto err_detect_irq_num_failed;
		}

		ret = request_irq(temp_irq, psensor_interrupt, IRQF_DISABLED | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			DRIVER_NAME, psensor);
		if(ret){
			pr_err("psensor request irq failed\n");
			goto err_request_irq;
		}
		psensor->irq[i] = temp_irq;
		temp_irq = 0;
	}

	// set current status
	psensor_work(&psensor->work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	psensor->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	psensor->early_suspend.suspend = func_psensor_early_suspend;
	psensor->early_suspend.resume = func_psensor_late_resume;
	register_early_suspend(&psensor->early_suspend);
#endif

	pr_info("psensor probe success\n");

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
err_request_gpio:
        for(i=0;i<psensor->count;i++){
                if(psensor->irq[i]>0 )
                        free_irq(psensor->irq[i],psensor);
        }
        for(i=0;i<psensor->count;i++){
                if(psensor->gpio[i]>0)
                        gpio_free(psensor->gpio[i]);
        }
	switch_dev_unregister(&psensor->sdev);
err_register_switch:
	kfree(psensor);
	return ret;
}

static int __devexit psensor_remove(struct platform_device *pdev)
{
	struct psensor_data *psensor = platform_get_drvdata(pdev);
	int i=0;

	cancel_work_sync(&psensor->work);
        for(i=0;i<psensor->count;i++){
                if(psensor->gpio[i]>0)
                        gpio_free(psensor->gpio[i]);
        }
	switch_dev_unregister(&psensor->sdev);
	kfree(psensor);

	return 0;
}

#ifdef CONFIG_PM
static int psensor_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int psensor_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver psensor_driver = {
	.probe      = psensor_probe,
	.remove     = __devexit_p(psensor_remove),
#ifdef CONFIG_PM
	.suspend    = psensor_suspend,
	.resume     = psensor_resume,
#endif
	.driver     = {
	.name   = "psensor",
	.owner  = THIS_MODULE,
	},
};

static int __init psensor_init(void)
{
	int err;
	err = platform_driver_register(&psensor_driver);
	if (err)
		goto err_exit;

	pr_info("psensor initial success\n");

	return 0;

err_exit:
	pr_err("psensor register Failed!\n");
	return err;
}

static void __exit psensor_exit(void)
{
	pr_info("psensor driver unregister\n");
	platform_driver_unregister(&psensor_driver);
}

module_init(psensor_init);
module_exit(psensor_exit);

MODULE_DESCRIPTION("Psensor Driver");
MODULE_LICENSE("GPL");
