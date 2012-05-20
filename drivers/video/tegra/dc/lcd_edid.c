#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/fb.h>
#include "edid.h"

struct lcd_edid {
    struct tegra_edid *edid;
    struct fb_monspecs *specs;
    int bus;
} *lcdedid;

int checkLCM(void)
{
    if(lcdedid->edid->data==NULL) {
        pr_info("lcd_edid: lcd edid data is NULL\n");
        return -1;
    }
    if(0x14 == lcdedid->edid->data->dc_edid.buf[17]) {
        pr_info("lcd_edid: lcd CABC pin is pulled low\n");
        return 0;
    } else if(0x15 == lcdedid->edid->data->dc_edid.buf[17]) {
        pr_info("lcd_edid: lcd CABC pin is pulled high\n");
        return 1;
    } else {
        pr_err("lcd_edid: edid 17 is not 0x14 or 0x15\n");
        return -1;
    }

    return 0;
}
EXPORT_SYMBOL(checkLCM);


static int __init lcd_edid_init(void)
{
    int ret;

    lcdedid = kzalloc(sizeof(struct lcd_edid), GFP_KERNEL);

    if (!lcdedid) {
        pr_err("lcd_edid: Unable to allocate memory!\n");
        return -ENOMEM;
    }

    lcdedid->specs = kzalloc(sizeof(struct fb_monspecs), GFP_KERNEL);

    if (!lcdedid->specs) {
        kfree(lcdedid);
        pr_err("lcd_edid: Unable to allocate memory!\n");
        return -ENOMEM;
    }

    lcdedid->bus = 2;

    lcdedid->edid = tegra_edid_create(lcdedid->bus);
    if (!lcdedid->edid) {
        pr_err("lcd_edid: can't create edid\n");
    }

    ret = tegra_edid_get_monspecs(lcdedid->edid, lcdedid->specs);
    if (ret < 0) {
        pr_err("error reading edid\n");
    }
    return 0;
}

static void __exit lcd_edid_exit(void)
{
    kfree(lcdedid->specs);
    kfree(lcdedid);

    return;
}
module_init(lcd_edid_init);
module_exit(lcd_edid_exit);
