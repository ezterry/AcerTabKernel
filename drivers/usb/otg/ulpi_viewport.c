/*
 * Copyright (C) 2011 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/io.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>

#define ULPI_VIEW_WAKEUP	(1 << 31)
#define ULPI_VIEW_RUN		(1 << 30)
#define ULPI_VIEW_WRITE		(1 << 29)
#define ULPI_VIEW_READ		(0 << 29)
#define ULPI_VIEW_ADDR(x)	(((x) & 0xff) << 16)
#define ULPI_VIEW_DATA_READ(x)	(((x) >> 8) & 0xff)
#define ULPI_VIEW_DATA_WRITE(x)	((x) & 0xff)

static int ulpi_viewport_wait(void __iomem *view, u32 mask)
{
	unsigned long usec = 5000;

	while (usec--) {
		if (!(readl(view) & mask))
			return 0;

		udelay(1);
	};

	return -ETIMEDOUT;
}

static int ulpi_viewport_read(struct otg_transceiver *otg, u32 reg)
{
	int ret;
	void __iomem *view = otg->io_priv;

	writel(ULPI_VIEW_WAKEUP | ULPI_VIEW_WRITE, view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_WAKEUP);
	if (ret)
		return ret;

	writel(ULPI_VIEW_RUN | ULPI_VIEW_READ | ULPI_VIEW_ADDR(reg), view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_RUN);
	if (ret)
		return ret;

	return ULPI_VIEW_DATA_READ(readl(view));
}

static int ulpi_viewport_write_1(struct otg_transceiver *otg, u32 val, u32 reg)
{
	int ret;
	void __iomem *view = otg->io_priv;

	writel(ULPI_VIEW_WAKEUP | ULPI_VIEW_WRITE, view);
	ret = ulpi_viewport_wait(view, ULPI_VIEW_WAKEUP);
	if (ret) {
		printk("%s() line : %d \n", __func__, __LINE__);
		return ret;
	}

	writel(ULPI_VIEW_RUN | ULPI_VIEW_WRITE | ULPI_VIEW_DATA_WRITE(val) |
						 ULPI_VIEW_ADDR(reg), view);

	ret = ulpi_viewport_wait(view, ULPI_VIEW_RUN);
	if (ret) {
		printk("%s() line : %d \n", __func__, __LINE__);
		return ret;
	}

}

static int ulpi_viewport_write(struct otg_transceiver *otg, u32 val, u32 reg)
{
	int ret;
	int retry = 1000;
	void __iomem *view = otg->io_priv;
	ret = ulpi_viewport_write_1(otg, val, reg);
	while((retry == 0) || (ret))
	{
		printk("%s(retry:%d) line : %d \n", __func__, retry, __LINE__);
		ret = ulpi_viewport_write_1(otg, val, reg);
		retry--;
	}
	return ret;
}

struct otg_io_access_ops ulpi_viewport_access_ops = {
	.read	= ulpi_viewport_read,
	.write	= ulpi_viewport_write,
};
