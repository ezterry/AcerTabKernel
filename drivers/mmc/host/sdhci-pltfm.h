/*
 * Copyright 2010 MontaVista Software, LLC.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DRIVERS_MMC_SDHCI_PLTFM_H
#define _DRIVERS_MMC_SDHCI_PLTFM_H

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/mmc/sdhci-pltfm.h>

struct sdhci_pltfm_host {
	struct clk *clk;
	void *priv; /* to handle quirks across io-accessor calls */
};

extern struct sdhci_pltfm_data sdhci_cns3xxx_pdata;
extern struct sdhci_pltfm_data sdhci_esdhc_imx_pdata;
extern struct sdhci_pltfm_data sdhci_dove_pdata;
extern struct sdhci_pltfm_data sdhci_tegra_pdata;

struct tegra_sdhci_host {
	bool	clk_enabled;
	struct regulator *vdd_io_reg;
	struct regulator *vdd_slot_reg;
	/* Pointer to the chip specific HW ops */
	struct tegra_sdhci_hw_ops *hw_ops;
	/* Host controller instance */
	unsigned int instance;
	/* vddio_min */
	unsigned int vddio_min_uv;
	/* vddio_max */
	unsigned int vddio_max_uv;
	/* max clk supported by the platform */
	unsigned int max_clk_limit;
};

#endif /* _DRIVERS_MMC_SDHCI_PLTFM_H */
