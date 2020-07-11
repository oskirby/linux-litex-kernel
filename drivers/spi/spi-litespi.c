// SPDX-License-Identifier: GPL-2.0
/*
 * LiteSPI controller (LiteX) Driver
 *
 * Copyright (C) 2019 Antmicro Ltd. <www.antmicro.com>
 */

#include <linux/device.h>
#include <linux/litex.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME "litespi"

#define LITESPI_OFF_CTRL	0x00
#define LITESPI_OFF_STAT	0x08
#define LITESPI_OFF_MOSI	0x0c
#define LITESPI_OFF_MISO	0x10
#define LITESPI_OFF_CS		0x14

#define LITESPI_SZ_CTRL		2
#define LITESPI_SZ_STAT		1
#define LITESPI_SZ_CS		1

#define LITESPI_CTRL_SHIFT_BPW	8
#define LITESPI_CTRL_START_BIT	0

struct litespi_hw {
	struct spi_master *master;
	void __iomem *base;
};

static inline void litespi_wait_xfer_end(struct litespi_hw *hw)
{
	while (!litex_get_reg(hw->base + LITESPI_OFF_STAT,
			      LITESPI_SZ_STAT))
		cpu_relax();
}

static int litespi_rxtx(struct spi_master *master, struct spi_device *spi,
			struct spi_transfer *t)
{
	struct litespi_hw *hw = spi_master_get_devdata(master);
	u16 ctl_word = t->bits_per_word << LITESPI_CTRL_SHIFT_BPW;
	int i;

	/* set chip select */
	litex_set_reg(hw->base + LITESPI_OFF_CS, LITESPI_SZ_CS, BIT(spi->chip_select));

	/* set word size */
	litex_set_reg(hw->base + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL, ctl_word);

	/* add start bit to ctl_word */
	ctl_word |= BIT(LITESPI_CTRL_START_BIT);

	/*
	 * Validated SPI transfer length is multiple of SPI word size, which
	 * is itself a power-of-two multiple, and fits within LITEX_SUBREG_SIZE
	 */
	if (t->bits_per_word <= 8) {
		const u8 *tx = t->tx_buf;
		u8 *rx = t->rx_buf;

		/* word size is 1 byte */
		for (i = 0; i < t->len; i++) {
			if (tx)
				litex_set_reg(hw->base + LITESPI_OFF_MOSI, 1, *tx++);

			litex_set_reg(hw->base + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = litex_get_reg(hw->base + LITESPI_OFF_MISO, 1);
		}
	} else if (t->bits_per_word <= 16) {
		const u16 *tx = t->tx_buf;
		u16 *rx = t->rx_buf;

		/* word size is 2 bytes */
		for (i = 0; i < t->len / 2; i++) {
			if (tx)
				litex_set_reg(hw->base + LITESPI_OFF_MOSI, 2,
						 be16_to_cpu(*tx++));

			litex_set_reg(hw->base + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = cpu_to_be16(litex_get_reg(hw->base + LITESPI_OFF_MISO, 2));
		}
	} else {
		const u32 *tx = t->tx_buf;
		u32 *rx = t->rx_buf;

		/* word size is 4 bytes */
		for (i = 0; i < t->len / 4; i++) {
			if (tx)
				litex_set_reg(hw->base + LITESPI_OFF_MOSI, 4,
						 be32_to_cpu(*tx++));

			litex_set_reg(hw->base + LITESPI_OFF_CTRL, LITESPI_SZ_CTRL, ctl_word);
			litespi_wait_xfer_end(hw);

			if (rx)
				*rx++ = cpu_to_be32(litex_get_reg(hw->base + LITESPI_OFF_MISO, 4));
		}
	}

	return 0;
}

static int litespi_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct litespi_hw *hw;
	struct spi_master *master;
	struct resource *res;
	int ret;
	u32 val;

	if (!litex_check_accessors())
		return -EPROBE_DEFER;

	master = spi_alloc_master(&pdev->dev, sizeof(*hw));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = pdev->id;
	master->transfer_one = litespi_rxtx;
	master->mode_bits = SPI_MODE_0 | SPI_CS_HIGH;
	master->flags = SPI_CONTROLLER_MUST_RX | SPI_CONTROLLER_MUST_TX;

	/* get bits per word property */
	ret = of_property_read_u32(node, "litespi,max-bpw", &val);
	if (ret)
		goto err;
	if (val > 32) {
		ret = -EINVAL;
		goto err;
	}
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, val);

	/* get sck frequency */
	ret = of_property_read_u32(node, "litespi,sck-frequency", &val);
	if (ret)
		goto err;
	master->max_speed_hz = val;

	/* get num cs */
	ret = of_property_read_u32(node, "litespi,num-cs", &val);
	if (ret)
		goto err;
	master->num_chipselect = val;

	hw = spi_master_get_devdata(master);
	hw->master = master;

	/* get base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base)) {
		ret = PTR_ERR(hw->base);
		goto err;
	}

	/* register controller */
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret)
		goto err;

	return 0;

err:
	spi_master_put(master);
	return ret;
}

static const struct of_device_id litespi_match[] = {
	{ .compatible = "litex,litespi" },
	{}
};
MODULE_DEVICE_TABLE(of, litespi_match);

static struct platform_driver litespi_driver = {
	.probe = litespi_probe,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(litespi_match)
	}
};
module_platform_driver(litespi_driver)

MODULE_AUTHOR("Antmicro Ltd <www.antmicro.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
