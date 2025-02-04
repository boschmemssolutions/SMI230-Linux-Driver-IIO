/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/version.h>

#include "smi230.h"

static int smi230acc_regmap_spi_read(void *context, const void *reg_buf,
				     size_t reg_size, void *val_buf,
				     size_t val_size)
{
	struct spi_device *spi = context;

	return spi_write_then_read(spi, reg_buf, reg_size, val_buf, val_size);
}

static int smi230acc_regmap_spi_write(void *context, const void *data,
				      size_t count)
{
	struct spi_device *spi = context;
	u8 *data_buff = (u8 *)data;

	data_buff[1] = data_buff[0];
	return spi_write(spi, data_buff + 1, count - 1);
}

static const struct regmap_bus smi230acc_regmap_bus = {
	.read = smi230acc_regmap_spi_read,
	.write = smi230acc_regmap_spi_write,
	.read_flag_mask = 0x80,
};

static const struct regmap_config smi230acc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.pad_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int smi230acc_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init(&spi->dev, &smi230acc_regmap_bus, &spi->dev,
				  &smi230acc_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "Failed to initialize SPI Regmap\n");

	return smi230acc_core_probe(&spi->dev, regmap);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void smi230acc_spi_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	smi230acc_core_remove(dev);
}
#else
static int smi230acc_spi_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	return smi230acc_core_remove(dev);
}
#endif

static int __maybe_unused smi230acc_spi_suspend(struct device *dev)
{
	return smi230acc_core_suspend(dev);
}

static int __maybe_unused smi230acc_spi_resume(struct device *dev)
{
	return smi230acc_core_resume(dev);
}

static const struct dev_pm_ops smi230acc_spi_pm_ops = { SET_SYSTEM_SLEEP_PM_OPS(
	smi230acc_spi_suspend, smi230acc_spi_resume) };

static const struct spi_device_id smi230acc_spi_id[] = { { "smi230acc", 0 },
							 {} };
MODULE_DEVICE_TABLE(spi, smi230acc_spi_id);

static const struct of_device_id smi230acc_of_match[] = {
	{ .compatible = "bosch,smi230acc" },
	{},
};
MODULE_DEVICE_TABLE(of, smi230acc_of_match);

static struct spi_driver smi230acc_spi_driver = {
	.probe = smi230acc_spi_probe,
	.remove = smi230acc_spi_remove,
	.id_table = smi230acc_spi_id,
	.driver = {
		.of_match_table = smi230acc_of_match,
		.name = "smi230acc_spi",
		.pm = &smi230acc_spi_pm_ops,
	},
};

//##################################### GYRO ########################################

static const struct regmap_config smi230gyro_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int smi230gyro_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &smi230gyro_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(regmap),
				     "Failed to initialize SPI Regmap\n");

	return smi230gyro_core_probe(&spi->dev, regmap);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void smi230gyro_spi_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	smi230gyro_core_remove(dev);
}
#else
static int smi230gyro_spi_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	return smi230gyro_core_remove(dev);
}
#endif

static int __maybe_unused smi230gyro_spi_suspend(struct device *dev)
{
	return smi230gyro_core_suspend(dev);
}

static int __maybe_unused smi230gyro_spi_resume(struct device *dev)
{
	return smi230gyro_core_resume(dev);
}

static const struct dev_pm_ops smi230gyro_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smi230gyro_spi_suspend, smi230gyro_spi_resume)
};

static const struct spi_device_id smi230gyro_spi_id[] = { { "smi230gyro", 0 },
							  {} };
MODULE_DEVICE_TABLE(spi, smi230gyro_spi_id);

static const struct of_device_id smi230gyro_of_match[] = {
	{ .compatible = "bosch,smi230gyro" },
	{},
};
MODULE_DEVICE_TABLE(of, smi230gyro_of_match);

static struct spi_driver smi230gyro_spi_driver = {
	.probe = smi230gyro_spi_probe,
	.remove = smi230gyro_spi_remove,
	.id_table = smi230gyro_spi_id,
	.driver = {
		.of_match_table = smi230gyro_of_match,
		.name = "smi230gyro_spi",
		.pm = &smi230gyro_spi_pm_ops,
	},
};

static int __init smi230_init(void)
{
	int ret;

	/* Register ACC SPI driver*/
	ret = spi_register_driver(&smi230acc_spi_driver);
	if (ret) {
		pr_info("Acc and Gyro SPI driver registered failed\n");
		return ret;
	}

	/* Register GYRO SPI driver*/
	ret = spi_register_driver(&smi230gyro_spi_driver);
	if (ret) {
		// Unregister driver 1 if driver 2 fails
		spi_unregister_driver(&smi230acc_spi_driver);
		pr_info("Acc and Gyro SPI driver registered failed\n");
		return ret;
	}

	pr_info("Acc and Gyro SPI driver registered successfully\n");
	return 0;
}

static void __exit smi230_exit(void)
{
	/* Unregister both SPI drivers */
	spi_unregister_driver(&smi230acc_spi_driver);
	spi_unregister_driver(&smi230gyro_spi_driver);

	pr_info("Acc and Gyro SPI driver unregistered successfully\n");
}

module_init(smi230_init);
module_exit(smi230_exit);
