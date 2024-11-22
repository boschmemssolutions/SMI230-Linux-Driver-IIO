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
#include <linux/i2c.h>
#include <linux/version.h>

#include "smi230.h"

static const struct regmap_config smi230acc_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static int smi230acc_i2c_probe(struct i2c_client *i2c)
#else
static int smi230acc_i2c_probe(struct i2c_client *i2c,
			       const struct i2c_device_id *id)
#endif
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &smi230acc_regmap_config);

	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize I2C Regmap\n");

	return smi230acc_core_probe(dev, regmap);
}

static const struct i2c_device_id smi230acc_i2c_id[] = { { "smi230acc", 0 },
							 {} };
MODULE_DEVICE_TABLE(i2c, smi230acc_i2c_id);

static const struct of_device_id smi230acc_of_match[] = {
	{ .compatible = "bosch,smi230acc" },
	{},
};
MODULE_DEVICE_TABLE(of, smi230acc_of_match);

static struct i2c_driver smi230acc_i2c_driver = {
	.probe = smi230acc_i2c_probe,
	.id_table = smi230acc_i2c_id,
	.driver = {
		.of_match_table = smi230acc_of_match,
		.name = "smi230acc_i2c",
	},
};

//################################### GYRO #####################################

static const struct regmap_config smi230gyro_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 6, 0)
static int smi230gyro_i2c_probe(struct i2c_client *i2c)
#else
static int smi230gyro_i2c_probe(struct i2c_client *i2c,
				const struct i2c_device_id *id)
#endif
{
	struct device *dev = &i2c->dev;
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &smi230gyro_regmap_config);

	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize I2C Regmap\n");

	return smi230gyro_core_probe(dev, regmap);
}

static const struct i2c_device_id smi230gyro_i2c_id[] = { { "smi230gyro", 0 },
							  {} };
MODULE_DEVICE_TABLE(i2c, smi230gyro_i2c_id);

static const struct of_device_id smi230gyro_of_match[] = {
	{ .compatible = "bosch,smi230gyro" },
	{},
};
MODULE_DEVICE_TABLE(of, smi230gyro_of_match);

static struct i2c_driver smi230gyro_i2c_driver = {
	.probe = smi230gyro_i2c_probe,
	.id_table = smi230gyro_i2c_id,
	.driver = {
		.of_match_table = smi230gyro_of_match,
		.name = "smi230gyro_i2c",
	},
};

static int __init smi230_init(void)
{
	int ret;

	/* Register ACC I2C driver*/
	ret = i2c_add_driver(&smi230acc_i2c_driver);
	if (ret) {
		pr_info("Acc and Gyro SPI driver registered failed\n");
		return ret;
	}

	/* Register GYRO I2C driver*/
	ret = i2c_add_driver(&smi230gyro_i2c_driver);
	if (ret) {
		// Unregister driver 1 if driver 2 fails
		i2c_del_driver(&smi230acc_i2c_driver);
		pr_info("Acc and Gyro SPI driver registered failed\n");
		return ret;
	}

	pr_info("Acc and Gyro I2C driver registered successfully\n");
	return 0;
}

static void __exit smi230_exit(void)
{
	/* Unregister both I2C drivers */
	i2c_del_driver(&smi230acc_i2c_driver);
	i2c_del_driver(&smi230gyro_i2c_driver);

	pr_info("Acc and Gyro I2C driver unregistered successfully\n");
}

module_init(smi230_init);
module_exit(smi230_exit);
