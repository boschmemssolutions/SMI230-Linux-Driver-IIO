// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2022 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2022 Robert Bosch GmbH. All rights reserved.
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

#include <linux/types.h>
#include <linux/spi/spi.h>
#include <linux/module.h>

#include "smi230_gyro.h"
#define MODULE_TAG		   MODULE_GYRO_NAME
#define SMI230_SPI_MAX_BUFFER_SIZE 32

static DEFINE_MUTEX(mutex);
static uint8_t *read_buf;
static struct spi_device *smi230_gyro_device;
static struct smi230_gyro_dev smi230_spi_dev;

static int8_t smi230_spi_write(uint8_t dev_addr, uint8_t reg_addr,
			       uint8_t *data, uint16_t len)
{
	struct spi_message msg;
	uint8_t buffer[SMI230_SPI_MAX_BUFFER_SIZE + 1];
	struct spi_transfer xfer = {
		.tx_buf = buffer,
		.len = len + 1,
	};

	if (len > SMI230_SPI_MAX_BUFFER_SIZE)
		return -EINVAL;

	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(smi230_gyro_device, &msg);
}

static int8_t smi230_spi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
			      uint16_t len)
{
	int ret;
	uint16_t index;
	struct spi_message msg;
	struct spi_transfer xfer[2] = {
		[0] = {
			.tx_buf = &reg_addr,
			.len = 1,
		},
		[1] = {
			.rx_buf = read_buf,
			.len = len,
		}
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer[0], &msg);
	spi_message_add_tail(&xfer[1], &msg);

	mutex_lock(&mutex);
	ret = spi_sync(smi230_gyro_device, &msg);
	if (ret) {
		mutex_unlock(&mutex);
		return ret;
	}

	for (index = 0; index < len; index++)
		data[index] = read_buf[index];
	mutex_unlock(&mutex);

	return ret;
}

static int smi230_gyro_spi_probe(struct spi_device *device)
{
	int err;

	dev_dbg(&device->dev, "smi230 gyro spi probe!");
	device->bits_per_word = 8;
	err = spi_setup(device);
	if (err < 0) {
		dev_dbg(&device->dev, "spi_setup err!");
		return err;
	}

	if (read_buf == NULL)
		read_buf = kmalloc(CONFIG_SMI230_GYRO_MAX_BUFFER_LEN + 1,
				   GFP_KERNEL);
	if (!read_buf)
		return -ENOMEM;

	smi230_gyro_device = device;

	return smi230_gyro_core_probe(&device->dev, &smi230_spi_dev);
}

static int smi230_gyro_spi_remove(struct spi_device *device)
{
	if (read_buf != NULL) {
		mutex_lock(&mutex);
		kfree(read_buf);
		read_buf = NULL;
		mutex_unlock(&mutex);
	}
	return smi230_gyro_core_remove(&device->dev);
}

static const struct spi_device_id smi230_gyro_id[] = { { SENSOR_GYRO_NAME, 0 },
						       {} };
MODULE_DEVICE_TABLE(spi, smi230_gyro_id);

static const struct of_device_id smi230_gyro_of_match[] = {
	{
		.compatible = SENSOR_GYRO_NAME,
	},
	{}
};
MODULE_DEVICE_TABLE(of, smi230_gyro_of_match);

static struct spi_driver smi230_gyro_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = SENSOR_GYRO_NAME,
		.of_match_table = smi230_gyro_of_match,
	},
	.id_table = smi230_gyro_id,
	.probe    = smi230_gyro_spi_probe,
	.remove	= smi230_gyro_spi_remove,
};

static int __init smi230_module_init(void)
{
	int err = 0;

	smi230_spi_dev.delay_ms = smi230_gyro_delay;
	smi230_spi_dev.read_write_len = 32;
	smi230_spi_dev.intf = SMI230_GYRO_SPI_INTF;
	smi230_spi_dev.read = smi230_spi_read;
	smi230_spi_dev.write = smi230_spi_write;

	err |= spi_register_driver(&smi230_gyro_driver);
	return err;
}

static void __exit smi230_module_exit(void)
{
	spi_unregister_driver(&smi230_gyro_driver);
}

module_init(smi230_module_init);
module_exit(smi230_module_exit);

MODULE_DESCRIPTION("SMI230 GYRO SENSOR SPI DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
