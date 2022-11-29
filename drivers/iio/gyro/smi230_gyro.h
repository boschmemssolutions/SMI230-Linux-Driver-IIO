/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2022 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
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

/*! \file smi230.h
 * \brief Sensor Driver for SMI230 sensors
 */
#ifndef _SMI230_GYRO_H
#define _SMI230_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************/
/* header files */
#include <linux/module.h>
#include <linux/delay.h>

#define DRIVER_VERSION	 "1.2.0"
#define MODULE_GYRO_NAME "SMI230GYRO"
#define SENSOR_GYRO_NAME "SMI230GYRO"

#define SMI230_GYRO_BW_523_ODR_2000_HZ UINT8_C(0x00)
#define SMI230_GYRO_BW_230_ODR_2000_HZ UINT8_C(0x01)
#define SMI230_GYRO_BW_116_ODR_1000_HZ UINT8_C(0x02)
#define SMI230_GYRO_BW_47_ODR_400_HZ   UINT8_C(0x03)
#define SMI230_GYRO_BW_23_ODR_200_HZ   UINT8_C(0x04)
#define SMI230_GYRO_BW_12_ODR_100_HZ   UINT8_C(0x05)
#define SMI230_GYRO_BW_64_ODR_200_HZ   UINT8_C(0x06)
#define SMI230_GYRO_BW_32_ODR_100_HZ   UINT8_C(0x07)

enum smi230_gyro_intf {
	/*! I2C interface */
	SMI230_GYRO_I2C_INTF,

	/*! SPI interface */
	SMI230_GYRO_SPI_INTF
};

struct smi230_gyro_cfg {
	/*! power mode */
	uint8_t power;

	/*! range */
	uint8_t range;

	/*! bandwidth */
	uint8_t bw;

	/*! output data rate */
	uint8_t odr;
};

typedef int8_t (*smi230_gyro_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr,
					 uint8_t *data, uint16_t len);

typedef void (*smi230_gyro_delay_fptr_t)(uint32_t period);

struct smi230_gyro_dev {
	/*! Accel chip Id */
	uint8_t accel_chip_id;

	/*! Gyro chip Id */
	uint8_t gyro_chip_id;

	/*! Accel device Id */
	uint8_t accel_id;

	/*! Gyro device Id */
	uint8_t gyro_id;

	/*! 0 - I2C , 1 - SPI Interface */
	enum smi230_gyro_intf intf;

	/*! Decide SPI or I2C read mechanism */
	uint8_t dummy_byte;

	/*! Structure to configure gyro sensor  */
	struct smi230_gyro_cfg gyro_cfg;

	/*! Config stream data buffer address will be assigned */
	const uint8_t *config_file_ptr;

	/*! Max read/write length (maximum supported length is 32). To be set by the user */
	uint8_t read_write_len;

	/*! Read function pointer */
	smi230_gyro_com_fptr_t read;

	/*! Write function pointer */
	smi230_gyro_com_fptr_t write;

	/*! Delay function pointer */
	smi230_gyro_delay_fptr_t delay_ms;
};

int smi230_gyro_core_probe(struct device *dev,
			   struct smi230_gyro_dev *p_smi230_dev);

int smi230_gyro_core_remove(struct device *dev);

static inline void smi230_gyro_delay(uint32_t msec)
{
	unsigned long mseond = msec;
	unsigned long min = mseond * (1000);
	/* if the time less than 20ms */
	if (msec <= 20)
		usleep_range(min, (min + 1000));
	else
		msleep(msec);
}

int8_t smi230_gyro_set_meas_conf_ex(uint8_t gyro_odr, uint8_t gyro_bw);
int8_t smi230_gyro_get_data_ex(int16_t *x, int16_t *y, int16_t *z);
#ifdef __cplusplus
}
#endif

#endif /* _SMI230_GRYO_H */

/** @}*/
