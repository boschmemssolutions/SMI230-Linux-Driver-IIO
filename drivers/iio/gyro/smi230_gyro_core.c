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

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/of_gpio.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/kernel.h>

#include "smi230_gyro.h"

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)  S8_C(x)
#define UINT8_C(x) U8_C(x)
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void *)0)
#endif
#endif

#ifndef TRUE
#define TRUE UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE UINT8_C(0)
#endif

/* Gyro registers */
#define SMI230_GYRO_CHIP_ID_REG		  UINT8_C(0x00)
#define SMI230_GYRO_X_LSB_REG		  UINT8_C(0x02)
#define SMI230_GYRO_X_MSB_REG		  UINT8_C(0x03)
#define SMI230_GYRO_Y_LSB_REG		  UINT8_C(0x04)
#define SMI230_GYRO_Y_MSB_REG		  UINT8_C(0x05)
#define SMI230_GYRO_Z_LSB_REG		  UINT8_C(0x06)
#define SMI230_GYRO_Z_MSB_REG		  UINT8_C(0x07)
#define SMI230_GYRO_INT_STAT_1_REG	  UINT8_C(0x0A)
#define SMI230_GYRO_RANGE_REG		  UINT8_C(0x0F)
#define SMI230_GYRO_BANDWIDTH_REG	  UINT8_C(0x10)
#define SMI230_GYRO_LPM1_REG		  UINT8_C(0x11)
#define SMI230_GYRO_SOFTRESET_REG	  UINT8_C(0x14)
#define SMI230_GYRO_INT_CTRL_REG	  UINT8_C(0x15)
#define SMI230_GYRO_INT3_INT4_IO_CONF_REG UINT8_C(0x16)
#define SMI230_GYRO_INT3_INT4_IO_MAP_REG  UINT8_C(0x18)
#define SMI230_GYRO_WM_INT_REG		  UINT8_C(0x1E)
#define SMI230_GYRO_FIFO_EXT_INT_S_REG	  UINT8_C(0x34)
#define SMI230_GYRO_SELF_TEST_REG	  UINT8_C(0x3C)
#define SMI230_GYRO_CHIP_ID		  UINT8_C(0x0F)
#define SMI230_GYRO_I2C_ADDR_PRIMARY	  UINT8_C(0x68)
#define SMI230_GYRO_I2C_ADDR_SECONDARY	  UINT8_C(0x69)

#define SMI230_GYRO_FIFO_STATUS_ADDR   UINT8_C(0x0E)
#define SMI230_GYRO_FIFO_CONFIG_0_ADDR UINT8_C(0x3D)
#define SMI230_GYRO_FIFO_CONFIG_1_ADDR UINT8_C(0x3E)
#define SMI230_GYRO_FIFO_DATA_ADDR     UINT8_C(0x3F)

#define SMI230_GYRO_INT_STAT_DRDY UINT8_C(0x80)
#define SMI230_GYRO_INT_STAT_FIFO UINT8_C(0x10)

#define SMI230_FIFO_GYRO_FRAME_LENGTH UINT8_C(6)
#define SMI230_GYRO_STREAM_MODE	      UINT8_C(0x80)
#define SMI230_GYRO_FIFO_MODE	      UINT8_C(0x40)

#define SMI230_W_FIFO_EMPTY INT8_C(1)

#define SMI230_GYRO_RANGE_2000_DPS	       UINT8_C(0x00)
#define SMI230_GYRO_RANGE_1000_DPS	       UINT8_C(0x01)
#define SMI230_GYRO_RANGE_500_DPS	       UINT8_C(0x02)
#define SMI230_GYRO_RANGE_250_DPS	       UINT8_C(0x03)
#define SMI230_GYRO_RANGE_125_DPS	       UINT8_C(0x04)
#define SMI230_GYRO_BW_523_ODR_2000_HZ	       UINT8_C(0x00)
#define SMI230_GYRO_BW_230_ODR_2000_HZ	       UINT8_C(0x01)
#define SMI230_GYRO_BW_116_ODR_1000_HZ	       UINT8_C(0x02)
#define SMI230_GYRO_BW_47_ODR_400_HZ	       UINT8_C(0x03)
#define SMI230_GYRO_BW_23_ODR_200_HZ	       UINT8_C(0x04)
#define SMI230_GYRO_BW_12_ODR_100_HZ	       UINT8_C(0x05)
#define SMI230_GYRO_BW_64_ODR_200_HZ	       UINT8_C(0x06)
#define SMI230_GYRO_BW_32_ODR_100_HZ	       UINT8_C(0x07)
#define SMI230_GYRO_ODR_RESET_VAL	       UINT8_C(0x80)
#define SMI230_GYRO_PM_NORMAL		       UINT8_C(0x00)
#define SMI230_GYRO_PM_DEEP_SUSPEND	       UINT8_C(0x20)
#define SMI230_GYRO_PM_SUSPEND		       UINT8_C(0x80)
#define SMI230_GYRO_DRDY_INT_DISABLE_VAL       UINT8_C(0x00)
#define SMI230_GYRO_DRDY_INT_ENABLE_VAL	       UINT8_C(0x80)
#define SMI230_GYRO_FIFO_INT_DISABLE_VAL       UINT8_C(0x00)
#define SMI230_GYRO_FIFO_INT_ENABLE_VAL	       UINT8_C(0x40)
#define SMI230_GYRO_MAP_DRDY_TO_INT3	       UINT8_C(0x01)
#define SMI230_GYRO_MAP_DRDY_TO_INT4	       UINT8_C(0x80)
#define SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 UINT8_C(0x81)
#define SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4 UINT8_C(0x24)
#define SMI230_GYRO_SOFTRESET_DELAY	       UINT8_C(200)
#define SMI230_GYRO_POWER_MODE_CONFIG_DELAY    UINT8_C(30)
#define SMI230_GYRO_RANGE_MASK		       UINT8_C(0x07)
#define SMI230_GYRO_BW_MASK		       UINT8_C(0x0F)
#define SMI230_GYRO_POWER_MASK		       UINT8_C(0xA0)
#define SMI230_GYRO_POWER_POS		       UINT8_C(5)
#define SMI230_GYRO_DATA_EN_MASK	       UINT8_C(0x80)
#define SMI230_GYRO_DATA_EN_POS		       UINT8_C(7)
#define SMI230_GYRO_FIFO_EN_MASK	       UINT8_C(0x40)
#define SMI230_GYRO_FIFO_EN_POS		       UINT8_C(6)
#define SMI230_GYRO_INT3_LVL_MASK	       UINT8_C(0x01)
#define SMI230_GYRO_INT3_OD_MASK	       UINT8_C(0x02)
#define SMI230_GYRO_INT4_LVL_MASK	       UINT8_C(0x04)
#define SMI230_GYRO_INT4_OD_MASK	       UINT8_C(0x08)
#define SMI230_GYRO_INT3_OD_POS		       UINT8_C(1)
#define SMI230_GYRO_INT4_LVL_POS	       UINT8_C(2)
#define SMI230_GYRO_INT4_OD_POS		       UINT8_C(3)
#define SMI230_GYRO_INT_EN_MASK		       UINT8_C(0x80)
#define SMI230_GYRO_INT_EN_POS		       UINT8_C(7)
#define SMI230_GYRO_INT3_MAP_MASK	       UINT8_C(0x01)
#define SMI230_GYRO_INT4_MAP_MASK	       UINT8_C(0x80)
#define SMI230_GYRO_FIFO_INT3_MAP_MASK	       UINT8_C(0x04)
#define SMI230_GYRO_FIFO_INT4_MAP_MASK	       UINT8_C(0x20)
#define SMI230_GYRO_INT3_MAP_POS	       UINT8_C(0)
#define SMI230_GYRO_INT4_MAP_POS	       UINT8_C(7)
#define SMI230_GYRO_FIFO_INT3_MAP_POS	       UINT8_C(2)
#define SMI230_GYRO_FIFO_INT4_MAP_POS	       UINT8_C(5)
#define SMI230_GYRO_SELF_TEST_EN_MASK	       UINT8_C(0x01)
#define SMI230_GYRO_SELF_TEST_RDY_MASK	       UINT8_C(0x02)
#define SMI230_GYRO_SELF_TEST_RESULT_MASK      UINT8_C(0x04)
#define SMI230_GYRO_SELF_TEST_FUNCTION_MASK    UINT8_C(0x08)
#define SMI230_GYRO_SELF_TEST_RDY_POS	       UINT8_C(1)
#define SMI230_GYRO_SELF_TEST_RESULT_POS       UINT8_C(2)
#define SMI230_GYRO_SELF_TEST_FUNCTION_POS     UINT8_C(3)

#define SMI230_INT_MODE_PUSH_PULL  UINT8_C(0)
#define SMI230_INT_MODE_OPEN_DRAIN UINT8_C(1)
#define SMI230_INT_ACTIVE_LOW	   UINT8_C(0)
#define SMI230_INT_ACTIVE_HIGH	   UINT8_C(1)

#define SMI230_SPI_RD_MASK UINT8_C(0x80)
#define SMI230_SPI_WR_MASK UINT8_C(0x7F)

#define SMI230_OK		       INT8_C(0)
#define SMI230_E_NULL_PTR	       INT8_C(-1)
#define SMI230_E_COM_FAIL	       INT8_C(-2)
#define SMI230_E_DEV_NOT_FOUND	       INT8_C(-3)
#define SMI230_E_OUT_OF_RANGE	       INT8_C(-4)
#define SMI230_E_INVALID_INPUT	       INT8_C(-5)
#define SMI230_E_CONFIG_STREAM_ERROR   INT8_C(-6)
#define SMI230_E_RD_WR_LENGTH_INVALID  INT8_C(-7)
#define SMI230_E_INVALID_CONFIG	       INT8_C(-8)
#define SMI230_E_FEATURE_NOT_SUPPORTED INT8_C(-9)

#define SMI230_DISABLE UINT8_C(0)
#define SMI230_ENABLE  UINT8_C(1)

#define SMI230_SOFT_RESET_CMD UINT8_C(0xB6)
#define SMI230_FIFO_RESET_CMD UINT8_C(0xB0)

#define SMI230_SET_BITS(reg_var, bitname, val)                                 \
	((reg_var & ~(bitname##_MASK)) |                                       \
	 ((val << bitname##_POS) & bitname##_MASK))

#define SMI230_GET_BITS(reg_var, bitname)                                      \
	((reg_var & (bitname##_MASK)) >> (bitname##_POS))

#define SMI230_SET_BITS_POS_0(reg_var, bitname, val)                           \
	((reg_var & ~(bitname##_MASK)) | (val & bitname##_MASK))

#define SMI230_GET_BITS_POS_0(reg_var, bitname) (reg_var & (bitname##_MASK))

#define SMI230_SET_BIT_VAL_0(reg_var, bitname) (reg_var & ~(bitname##_MASK))

#define SMI230_CHANNEL(_type, _axis, _index)                                   \
	{                                                                      \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis,     \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),      \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
	}

enum smi230_gyro_int_channel { SMI230_INT_CHANNEL_3, SMI230_INT_CHANNEL_4 };

enum smi230_gyro_int_types { SMI230_GYRO_DATA_RDY_INT, SMI230_GYRO_FIFO_INT };

struct smi230_int_pin_cfg {
	uint8_t lvl : 1;
	uint8_t output_mode : 1;
	uint8_t enable_int_pin : 1;
};

struct smi230_gyro_int_channel_cfg {
	enum smi230_gyro_int_channel int_channel;
	enum smi230_gyro_int_types int_type;
	struct smi230_int_pin_cfg int_pin_cfg;
};

struct smi230_int_cfg {
	struct smi230_gyro_int_channel_cfg gyro_int_config_1;
	struct smi230_gyro_int_channel_cfg gyro_int_config_2;
};

enum smi230_scan_axis {
	SMI230_SCAN_GYRO_X = 0,
	SMI230_SCAN_GYRO_Y,
	SMI230_SCAN_GYRO_Z,
	SMI230_SCAN_TIMESTAMP,
};

static const struct iio_chan_spec smi230_channels[] = {
	SMI230_CHANNEL(IIO_ANGL_VEL, X, SMI230_SCAN_GYRO_X),
	SMI230_CHANNEL(IIO_ANGL_VEL, Y, SMI230_SCAN_GYRO_Y),
	SMI230_CHANNEL(IIO_ANGL_VEL, Z, SMI230_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(SMI230_SCAN_TIMESTAMP),
};

struct smi230_sensor_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

#define SMI230_MAX_GYRO_FIFO_FRAME 100
#define SMI230_MAX_GYRO_FIFO_BYTES                                             \
	(SMI230_MAX_GYRO_FIFO_FRAME * SMI230_FIFO_GYRO_FRAME_LENGTH)
static struct smi230_sensor_data fifo_gyro_data[SMI230_MAX_GYRO_FIFO_FRAME];

struct gyro_fifo_config {
	uint8_t mode;
	uint8_t wm_en;
	uint8_t int3_en;
	uint8_t int4_en;
};

struct smi230_fifo_frame {
	uint8_t *data;
	uint16_t length;
	uint16_t data_enable;
	uint16_t acc_byte_start_idx;
	uint32_t sensor_time;
	uint8_t skipped_frame_count;
	uint8_t data_int_map;
	uint16_t wm_lvl;
	uint8_t acc_frm_len;
	uint8_t all_frm_len;
};

static struct gyro_fifo_config fifo_config;
static uint8_t fifo_buf[SMI230_MAX_GYRO_FIFO_BYTES];
static struct smi230_int_cfg int_config;
static struct smi230_gyro_dev *g_smi230_gyro_dev;

static int8_t null_ptr_check(const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) ||
	    (dev->delay_ms == NULL)) {
		rslt = SMI230_E_NULL_PTR;
	} else {
		rslt = SMI230_OK;
	}

	return rslt;
}

static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_GYRO_SPI_INTF)
		reg_addr = (reg_addr | SMI230_SPI_RD_MASK);

	rslt = dev->read(dev->gyro_id, reg_addr, reg_data, len);
	if (rslt != SMI230_OK)
		rslt = SMI230_E_COM_FAIL;

	return rslt;
}

static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_GYRO_SPI_INTF) {
		/* Configuring reg_addr for SPI Interface */
		reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
	}

	rslt = dev->write(dev->gyro_id, reg_addr, reg_data, len);
	if (rslt != SMI230_OK)
		rslt = SMI230_E_COM_FAIL;

	return rslt;
}

int8_t smi230_gyro_chip_id_check(struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_CHIP_ID_REG, &chip_id, 1, dev);

		if (rslt == SMI230_OK) {
			if (chip_id == SMI230_GYRO_CHIP_ID)
				dev->gyro_chip_id = chip_id;
			else
				rslt = SMI230_E_DEV_NOT_FOUND;
		}
	}

	return rslt;
}

int8_t smi230_gyro_get_meas_conf(struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_RANGE_REG, data, 2, dev);

		if (rslt == SMI230_OK) {
			dev->gyro_cfg.range = data[0];
			dev->gyro_cfg.odr = (data[1] & SMI230_GYRO_BW_MASK);
			dev->gyro_cfg.bw = dev->gyro_cfg.odr;
		}
	}

	return rslt;
}

int8_t smi230_gyro_set_meas_conf(const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;
	uint8_t odr, range;
	uint8_t is_range_valid = TRUE, is_odr_valid = TRUE;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		odr = dev->gyro_cfg.odr;
		range = dev->gyro_cfg.range;

		if (odr > SMI230_GYRO_BW_32_ODR_100_HZ)
			is_odr_valid = FALSE;

		if (range > SMI230_GYRO_RANGE_125_DPS)
			is_range_valid = FALSE;

		if (is_odr_valid && is_range_valid) {
			rslt = get_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1,
					dev);
			if (rslt == SMI230_OK) {
				data = SMI230_SET_BITS_POS_0(
					data, SMI230_GYRO_BW, odr);
				rslt = set_regs(SMI230_GYRO_BANDWIDTH_REG,
						&data, 1, dev);
				if (rslt == SMI230_OK) {
					rslt = get_regs(SMI230_GYRO_RANGE_REG,
							&data, 1, dev);
					if (rslt == SMI230_OK) {
						data = SMI230_SET_BITS_POS_0(
							data, SMI230_GYRO_RANGE,
							range);
						rslt = set_regs(
							SMI230_GYRO_RANGE_REG,
							&data, 1, dev);
					}
				}
			}

		} else {
			rslt = SMI230_E_INVALID_CONFIG;
		}
	}

	return rslt;
}
int8_t smi230_gyro_set_meas_conf_ex(uint8_t gyro_odr, uint8_t gyro_bw)
{
	g_smi230_gyro_dev->gyro_cfg.odr = gyro_odr;
	g_smi230_gyro_dev->gyro_cfg.bw = gyro_bw;
	return smi230_gyro_set_meas_conf(g_smi230_gyro_dev);
}
EXPORT_SYMBOL(smi230_gyro_set_meas_conf_ex);

static int8_t
set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config,
		   const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);

	if (rslt == SMI230_OK) {
		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_3:
			data = SMI230_SET_BITS_POS_0(
				data, SMI230_GYRO_INT3_LVL,
				int_config->int_pin_cfg.lvl);
			data = SMI230_SET_BITS(
				data, SMI230_GYRO_INT3_OD,
				int_config->int_pin_cfg.output_mode);
			break;

		case SMI230_INT_CHANNEL_4:
			data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_LVL,
					       int_config->int_pin_cfg.lvl);
			data = SMI230_SET_BITS(
				data, SMI230_GYRO_INT4_OD,
				int_config->int_pin_cfg.output_mode);
			break;

		default:
			break;
		}

		rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1,
				dev);
	}

	return rslt;
}

static int8_t
set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
			const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t conf, data[2] = { 0 };

	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

	if (rslt == SMI230_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_3:
			data[0] = SMI230_SET_BITS_POS_0(
				data[0], SMI230_GYRO_INT3_MAP, conf);
			break;

		case SMI230_INT_CHANNEL_4:
			data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_INT4_MAP,
						  conf);
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			/* condition to check disabling the interrupt in single channel when both
			 * interrupts channels are enabled
			 */
			if (data[0] & SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4)
				data[1] = SMI230_GYRO_DRDY_INT_ENABLE_VAL;
			else
				data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;

			rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG,
					&data[0], 1, dev);
			if (rslt == SMI230_OK) {
				rslt = set_int_pin_config(int_config, dev);
				if (rslt == SMI230_OK)
					rslt = set_regs(
						SMI230_GYRO_INT_CTRL_REG,
						&data[1], 1, dev);
			}
		}
	}

	return rslt;
}

static int8_t
set_gyro_fifo_int(const struct smi230_gyro_int_channel_cfg *int_config,
		  const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t conf, data[2] = { 0 };

	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);
	if (rslt == SMI230_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;
		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_3:
			data[0] = SMI230_SET_BITS(
				data[0], SMI230_GYRO_FIFO_INT3_MAP, conf);
			break;

		case SMI230_INT_CHANNEL_4:
			data[0] = SMI230_SET_BITS(
				data[0], SMI230_GYRO_FIFO_INT4_MAP, conf);
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			if (data[0] & SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4)
				data[1] = SMI230_GYRO_FIFO_INT_ENABLE_VAL;
			else
				data[1] = SMI230_GYRO_FIFO_INT_DISABLE_VAL;

			rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG,
					&data[0], 1, dev);
			if (rslt == SMI230_OK) {
				rslt = set_int_pin_config(int_config, dev);
				if (rslt == SMI230_OK) {
					rslt = set_regs(
						SMI230_GYRO_INT_CTRL_REG,
						&data[1], 1, dev);
				}
			}
		}
	}

	return rslt;
}

static int8_t
smi230_gyro_set_int_config(const struct smi230_gyro_int_channel_cfg *int_config,
			   const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (int_config != NULL)) {
		switch (int_config->int_type) {
		case SMI230_GYRO_DATA_RDY_INT:
			rslt = set_gyro_data_ready_int(int_config, dev);
			break;
		case SMI230_GYRO_FIFO_INT:
			rslt = set_gyro_fifo_int(int_config, dev);
			break;

		default:
			rslt = SMI230_E_INVALID_CONFIG;
			break;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t smi230_gyro_get_data(struct smi230_sensor_data *gyro,
				   const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (gyro != NULL)) {
		rslt = get_regs(SMI230_GYRO_X_LSB_REG, data, 6, dev);
		if (rslt == SMI230_OK) {
			lsb = data[0];
			msb = data[1];
			msblsb = (msb << 8) | lsb;
			gyro->x = (int16_t)msblsb;

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			gyro->y = (int16_t)msblsb;

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			gyro->z = (int16_t)msblsb;
		}

	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

int8_t smi230_gyro_get_data_ex(int16_t *x, int16_t *y, int16_t *z)
{
	int8_t rslt;
	struct smi230_sensor_data data = { 0 };

	rslt = smi230_gyro_get_data(&data, g_smi230_gyro_dev);
	*x = data.x;
	*y = data.y;
	*z = data.z;

	return rslt;
}
EXPORT_SYMBOL(smi230_gyro_get_data_ex);

int8_t smi230_gyro_set_power_mode(const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t power_mode, data;
	uint8_t is_power_switching_mode_valid = TRUE;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);
		if (rslt == SMI230_OK) {
			power_mode = dev->gyro_cfg.power;

			/* switching between normal mode and the suspend modes is allowed, it is not possible to switch
			 * between suspend and deep suspend and vice versa. Check for invalid power switching (i.e)
			 * deep suspend to suspend
			 */
			if ((power_mode == SMI230_GYRO_PM_SUSPEND) &&
			    (data == SMI230_GYRO_PM_DEEP_SUSPEND))
				is_power_switching_mode_valid = FALSE;

			/* Check for invalid power switching (i.e) from suspend to deep suspend */
			if ((power_mode == SMI230_GYRO_PM_DEEP_SUSPEND) &&
			    (data == SMI230_GYRO_PM_SUSPEND))
				is_power_switching_mode_valid = FALSE;

			/* Check if power switching mode is valid*/
			if (is_power_switching_mode_valid) {
				rslt = set_regs(SMI230_GYRO_LPM1_REG,
						&power_mode, 1, dev);
				if (rslt == SMI230_OK)
					dev->delay_ms(
						SMI230_GYRO_POWER_MODE_CONFIG_DELAY);

			} else
				rslt = SMI230_E_INVALID_INPUT;
		}
	}

	return rslt;
}

static int smi230_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	int ret;
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = smi230_gyro_get_data(&data, p_smi230_gyro_dev);
		if (ret != SMI230_OK)
			return 0;
		switch (chan->channel2) {
		case IIO_MOD_X:
			*val = data.x;
			break;
		case IIO_MOD_Y:
			*val = data.y;
			break;
		case IIO_MOD_Z:
			*val = data.z;
			break;
		}
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = smi230_gyro_get_meas_conf(p_smi230_gyro_dev);
		if (ret)
			return ret;
		switch (p_smi230_gyro_dev->gyro_cfg.odr) {
		case SMI230_GYRO_BW_523_ODR_2000_HZ:
			*val = 2000;
			break;
		case SMI230_GYRO_BW_116_ODR_1000_HZ:
			*val = 1000;
			break;
		case SMI230_GYRO_BW_47_ODR_400_HZ:
			*val = 400;
			break;
		case SMI230_GYRO_BW_64_ODR_200_HZ:
			*val = 200;
			break;
		case SMI230_GYRO_BW_32_ODR_100_HZ:
			*val = 100;
			break;
		default:
			return -EINVAL;
		}
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);

#ifdef CONFIG_SMI230_ACC_DATA_SYNC
	dev_info(&indio_dev->dev,
		 "in datasync mode, setting freq. is only possible in ACC.");
	return 0;
#endif
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (val) {
		case 2000:
			p_smi230_gyro_dev->gyro_cfg.odr =
				SMI230_GYRO_BW_523_ODR_2000_HZ;
			break;
		case 1000:
			p_smi230_gyro_dev->gyro_cfg.odr =
				SMI230_GYRO_BW_116_ODR_1000_HZ;
			break;
		case 400:
			p_smi230_gyro_dev->gyro_cfg.odr =
				SMI230_GYRO_BW_47_ODR_400_HZ;
			break;
		case 200:
			p_smi230_gyro_dev->gyro_cfg.odr =
				SMI230_GYRO_BW_64_ODR_200_HZ;
			break;
		case 100:
			p_smi230_gyro_dev->gyro_cfg.odr =
				SMI230_GYRO_BW_32_ODR_100_HZ;
			break;
		default:
			return -EINVAL;
		}

		ret = smi230_gyro_set_meas_conf(p_smi230_gyro_dev);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static ssize_t self_test_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int err;
	uint8_t data = 0x1;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = set_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, p_smi230_gyro_dev);
	smi230_gyro_delay(100);
	err |= get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, p_smi230_gyro_dev);
	if (err)
		goto done;

	if (data & 0x2) {
		if (data & 0x4)
			return snprintf(buf, PAGE_SIZE,
					"selftest result: Failure\n");
		else
			return snprintf(buf, PAGE_SIZE,
					"selftest result: OK\n");
	}

done:
	return snprintf(buf, PAGE_SIZE, "selftest not performed\n");
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	for (i = 0; i <= 0x3F; i++) {
		err = get_regs(i, &data, 1, p_smi230_gyro_dev);
		if (err) {
			pr_err("register dump failed");
			return err;
		}
		pr_info("0x%x = 0x%x", i, data);
		if (i % 15 == 0)
			pr_info("\n");
	}

	return 0;
}

static ssize_t anglvel_raw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	ret = smi230_gyro_get_data(&data, p_smi230_gyro_dev);
	if (ret != SMI230_OK)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd\n", data.x, data.y,
			data.z);
}

int8_t smi230_gyro_get_power_mode(struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);

		if (rslt == SMI230_OK)
			dev->gyro_cfg.power = data;
	}

	return rslt;
}

static ssize_t power_mode_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err = 0;
	unsigned long pwr_cfg;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = kstrtoul(buf, 10, &pwr_cfg);
	if (err)
		return err;
	if (pwr_cfg == 0) {
		p_smi230_gyro_dev->gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
		err = smi230_gyro_set_power_mode(p_smi230_gyro_dev);
	} else if (pwr_cfg == 1) {
		p_smi230_gyro_dev->gyro_cfg.power = SMI230_GYRO_PM_SUSPEND;
		err = smi230_gyro_set_power_mode(p_smi230_gyro_dev);
	} else if (pwr_cfg == 2) {
		p_smi230_gyro_dev->gyro_cfg.power = SMI230_GYRO_PM_DEEP_SUSPEND;
		err = smi230_gyro_set_power_mode(p_smi230_gyro_dev);
	} else {
		dev_info(dev, "invalid param");
		return count;
	}

	dev_info(dev, "set power cfg to %ld, err %d", pwr_cfg, err);

	if (err) {
		dev_info(dev, "setting power config failed");
		return err;
	}
	return count;
}

static ssize_t power_mode_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = smi230_gyro_get_power_mode(p_smi230_gyro_dev);
	if (err) {
		dev_info(dev, "read failed");
		return err;
	}
	return snprintf(buf, PAGE_SIZE,
			"%x (0:active 1:suspend 2:deep suspend)\n",
			p_smi230_gyro_dev->gyro_cfg.power);
}

static int8_t smi230_gyro_soft_reset(const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_SOFT_RESET_CMD;
		rslt = set_regs(SMI230_GYRO_SOFTRESET_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			/* delay 30 ms after writing reset value to its register */
			dev->delay_ms(SMI230_GYRO_SOFTRESET_DELAY);
		}
	}

	return rslt;
}

static ssize_t bw_odr_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int err;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = smi230_gyro_get_meas_conf(p_smi230_gyro_dev);
	if (err) {
		dev_info(dev, "read ODR failed");
		return err;
	}

	switch (p_smi230_gyro_dev->gyro_cfg.odr) {
	case SMI230_GYRO_BW_523_ODR_2000_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:523 ODR:2000");
	case SMI230_GYRO_BW_230_ODR_2000_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:230 ODR:2000");
	case SMI230_GYRO_BW_116_ODR_1000_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:116 ODR:1000");
	case SMI230_GYRO_BW_47_ODR_400_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:47 ODR:400");
	case SMI230_GYRO_BW_23_ODR_200_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:23 ODR:200");
	case SMI230_GYRO_BW_12_ODR_100_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:12 ODR:100");
	case SMI230_GYRO_BW_64_ODR_200_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:64 ODR:200");
	case SMI230_GYRO_BW_32_ODR_100_HZ:
		return snprintf(buf, PAGE_SIZE, "%s\n", "BW:32 ODR:100");
	default:
		return snprintf(buf, PAGE_SIZE, "%s\n", "error");
	}
}

static ssize_t bw_odr_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int err = 0, bw, odr;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &bw);
	if (err) {
		dev_info(dev, "read ODR failed");
		return err;
	}

	switch (bw) {
	case 523:
		p_smi230_gyro_dev->gyro_cfg.odr =
			SMI230_GYRO_BW_523_ODR_2000_HZ;
		odr = 2000;
		break;
	case 230:
		p_smi230_gyro_dev->gyro_cfg.odr =
			SMI230_GYRO_BW_230_ODR_2000_HZ;
		odr = 2000;
		break;
	case 116:
		p_smi230_gyro_dev->gyro_cfg.odr =
			SMI230_GYRO_BW_116_ODR_1000_HZ;
		odr = 1000;
		break;
	case 47:
		p_smi230_gyro_dev->gyro_cfg.odr = SMI230_GYRO_BW_47_ODR_400_HZ;
		odr = 400;
		break;
	case 23:
		p_smi230_gyro_dev->gyro_cfg.odr = SMI230_GYRO_BW_23_ODR_200_HZ;
		odr = 200;
		break;
	case 12:
		p_smi230_gyro_dev->gyro_cfg.odr = SMI230_GYRO_BW_12_ODR_100_HZ;
		odr = 100;
		break;
	case 64:
		p_smi230_gyro_dev->gyro_cfg.odr = SMI230_GYRO_BW_64_ODR_200_HZ;
		odr = 200;
		break;
	case 32:
		p_smi230_gyro_dev->gyro_cfg.odr = SMI230_GYRO_BW_32_ODR_100_HZ;
		odr = 100;
		break;
	default:
		dev_info(dev, "ODR not supported");
		return count;
	}

	err |= smi230_gyro_set_meas_conf(p_smi230_gyro_dev);

	dev_info(dev, "set bw to %d, odr to %d, err %d", bw, odr, err);

	if (err) {
		dev_info(dev, "setting ODR failed");
		return err;
	}
	return count;
}

static ssize_t range_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int err, range = 0;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = smi230_gyro_get_meas_conf(p_smi230_gyro_dev);
	if (err) {
		dev_info(dev, "read range failed");
		return err;
	}

	switch (p_smi230_gyro_dev->gyro_cfg.range) {
	case SMI230_GYRO_RANGE_2000_DPS:
		range = 2000;
		break;
	case SMI230_GYRO_RANGE_1000_DPS:
		range = 1000;
		break;
	case SMI230_GYRO_RANGE_500_DPS:
		range = 500;
		break;
	case SMI230_GYRO_RANGE_250_DPS:
		range = 250;
		break;
	case SMI230_GYRO_RANGE_125_DPS:
		range = 125;
		break;
	default:
		dev_info(dev, "wrong range read");
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", range);
}

static ssize_t range_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int err = 0, range;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = kstrtoint(buf, 10, &range);
	if (err) {
		dev_info(dev, "invalid params");
		return err;
	}

	switch (range) {
	case 2000:
		p_smi230_gyro_dev->gyro_cfg.range = SMI230_GYRO_RANGE_2000_DPS;
		break;
	case 1000:
		p_smi230_gyro_dev->gyro_cfg.range = SMI230_GYRO_RANGE_1000_DPS;
		break;
	case 500:
		p_smi230_gyro_dev->gyro_cfg.range = SMI230_GYRO_RANGE_500_DPS;
		break;
	case 250:
		p_smi230_gyro_dev->gyro_cfg.range = SMI230_GYRO_RANGE_250_DPS;
		break;
	case 125:
		p_smi230_gyro_dev->gyro_cfg.range = SMI230_GYRO_RANGE_125_DPS;
		break;
	default:
		dev_info(dev, "range not supported");
		return count;
	}

	err |= smi230_gyro_set_meas_conf(p_smi230_gyro_dev);

	dev_info(dev, "set range to %d, err %d", range, err);

	if (err) {
		dev_info(dev, "setting range failed");
		return err;
	}
	return count;
}

static int smi230_get_odr_value(const struct smi230_gyro_dev *dev)
{
	switch (dev->gyro_cfg.odr) {
	case SMI230_GYRO_BW_523_ODR_2000_HZ:
		return 2000;
	case SMI230_GYRO_BW_116_ODR_1000_HZ:
		return 1000;
	case SMI230_GYRO_BW_47_ODR_400_HZ:
		return 400;
	case SMI230_GYRO_BW_64_ODR_200_HZ:
		return 200;
	case SMI230_GYRO_BW_32_ODR_100_HZ:
		return 100;
	default:
		return -EINVAL;
	}
}

int8_t smi230_gyro_get_fifo_wm(uint8_t *wm, const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, &data, 1, dev);
		if ((rslt == SMI230_OK) && (wm != NULL))
			*wm = data & 0x7F;
		else
			rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

int8_t smi230_gyro_set_fifo_config(struct gyro_fifo_config *config,
				   const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = set_regs(SMI230_GYRO_FIFO_CONFIG_1_ADDR, &config->mode,
				1, dev);
		rslt |= set_regs(SMI230_GYRO_WM_INT_REG, &config->wm_en, 1,
				 dev);

		if (config->int3_en ^ config->int4_en) {
			if (config->int3_en)
				data = 0x10;
			else
				data = 0x18;
			rslt |= set_regs(SMI230_GYRO_FIFO_EXT_INT_S_REG, &data,
					 1, dev);
		} else
			rslt |= set_regs(SMI230_GYRO_FIFO_EXT_INT_S_REG, &data,
					 1, dev);
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t smi230_gyro_set_fifo_wm(uint8_t wm,
				      const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK)
		rslt = set_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, &wm, 1, dev);

	return rslt;
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);
	uint8_t wm;

	smi230_gyro_get_fifo_wm(&wm, p_smi230_gyro_dev);

	return sprintf(buf, "%d\n", wm);
}

static ssize_t hwfifo_watermark_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int err = 0;
	uint16_t fifo_wm;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);

	err = kstrtou16(buf, 10, &fifo_wm);
	if (err) {
		dev_err(dev, "Invalid argument");
		return err;
	}

	dev_info(indio_dev->dev.parent, "fifo watermark set to %d", fifo_wm);

	err = smi230_gyro_set_fifo_wm(fifo_wm, p_smi230_gyro_dev);
	if (err != SMI230_OK) {
		dev_err(dev, "Failed to set gyro FIFO WM.");
		return err;
	}

	return count;
}

static ssize_t hwfifo_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", fifo_config.wm_en);
}

static void unpack_gyro_data(struct smi230_sensor_data *gyro,
			     uint16_t data_start_index,
			     const struct smi230_fifo_frame *fifo)
{
	uint16_t data_lsb;
	uint16_t data_msb;

	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->x = (int16_t)((data_msb << 8) | data_lsb);

	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->y = (int16_t)((data_msb << 8) | data_lsb);

	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->z = (int16_t)((data_msb << 8) | data_lsb);
}

static int8_t extract_gyro_headerless_mode(struct smi230_sensor_data *gyro,
					   uint8_t *fifo_length,
					   struct smi230_fifo_frame *fifo)
{
	int8_t rslt = SMI230_OK;
	uint16_t data_index;
	uint16_t gyro_index = 0;
	uint8_t frame_to_read = *fifo_length;

	for (data_index = 0; data_index < fifo->length;) {
		unpack_gyro_data(&gyro[gyro_index], data_index, fifo);

		data_index += SMI230_FIFO_GYRO_FRAME_LENGTH;
		gyro_index++;

		/* Break if Number of frames to be read is complete or FIFO is mpty */
		if ((frame_to_read == gyro_index) ||
		    (rslt == SMI230_W_FIFO_EMPTY)) {
			break;
		}
	}

	(*fifo_length) = gyro_index;

	return rslt;
}

static int8_t smi230_gyro_extract_fifo(struct smi230_sensor_data *gyro_data,
				       uint8_t *fifo_length,
				       struct smi230_fifo_frame *fifo,
				       const struct smi230_gyro_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (gyro_data != NULL) &&
	    (fifo_length != NULL) && (fifo != NULL)) {
		rslt = extract_gyro_headerless_mode(gyro_data, fifo_length,
						    fifo);
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int smi230_gyro_init(struct smi230_gyro_dev *dev)
{
	int err = 0;

	err = smi230_gyro_chip_id_check(dev);
	err |= smi230_gyro_soft_reset(dev);

	dev->gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
	err |= smi230_gyro_set_power_mode(dev);

	err |= smi230_gyro_get_meas_conf(dev);

	smi230_gyro_delay(100);

	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin =
		SMI230_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
		SMI230_DISABLE;

	if (IS_ENABLED(CONFIG_SMI230_GYRO_INT_ACTIVE_HIGH)) {
		int_config.gyro_int_config_1.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_2.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_HIGH;
	} else {
		int_config.gyro_int_config_1.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_LOW;
		int_config.gyro_int_config_2.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_LOW;
	}

	int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
	int_config.gyro_int_config_1.int_pin_cfg.output_mode =
		SMI230_INT_MODE_PUSH_PULL;
	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode =
		SMI230_INT_MODE_PUSH_PULL;

	if (IS_ENABLED(CONFIG_SMI230_GYRO_NEW_DATA)) {
		int_config.gyro_int_config_1.int_type =
			SMI230_GYRO_DATA_RDY_INT;
		int_config.gyro_int_config_2.int_type =
			SMI230_GYRO_DATA_RDY_INT;
	} else if (IS_ENABLED(CONFIG_SMI230_GYRO_FIFO)) {
		int_config.gyro_int_config_1.int_type = SMI230_GYRO_FIFO_INT;
		int_config.gyro_int_config_2.int_type = SMI230_GYRO_FIFO_INT;

		fifo_config.mode = SMI230_GYRO_FIFO_MODE;
		/* 0x88 to enable wm int, 0x80 to disable */
		fifo_config.wm_en = 0x88;
		err |= smi230_gyro_set_fifo_wm(100, dev);

		fifo_config.int3_en = 0;
		fifo_config.int4_en = 0;

		err |= smi230_gyro_set_fifo_config(&fifo_config, dev);
		if (err != SMI230_OK)
			return err;

		smi230_gyro_delay(100);
	}

	if (IS_ENABLED(CONFIG_SMI230_GYRO_DATA_SYNC)) {
		int_config.gyro_int_config_1.int_channel = SMI230_INT_CHANNEL_3;
		int_config.gyro_int_config_1.int_type =
			SMI230_GYRO_DATA_RDY_INT;
		int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin =
			SMI230_ENABLE;
		int_config.gyro_int_config_1.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_1.int_pin_cfg.output_mode =
			SMI230_INT_MODE_PUSH_PULL;

		int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
		int_config.gyro_int_config_2.int_type =
			SMI230_GYRO_DATA_RDY_INT;
		int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin =
			SMI230_DISABLE;
		int_config.gyro_int_config_2.int_pin_cfg.lvl =
			SMI230_INT_ACTIVE_HIGH;
		int_config.gyro_int_config_2.int_pin_cfg.output_mode =
			SMI230_INT_MODE_PUSH_PULL;
	}

	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_1, dev);
	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_2, dev);

	smi230_gyro_delay(100);

	return err;
}

static ssize_t soft_reset_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int err;
	struct smi230_gyro_dev *p_smi230_gyro_dev = dev_get_drvdata(dev);

	err = smi230_gyro_init(p_smi230_gyro_dev);
	if (err)
		return snprintf(buf, PAGE_SIZE, "%s\n", "softreset failed");

	return snprintf(buf, PAGE_SIZE, "softreset performed successfully\n");
}

static IIO_CONST_ATTR(hwfifo_watermark_min, "10");
static IIO_CONST_ATTR(hwfifo_watermark_max, "100");
static IIO_DEVICE_ATTR_RO(hwfifo_enabled, 0);
static IIO_DEVICE_ATTR_RW(hwfifo_watermark, 0);

static DEVICE_ATTR_RO(anglvel_raw);

static DEVICE_ATTR_RO(self_test);
static DEVICE_ATTR_RO(reg_dump);
static DEVICE_ATTR_RW(power_mode);

#ifndef CONFIG_SMI230_ACC_DATA_SYNC
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("100 200 400 1000 2000");
#endif
static IIO_CONST_ATTR(anglvel_scale_available,
		      "0.008 0.004 0.002 0.001 0.0005");
static DEVICE_ATTR_RW(bw_odr);
static DEVICE_ATTR_RO(soft_reset);
static DEVICE_ATTR_RW(range);

static struct attribute *smi230_attrs[] = {
#ifndef CONFIG_SMI230_ACC_DATA_SYNC
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
#endif
	&iio_const_attr_anglvel_scale_available.dev_attr.attr,
	&dev_attr_self_test.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_anglvel_raw.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_bw_odr.attr,
	&dev_attr_range.attr,
	&dev_attr_soft_reset.attr,
	NULL,
};

static const struct attribute *smi230_fifo_attributes[] = {
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi230_attrs_group = {
	.attrs = smi230_attrs,
};

static const struct iio_info smi230_info = {
	.read_raw = smi230_read_raw,
	.write_raw = smi230_write_raw,
	.attrs = &smi230_attrs_group,
};

static int smi230_read_channel(struct smi230_sensor_data *data, int i,
			       s16 *sample)
{
	switch (i) {
	case 0:
		*sample = data->x;
		break;
	case 1:
		*sample = data->y;
		break;
	case 2:
		*sample = data->z;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int8_t smi230_gyro_read_fifo_data(struct smi230_fifo_frame *fifo,
					 const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t addr = SMI230_GYRO_FIFO_DATA_ADDR;

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (fifo != NULL))
		rslt = get_regs(addr, fifo->data, fifo->length, dev);
	else
		rslt = SMI230_E_NULL_PTR;

	return rslt;
}

static int8_t smi230_gyro_get_fifo_length(uint8_t *fifo_length,
					  const struct smi230_gyro_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (fifo_length != NULL)) {
		rslt = get_regs(SMI230_GYRO_FIFO_STATUS_ADDR, &data, 1, dev);
		if (rslt == SMI230_OK)
			(*fifo_length) = data & 0x7F;
		else
			rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static irqreturn_t smi230_fifo_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);

	struct smi230_fifo_frame fifo;
	int err = 0, i, j, k, time_step_ns;
	uint8_t fifo_length, extract_length, int_stat;
	s16 buf[8];
	s16 sample;

	err = get_regs(SMI230_GYRO_INT_STAT_1_REG, &int_stat, 1,
		       p_smi230_gyro_dev);
	if (err != SMI230_OK) {
		dev_info(indio_dev->dev.parent, "FIFO interrupt error!");
		goto done;
	}

	dev_info(indio_dev->dev.parent, "int_stat %d", int_stat);
	if ((int_stat & (SMI230_GYRO_INT_STAT_FIFO)) == 0)
		goto done;

	err = smi230_gyro_get_fifo_length(&fifo_length, p_smi230_gyro_dev);
	if (err != SMI230_OK) {
		dev_info(indio_dev->dev.parent, "FIFO get length error!");
		goto done;
	}
	dev_info(indio_dev->dev.parent, "fifo length %d", fifo_length);

	fifo.data = fifo_buf;
	fifo.length = fifo_length * SMI230_FIFO_GYRO_FRAME_LENGTH;
	err = smi230_gyro_read_fifo_data(&fifo, p_smi230_gyro_dev);
	if (err != SMI230_OK) {
		dev_info(indio_dev->dev.parent, "FIFO read data error %d", err);
		goto done;
	}

	extract_length = SMI230_MAX_GYRO_FIFO_FRAME;
	err = smi230_gyro_extract_fifo(fifo_gyro_data, &extract_length, &fifo,
				       p_smi230_gyro_dev);

	time_step_ns = 1000000000 / smi230_get_odr_value(p_smi230_gyro_dev);
	dev_info(indio_dev->dev.parent,
		 "fifo length extracted %d, odr %d, step %d ns", extract_length,
		 p_smi230_gyro_dev->gyro_cfg.odr, time_step_ns);
	for (k = 0; k < extract_length; k++) {
		j = 0;
		for_each_set_bit(i, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			err = smi230_read_channel(&fifo_gyro_data[k], i,
						  &sample);
			if (err) {
				dev_info(indio_dev->dev.parent,
					 "Read channel %d failed", i);
				goto done;
			}
			//dev_info(indio_dev->dev.parent, "mask %d, sample %d", i, sample);
			buf[j++] = sample;
		}
		err = iio_push_to_buffers_with_timestamp(
			indio_dev, buf,
			pf->timestamp - time_step_ns * (extract_length - k));
		dev_info(indio_dev->dev.parent, "process %d in fifo, time %lld",
			 k,
			 pf->timestamp - time_step_ns * (extract_length - k));
		if (err) {
			dev_info(indio_dev->dev.parent,
				 "Push to buffer failed");
			goto done;
		}
	}

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static irqreturn_t smi230_new_data_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);

	/* hand code the buffer size for now */
	s16 buf[8];
	s16 sample;
	int ret, i, j = 0;
	struct smi230_sensor_data sensor_data = { 0 };

	dev_info(indio_dev->dev.parent, "New data handler!");
	ret = smi230_gyro_get_data(&sensor_data, p_smi230_gyro_dev);
	if (ret) {
		dev_info(indio_dev->dev.parent, "Reading sensor data failed");
		goto done;
	}

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = smi230_read_channel(&sensor_data, i, &sample);
		if (ret) {
			dev_info(indio_dev->dev.parent,
				 "Read channel %d failed", i);
			goto done;
		}
		buf[j++] = sample;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, pf->timestamp);
	if (ret)
		dev_info(indio_dev->dev.parent, "Push to buffer failed");
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int smi230_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230_gyro_dev *p_smi230_gyro_dev =
		iio_device_get_drvdata(indio_dev);
	u8 en;

	dev_info(indio_dev->dev.parent, "trigger set state %d", enable);
	if (enable)
		en = SMI230_ENABLE;
	else
		en = SMI230_DISABLE;

	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = en;
	return smi230_gyro_set_int_config(&int_config.gyro_int_config_2,
					  p_smi230_gyro_dev);
}

static const struct iio_trigger_ops smi230_trigger_ops = {
	.set_trigger_state = &smi230_trigger_set_state,
};

static int smi230_get_irq(struct device *dev, int *irq)
{
	int gpio_pin, ret;

	gpio_pin = of_get_named_gpio_flags(dev->of_node, "gpio_irq", 0, NULL);

	dev_info(dev, "gpio pin %d", gpio_pin);
	ret = gpio_request_one(gpio_pin, GPIOF_IN, "smi230_gyro_interrupt");
	if (ret) {
		dev_err(dev, "Request GPIO pin %d failed", gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(gpio_pin);
	if (ret) {
		dev_err(dev, "Set GPIO direction for pin %d failed", gpio_pin);
		gpio_free(gpio_pin);
		return ret;
	}

	*irq = gpio_to_irq(gpio_pin);

	return ret;
}

int smi230_gyro_core_probe(struct device *dev,
			   struct smi230_gyro_dev *p_smi230_gyro_dev)
{
	int ret = 0;
	int irq;
	struct iio_dev *indio_dev;

	ret = smi230_gyro_init(p_smi230_gyro_dev);
	if (ret == SMI230_OK) {
		dev_info(dev, "Bosch Sensor %s hardware initialized",
			 SENSOR_GYRO_NAME);
	} else {
		dev_err(dev,
			"Bosch Sensor %s hardware initialization failed, error %d",
			SENSOR_GYRO_NAME, ret);
		return ret;
	}

	/*store the dev handler for external call use*/
	g_smi230_gyro_dev = p_smi230_gyro_dev;

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev) {
		dev_err(dev, "Bosch Sensor %s iio device alloc failed",
			SENSOR_GYRO_NAME);
		return -ENOMEM;
	}

	iio_device_set_drvdata(indio_dev, p_smi230_gyro_dev);
	dev_set_drvdata(dev, p_smi230_gyro_dev);
	indio_dev->dev.parent = dev;
	indio_dev->name = MODULE_GYRO_NAME;
	indio_dev->info = &smi230_info;

	if (IS_ENABLED(CONFIG_SMI230_GYRO_DATA_SYNC))
		return devm_iio_device_register(dev, indio_dev);

	indio_dev->channels = smi230_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi230_channels);
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-trigger",
						 indio_dev->name);
	if (indio_dev->trig == NULL)
		return -ENOMEM;

	dev_info(dev, "Bosch Sensor %s device alloced", SENSOR_GYRO_NAME);

	indio_dev->trig->dev.parent = dev;
	indio_dev->trig->ops = &smi230_trigger_ops;

	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, indio_dev->trig);
	if (ret)
		return ret;

	dev_info(dev, "Bosch Sensor %s trigger registered", SENSOR_GYRO_NAME);

	if (IS_ENABLED(CONFIG_SMI230_GYRO_FIFO))
		ret = devm_iio_triggered_buffer_setup_ext(
			dev, indio_dev, iio_pollfunc_store_time,
			smi230_fifo_trigger_handler, NULL,
			smi230_fifo_attributes);
	else
		ret = devm_iio_triggered_buffer_setup(
			dev, indio_dev, iio_pollfunc_store_time,
			smi230_new_data_trigger_handler, NULL);

	if (ret) {
		dev_err(dev, "Setup triggered buffer failed");
		return ret;
	}

	dev_info(dev, "Bosch Sensor %s trigger buffer registered",
		 SENSOR_GYRO_NAME);

	ret = smi230_get_irq(dev, &irq);
	if (ret) {
		dev_err(dev, "Bosch Sensor %s gpio to irq failed",
			SENSOR_GYRO_NAME);
		return ret;
	}

	dev_info(dev, "irq number %d", irq);
	ret = devm_request_irq(&indio_dev->dev, irq,
			       &iio_trigger_generic_data_rdy_poll,
			       IRQF_TRIGGER_RISING, indio_dev->name,
			       indio_dev->trig);
	if (ret) {
		dev_err(dev, "Bosch Sensor %s request irq failed",
			SENSOR_GYRO_NAME);
		goto free_gpio;
	}

	dev_info(dev, "Bosch Sensor %s irq alloced", SENSOR_GYRO_NAME);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Bosch Sensor %s iio device register failed",
			SENSOR_GYRO_NAME);
		goto free_gpio;
	}

	return ret;

free_gpio:
	smi230_gyro_core_remove(dev);
	return ret;
}

int smi230_gyro_core_remove(struct device *dev)
{
	int gpio_pin;

	gpio_pin = of_get_named_gpio_flags(dev->of_node, "gpio_irq", 0, NULL);
	dev_info(dev, "free gpio pin %d", gpio_pin);
	gpio_free(gpio_pin);
	return 0;
}

MODULE_DESCRIPTION("SMI230 GYRO SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
