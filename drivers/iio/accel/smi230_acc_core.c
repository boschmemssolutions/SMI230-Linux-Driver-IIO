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

#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/of_gpio.h>
#include <linux/ioport.h>

#include "smi230_acc.h"

#define SMI230_CHANNEL(_type, _axis, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
}

static uint8_t temp_buff[CONFIG_IIO_SMI230_ACC_MAX_BUFFER_LEN + 1];

static const struct iio_chan_spec smi230_channels[] = {
	SMI230_CHANNEL(IIO_ACCEL, X, 0),
	SMI230_CHANNEL(IIO_ACCEL, Y, 1),
	SMI230_CHANNEL(IIO_ACCEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct smi230_int_cfg int_config;

int8_t null_ptr_check(const struct smi230_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL)
	    || (dev->delay_ms == NULL))
		rslt = SMI230_E_NULL_PTR;
	else
		rslt = SMI230_OK;

	return rslt;
}

static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_dev *dev)
{
	int8_t rslt;
	uint16_t index;
	uint16_t temp_len = len + dev->dummy_byte;

	if (WARN(temp_len > CONFIG_IIO_SMI230_ACC_MAX_BUFFER_LEN,
			"SMI230 buffer overflow\n"))
		return SMI230_E_COM_FAIL;

	if (dev->intf == SMI230_SPI_INTF)
		reg_addr = reg_addr | SMI230_SPI_RD_MASK;

	rslt = dev->read(dev->accel_id, reg_addr, temp_buff, temp_len);

	if (rslt == SMI230_OK) {
		for (index = 0; index < len; index++)
			reg_data[index] = temp_buff[index + dev->dummy_byte];
	} else {
		rslt = SMI230_E_COM_FAIL;
	}

	return rslt;
}

static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_SPI_INTF) {
		reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
	}

	rslt = dev->write(dev->accel_id, reg_addr, reg_data, len);

	if (rslt != SMI230_OK)
		rslt = SMI230_E_COM_FAIL;

	return rslt;
}

static int8_t set_int_pin_config(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = 0;
	uint8_t data;
	uint8_t is_channel_invalid = FALSE;

	switch (int_config->int_channel) {
	case SMI230_INT_CHANNEL_1:
		reg_addr = SMI230_ACCEL_INT1_IO_CONF_REG;
		break;

	case SMI230_INT_CHANNEL_2:
		reg_addr = SMI230_ACCEL_INT2_IO_CONF_REG;
		break;

	default:
		is_channel_invalid = TRUE;
		break;
	}

	if (!is_channel_invalid) {
		rslt = get_regs(reg_addr, &data, 1, dev);

		if (rslt == SMI230_OK) {
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_LVL,
					int_config->int_pin_cfg.lvl);

			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_OD,
					    int_config->int_pin_cfg.output_mode);

			if (int_config->int_type == SMI230_ACCEL_SYNC_INPUT) {
				data = SMI230_SET_BITS_POS_0(data, SMI230_ACCEL_INT_EDGE,
							  SMI230_ENABLE);

				data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IN,
						    int_config->int_pin_cfg.enable_int_pin);

				data = SMI230_SET_BIT_VAL_0(data, SMI230_ACCEL_INT_IO);
			} else {
				data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IO,
						int_config->int_pin_cfg.enable_int_pin);

				data = SMI230_SET_BIT_VAL_0(data, SMI230_ACCEL_INT_IN);
			}

			rslt = set_regs(reg_addr, &data, 1, dev);
		}
	} else {
		rslt = SMI230_E_INVALID_INPUT;
	}

	return rslt;
}

static int8_t set_accel_data_ready_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0, conf;

	rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

	if (rslt == SMI230_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_DRDY, conf);
			break;

		case SMI230_INT_CHANNEL_2:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_DRDY, conf);
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = set_int_pin_config(int_config, dev);
			if (rslt == SMI230_OK)
				rslt = set_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG,
						&data, 1, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_sync_data_ready_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {

		data = SMI230_ACCEL_INTA_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data = SMI230_ACCEL_INTA_ENABLE;

			rslt = set_regs(reg_addr, &data, 1, dev);
			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_sync_input(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK)
		rslt = set_int_pin_config(int_config, dev);

	return rslt;
}

static int8_t set_accel_anymotion_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_INTB_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data = SMI230_ACCEL_INTB_ENABLE;

			rslt = set_regs(reg_addr, &data, 1, dev);
			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_high_g_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_HIGH_G_INT_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(reg_addr, &data, 1, dev);

			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data |= SMI230_ACCEL_HIGH_G_INT_ENABLE;
			else
				data &= ~SMI230_ACCEL_HIGH_G_INT_ENABLE;

			if (rslt == SMI230_OK)
				rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_low_g_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_LOW_G_INT_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(reg_addr, &data, 1, dev);

			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data |= SMI230_ACCEL_LOW_G_INT_ENABLE;
			else
				data &= ~SMI230_ACCEL_LOW_G_INT_DISABLE;

			if (rslt == SMI230_OK)
				rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_orient_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_ORIENT_INT_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(reg_addr, &data, 1, dev);

			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data |= SMI230_ACCEL_ORIENT_INT_ENABLE;
			else
				data &= ~SMI230_ACCEL_ORIENT_INT_ENABLE;

			if (rslt == SMI230_OK)
				rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_no_motion_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_NO_MOT_INT_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(reg_addr, &data, 1, dev);

			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data |= SMI230_ACCEL_NO_MOT_INT_ENABLE;
			else
				data &= ~SMI230_ACCEL_NO_MOT_INT_ENABLE;

			if (rslt == SMI230_OK)
				rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t set_accel_err_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data, reg_addr = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_ACCEL_ERR_INT_DISABLE;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			break;

		case SMI230_INT_CHANNEL_2:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(reg_addr, &data, 1, dev);

			if (int_config->int_pin_cfg.enable_int_pin == SMI230_ENABLE)
				data |= SMI230_ACCEL_ERR_INT_ENABLE;
			else
				data &= ~SMI230_ACCEL_ERR_INT_ENABLE;

			if (rslt == SMI230_OK)
				rslt = set_regs(reg_addr, &data, 1, dev);

			if (rslt == SMI230_OK)
				rslt = set_int_pin_config(int_config, dev);
		}
	}

	return rslt;
}

static int8_t smi230_acc_soft_reset(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		data = SMI230_SOFT_RESET_CMD;
		rslt = set_regs(SMI230_ACCEL_SOFTRESET_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			dev->delay_ms(SMI230_ACCEL_SOFTRESET_DELAY_MS);

			/* After soft reset SPI mode in the initialization phase, need to
			 * perform a dummy SPI read operation.
			 * The soft-reset performs a fundamental reset to the device,
			 * which is largely equivalent to a power cycle.
			 */
			if (dev->intf == SMI230_SPI_INTF) {
				/* Dummy SPI read operation of Chip-ID */
				rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &data, 1, dev);
			}
		}
	}

	return rslt;
}

static int8_t smi230_acc_get_meas_conf(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);

		if (rslt == SMI230_OK) {
			dev->accel_cfg.odr = data[0] & SMI230_ACCEL_ODR_MASK;
			dev->accel_cfg.bw = (data[0] & SMI230_ACCEL_BW_MASK) >> 4;
			dev->accel_cfg.range = data[1] & SMI230_ACCEL_RANGE_MASK;
		}
	}

	return rslt;
}

static int8_t smi230_acc_set_meas_conf(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };
	uint8_t bw, range, odr;
	uint8_t is_odr_invalid = FALSE;
	uint8_t is_bw_invalid = FALSE;
	uint8_t is_range_invalid = FALSE;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		odr = dev->accel_cfg.odr;
		bw = dev->accel_cfg.bw;
		range = dev->accel_cfg.range;

		if ((odr < SMI230_ACCEL_ODR_12_5_HZ)
				|| (odr > SMI230_ACCEL_ODR_1600_HZ)) {
			is_odr_invalid = TRUE;
		}

		if (bw > SMI230_ACCEL_BW_NORMAL)
			is_bw_invalid = TRUE;

		if (range > SMI230_ACCEL_RANGE_16G)
			is_range_invalid = TRUE;

		/* If ODR, BW and Range are valid, write it to accel config registers */
		if ((!is_odr_invalid) && (!is_bw_invalid) && (!is_range_invalid)) {
			rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
			if (rslt == SMI230_OK) {
				/* Update data with new odr and bw values */
				data[0] = SMI230_SET_BITS_POS_0(data[0], SMI230_ACCEL_ODR, odr);
				data[0] = SMI230_SET_BITS(data[0], SMI230_ACCEL_BW, bw);

				/* Update data with current range values */
				data[1] = SMI230_SET_BITS_POS_0(data[1], SMI230_ACCEL_RANGE,
						range);

				/* write to range register */
				rslt = set_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
			}
		} else {
			rslt = SMI230_E_INVALID_CONFIG;
		}
	}

	return rslt;
}

static int8_t smi230_acc_set_power_mode(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t power_mode;
	uint8_t data[2];

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		power_mode = dev->accel_cfg.power;

		if (power_mode == SMI230_ACCEL_PM_ACTIVE) {
			data[0] = SMI230_ACCEL_PM_ACTIVE;
			data[1] = SMI230_ACCEL_POWER_ENABLE;
		} else if (power_mode == SMI230_ACCEL_PM_SUSPEND) {
			data[0] = SMI230_ACCEL_PM_SUSPEND;
			data[1] = SMI230_ACCEL_POWER_DISABLE;
		} else {
			/* Invalid power input */
			rslt = SMI230_E_INVALID_INPUT;
		}

		if (rslt == SMI230_OK) {
			/*enable accel sensor */
			rslt = set_regs(SMI230_ACCEL_PWR_CONF_REG, &data[0], 1, dev);

			if (rslt == SMI230_OK) {
				/*delay between power ctrl and power config */
				dev->delay_ms(SMI230_POWER_CONFIG_DELAY);

				rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &data[1], 1, dev);

				if (rslt == SMI230_OK) {
					/*delay required to switch power modes */
					dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
				}
			}

		}
	}

	return rslt;
}

static int8_t smi230_acc_get_data(struct smi230_sensor_data *accel,
				  const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;

	rslt = null_ptr_check(dev);

	if ((rslt == SMI230_OK) && (accel != NULL)) {
		rslt = get_regs(SMI230_ACCEL_X_LSB_REG, data, 6, dev);

		if (rslt == SMI230_OK) {
			lsb = data[0];
			msb = data[1];
			msblsb = (msb << 8) | lsb;
			accel->x = ((int16_t) msblsb);	/* Data in X axis */

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			accel->y = ((int16_t) msblsb);	/* Data in Y axis */

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			accel->z = ((int16_t) msblsb);	/* Data in Z axis */
		}

	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t smi230_acc_set_int_config(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
	int8_t rslt;

	rslt = null_ptr_check(dev);

	if ((rslt == SMI230_OK) && (int_config != NULL)) {
		switch (int_config->int_type) {
		case SMI230_ACCEL_DATA_RDY_INT:
			rslt = set_accel_data_ready_int(int_config, dev);
			break;
		case SMI230_ACCEL_SYNC_DATA_RDY_INT:
			rslt = set_accel_sync_data_ready_int(int_config, dev);
			break;
		case SMI230_ACCEL_SYNC_INPUT:
			rslt = set_accel_sync_input(int_config, dev);
			break;
		case SMI230_ACCEL_ANYMOTION_INT:
			rslt = set_accel_anymotion_int(int_config, dev);
			break;
		case SMI230_ACCEL_HIGH_G_INT:
			rslt = set_accel_high_g_int(int_config, dev);
			break;
		case SMI230_ACCEL_LOW_G_INT:
			rslt = set_accel_low_g_int(int_config, dev);
			break;
		case SMI230_ACCEL_ORIENT_INT:
			rslt = set_accel_orient_int(int_config, dev);
			break;
		case SMI230_ACCEL_NO_MOTION_INT:
			rslt = set_accel_no_motion_int(int_config, dev);
			break;
		case SMI230_ACCEL_ERROR_INT:
			rslt = set_accel_err_int(int_config, dev);
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

void smi230_delay(uint32_t msec)
{
	unsigned long mseond = msec;
	unsigned long min = mseond * (1000);
	if (msec <= 20)
		usleep_range(min, (min + 1000));
	else
		msleep(msec);
}

int8_t smi230_acc_init(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		if (dev->intf == SMI230_SPI_INTF) {
			/* Set dummy byte in case of SPI interface */
			dev->dummy_byte = SMI230_ENABLE;

			/* Dummy read of Chip-ID in SPI mode */
			rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);
		} else {
			/* Make dummy byte 0 in case of I2C interface */
			dev->dummy_byte = SMI230_DISABLE;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1, dev);

			if (rslt == SMI230_OK) {
				if (chip_id == SMI230_ACCEL_CHIP_ID) {
					dev->accel_chip_id = chip_id;
				} else
					rslt = SMI230_E_DEV_NOT_FOUND;
			}
		}
	}

	return rslt;
}

static int smi230_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = smi230_acc_get_data(&data, p_smi230_dev);
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
		ret = smi230_acc_get_meas_conf(p_smi230_dev);
		if (ret)
			return ret;

		switch (p_smi230_dev->accel_cfg.odr) {
		case SMI230_ACCEL_ODR_12_5_HZ:
			*val = 12;
			*val2 = 5000000;
			break;
		case SMI230_ACCEL_ODR_25_HZ:
			*val = 25;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_50_HZ:
			*val = 50;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_100_HZ:
			*val = 100;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_200_HZ:
			*val = 200;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_400_HZ:
			*val = 400;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_800_HZ:
			*val = 800;
			*val2 = 0;
			break;
		case SMI230_ACCEL_ODR_1600_HZ:
			*val = 1600;
			*val2 = 0;
			break;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	int ret;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (val) {
		case 12:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_12_5_HZ;
			break;
		case 25:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_25_HZ;
			break;
		case 50:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_50_HZ;
			break;
		case 100:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
			break;
		case 200:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_200_HZ;
			break;
		case 400:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
			break;
		case 800:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
			break;
		case 1600:
			p_smi230_dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
			break;
		default:
			return -EINVAL;
		}

		ret = smi230_acc_set_meas_conf(p_smi230_dev);
		break;
	default:
		return -EINVAL;
	}

	return ret;

}

static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
	       "12.5 25 50 100 200 400 800 1600");

/* 8192 LSB/g */
static IIO_CONST_ATTR(in_accel_scale_available, "0.0001221");

static struct attribute *smi230_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
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

static int smi230_init(struct smi230_dev *dev)
{
	int err = 0;

	/* Reset the accelerometer and wait for 1 ms
	 * delay taken care inside the function
	 */
	err |= smi230_acc_soft_reset(dev);

	/*set accel power mode */
	dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	err |= smi230_acc_set_power_mode(dev);

	/*disable acc int1 */
	int_config.accel_int_config_1.int_pin_cfg.enable_int_pin =
	    SMI230_DISABLE;

	/*configure int2 as accel FIFO interrupt pin */
	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin =
	    SMI230_ENABLE;

	dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
	dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;

	err |= smi230_acc_set_meas_conf(dev);
	smi230_delay(100);

	int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
	int_config.accel_int_config_2.int_type = SMI230_ACCEL_DATA_RDY_INT;
	int_config.accel_int_config_2.int_pin_cfg.output_mode =
	    SMI230_INT_MODE_PUSH_PULL;
	int_config.accel_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;

	err |= smi230_acc_set_int_config(&int_config.accel_int_config_2, dev);

	smi230_delay(100);

	return err;
}

static int smi230_read_channel(struct smi230_sensor_data *sdata, int i,
			       s16 *sample)
{
	switch (i) {
	case 0:
		*sample = sdata->x;
		break;
	case 1:
		*sample = sdata->y;
		break;
	case 2:
		*sample = sdata->z;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t smi230_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	s16 buf[8];
	s16 sample;
	int ret, i, j = 0;

	struct smi230_sensor_data sensor_data = { 0 };

	ret = smi230_acc_get_data(&sensor_data, p_smi230_dev);
	if (ret) {
		pr_err("Reading sensor data failed");
		goto done;
	}

	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		ret = smi230_read_channel(&sensor_data, i, &sample);
		if (ret) {
			pr_err("Read channel %d failed", i);
			goto done;
		}
		buf[j++] = sample;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, pf->timestamp);
	if (ret)
		pr_err("Push to buffer failed");
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int smi230_get_irq(struct device *dev, int *irq)
{
	int gpio_pin, ret;

	gpio_pin = of_get_named_gpio_flags(dev->of_node, "gpio_irq", 0, NULL);

	ret = gpio_request_one(gpio_pin, GPIOF_IN, "smi230_acc_interrupt");
	if (ret) {
		pr_err("Request GPIO pin %d failed", gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(gpio_pin);
	if (ret)
		return ret;

	*irq = gpio_to_irq(gpio_pin);

	return ret;
}

static int smi230_new_data_trigger_set_state(
		struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);
	u8 en;

	dev_dbg(indio_dev->dev.parent, "trigger set state %d", enable);
	if (enable)
		en = SMI230_ENABLE;
	else
		en = SMI230_DISABLE;

	int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = en;
	return smi230_acc_set_int_config(
			&int_config.accel_int_config_2, p_smi230_dev);
}

static const struct iio_trigger_ops smi230_trigger_ops = {
	.set_trigger_state = &smi230_new_data_trigger_set_state,
};

int smi230_acc_probe(struct device *dev, struct smi230_dev *p_smi230_dev)
{
	int ret = 0;
	int irq;
	struct iio_dev *indio_dev;

	ret = smi230_init(p_smi230_dev);
	if (ret == SMI230_OK) {
		dev_dbg(dev, "Bosch Sensor %s hardware initialized", SENSOR_ACC_NAME);
	} else {
		dev_dbg(dev, "Bosch Sensor %s initialization failed %d",
				SENSOR_ACC_NAME, ret);
		return ret;
	}

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, p_smi230_dev);
	dev_set_drvdata(dev, indio_dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = smi230_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi230_channels);
	indio_dev->name = MODULE_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->info = &smi230_info;
	indio_dev->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-dev%d",
			indio_dev->name, indio_dev->id);

	if (indio_dev->trig == NULL)
		return -ENOMEM;

	ret = smi230_get_irq(dev, &irq);
	if (ret) {
		dev_dbg(dev, "Failed to request GPIO pin");
		return ret;
	}

	ret = devm_request_irq(&indio_dev->dev, irq,
			&iio_trigger_generic_data_rdy_poll,
			IRQF_TRIGGER_RISING,
			indio_dev->name,
			indio_dev->trig);

	if (ret) {
		dev_dbg(dev, "Failed to request irq for pin %d", irq);
		return ret;
	}

	indio_dev->trig->dev.parent = dev;
	indio_dev->trig->ops = &smi230_trigger_ops;
	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, indio_dev->trig);
	if (ret) {
		dev_dbg(dev, "Failed to register trigger");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      smi230_trigger_handler, NULL);
	if (ret) {
		dev_dbg(dev, "Setup triggered buffer failed");
		return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}
