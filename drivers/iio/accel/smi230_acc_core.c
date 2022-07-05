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

struct smi230_accel_fifo_config
{
    uint8_t mode;
    uint8_t accel_en;
    uint8_t int1_en;
    uint8_t int2_en;
};

struct smi230_fifo_frame
{
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
    struct smi230_accel_fifo_config fifo_conf;
};

static uint8_t temp_buff[CONFIG_IIO_SMI230_ACC_MAX_BUFFER_LEN + 1];
static uint8_t fifo_buf[SMI230_MAX_ACC_FIFO_BYTES];
static struct smi230_sensor_data fifo_accel_data[SMI230_MAX_ACC_FIFO_FRAME];

static const struct iio_chan_spec smi230_channels[] = {
	SMI230_CHANNEL(IIO_ACCEL, X, 0),
	SMI230_CHANNEL(IIO_ACCEL, Y, 1),
	SMI230_CHANNEL(IIO_ACCEL, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

static struct smi230_int_cfg int_config;

static int8_t null_ptr_check(const struct smi230_dev *dev)
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

static int8_t smi230_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (reg_data != NULL)) {
        if (len > 0)
            rslt = get_regs(reg_addr, reg_data, len, dev);
        else
            rslt = SMI230_E_RD_WR_LENGTH_INVALID;
    } else
        rslt = SMI230_E_NULL_PTR;

    return rslt;
}

static int8_t smi230_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data,
		uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (reg_data != NULL)) {
        if (len > 0)
            rslt = set_regs(reg_addr, reg_data, len, dev);
        else
            rslt = SMI230_E_RD_WR_LENGTH_INVALID;
    } else
        rslt = SMI230_E_NULL_PTR;

    return rslt;
}

static int8_t smi230_acc_set_fifo_wm(uint16_t wm, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK) {
        data[0] = SMI230_GET_LSB(wm);
        data[1] = SMI230_GET_MSB(wm);

        rslt = smi230_acc_set_regs(SMI230_FIFO_WTM_0_ADDR, data,
        		SMI230_FIFO_WTM_LENGTH, dev);
    }

    return rslt;
}

static int8_t smi230_acc_get_fifo_length(uint16_t *fifo_length,
		const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[SMI230_FIFO_DATA_LENGTH] = { 0 };

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (fifo_length != NULL)) {
        rslt = smi230_acc_get_regs(SMI230_FIFO_LENGTH_0_ADDR, data,
        		SMI230_FIFO_DATA_LENGTH, dev);
        if (rslt == SMI230_OK) {
            data[1] = SMI230_GET_BITS_POS_0(data[1], SMI230_FIFO_BYTE_COUNTER_MSB);
            (*fifo_length) = (uint16_t)((uint16_t)(data[1] << 8) | data[0]);
        }
        else
            rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

static int8_t smi230_acc_read_fifo_data(struct smi230_fifo_frame *fifo,
		const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t config_data = 0;
    uint8_t addr = SMI230_FIFO_DATA_ADDR;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (fifo != NULL)) {
    	fifo->acc_byte_start_idx = 0;
		fifo->sensor_time = 0;
		fifo->skipped_frame_count = 0;

        rslt = smi230_acc_get_regs(addr, fifo->data, fifo->length, dev);
        if (rslt == SMI230_OK) {
            rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_1_ADDR, &config_data, 1, dev);
            if (rslt == SMI230_OK)
                fifo->data_enable = (uint16_t)((uint16_t)config_data & SMI230_ACCEL_EN_MASK);
        }
        else
            rslt = SMI230_E_COM_FAIL;
    }
    else
        rslt = SMI230_E_NULL_PTR;

    return rslt;
}

static void unpack_accel_data(struct smi230_sensor_data *acc,
                              uint16_t data_start_index,
                              const struct smi230_fifo_frame *fifo)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->x = (int16_t)((data_msb << 8) | data_lsb);

    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->y = (int16_t)((data_msb << 8) | data_lsb);

    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    acc->z = (int16_t)((data_msb << 8) | data_lsb);

}

static int8_t unpack_accel_frame(struct smi230_sensor_data *acc,
                                 uint16_t *idx,
                                 uint16_t *acc_idx,
                                 uint16_t frame,
                                 const struct smi230_fifo_frame *fifo)
{
    int8_t rslt = SMI230_OK;

    switch (frame) {
        case SMI230_FIFO_HEADER_ACC_FRM:
            if (((*idx) + SMI230_FIFO_ACCEL_LENGTH) > fifo->length) {
                (*idx) = fifo->length;
                rslt = SMI230_W_FIFO_EMPTY;
                break;
            }

            unpack_accel_data(&acc[(*acc_idx)], *idx, fifo);
            (*idx) = (*idx) + SMI230_FIFO_ACCEL_LENGTH;
            (*acc_idx)++;
            break;
        default:
            (*idx) = fifo->length;
            rslt = SMI230_W_FIFO_EMPTY;
            break;
    }

    return rslt;
}

static int8_t move_next_frame(uint16_t *data_index,
		uint8_t current_frame_length, const struct smi230_fifo_frame *fifo)
{
    int8_t rslt = SMI230_OK;

    if (((*data_index) + current_frame_length) > fifo->length) {
        (*data_index) = fifo->length;
        rslt = SMI230_W_FIFO_EMPTY;
    } else
        (*data_index) = (*data_index) + current_frame_length;

    return rslt;
}

static int8_t unpack_sensortime_frame(uint16_t *data_index,
		struct smi230_fifo_frame *fifo)
{
    int8_t rslt = SMI230_OK;
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    if (((*data_index) + SMI230_SENSOR_TIME_LENGTH) > fifo->length) {
        (*data_index) = fifo->length;
        rslt = SMI230_W_FIFO_EMPTY;
    } else {
        sensor_time_byte3 = fifo->data[(*data_index) + SMI230_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + SMI230_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        (*data_index) = (*data_index) + SMI230_SENSOR_TIME_LENGTH;
    }

    return rslt;
}

static int8_t unpack_skipped_frame(uint16_t *data_index,
		struct smi230_fifo_frame *fifo)
{
    int8_t rslt = SMI230_OK;

    if ((*data_index) >= fifo->length) {
        (*data_index) = fifo->length;
        rslt = SMI230_W_FIFO_EMPTY;
    } else {
        fifo->skipped_frame_count = fifo->data[(*data_index)];
        (*data_index) = (*data_index) + 1;
        rslt = SMI230_W_PARTIAL_READ;
    }

    return rslt;
}

static int8_t smi230_acc_extract_accel(struct smi230_sensor_data *accel_data,
		uint16_t *accel_length, struct smi230_fifo_frame *fifo)
{
	int8_t rslt = SMI230_OK;
	uint8_t frame_header = 0;
	uint16_t data_index;
	uint16_t accel_index = 0;
	uint16_t frame_to_read = *accel_length;

	for (data_index = fifo->acc_byte_start_idx; data_index < fifo->length;) {
		frame_header = fifo->data[data_index];
		data_index++;
		switch (frame_header) {
			case SMI230_FIFO_HEADER_ACC_FRM:
			case SMI230_FIFO_HEADER_ALL_FRM:
				rslt = unpack_accel_frame(accel_data, &data_index, &accel_index,
						frame_header, fifo);
				break;

			case SMI230_FIFO_HEADER_SENS_TIME_FRM:
				rslt = unpack_sensortime_frame(&data_index, fifo);
				break;

			case SMI230_FIFO_HEADER_SKIP_FRM:
				rslt = unpack_skipped_frame(&data_index, fifo);
				break;

			case SMI230_FIFO_HEADER_INPUT_CFG_FRM:
				rslt = move_next_frame(&data_index,
						SMI230_FIFO_INPUT_CFG_LENGTH, fifo);
				break;

			case SMI230_FIFO_SAMPLE_DROP_FRM:
				rslt = move_next_frame(&data_index,
						SMI230_FIFO_INPUT_CFG_LENGTH, fifo);
				break;

			case SMI230_FIFO_HEAD_OVER_READ_MSB:
				data_index = fifo->length;
				rslt = SMI230_W_FIFO_EMPTY;
				break;
			default:
				data_index = fifo->length;
				rslt = SMI230_W_FIFO_EMPTY;
				break;
		}

		if ((frame_to_read == accel_index) || (rslt == SMI230_W_FIFO_EMPTY))
			break;
	}

	(*accel_length) = accel_index;
	fifo->acc_byte_start_idx = data_index;

	return rslt;
}

static int8_t smi230_acc_set_fifo_config(
		const struct smi230_accel_fifo_config *config,
		const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data_array[2] = { 0 };

    rslt = null_ptr_check(dev);

    if (rslt == SMI230_OK) {
        rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        if (rslt == SMI230_OK) {
            data_array[0] = SMI230_SET_BITS_POS_0(data_array[0], SMI230_ACC_FIFO_MODE_CONFIG, config->mode);
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_EN, config->accel_en);
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_INT1_EN, config->int1_en);
            data_array[1] = SMI230_SET_BITS(data_array[1], SMI230_ACCEL_INT2_EN, config->int2_en);
            rslt = smi230_acc_set_regs(SMI230_FIFO_CONFIG_0_ADDR, data_array, 2, dev);
        }
    }
    else
        rslt = SMI230_E_NULL_PTR;

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

static int8_t set_fifo_wm_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data = 0;

    rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == SMI230_OK) {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_FWM, conf);
			break;

		case SMI230_INT_CHANNEL_2:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FWM, conf);
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

static int8_t set_fifo_full_int(
		const struct smi230_accel_int_channel_cfg *int_config,
		const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data = 0;

    rslt = get_regs(SMI230_ACCEL_INT1_INT2_MAP_DATA_REG, &data, 1, dev);

    if (rslt == SMI230_OK) {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_1:
			data = SMI230_SET_BITS_POS_0(data, SMI230_ACCEL_INT1_FFULL, conf);
			break;

		case SMI230_INT_CHANNEL_2:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FFULL, conf);
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

static int8_t smi230_acc_get_power_mode(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK) {
        rslt = get_regs(SMI230_ACCEL_PWR_CONF_REG, &data, 1, dev);
        if (rslt == SMI230_OK)
            dev->accel_cfg.power = data;
    }

    return rslt;
}

static int8_t smi230_acc_set_power_mode(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		if (dev->accel_cfg.power == SMI230_ACCEL_PM_ACTIVE) {
			data[0] = SMI230_ACCEL_PM_ACTIVE;
			data[1] = SMI230_ACCEL_POWER_ENABLE;
		} else if (dev->accel_cfg.power == SMI230_ACCEL_PM_SUSPEND) {
			data[0] = SMI230_ACCEL_PM_SUSPEND;
			data[1] = SMI230_ACCEL_POWER_DISABLE;
		} else {
			rslt = SMI230_E_INVALID_INPUT;
		}

		if (rslt == SMI230_OK) {
			rslt = set_regs(SMI230_ACCEL_PWR_CONF_REG, &data[0], 1, dev);

			if (rslt == SMI230_OK) {
				dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
				rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &data[1], 1, dev);
				if (rslt == SMI230_OK)
					dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
			}

		}
	}

	return rslt;
}

static int8_t smi230_acc_get_data(struct smi230_sensor_data *accel,
				  const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[9];
	uint8_t lsb, msb;
	uint16_t msblsb;

	rslt = null_ptr_check(dev);

	if ((rslt == SMI230_OK) && (accel != NULL)) {
		rslt = get_regs(SMI230_ACCEL_X_LSB_REG, data, 9, dev);

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

			accel->sensor_time=(data[8] << 16) | (data[7] << 8) | data[6]; /* sensor intern time stamp*/
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
		case SMI230_ACCEL_FIFO_WM_INT:
			rslt = set_fifo_wm_int(int_config, dev);
			break;
		case SMI230_ACCEL_FIFO_FULL_INT:
			rslt = set_fifo_full_int(int_config, dev);
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

static int8_t smi230_acc_get_fifo_wm(uint16_t *wm, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK) {
        rslt = smi230_acc_get_regs(SMI230_FIFO_WTM_0_ADDR, data,
        		SMI230_FIFO_WTM_LENGTH, dev);
        if ((rslt == SMI230_OK) && (wm != NULL))
            *wm = (data[1] << 8) | (data[0]);
        else
            rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

static int8_t smi230_acc_fifo_reset(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK) {
        data = SMI230_FIFO_RESET_CMD;

        rslt = set_regs(SMI230_ACCEL_SOFTRESET_REG, &data, 1, dev);

        if (rslt == SMI230_OK)
            /* Delay 1 ms after reset value is written to its register */
            dev->delay_ms(SMI230_ACCEL_SOFTRESET_DELAY_MS);
    }

    return rslt;
}

static int8_t smi230_acc_get_sensor_temperature(const struct smi230_dev *dev,
		int32_t *sensor_temp)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };
    uint16_t msb, lsb;
    uint16_t msblsb;
    int16_t temp;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (sensor_temp != NULL)) {
        rslt = get_regs(SMI230_TEMP_MSB_REG, data, 2, dev);

        if (rslt == SMI230_OK) {
            msb = (data[0] << 3);
            lsb = (data[1] >> 5);
            msblsb = (uint16_t) (msb + lsb);

            if (msblsb > 1023)
                temp = (int16_t) (msblsb - 2048);
            else
                temp = (int16_t) msblsb;

            *sensor_temp = (temp * 125) + 23000;
        }
    } else
        rslt = SMI230_E_NULL_PTR;

    return rslt;
}

static int8_t smi230_acc_self_test(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;
	int16_t x, y, z;
	struct smi230_sensor_data sdata = {0};

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;
	dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
	dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;

	rslt = smi230_acc_set_meas_conf(dev);
	if (rslt != SMI230_OK)
		return rslt;

	smi230_delay(5);

	data = SMI230_ACCEL_POSITIVE_SELF_TEST;
	rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &data, 1, dev);
	if (rslt != SMI230_OK)
		return rslt;

	smi230_delay(75);

	rslt = smi230_acc_get_data(&sdata, dev);
	if (rslt != SMI230_OK)
		return rslt;

	x = sdata.x;
	y = sdata.y;
	z = sdata.z;

	data = SMI230_ACCEL_NEGATIVE_SELF_TEST;
	rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &data, 1, dev);
	if (rslt != SMI230_OK)
		return rslt;

	smi230_delay(75);

	rslt = smi230_acc_get_data(&sdata, dev);
	if (rslt != SMI230_OK)
		return rslt;

	data = SMI230_ACCEL_SWITCH_OFF_SELF_TEST;
	rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &data, 1, dev);
	if (rslt != SMI230_OK)
		return rslt;

	if ((x - sdata.x) < 2048)
		rslt = SMI230_W_SELF_TEST_FAIL;
	if ((y - sdata.y) < 2048)
		rslt = SMI230_W_SELF_TEST_FAIL;
	if ((z - sdata.z) < 1024)
		rslt = SMI230_W_SELF_TEST_FAIL;

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
			return 0;

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

static ssize_t smi230_acc_store_fifo_wm(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	uint16_t fifo_wm;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	err = kstrtou16(buf, 10, &fifo_wm);
	if (err) {
		dev_err(dev, "Invalid argument");
		return err;
	}

	err = smi230_acc_set_fifo_wm(fifo_wm, p_smi230_dev);
	if (err != SMI230_OK) {
		dev_err(dev, "Failed to set FIFO WM.");
		return err;
	}

	return count;
}

static int smi230_extract_accel_fifo_data(const struct smi230_dev *dev,
		struct smi230_sensor_data *data, uint16_t *count)
{
	int ret;
	struct smi230_fifo_frame fifo;

	fifo.data = fifo_buf;
	ret = smi230_acc_get_fifo_length(&fifo.length, dev);
	if (ret != SMI230_OK) return ret;
	ret = smi230_acc_read_fifo_data(&fifo, dev);
	if (ret != SMI230_OK) return ret;
	ret = smi230_acc_extract_accel(data, count, &fifo);

	return ret;
}

static int smi230_fifo_flush(struct iio_dev *indio_dev, unsigned count)
{
	int ret = 0;
	int i;
	int16_t buf[8];
	int16_t sample;
	uint16_t frame_count = count;
	uint32_t tsamp;
	int64_t timestamp = iio_get_time_ns(indio_dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_extract_accel_fifo_data(p_smi230_dev, fifo_accel_data,
			&frame_count);

	if (ret) {
		dev_err(indio_dev->dev.parent, "Error extracting FIFO data.");
		return -1;
	}

	switch (p_smi230_dev->accel_cfg.odr) {
	case SMI230_ACCEL_ODR_12_5_HZ:
		tsamp = 80000000;
		break;
	case SMI230_ACCEL_ODR_25_HZ:
		tsamp = 40000000;
		break;
	case SMI230_ACCEL_ODR_50_HZ:
		tsamp = 20000000;
		break;
	case SMI230_ACCEL_ODR_100_HZ:
		tsamp = 10000000;
		break;
	case SMI230_ACCEL_ODR_200_HZ:
		tsamp = 5000000;
		break;
	case SMI230_ACCEL_ODR_400_HZ:
		tsamp = 2500000;
		break;
	case SMI230_ACCEL_ODR_800_HZ:
		tsamp = 1250000;
		break;
	case SMI230_ACCEL_ODR_1600_HZ:
		tsamp = 625000;
		break;
	}

	timestamp -= tsamp * (frame_count - 1);

	for (i = 0; i < frame_count; i++) {
		int j = 0;
		int k = 0;
		for_each_set_bit(k, indio_dev->active_scan_mask, indio_dev->masklength) {
			ret |= smi230_read_channel(&fifo_accel_data[i], k, &sample);
			buf[j++] = sample;
		}

		iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
		timestamp += tsamp;
	}

	if (ret)
		return -1;

	return frame_count;
}

static int smi230_data_ready_handler(struct iio_dev *indio_dev, int64_t timestamp)
{
	s16 buf[8];
	s16 sample;
	int ret, i, j = 0;
	struct smi230_sensor_data sensor_data = { 0 };
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_data(&sensor_data, p_smi230_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent, "Reading sensor data failed");
		return ret;
	}

	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		ret = smi230_read_channel(&sensor_data, i, &sample);
		if (ret) {
			dev_err(indio_dev->dev.parent, "Read channel %d failed", i);
			return ret;
		}
		buf[j++] = sample;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	if (ret)
		pr_err("Push to buffer failed");

	return ret;
}


static int smi230_get_in_accel_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	ret = smi230_acc_get_data(&data, p_smi230_dev);
	if (ret != SMI230_OK) return 0;
	return snprintf(buf, PAGE_SIZE, "%d %hd %hd %hd\n",
			data.sensor_time, data.x, data.y, data.z);
}

static int smi230_get_accel_range(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret, range;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_meas_conf(p_smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Error reading meas config.");
		return ret;
	}

	switch (p_smi230_dev->accel_cfg.range) {
	case SMI230_ACCEL_RANGE_2G:
		range = 2;
		break;
	case SMI230_ACCEL_RANGE_4G:
		range = 4;
		break;
	case SMI230_ACCEL_RANGE_8G:
		range = 8;
		break;
	case SMI230_ACCEL_RANGE_16G:
		range = 16;
		break;
	default:
		dev_err(dev, "Invalid measurement range set.");
		range = 0;
		break;
	}
	return sprintf(buf, "%d\n", range);
}

static int smi230_get_accel_bw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_meas_conf(p_smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Error reading meas config.");
		return ret;
	}

	switch (p_smi230_dev->accel_cfg.bw) {
	case SMI230_ACCEL_BW_NORMAL:
		return sprintf(buf, "normal\n");
	case SMI230_ACCEL_BW_OSR2:
		return sprintf(buf, "osr2\n");
	case SMI230_ACCEL_BW_OSR4:
		return sprintf(buf, "osr4\n");
	default:
		return sprintf(buf, "error\n");
	}
}

static int smi230_get_accel_power_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_power_mode(p_smi230_dev);
	if (ret) {
		dev_err(dev, "Failed to read power config.");
		return ret;
	}

	switch (p_smi230_dev->accel_cfg.power) {
	case SMI230_ACCEL_PM_ACTIVE:
		return sprintf(buf, "normal\n");
	case SMI230_ACCEL_PM_SUSPEND:
		return sprintf(buf, "suspend\n");
	default:
		return sprintf(buf, "error\n");
	}
}

static int smi230_get_accel_self_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret, success;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_self_test(p_smi230_dev);
	if (ret == SMI230_OK)
		success = 1;
	else
		success = 0;

	smi230_acc_soft_reset(p_smi230_dev);
	smi230_delay(100);

	if (success)
		return snprintf(buf, PAGE_SIZE, "success. soft reset performed.\n");
	else
		return snprintf(buf, PAGE_SIZE, "failure. soft reset performed.\n");
}

static int smi230_set_accel_range(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, range;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = kstrtoint(buf, 10, &range);
	if (ret) {
		dev_err(dev, "Invalid argument for range.");
		return ret;
	}

	switch (range) {
	case 2:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_2G;
		break;
	case 4:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;
		break;
	case 8:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_8G;
		break;
	case 16:
		p_smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;
		break;
	default:
		dev_err(dev, "Invalid argument for range.");
		return -EINVAL;
	}

	ret = smi230_acc_set_meas_conf(p_smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Failed to set accel range.");
		return ret;
	}
	return count;
}

static int smi230_set_accel_bw(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	if (strncmp(buf, "normal", 6) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	else if (strncmp(buf, "osr2", 4) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR2;
	else if (strncmp(buf, "osr4", 4) == 0)
		p_smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR4;
	else {
		dev_err(dev, "Invalid argument for bw.");
		return -EINVAL;
	}

	ret = smi230_acc_set_meas_conf(p_smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Failed to set accel bw.");
		return ret;
	}
	return count;
}

static int smi230_set_accel_power_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	if (strncmp(buf, "normal", 6) == 0) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	} else if (strncmp(buf, "suspend", 7) == 0) {
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	} else {
		dev_err(dev, "Invalid argument for power mode.");
		return -EINVAL;
	}

	ret = smi230_acc_set_power_mode(p_smi230_dev);
	if (ret)
		dev_err(dev, "Failed to set power mode.");

	return count;
}

static int smi230_accel_soft_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, val;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = kstrtoint(buf, 10, &val);
	if (ret) return -EINVAL;

	if (val) {
		smi230_acc_soft_reset(p_smi230_dev);
		smi230_delay(100);
	}
	return count;
}

static int smi230_accel_show_sensor_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	int32_t temp;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_sensor_temperature(p_smi230_dev, &temp);
	if (ret != SMI230_OK)
		return snprintf(buf, PAGE_SIZE, "error\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}

static IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		"12.5 25 50 100 200 400 800 1600");
static IIO_CONST_ATTR(in_accel_scale_available,
		"2g: 1/16384 4g: 1/8192 8g: 1/4096 16g: 1/2048 (range: scale)");
static IIO_DEVICE_ATTR(in_accel_raw, S_IRUGO, smi230_get_in_accel_raw, NULL, 0);
static IIO_DEVICE_ATTR(in_accel_range, S_IRUGO|S_IWUSR|S_IWGRP,
		smi230_get_accel_range, smi230_set_accel_range, 0);
static IIO_DEVICE_ATTR(in_accel_bw, S_IRUGO|S_IWUSR|S_IWGRP,
		smi230_get_accel_bw, smi230_set_accel_bw, 0);
static IIO_DEVICE_ATTR(in_accel_power_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		smi230_get_accel_power_mode, smi230_set_accel_power_mode, 0);
static IIO_DEVICE_ATTR(soft_reset, S_IWUSR|S_IWGRP, NULL,
		smi230_accel_soft_reset, 0);
static IIO_DEVICE_ATTR(self_test, S_IRUGO, smi230_get_accel_self_test, NULL, 0);
static IIO_DEVICE_ATTR(in_temp_accel, S_IRUGO, smi230_accel_show_sensor_temp,
		NULL, 0);

static struct attribute *smi230_attrs[] = {
	&iio_dev_attr_in_accel_raw.dev_attr.attr,
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_in_accel_range.dev_attr.attr,
	&iio_dev_attr_in_accel_bw.dev_attr.attr,
	&iio_dev_attr_in_accel_power_mode.dev_attr.attr,
	&iio_dev_attr_soft_reset.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_in_temp_accel.dev_attr.attr,
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
	uint8_t int_lvl, int_mode;
	struct smi230_accel_fifo_config fifo_config;

	/* Reset the accelerometer and wait for 1 ms
	 * delay taken care inside the function
	 */
	err = smi230_acc_soft_reset(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 soft reset failed.");
		return err;
	}

	dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	err = smi230_acc_set_power_mode(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set power mode failed.");
		return err;
	}

	dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
	dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;

	err = smi230_acc_set_meas_conf(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set meas config failed.");
		return err;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_PUSH_PULL))
		int_mode = SMI230_INT_MODE_PUSH_PULL;
	else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_OPEN_DRAIN))
		int_mode = SMI230_INT_MODE_OPEN_DRAIN;

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_ACTIVE_LOW))
		int_lvl = SMI230_INT_ACTIVE_LOW;
	else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_ACTIVE_HIGH))
		int_lvl = SMI230_INT_ACTIVE_HIGH;

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT1)) {
		int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
		int_config.accel_int_config_1.int_channel = SMI230_INT_CHANNEL_1;
		int_config.accel_int_config_1.int_pin_cfg.output_mode = int_mode;
		int_config.accel_int_config_1.int_pin_cfg.lvl = int_lvl;

		if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO_WM))
			int_config.accel_int_config_1.int_type = SMI230_ACCEL_FIFO_WM_INT;
		else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO_FULL))
			int_config.accel_int_config_1.int_type = SMI230_ACCEL_FIFO_FULL_INT;
		else
			int_config.accel_int_config_1.int_type = SMI230_ACCEL_DATA_RDY_INT;
	} else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT2)) {
		int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
		int_config.accel_int_config_2.int_channel = SMI230_INT_CHANNEL_2;
		int_config.accel_int_config_2.int_pin_cfg.output_mode = int_mode;
		int_config.accel_int_config_2.int_pin_cfg.lvl = int_lvl;

		if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO_WM))
			int_config.accel_int_config_2.int_type = SMI230_ACCEL_FIFO_WM_INT;
		else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO_FULL))
			int_config.accel_int_config_2.int_type = SMI230_ACCEL_FIFO_FULL_INT;
		else
			int_config.accel_int_config_2.int_type = SMI230_ACCEL_DATA_RDY_INT;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO_WM)) {
		err = smi230_acc_set_fifo_wm(512, dev);
		if (err != SMI230_OK) {
			pr_err("SMI230 set fifo wm failed.");
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT1))
		err = smi230_acc_set_int_config(&int_config.accel_int_config_1, dev);
	else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT2))
		err = smi230_acc_set_int_config(&int_config.accel_int_config_2, dev);

	if (err != SMI230_OK) {
		pr_err("SMI230 set int config failed.");
		return err;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO)) {
		fifo_config.mode = SMI230_ACC_FIFO_MODE;
		fifo_config.accel_en = 1;
		err = smi230_acc_set_fifo_config(&fifo_config, dev);
		if (err != SMI230_OK) {
			pr_err("SMI230 set fifo config failed.");
			return err;
		}
	}

	dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	err = smi230_acc_set_power_mode(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set power mode failed.");
		return err;
	}

	smi230_delay(100);

	return err;
}

static irqreturn_t smi230_trigger_handler(int irq, void *p)
{
	int ret;
	uint8_t int_stat;
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_regs(SMI230_ACCEL_INT_STAT_1_REG,
			&int_stat, 1, p_smi230_dev);
	if (ret != SMI230_OK)
		dev_err(indio_dev->dev.parent,
				"Failed to read interrupt status register");

	if ((int_stat & (SMI230_ACCEL_FIFO_WM_INT_ENABLE | SMI230_ACCEL_FIFO_FULL_INT_ENABLE)) != 0) {
		ret = smi230_fifo_flush(indio_dev, SMI230_MAX_ACC_FIFO_FRAME);
		if (ret < 0)
			dev_err(indio_dev->dev.parent, "Failed to read FIFO data");
	} else if ((int_stat & SMI230_ACCEL_DATA_RDY_INT_ENABLE) != 0) {
		ret = smi230_data_ready_handler(indio_dev, pf->timestamp);
		if (ret != SMI230_OK)
			dev_err(indio_dev->dev.parent, "Failed to read new data");
	}

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
	int ret = 0;
	uint8_t en;
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	dev_dbg(indio_dev->dev.parent, "trigger set state %d", enable);

	if (enable) {
		en = SMI230_ENABLE;
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	} else {
		en = SMI230_DISABLE;
		p_smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	}

	ret = smi230_acc_set_power_mode(p_smi230_dev);
	if (ret) {
		dev_err(indio_dev->dev.parent, "Failed to set power mode.");
		return ret;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT1)) {
		int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = en;
		ret = smi230_acc_set_int_config(
				&int_config.accel_int_config_1, p_smi230_dev);
	} else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_INT2)) {
		int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = en;
		ret = smi230_acc_set_int_config(
				&int_config.accel_int_config_2, p_smi230_dev);
	}

	if (ret) {
		dev_err(indio_dev->dev.parent, "Failed to set interrupt config.");
		return ret;
	}

	return ret;
}

static const struct iio_trigger_ops smi230_trigger_ops = {
	.set_trigger_state = &smi230_new_data_trigger_set_state,
};

static ssize_t smi230_get_fifo_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

static ssize_t smi230_get_fifo_watermark(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t wm;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_fifo_wm(&wm, p_smi230_dev);

	if (ret == SMI230_OK)
		return sprintf(buf, "%d\n", wm);
	else
		return sprintf(buf, "0\n");
}

static ssize_t smi230_fifo_reset(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int err, val;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	err = kstrtoint(buf, 10, &val);
	if (err) {
		dev_err(dev, "Invalid argument");
		return err;
	}

	if (val) {
		err = smi230_acc_fifo_reset(p_smi230_dev);
		if (err != SMI230_OK) {
			dev_err(dev, "Failed to reset fifo.");
			return err;
		}
	}
	return count;
}

static IIO_CONST_ATTR(hwfifo_watermark_min, "1");
static IIO_CONST_ATTR(hwfifo_watermark_max,
		__stringify(SMI230_MAX_ACC_FIFO_FRAME));
static IIO_DEVICE_ATTR(hwfifo_enabled, S_IRUGO,
		smi230_get_fifo_state, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, S_IRUGO|S_IWUSR|S_IWGRP,
		smi230_get_fifo_watermark, smi230_acc_store_fifo_wm, 0);
static IIO_DEVICE_ATTR(hwfifo_reset, S_IWUSR|S_IWGRP, NULL,
		smi230_fifo_reset, 0);

static const struct attribute *smi230_fifo_attributes[] = {
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_reset.dev_attr.attr,
	NULL,
};

int smi230_acc_probe(struct device *dev, struct smi230_dev *p_smi230_dev)
{
	int ret = 0;
	int irq;
	unsigned long irqf;
	struct iio_dev *indio_dev;

	ret = smi230_init(p_smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Bosch Sensor %s initialization failed %d",
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
		dev_err(dev, "Failed to request GPIO pin");
		return ret;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_ACTIVE_LOW))
		irqf = IRQF_TRIGGER_FALLING;
	else if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_ACTIVE_HIGH))
		irqf = IRQF_TRIGGER_RISING;

	ret = devm_request_irq(&indio_dev->dev, irq,
			&iio_trigger_generic_data_rdy_poll,
			irqf,
			indio_dev->name,
			indio_dev->trig);

	if (ret) {
		dev_err(dev, "Failed to request irq for pin %d", irq);
		return ret;
	}

	indio_dev->trig->dev.parent = dev;
	indio_dev->trig->ops = &smi230_trigger_ops;
	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, indio_dev->trig);
	if (ret) {
		dev_err(dev, "Failed to register trigger");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      &iio_pollfunc_store_time,
					      smi230_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "Setup triggered buffer failed");
		return ret;
	}

	if (IS_ENABLED(CONFIG_IIO_SMI230_ACC_FIFO))
		iio_buffer_set_attrs(indio_dev->buffer, smi230_fifo_attributes);

	return devm_iio_device_register(dev, indio_dev);
}
