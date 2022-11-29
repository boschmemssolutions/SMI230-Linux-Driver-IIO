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
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/sysfs.h>
#include <linux/ioport.h>
#include <linux/of_gpio.h>

#include "smi230_acc.h"
#include "../gyro/smi230_gyro.h"
#include "smi230_config_file.h"

struct smi230_accel_fifo_config {
	uint8_t mode;
	uint8_t accel_en;
	uint8_t int1_en;
	uint8_t int2_en;
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
	struct smi230_accel_fifo_config fifo_conf;
};

enum {
	SMI230_ACC_X,
	SMI230_ACC_Y,
	SMI230_ACC_Z,
#ifndef CONFIG_SMI230_ACC_DATA_SYNC
	SMI230_ACC_TIMESTAMP,
#endif
	SMI230_GYRO_X,
	SMI230_GYRO_Y,
	SMI230_GYRO_Z,
#ifdef CONFIG_SMI230_ACC_DATA_SYNC
	SMI230_ACC_TIMESTAMP,
#endif
};

static uint8_t dsync;
static uint8_t temp_buff[CONFIG_SMI230_ACC_MAX_BUFFER_LEN + 1];
static uint8_t fifo_buf[SMI230_MAX_ACC_FIFO_BYTES];
static struct smi230_sensor_data fifo_accel_data[SMI230_MAX_ACC_FIFO_FRAME];

static struct iio_event_spec smi230_events[] = {
	/* Any-Motion */
	{
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	/* No-Motion */
	{
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	/* High-G Positive*/
	{
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_PERIOD) |
				      BIT(IIO_EV_INFO_HYSTERESIS),
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	/* High-G Negative*/
	{
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_FALLING,
	},
	/* Low-G */
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_PERIOD) |
			BIT(IIO_EV_INFO_HYSTERESIS) | BIT(IIO_EV_INFO_ENABLE),
	},
	/* Orientation Portrait Upright */
	{
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_RISING,
	},
	/* Orientation Portrait Upside Down */
	{
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_FALLING,
	},
	/* Orientation Landscape Left */
	{
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_EITHER,
	},
	/* Orientation Landscape Right */
	{
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_NONE,
		.mask_shared_by_all =
			BIT(IIO_EV_INFO_VALUE) | BIT(IIO_EV_INFO_HYSTERESIS) |
			BIT(IIO_EV_INFO_PERIOD) | BIT(IIO_EV_INFO_TIMEOUT) |
			BIT(IIO_EV_INFO_ENABLE),
	},
	/* Orientation Face Up */
	{
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE),
	},
	/* Orientation Face Down */
	{
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_FALLING,
	},
};

#ifdef CONFIG_SMI230_ACC_DATA_SYNC
#define SMI230_CHANNEL(_type, _axis, _index)                                     \
	{                                                                        \
		.type = _type,                                                 \
		.modified = 1,                                                 \
		.channel2 = IIO_MOD_##_axis,                                   \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),      \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
		.event_spec = smi230_events,                                   \
		.num_event_specs = ARRAY_SIZE(smi230_events), \
	}

#else

#define SMI230_CHANNEL(_type, _axis, _index)                                     \
	{                                                                        \
		.type = _type,                                                 \
		.modified = 1,                                                 \
		.channel2 = IIO_MOD_##_axis,                                   \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),      \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
		.event_spec = smi230_events,                                   \
		.num_event_specs = ARRAY_SIZE(smi230_events), \
	}
#endif

static const struct iio_chan_spec smi230_channels[] = {
	SMI230_CHANNEL(IIO_ACCEL, X, SMI230_ACC_X),
	SMI230_CHANNEL(IIO_ACCEL, Y, SMI230_ACC_Y),
	SMI230_CHANNEL(IIO_ACCEL, Z, SMI230_ACC_Z),
#ifdef CONFIG_SMI230_ACC_DATA_SYNC
	SMI230_CHANNEL(IIO_ANGL_VEL, X, SMI230_GYRO_X),
	SMI230_CHANNEL(IIO_ANGL_VEL, Y, SMI230_GYRO_Y),
	SMI230_CHANNEL(IIO_ANGL_VEL, Z, SMI230_GYRO_Z),
#endif
	IIO_CHAN_SOFT_TIMESTAMP(SMI230_ACC_TIMESTAMP),
};

static int8_t null_ptr_check(const struct smi230_acc_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) ||
	    (dev->delay_ms == NULL))
		rslt = SMI230_E_NULL_PTR;
	else
		rslt = SMI230_OK;

	return rslt;
}

static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t index;
	uint16_t temp_len = len + dev->dummy_byte;

	if (WARN(temp_len > CONFIG_SMI230_ACC_MAX_BUFFER_LEN,
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
		       const struct smi230_acc_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_SPI_INTF)
		reg_addr = (reg_addr & SMI230_SPI_WR_MASK);

	rslt = dev->write(dev->accel_id, reg_addr, reg_data, len);

	if (rslt != SMI230_OK)
		rslt = SMI230_E_COM_FAIL;

	return rslt;
}

static int8_t smi230_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data,
				  uint16_t len,
				  const struct smi230_acc_dev *dev)
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
				  uint16_t len,
				  const struct smi230_acc_dev *dev)
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

static int8_t smi230_acc_set_fifo_wm(uint16_t wm,
				     const struct smi230_acc_dev *dev)
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
					 const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data[SMI230_FIFO_DATA_LENGTH] = { 0 };

	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (fifo_length != NULL)) {
		rslt = smi230_acc_get_regs(SMI230_FIFO_LENGTH_0_ADDR, data,
					   SMI230_FIFO_DATA_LENGTH, dev);
		if (rslt == SMI230_OK) {
			data[1] = SMI230_GET_BITS_POS_0(
				data[1], SMI230_FIFO_BYTE_COUNTER_MSB);
			(*fifo_length) =
				(uint16_t)((uint16_t)(data[1] << 8) | data[0]);
		} else {
			rslt = SMI230_E_NULL_PTR;
		}
	}

	return rslt;
}

static int8_t smi230_acc_read_fifo_data(struct smi230_fifo_frame *fifo,
					const struct smi230_acc_dev *dev)
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
			rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_1_ADDR,
						   &config_data, 1, dev);
			if (rslt == SMI230_OK)
				fifo->data_enable =
					(uint16_t)((uint16_t)config_data &
						   SMI230_ACCEL_EN_MASK);
		} else {
			rslt = SMI230_E_COM_FAIL;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

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

static int8_t unpack_accel_frame(struct smi230_sensor_data *acc, uint16_t *idx,
				 uint16_t *acc_idx, uint16_t frame,
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
			      uint8_t current_frame_length,
			      const struct smi230_fifo_frame *fifo)
{
	int8_t rslt = SMI230_OK;

	if (((*data_index) + current_frame_length) > fifo->length) {
		(*data_index) = fifo->length;
		rslt = SMI230_W_FIFO_EMPTY;
	} else {
		(*data_index) = (*data_index) + current_frame_length;
	}

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
		sensor_time_byte3 =
			fifo->data[(*data_index) + SMI230_SENSOR_TIME_MSB_BYTE]
			<< 16;
		sensor_time_byte2 =
			fifo->data[(*data_index) + SMI230_SENSOR_TIME_XLSB_BYTE]
			<< 8;
		sensor_time_byte1 = fifo->data[(*data_index)];

		fifo->sensor_time =
			(uint32_t)(sensor_time_byte3 | sensor_time_byte2 |
				   sensor_time_byte1);
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
				       uint16_t *accel_length,
				       struct smi230_fifo_frame *fifo)
{
	int8_t rslt = SMI230_OK;
	uint8_t frame_header = 0;
	uint16_t data_index;
	uint16_t accel_index = 0;
	uint16_t frame_to_read = *accel_length;

	for (data_index = fifo->acc_byte_start_idx;
	     data_index < fifo->length;) {
		frame_header = fifo->data[data_index];
		data_index++;

		switch (frame_header) {
		case SMI230_FIFO_HEADER_ACC_FRM:
		case SMI230_FIFO_HEADER_ALL_FRM:
			rslt = unpack_accel_frame(accel_data, &data_index,
						  &accel_index, frame_header,
						  fifo);
			break;

		case SMI230_FIFO_HEADER_SENS_TIME_FRM:
			rslt = unpack_sensortime_frame(&data_index, fifo);
			break;

		case SMI230_FIFO_HEADER_SKIP_FRM:
			rslt = unpack_skipped_frame(&data_index, fifo);
			break;

		case SMI230_FIFO_HEADER_INPUT_CFG_FRM:
			rslt = move_next_frame(&data_index,
					       SMI230_FIFO_INPUT_CFG_LENGTH,
					       fifo);
			break;

		case SMI230_FIFO_SAMPLE_DROP_FRM:
			rslt = move_next_frame(&data_index,
					       SMI230_FIFO_INPUT_CFG_LENGTH,
					       fifo);
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

		if ((frame_to_read == accel_index) ||
		    (rslt == SMI230_W_FIFO_EMPTY))
			break;
	}

	(*accel_length) = accel_index;
	fifo->acc_byte_start_idx = data_index;

	return rslt;
}

static int8_t
smi230_acc_set_fifo_config(const struct smi230_accel_fifo_config *config,
			   const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data_array[2] = { 0 };

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = smi230_acc_get_regs(SMI230_FIFO_CONFIG_0_ADDR,
					   data_array, 2, dev);
		if (rslt == SMI230_OK) {
			data_array[0] = SMI230_SET_BITS_POS_0(
				data_array[0], SMI230_ACC_FIFO_MODE_CONFIG,
				config->mode);
			data_array[1] =
				SMI230_SET_BITS(data_array[1], SMI230_ACCEL_EN,
						config->accel_en);
			data_array[1] = SMI230_SET_BITS(data_array[1],
							SMI230_ACCEL_INT1_EN,
							config->int1_en);
			data_array[1] = SMI230_SET_BITS(data_array[1],
							SMI230_ACCEL_INT2_EN,
							config->int2_en);
			rslt = smi230_acc_set_regs(SMI230_FIFO_CONFIG_0_ADDR,
						   data_array, 2, dev);
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t
smi230_acc_set_int_pin_config(const struct smi230_acc_dev *dev,
			      enum smi230_accel_int_channel int_channel)
{
	uint8_t reg_addr;
	uint8_t data = 0;
	struct smi230_int_pin_cfg int_pin_cfg;

	switch (int_channel) {
	case SMI230_INT_CHANNEL_1:
		reg_addr = SMI230_ACCEL_INT1_IO_CONF_REG;
		int_pin_cfg = dev->int_channel_1.int_pin_cfg;
		break;
	case SMI230_INT_CHANNEL_2:
		reg_addr = SMI230_ACCEL_INT2_IO_CONF_REG;
		int_pin_cfg = dev->int_channel_2.int_pin_cfg;
		break;
	default:
		return SMI230_E_INVALID_INPUT;
	}

	data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_LVL, int_pin_cfg.lvl);

	data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_OD,
			       int_pin_cfg.output_mode);

	data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IO,
			       int_pin_cfg.output_en);

	data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_IN, int_pin_cfg.input_en);

	return smi230_acc_set_regs(reg_addr, &data, 1, dev);
}

static int8_t smi230_acc_set_int_type_config(const struct smi230_acc_dev *dev)
{
	uint8_t data = 0;
	uint8_t reg_addr = SMI230_ACCEL_INT1_INT2_MAP_DATA_REG;

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1))
		switch (dev->int_channel_1.int_type) {
		case SMI230_ACCEL_NO_INT:
			break;
		case SMI230_ACCEL_DATA_RDY_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_DRDY, 1);
			break;
		case SMI230_ACCEL_DATA_SYNC_INT:
			reg_addr = SMI230_ACCEL_INT1_MAP_REG;
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_DATA_SYNC,
					       1);
			break;
		case SMI230_ACCEL_FIFO_WM_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_FWM, 1);
			break;
		case SMI230_ACCEL_FIFO_FULL_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT1_FFUL, 1);
			break;
		default:
			return SMI230_E_INVALID_CONFIG;
		}
	else
		switch (dev->int_channel_2.int_type) {
		case SMI230_ACCEL_NO_INT:
			break;
		case SMI230_ACCEL_DATA_RDY_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_DRDY, 1);
			break;
		case SMI230_ACCEL_DATA_SYNC_INT:
			reg_addr = SMI230_ACCEL_INT2_MAP_REG;
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_DATA_SYNC,
					       1);
			break;
		case SMI230_ACCEL_FIFO_WM_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FWM, 1);
			break;
		case SMI230_ACCEL_FIFO_FULL_INT:
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT2_FFUL, 1);
			break;
		default:
			return SMI230_E_INVALID_CONFIG;
		}

	return smi230_acc_set_regs(reg_addr, &data, 1, dev);
}

static int8_t
smi230_acc_set_int_feature_config(const struct smi230_acc_dev *dev,
				  enum smi230_accel_int_channel int_channel)
{
	uint8_t reg_addr;
	uint8_t data;

	switch (int_channel) {
	case SMI230_INT_CHANNEL_1:
		reg_addr = SMI230_ACCEL_INT1_MAP_REG;
		data = dev->int_channel_1.int_features;
		break;
	case SMI230_INT_CHANNEL_2:
		reg_addr = SMI230_ACCEL_INT2_MAP_REG;
		data = dev->int_channel_2.int_features;
		break;
	default:
		return SMI230_E_INVALID_INPUT;
	}

	return smi230_acc_set_regs(reg_addr, &data, 1, dev);
}

static int8_t smi230_acc_soft_reset(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		data = SMI230_SOFT_RESET_CMD;
		rslt = set_regs(SMI230_ACCEL_SOFTRESET_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			dev->delay_ms(SMI230_SOFTRESET_DELAY_MS);

			/*
			 * After soft reset, in SPI mode a dummy SPI read
			 * operation needs to be performed again.
			 * The soft-reset performs a fundamental reset to the
			 * device, which is largely equivalent to a power cycle.
			 */
			if (dev->intf == SMI230_SPI_INTF) {
				/* Dummy SPI read operation of Chip-ID */
				rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &data,
						1, dev);
			}
		}
	}

	return rslt;
}

static int8_t smi230_acc_write_config_file(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data = SMI230_DISABLE;
	uint8_t bin_pointer[2];
	int i;

	rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG, &data, 1, dev);
	if (rslt != SMI230_OK) {
		pr_err("write pwr ctrl reg failed");
		return rslt;
	}

	dev->delay_ms(SMI230_POWER_CONFIG_DELAY);

	rslt = set_regs(SMI230_ACCEL_PWR_CONF_REG, &data, 1, dev);
	if (rslt != SMI230_OK) {
		pr_err("write pwr conf reg failed");
		return rslt;
	}

	dev->delay_ms(SMI230_POWER_CONFIG_DELAY);

	rslt = set_regs(SMI230_ACCEL_INIT_CTRL_REG, &data, 1, dev);
	if (rslt != SMI230_OK) {
		pr_err("write init ctrl reg failed");
		return rslt;
	}

	dev->delay_ms(SMI230_ASIC_INIT_TIME_MS);

	for (i = 0; i < SMI230_CONFIG_STREAM_SIZE; i += dev->read_write_len) {
		bin_pointer[1] = (uint8_t)(i >> 5);
		bin_pointer[0] = (uint8_t)((i >> 1) & 0x0F);
		rslt = set_regs(SMI230_ACCEL_RESERVED_5B_REG, bin_pointer, 2,
				dev);
		if (rslt != SMI230_OK) {
			pr_err("write bin_pointer failed at %d", i);
			return rslt;
		}
		rslt = set_regs(SMI230_ACCEL_FEATURE_CFG_REG,
				(uint8_t *)(dev->config_file_ptr + i),
				dev->read_write_len, dev);
		if (rslt != SMI230_OK) {
			pr_err("write feature cfg failed at %d", i);
			return rslt;
		}
	}

	data = SMI230_ENABLE;
	rslt = set_regs(SMI230_ACCEL_INIT_CTRL_REG, &data, 1, dev);
	if (rslt != SMI230_OK) {
		pr_err("enable init ctrl reg failed");
		return rslt;
	}
	dev->delay_ms(SMI230_ASIC_INIT_TIME_MS);

	rslt = get_regs(SMI230_ACCEL_INTERNAL_STAT_REG, &data, 1, dev);
	if (rslt != SMI230_OK) {
		pr_err("get internal status failed");
		return rslt;
	}

	if ((data & SMI230_CONFIG_STATUS_MASK) == 0x01)
		rslt = SMI230_OK;
	else {
		pr_err("config invalid. status 0x%02x", data);
		rslt = SMI230_E_CONFIG_STREAM_ERROR;
	}

	dev->delay_ms(SMI230_ASIC_INIT_TIME_MS);
	return rslt;
}

static int8_t smi230_acc_get_meas_conf(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);

		if (rslt == SMI230_OK) {
			dev->accel_cfg.odr = data[0] & SMI230_ACCEL_ODR_MASK;
			dev->accel_cfg.bw =
				(data[0] & SMI230_ACCEL_BW_MASK) >> 4;
			dev->accel_cfg.range =
				data[1] & SMI230_ACCEL_RANGE_MASK;
		}
	}

	return rslt;
}

static int8_t smi230_acc_set_meas_conf(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data[2] = { 0 };
	uint8_t bw, range, odr;
	uint8_t is_odr_invalid = FALSE;
	uint8_t is_bw_invalid = FALSE;
	uint8_t is_range_invalid = FALSE;

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return SMI230_E_NULL_PTR;

	odr = dev->accel_cfg.odr;
	bw = dev->accel_cfg.bw;
	range = dev->accel_cfg.range;

	if ((odr < SMI230_ACCEL_ODR_12_5_HZ) ||
	    (odr > SMI230_ACCEL_ODR_1600_HZ))
		is_odr_invalid = TRUE;

	if (bw > SMI230_ACCEL_BW_NORMAL)
		is_bw_invalid = TRUE;

	if (range > SMI230_ACCEL_RANGE_16G)
		is_range_invalid = TRUE;

	if ((!is_odr_invalid) && (!is_bw_invalid) && (!is_range_invalid)) {
		rslt = get_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
		if (rslt == SMI230_OK) {
			data[0] = SMI230_SET_BITS_POS_0(data[0],
							SMI230_ACCEL_ODR, odr);
			data[0] = SMI230_SET_BITS(data[0], SMI230_ACCEL_BW, bw);
			data[1] = SMI230_SET_BITS_POS_0(
				data[1], SMI230_ACCEL_RANGE, range);

			rslt = set_regs(SMI230_ACCEL_CONF_REG, data, 2, dev);
			dev->delay_ms(SMI230_ACC_CONF_DELAY_MS);
		}
	} else {
		rslt = SMI230_E_INVALID_CONFIG;
	}

	return rslt;
}

static int8_t smi230_acc_get_power_mode(struct smi230_acc_dev *dev)
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

static int8_t smi230_acc_set_power_mode(const struct smi230_acc_dev *dev)
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
			rslt = set_regs(SMI230_ACCEL_PWR_CONF_REG, &data[0], 1,
					dev);

			if (rslt == SMI230_OK) {
				dev->delay_ms(SMI230_POWER_CONFIG_DELAY);
				rslt = set_regs(SMI230_ACCEL_PWR_CTRL_REG,
						&data[1], 1, dev);
				if (rslt == SMI230_OK)
					dev->delay_ms(
						SMI230_POWER_CONFIG_DELAY);
			}
		}
	}

	return rslt;
}

static int8_t smi230_acc_get_data(struct smi230_sensor_data *accel,
				  const struct smi230_acc_dev *dev)
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
			accel->x = ((int16_t)msblsb); /* Data in X axis */

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			accel->y = ((int16_t)msblsb); /* Data in Y axis */

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			accel->z = ((int16_t)msblsb); /* Data in Z axis */

			accel->sensor_time =
				(data[8] << 16) | (data[7] << 8) | data[6];
		}

	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t smi230_acc_get_fifo_wm(uint16_t *wm,
				     const struct smi230_acc_dev *dev)
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

static int8_t smi230_acc_fifo_reset(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		data = SMI230_FIFO_RESET_CMD;

		rslt = set_regs(SMI230_ACCEL_SOFTRESET_REG, &data, 1, dev);

		if (rslt == SMI230_OK)
			dev->delay_ms(SMI230_SOFTRESET_DELAY_MS);
	}

	return rslt;
}

static int8_t
smi230_acc_get_sensor_temperature(const struct smi230_acc_dev *dev,
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
			msblsb = (uint16_t)(msb + lsb);

			if (msblsb > 1023)
				temp = (int16_t)(msblsb - 2048);
			else
				temp = (int16_t)msblsb;

			*sensor_temp = (temp * 125) + 23000;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

static int8_t smi230_acc_self_test(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t data;
	int16_t x, y, z;
	struct smi230_sensor_data sdata = { 0 };

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;
	dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
	dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;

	rslt = smi230_acc_set_meas_conf(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data = SMI230_ACCEL_POSITIVE_SELF_TEST;
	rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &data, 1, dev);
	if (rslt != SMI230_OK)
		return rslt;

	dev->delay_ms(SMI230_SELF_TEST_DELAY_MS);

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

	dev->delay_ms(SMI230_SELF_TEST_DELAY_MS);

	rslt = smi230_acc_get_data(&sdata, dev);
	if (rslt != SMI230_OK)
		return rslt;

	data = SMI230_ACCEL_SWITCH_OFF_SELF_TEST;
	rslt = set_regs(SMI230_ACCEL_SELF_TEST_REG, &data, 1, dev);
	if (rslt != SMI230_OK)
		return rslt;

	if ((x - sdata.x) < 2048 || (y - sdata.y) < 2048 ||
	    (z - sdata.z) < 1024)
		rslt = SMI230_W_SELF_TEST_FAIL;

	dev->delay_ms(SMI230_SELF_TEST_DELAY_MS);

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

int8_t smi230_acc_init(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		if (dev->intf == SMI230_SPI_INTF) {
			/* Set dummy byte in case of SPI interface */
			dev->dummy_byte = SMI230_ENABLE;

			/* Dummy read of Chip-ID in SPI mode */
			rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1,
					dev);
		} else {
			/* Make dummy byte 0 in case of I2C interface */
			dev->dummy_byte = SMI230_DISABLE;
		}

		if (rslt == SMI230_OK) {
			rslt = get_regs(SMI230_ACCEL_CHIP_ID_REG, &chip_id, 1,
					dev);

			if (rslt == SMI230_OK) {
				if (chip_id == SMI230_ACCEL_CHIP_ID)
					dev->accel_chip_id = chip_id;
				else
					rslt = SMI230_E_DEV_NOT_FOUND;
			}
		}
	}

	return rslt;
}

static int8_t smi230_acc_write_feature_config(uint8_t reg_addr,
					      const uint16_t *reg_data,
					      uint8_t len,
					      const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t bin_pointer[2];
	int i;

	if (WARN(len * 2 > CONFIG_SMI230_ACC_MAX_BUFFER_LEN,
		 "SMI230 buffer overflow\n"))
		return SMI230_E_COM_FAIL;

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	bin_pointer[1] = 0x10;
	bin_pointer[0] = reg_addr;

	rslt = set_regs(SMI230_ACCEL_RESERVED_5B_REG, bin_pointer, 2, dev);
	if (rslt != SMI230_OK) {
		pr_err("write bin_pointer failed");
		return rslt;
	}

	for (i = 0; i < len; ++i) {
		temp_buff[i * 2] = reg_data[i] & 0x00FF;
		temp_buff[i * 2 + 1] = (reg_data[i] & 0xFF00) >> 8;
	}

	rslt = set_regs(SMI230_ACCEL_FEATURE_CFG_REG, temp_buff, len * 2, dev);

	return rslt;
}

static int8_t smi230_acc_read_feature_config(uint8_t reg_addr,
					     uint16_t *reg_data, uint8_t len,
					     const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t bin_pointer[2];
	int i;

	if (WARN(len * 2 > CONFIG_SMI230_ACC_MAX_BUFFER_LEN,
		 "SMI230 buffer overflow\n"))
		return SMI230_E_COM_FAIL;

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	bin_pointer[1] = 0x10;
	bin_pointer[0] = reg_addr;

	rslt = set_regs(SMI230_ACCEL_RESERVED_5B_REG, bin_pointer, 2, dev);
	if (rslt != SMI230_OK) {
		pr_err("write bin_pointer failed");
		return rslt;
	}

	rslt = smi230_acc_get_regs(SMI230_ACCEL_FEATURE_CFG_REG, temp_buff,
				   len * 2, dev);

	if (rslt != SMI230_OK)
		return rslt;

	for (i = 0; i < len; ++i)
		reg_data[i] = temp_buff[i * 2] | (temp_buff[(i * 2) + 1] << 8);

	return rslt;
}

static int8_t smi230_get_anymotion_config(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_ACCEL_ANYMOTION_LEN];

	rslt = smi230_acc_read_feature_config(SMI230_ACCEL_ANYMOTION_ADR, data,
					      SMI230_ACCEL_ANYMOTION_LEN, dev);

	if (rslt != SMI230_OK)
		return rslt;

	dev->anymotion_cfg.threshold =
		data[0] & SMI230_ACCEL_ANYMOTION_THRESHOLD_MASK;
	dev->anymotion_cfg.enable =
		(data[0] & SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_MASK) >>
		SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT;
	dev->anymotion_cfg.duration =
		data[1] & SMI230_ACCEL_ANYMOTION_DURATION_MASK;
	dev->anymotion_cfg.x_en =
		(data[1] & SMI230_ACCEL_ANYMOTION_X_EN_MASK) >>
		SMI230_ACCEL_ANYMOTION_X_EN_SHIFT;
	dev->anymotion_cfg.y_en =
		(data[1] & SMI230_ACCEL_ANYMOTION_Y_EN_MASK) >>
		SMI230_ACCEL_ANYMOTION_Y_EN_SHIFT;
	dev->anymotion_cfg.z_en =
		(data[1] & SMI230_ACCEL_ANYMOTION_Z_EN_MASK) >>
		SMI230_ACCEL_ANYMOTION_Z_EN_SHIFT;

	return rslt;
}

static int8_t smi230_set_anymotion_config(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_ACCEL_ANYMOTION_LEN];

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data[0] = dev->anymotion_cfg.threshold &
		  SMI230_ACCEL_ANYMOTION_THRESHOLD_MASK;
	data[0] |= (dev->anymotion_cfg.enable
		    << SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT) &
		   SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_MASK;
	data[1] = dev->anymotion_cfg.duration &
		  SMI230_ACCEL_ANYMOTION_DURATION_MASK;
	data[1] |=
		(dev->anymotion_cfg.x_en << SMI230_ACCEL_ANYMOTION_X_EN_SHIFT) &
		SMI230_ACCEL_ANYMOTION_X_EN_MASK;
	data[1] |=
		(dev->anymotion_cfg.y_en << SMI230_ACCEL_ANYMOTION_Y_EN_SHIFT) &
		SMI230_ACCEL_ANYMOTION_Y_EN_MASK;
	data[1] |=
		(dev->anymotion_cfg.z_en << SMI230_ACCEL_ANYMOTION_Z_EN_SHIFT) &
		SMI230_ACCEL_ANYMOTION_Z_EN_MASK;
	rslt = smi230_acc_write_feature_config(SMI230_ACCEL_ANYMOTION_ADR, data,
					       SMI230_ACCEL_ANYMOTION_LEN, dev);

	return rslt;
}

static int8_t smi230_get_nomotion_config(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_NOMOTION_LEN];

	rslt = smi230_acc_read_feature_config(SMI230_NOMOTION_START_ADR, data,
					      SMI230_NOMOTION_LEN, dev);

	if (rslt != SMI230_OK)
		return rslt;

	dev->nomotion_cfg.threshold = data[0] & SMI230_NOMOTION_THRESHOLD_MASK;
	dev->nomotion_cfg.enable =
		(data[0] & SMI230_NOMOTION_EN_MASK) >> SMI230_NOMOTION_EN_POS;
	dev->nomotion_cfg.duration = data[1] & SMI230_NOMOTION_DURATION_MASK;
	dev->nomotion_cfg.x_en = (data[1] & SMI230_NOMOTION_X_EN_MASK) >>
				 SMI230_NOMOTION_X_EN_POS;
	dev->nomotion_cfg.y_en = (data[1] & SMI230_NOMOTION_Y_EN_MASK) >>
				 SMI230_NOMOTION_Y_EN_POS;
	dev->nomotion_cfg.z_en = (data[1] & SMI230_NOMOTION_Z_EN_MASK) >>
				 SMI230_NOMOTION_Z_EN_POS;

	return rslt;
}

static int8_t smi230_set_nomotion_config(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_NOMOTION_LEN];

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data[0] = dev->nomotion_cfg.threshold & SMI230_NOMOTION_THRESHOLD_MASK;
	data[0] |= (dev->nomotion_cfg.enable << SMI230_NOMOTION_EN_POS) &
		   SMI230_NOMOTION_EN_MASK;
	data[1] = dev->nomotion_cfg.duration & SMI230_NOMOTION_DURATION_MASK;
	data[1] |= (dev->nomotion_cfg.x_en << SMI230_NOMOTION_X_EN_POS) &
		   SMI230_NOMOTION_X_EN_MASK;
	data[1] |= (dev->nomotion_cfg.y_en << SMI230_NOMOTION_Y_EN_POS) &
		   SMI230_NOMOTION_Y_EN_MASK;
	data[1] |= (dev->nomotion_cfg.z_en << SMI230_NOMOTION_Z_EN_POS) &
		   SMI230_NOMOTION_Z_EN_MASK;
	rslt = smi230_acc_write_feature_config(SMI230_NOMOTION_START_ADR, data,
					       SMI230_NOMOTION_LEN, dev);

	return rslt;
}

static int8_t smi230_get_high_g_config(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_HIGH_G_LEN];

	rslt = smi230_acc_read_feature_config(SMI230_HIGH_G_START_ADR, data,
					      SMI230_HIGH_G_LEN, dev);

	if (rslt != SMI230_OK)
		return rslt;

	dev->high_g_cfg.threshold =
		SMI230_GET_BITS(data[0], SMI230_HIGH_G_THRESHOLD);
	dev->high_g_cfg.enable = SMI230_GET_BITS(data[1], SMI230_HIGH_G_ENABLE);
	dev->high_g_cfg.x_en = SMI230_GET_BITS(data[1], SMI230_HIGH_G_X_EN);
	dev->high_g_cfg.y_en = SMI230_GET_BITS(data[1], SMI230_HIGH_G_Y_EN);
	dev->high_g_cfg.z_en = SMI230_GET_BITS(data[1], SMI230_HIGH_G_Z_EN);
	dev->high_g_cfg.hysteresis =
		SMI230_GET_BITS(data[1], SMI230_HIGH_G_HYSTERESIS);
	dev->high_g_cfg.duration =
		SMI230_GET_BITS(data[2], SMI230_HIGH_G_DURATION);

	return rslt;
}

static int8_t smi230_set_high_g_config(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_HIGH_G_LEN];

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data[0] = SMI230_SET_BITS(data[0], SMI230_HIGH_G_THRESHOLD,
				  dev->high_g_cfg.threshold);
	data[1] = SMI230_SET_BITS(data[1], SMI230_HIGH_G_ENABLE,
				  dev->high_g_cfg.enable);
	data[1] = SMI230_SET_BITS(data[1], SMI230_HIGH_G_X_EN,
				  dev->high_g_cfg.x_en);
	data[1] = SMI230_SET_BITS(data[1], SMI230_HIGH_G_Y_EN,
				  dev->high_g_cfg.y_en);
	data[1] = SMI230_SET_BITS(data[1], SMI230_HIGH_G_Z_EN,
				  dev->high_g_cfg.z_en);
	data[1] = SMI230_SET_BITS(data[1], SMI230_HIGH_G_HYSTERESIS,
				  dev->high_g_cfg.hysteresis);
	data[2] = SMI230_SET_BITS(data[2], SMI230_HIGH_G_DURATION,
				  dev->high_g_cfg.duration);

	rslt = smi230_acc_write_feature_config(SMI230_HIGH_G_START_ADR, data,
					       SMI230_HIGH_G_LEN, dev);

	return rslt;
}

static int8_t smi230_get_low_g_config(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_LOW_G_LEN];

	rslt = smi230_acc_read_feature_config(SMI230_LOW_G_START_ADR, data,
					      SMI230_LOW_G_LEN, dev);

	if (rslt != SMI230_OK)
		return rslt;

	dev->low_g_cfg.threshold =
		SMI230_GET_BITS(data[0], SMI230_LOW_G_THRESHOLD);
	dev->low_g_cfg.enable = SMI230_GET_BITS(data[1], SMI230_LOW_G_ENABLE);
	dev->low_g_cfg.hysteresis =
		SMI230_GET_BITS(data[1], SMI230_LOW_G_HYSTERESIS);
	dev->low_g_cfg.duration =
		SMI230_GET_BITS(data[2], SMI230_LOW_G_DURATION);

	return rslt;
}

static int8_t smi230_set_low_g_config(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_LOW_G_LEN];

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data[0] = SMI230_SET_BITS(data[0], SMI230_LOW_G_THRESHOLD,
				  dev->low_g_cfg.threshold);
	data[1] = SMI230_SET_BITS(data[1], SMI230_LOW_G_ENABLE,
				  dev->low_g_cfg.enable);
	data[1] = SMI230_SET_BITS(data[1], SMI230_LOW_G_HYSTERESIS,
				  dev->low_g_cfg.hysteresis);
	data[2] = SMI230_SET_BITS(data[2], SMI230_LOW_G_DURATION,
				  dev->low_g_cfg.duration);

	rslt = smi230_acc_write_feature_config(SMI230_LOW_G_START_ADR, data,
					       SMI230_LOW_G_LEN, dev);

	return rslt;
}

static int8_t smi230_get_orientation_config(struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_ORIENT_LEN];

	rslt = smi230_acc_read_feature_config(SMI230_ORIENT_START_ADR, data,
					      SMI230_ORIENT_LEN, dev);

	if (rslt != SMI230_OK)
		return rslt;

	dev->orient_cfg.enable = SMI230_GET_BITS(data[0], SMI230_ORIENT_ENABLE);
	dev->orient_cfg.ud_en = SMI230_GET_BITS(data[0], SMI230_ORIENT_UP_DOWN);
	dev->orient_cfg.mode =
		SMI230_GET_BITS(data[0], SMI230_ORIENT_SYMM_MODE);
	dev->orient_cfg.blocking =
		SMI230_GET_BITS(data[0], SMI230_ORIENT_BLOCK_MODE);
	dev->orient_cfg.theta = SMI230_GET_BITS(data[0], SMI230_ORIENT_THETA);
	dev->orient_cfg.hysteresis =
		SMI230_GET_BITS(data[1], SMI230_ORIENT_HYST);

	return rslt;
}

static int8_t smi230_set_orientation_config(const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint16_t data[SMI230_ORIENT_LEN];

	rslt = null_ptr_check(dev);
	if (rslt != SMI230_OK)
		return rslt;

	data[0] = SMI230_SET_BITS(data[0], SMI230_ORIENT_ENABLE,
				  dev->orient_cfg.enable);
	data[0] = SMI230_SET_BITS(data[0], SMI230_ORIENT_UP_DOWN,
				  dev->orient_cfg.ud_en);
	data[0] = SMI230_SET_BITS(data[0], SMI230_ORIENT_SYMM_MODE,
				  dev->orient_cfg.mode);
	data[0] = SMI230_SET_BITS(data[0], SMI230_ORIENT_BLOCK_MODE,
				  dev->orient_cfg.blocking);
	data[0] = SMI230_SET_BITS(data[0], SMI230_ORIENT_THETA,
				  dev->orient_cfg.theta);
	data[1] = SMI230_SET_BITS(data[1], SMI230_ORIENT_HYST,
				  dev->orient_cfg.hysteresis);

	rslt = smi230_acc_write_feature_config(SMI230_ORIENT_START_ADR, data,
					       SMI230_ORIENT_LEN, dev);

	return rslt;
}

static int smi230_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	int ret;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = smi230_acc_get_data(&data, dev);
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
		ret = smi230_acc_get_meas_conf(dev);
		if (ret)
			return 0;

		if (IS_ENABLED(CONFIG_SMI230_ACC_DATA_SYNC))
			switch (dev->sync_cfg.mode) {
			case SMI230_ACCEL_DATA_SYNC_MODE_100HZ:
				*val = 100;
				*val2 = 0;
				break;
			case SMI230_ACCEL_DATA_SYNC_MODE_200HZ:
				*val = 200;
				*val2 = 0;
				break;
			case SMI230_ACCEL_DATA_SYNC_MODE_400HZ:
				*val = 400;
				*val2 = 0;
				break;
			case SMI230_ACCEL_DATA_SYNC_MODE_1000HZ:
				*val = 1000;
				*val2 = 0;
				break;
			case SMI230_ACCEL_DATA_SYNC_MODE_2000HZ:
				*val = 2000;
				*val2 = 0;
				break;
			}
		else
			switch (dev->accel_cfg.odr) {
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

static int8_t
smi230_configure_data_synchronization(struct smi230_data_sync_cfg sync_cfg,
				      struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t gyro_odr, gyro_bw;
	uint16_t data[SMI230_ACCEL_DATA_SYNC_LEN];

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		/* Change sensor meas config */
		switch (sync_cfg.mode) {
		case SMI230_ACCEL_DATA_SYNC_MODE_2000HZ:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
			dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
			gyro_odr = SMI230_GYRO_BW_230_ODR_2000_HZ;
			gyro_bw = SMI230_GYRO_BW_230_ODR_2000_HZ;
			break;
		case SMI230_ACCEL_DATA_SYNC_MODE_1000HZ:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
			dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
			gyro_odr = SMI230_GYRO_BW_116_ODR_1000_HZ;
			gyro_bw = SMI230_GYRO_BW_116_ODR_1000_HZ;
			break;
		case SMI230_ACCEL_DATA_SYNC_MODE_400HZ:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
			dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
			gyro_odr = SMI230_GYRO_BW_47_ODR_400_HZ;
			gyro_bw = SMI230_GYRO_BW_47_ODR_400_HZ;
			break;
		case SMI230_ACCEL_DATA_SYNC_MODE_100HZ:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
			dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR4;
			gyro_odr = SMI230_GYRO_BW_32_ODR_100_HZ;
			gyro_bw = SMI230_GYRO_BW_32_ODR_100_HZ;
			break;
		case SMI230_ACCEL_DATA_SYNC_MODE_200HZ:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
			dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR4;
			gyro_odr = SMI230_GYRO_BW_64_ODR_200_HZ;
			gyro_bw = SMI230_GYRO_BW_64_ODR_200_HZ;
			break;
		default:
			break;
		}
		rslt = smi230_acc_set_meas_conf(dev);
		if (rslt != SMI230_OK)
			return rslt;

		rslt = smi230_gyro_set_meas_conf_ex(gyro_odr, gyro_bw);
		if (rslt != SMI230_OK)
			return rslt;

		/* Enable data synchronization */
		data[0] = (sync_cfg.mode & SMI230_ACCEL_DATA_SYNC_MODE_MASK);
		rslt = smi230_acc_write_feature_config(
			SMI230_ACCEL_DATA_SYNC_ADR, &data[0],
			SMI230_ACCEL_DATA_SYNC_LEN, dev);
	}

	return rslt;
}

#ifdef CONFIG_SMI230_ACC_DATA_SYNC
static int smi230_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (val) {
		case 100:
			dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_100HZ;
			break;
		case 200:
			dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_200HZ;
			break;
		case 400:
			dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_400HZ;
			break;
		case 1000:
			dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_1000HZ;
			break;
		case 2000:
			dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
			break;
		default:
			return -EINVAL;
		}

		ret = smi230_configure_data_synchronization(dev->sync_cfg, dev);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
#else
static int smi230_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (val) {
		case 12:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_12_5_HZ;
			break;
		case 25:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_25_HZ;
			break;
		case 50:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_50_HZ;
			break;
		case 100:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_100_HZ;
			break;
		case 200:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_200_HZ;
			break;
		case 400:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_400_HZ;
			break;
		case 800:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_800_HZ;
			break;
		case 1600:
			dev->accel_cfg.odr = SMI230_ACCEL_ODR_1600_HZ;
			break;
		default:
			return -EINVAL;
		}

		ret = smi230_acc_set_meas_conf(dev);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}
#endif

static int smi230_read_channel(struct smi230_sensor_data *sdata, int i,
			       int16_t *sample)
{
	switch (i) {
	case SMI230_ACC_X:
		*sample = sdata->x;
		break;
	case SMI230_ACC_Y:
		*sample = sdata->y;
		break;
	case SMI230_ACC_Z:
		*sample = sdata->z;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230_read_channel_datasync(struct smi230_sensor_data *acc_data,
					struct smi230_sensor_data *gyro_data,
					int i, int16_t *sample)
{
	switch (i) {
	case SMI230_ACC_X:
		*sample = acc_data->x;
		break;
	case SMI230_ACC_Y:
		*sample = acc_data->y;
		break;
	case SMI230_ACC_Z:
		*sample = acc_data->z;
		break;
	case SMI230_GYRO_X:
		*sample = gyro_data->x;
		break;
	case SMI230_GYRO_Y:
		*sample = gyro_data->y;
		break;
	case SMI230_GYRO_Z:
		*sample = gyro_data->z;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t hwfifo_watermark_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int err = 0;
	uint16_t fifo_wm;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	err = kstrtou16(buf, 10, &fifo_wm);
	if (err) {
		dev_err(dev, "Invalid argument");
		return err;
	}

	err = smi230_acc_set_fifo_wm(fifo_wm, smi230_dev);
	if (err != SMI230_OK) {
		dev_err(dev, "Failed to set FIFO WM.");
		return err;
	}

	return count;
}

static int smi230_extract_accel_fifo_data(const struct smi230_acc_dev *dev,
					  struct smi230_sensor_data *data,
					  uint16_t *count)
{
	int ret;
	struct smi230_fifo_frame fifo;

	fifo.data = fifo_buf;
	ret = smi230_acc_get_fifo_length(&fifo.length, dev);
	if (ret != SMI230_OK)
		return ret;
	ret = smi230_acc_read_fifo_data(&fifo, dev);
	if (ret != SMI230_OK)
		return ret;
	ret = smi230_acc_extract_accel(data, count, &fifo);

	return ret;
}

static int smi230_fifo_flush(struct iio_dev *indio_dev, int64_t timestamp,
			     unsigned int count)
{
	int ret = 0;
	int i;
	int16_t buf[8];
	int16_t sample;
	uint16_t frame_count = count;
	uint32_t tsamp;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_extract_accel_fifo_data(dev, fifo_accel_data,
					     &frame_count);

	if (ret) {
		dev_err(indio_dev->dev.parent, "Error extracting FIFO data.");
		return -1;
	}

	switch (dev->accel_cfg.odr) {
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

		for_each_set_bit(k, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			ret |= smi230_read_channel(&fifo_accel_data[i], k,
						   &sample);
			buf[j++] = sample;
		}

		iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
		timestamp += tsamp;
	}

	if (ret)
		return -1;

	return frame_count;
}

static int8_t smi230_get_synchronized_data(struct smi230_sensor_data *accel,
					   struct smi230_sensor_data *gyro,
					   const struct smi230_acc_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr, data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;

	rslt = null_ptr_check(dev);

	if ((rslt != SMI230_OK) || (accel == NULL) || (gyro == NULL))
		return SMI230_E_NULL_PTR;

	reg_addr = SMI230_ACCEL_GP_0_REG;
	rslt = smi230_acc_get_regs(reg_addr, &data[0], 4, dev);
	reg_addr = SMI230_ACCEL_GP_4_REG;
	rslt |= smi230_acc_get_regs(reg_addr, &data[4], 2, dev);

	if (rslt != SMI230_OK)
		return rslt;

	lsb = data[0];
	msb = data[1];
	msblsb = (msb << 8) | lsb;
	accel->x = ((int16_t)msblsb);

	lsb = data[2];
	msb = data[3];
	msblsb = (msb << 8) | lsb;
	accel->y = ((int16_t)msblsb);

	lsb = data[4];
	msb = data[5];
	msblsb = (msb << 8) | lsb;
	accel->z = ((int16_t)msblsb);

	return smi230_gyro_get_data_ex(&gyro->x, &gyro->y, &gyro->z);
}

static int smi230_data_ready_handler(struct iio_dev *indio_dev,
				     int64_t timestamp)
{
	int16_t buf[8];
	int16_t sample;
	int ret, i, j = 0;
	struct smi230_sensor_data sensor_data = { 0 };
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_data(&sensor_data, dev);
	if (ret) {
		dev_err(indio_dev->dev.parent, "Reading sensor data failed");
		return ret;
	}

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = smi230_read_channel(&sensor_data, i, &sample);
		if (ret) {
			dev_err(indio_dev->dev.parent, "Read channel %d failed",
				i);
			return ret;
		}
		buf[j++] = sample;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	if (ret)
		pr_err("Push to buffer failed");

	return ret;
}

static int smi230_data_sync_handle(struct iio_dev *indio_dev, int64_t timestamp)
{
	int16_t buf[16];
	int16_t sample;
	int ret, i, j = 0;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data gyro_data;
	struct smi230_sensor_data accel_sync;

	ret = smi230_get_synchronized_data(&accel_sync, &gyro_data, dev);
	if (ret != SMI230_OK)
		return ret;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = smi230_read_channel_datasync(&accel_sync, &gyro_data, i,
						   &sample);
		if (ret) {
			dev_err(indio_dev->dev.parent, "Read channel %d failed",
				i);
			return ret;
		}
		buf[j++] = sample;
	}
	//	dev_info(indio_dev->dev.parent, "acc %d %d %d gyro %d %d %d", buf[0],
	//		 buf[1], buf[2], buf[3], buf[4], buf[5]);
	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);
	if (ret)
		pr_err("Push acc data to buffer failed");

	return ret;
}

static int smi230_init(struct smi230_acc_dev *dev)
{
	int err = 0;
	int i;
	uint8_t int_lvl, int_mode;
	struct smi230_accel_fifo_config fifo_config;

	for (i = 0; i < 10; ++i) {
		err = smi230_acc_soft_reset(dev);
		if (err != SMI230_OK)
			continue;

		err = smi230_acc_write_config_file(dev);
		if (err != SMI230_OK)
			continue;
		else
			break;
	}

	if (err != SMI230_OK) {
		pr_err("SMI230 write config file failed.");
		return err;
	}

	err = smi230_get_anymotion_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get anymotion config failed.");
		return err;
	}

	dev->anymotion_cfg.x_en = 0;
	dev->anymotion_cfg.y_en = 0;
	dev->anymotion_cfg.z_en = 0;
	err = smi230_set_anymotion_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set anymotion config failed.");
		return err;
	}

	err = smi230_get_nomotion_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get nomotion config failed.");
		return err;
	}

	dev->nomotion_cfg.x_en = 0;
	dev->nomotion_cfg.y_en = 0;
	dev->nomotion_cfg.z_en = 0;
	err = smi230_set_nomotion_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set nomotion config failed.");
		return err;
	}

	err = smi230_get_high_g_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get high-g config failed.");
		return err;
	}

	dev->high_g_cfg.x_en = 0;
	dev->high_g_cfg.y_en = 0;
	dev->high_g_cfg.z_en = 0;
	err = smi230_set_high_g_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 set high_g config failed.");
		return err;
	}

	err = smi230_get_low_g_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get low-g config failed.");
		return err;
	}

	err = smi230_get_orientation_config(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get orientation config failed.");
		return err;
	}

	err = smi230_acc_get_meas_conf(dev);
	if (err != SMI230_OK) {
		pr_err("SMI230 get meas config failed.");
		return err;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_PUSH_PULL))
		int_mode = SMI230_INT_MODE_PUSH_PULL;
	else if (IS_ENABLED(CONFIG_SMI230_ACC_OPEN_DRAIN))
		int_mode = SMI230_INT_MODE_OPEN_DRAIN;

	if (IS_ENABLED(CONFIG_SMI230_ACC_ACTIVE_LOW))
		int_lvl = SMI230_INT_ACTIVE_LOW;
	else if (IS_ENABLED(CONFIG_SMI230_ACC_ACTIVE_HIGH))
		int_lvl = SMI230_INT_ACTIVE_HIGH;

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1)) {
		dev->int_channel_1.int_pin_cfg.output_en = SMI230_ENABLE;
		dev->int_channel_1.int_pin_cfg.input_en = SMI230_DISABLE;
		dev->int_channel_1.int_pin_cfg.output_mode = int_mode;
		dev->int_channel_1.int_pin_cfg.lvl = int_lvl;
		dev->int_channel_1.int_type = SMI230_ACCEL_NO_INT;
		dev->int_channel_1.int_features = 0x00;
	} else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2)) {
		dev->int_channel_2.int_pin_cfg.output_en = SMI230_ENABLE;
		dev->int_channel_2.int_pin_cfg.input_en = SMI230_DISABLE;
		dev->int_channel_2.int_pin_cfg.output_mode = int_mode;
		dev->int_channel_2.int_pin_cfg.lvl = int_lvl;
		dev->int_channel_2.int_type = SMI230_ACCEL_NO_INT;
		dev->int_channel_2.int_features = 0x00;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO_WM)) {
		err = smi230_acc_set_fifo_wm(512, dev);
		if (err != SMI230_OK) {
			pr_err("SMI230 set fifo wm failed.");
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1))
		err |= smi230_acc_set_int_pin_config(dev, SMI230_INT_CHANNEL_1);
	else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2))
		err |= smi230_acc_set_int_pin_config(dev, SMI230_INT_CHANNEL_2);

	err |= smi230_acc_set_int_type_config(dev);

	if (err != SMI230_OK) {
		pr_err("SMI230 set int config failed.");
		return err;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_DATA_SYNC)) {
		dev->sync_cfg.mode = SMI230_ACCEL_DATA_SYNC_MODE_2000HZ;
		err = smi230_configure_data_synchronization(dev->sync_cfg, dev);
		/*set accel interrupt pin configuration*/
		/*configure host data ready interrupt */
		dev->int_channel_1.int_pin_cfg.output_en = SMI230_DISABLE;
		dev->int_channel_1.int_pin_cfg.input_en = SMI230_ENABLE;
		dev->int_channel_1.int_pin_cfg.output_mode =
			SMI230_INT_MODE_PUSH_PULL;
		dev->int_channel_1.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;
		dev->int_channel_1.int_type = SMI230_ACCEL_SYNC_INPUT;
		dev->int_channel_1.int_features = 0x00;

		err |= smi230_acc_set_int_pin_config(dev, SMI230_INT_CHANNEL_1);

		if (err != SMI230_OK) {
			pr_err("SMI230 config datasync failed.");
			return err;
		}
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO)) {
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

	dev->delay_ms(200);

	return err;
}

static int in_accel_raw_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);
	struct smi230_sensor_data data = { 0 };

	ret = smi230_acc_get_data(&data, smi230_dev);
	if (ret != SMI230_OK)
		return 0;
	return snprintf(buf, PAGE_SIZE, "%d %hd %hd %hd\n", data.sensor_time,
			data.x, data.y, data.z);
}

static int in_accel_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret, range;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_meas_conf(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Error reading meas config.");
		return ret;
	}

	switch (smi230_dev->accel_cfg.range) {
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

static int in_accel_bw_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_meas_conf(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Error reading meas config.");
		return ret;
	}

	switch (smi230_dev->accel_cfg.bw) {
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

static int power_mode_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_power_mode(smi230_dev);
	if (ret) {
		dev_err(dev, "Failed to read power config.");
		return ret;
	}

	switch (smi230_dev->accel_cfg.power) {
	case SMI230_ACCEL_PM_ACTIVE:
		return sprintf(buf, "normal\n");
	case SMI230_ACCEL_PM_SUSPEND:
		return sprintf(buf, "suspend\n");
	default:
		return sprintf(buf, "error\n");
	}
}

static int self_test_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int8_t ret;
	int8_t rslt;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	rslt = smi230_acc_self_test(smi230_dev);

	ret = smi230_init(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Sensor initialization failed.");
		return -EIO;
	}

	if (rslt == SMI230_OK)
		return snprintf(buf, PAGE_SIZE,
				"success. soft reset performed.\n");
	else
		return snprintf(buf, PAGE_SIZE,
				"failure. soft reset performed.\n");
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	uint8_t data = 0;
	int err = 0;
	int i;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	for (i = 0; i <= 0x7e; i++) {
		err = smi230_acc_get_regs(i, &data, 1, smi230_dev);
		if (err) {
			pr_err("falied");
			return err;
		}
		pr_info("0x%x = 0x%x", i, data);
		if (i % 15 == 0)
			pr_info("\n");
	}

	return 0;
}

static int in_accel_range_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int ret, range;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = kstrtoint(buf, 10, &range);
	if (ret) {
		dev_err(dev, "Invalid argument for range.");
		return ret;
	}

	switch (range) {
	case 2:
		smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_2G;
		break;
	case 4:
		smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_4G;
		break;
	case 8:
		smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_8G;
		break;
	case 16:
		smi230_dev->accel_cfg.range = SMI230_ACCEL_RANGE_16G;
		break;
	default:
		dev_err(dev, "Invalid argument for range.");
		return -EINVAL;
	}

	ret = smi230_acc_set_meas_conf(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Failed to set accel range.");
		return ret;
	}
	return count;
}

static int in_accel_bw_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	if (strncmp(buf, "normal", 6) == 0)
		smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_NORMAL;
	else if (strncmp(buf, "osr2", 4) == 0)
		smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR2;
	else if (strncmp(buf, "osr4", 4) == 0)
		smi230_dev->accel_cfg.bw = SMI230_ACCEL_BW_OSR4;
	else {
		dev_err(dev, "Invalid argument for bw.");
		return -EINVAL;
	}

	ret = smi230_acc_set_meas_conf(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Failed to set accel bw.");
		return ret;
	}
	return count;
}

static int power_mode_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	if (strncmp(buf, "normal", 6) == 0) {
		smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_ACTIVE;
	} else if (strncmp(buf, "suspend", 7) == 0) {
		smi230_dev->accel_cfg.power = SMI230_ACCEL_PM_SUSPEND;
	} else {
		dev_err(dev, "Invalid argument for power mode.");
		return -EINVAL;
	}

	ret = smi230_acc_set_power_mode(smi230_dev);
	if (ret)
		dev_err(dev, "Failed to set power mode.");

	return count;
}

static int soft_reset_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_init(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(indio_dev->dev.parent, "sensor initialization failed.");
		return snprintf(buf, PAGE_SIZE,
				"soft reset failed with error code %d\n", ret);
	}

	return snprintf(buf, PAGE_SIZE, "soft reset successful\n");
}

static int in_temp_accel_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int ret;
	int32_t temp;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_sensor_temperature(smi230_dev, &temp);
	if (ret != SMI230_OK)
		return snprintf(buf, PAGE_SIZE, "error\n");

	return snprintf(buf, PAGE_SIZE, "%d\n", temp);
}

#ifdef CONFIG_SMI230_ACC_DATA_SYNC
static IIO_CONST_ATTR(sampling_frequency_available, "100 200 400 1000 2000");
#else
static IIO_CONST_ATTR(in_accel_sampling_frequency_available,
		      "12.5 25 50 100 200 400 800 1600");
#endif

static IIO_CONST_ATTR(
	accel_scale_available,
	"2g: 1/16384 4g: 1/8192 8g: 1/4096 16g: 1/2048 (range: scale)");
static IIO_DEVICE_ATTR_RO(in_accel_raw, 0);
static IIO_DEVICE_ATTR_RW(in_accel_range, 0);
static IIO_DEVICE_ATTR_RW(in_accel_bw, 0);
static IIO_DEVICE_ATTR_RW(power_mode, 0);
static IIO_DEVICE_ATTR_RO(soft_reset, 0);
static IIO_DEVICE_ATTR_RO(self_test, 0);
static IIO_DEVICE_ATTR_RO(reg_dump, 0);
static IIO_DEVICE_ATTR_RO(in_temp_accel, 0);

static struct attribute *smi230_attrs[] = {
	&iio_dev_attr_in_accel_raw.dev_attr.attr,
#ifdef CONFIG_SMI230_ACC_DATA_SYNC
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
#else
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
#endif
	&iio_const_attr_accel_scale_available.dev_attr.attr,
	&iio_dev_attr_in_accel_range.dev_attr.attr,
	&iio_dev_attr_in_accel_bw.dev_attr.attr,
	&iio_dev_attr_power_mode.dev_attr.attr,
	&iio_dev_attr_soft_reset.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_reg_dump.dev_attr.attr,
	&iio_dev_attr_in_temp_accel.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi230_attrs_group = {
	.attrs = smi230_attrs,
};

static int smi230_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	int ret;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				ret = dev->anymotion_cfg.x_en;
				break;
			case IIO_MOD_Y:
				ret = dev->anymotion_cfg.y_en;
				break;
			case IIO_MOD_Z:
				ret = dev->anymotion_cfg.z_en;
				break;
			}
		} else if (dir == IIO_EV_DIR_FALLING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				ret = dev->nomotion_cfg.x_en;
				break;
			case IIO_MOD_Y:
				ret = dev->nomotion_cfg.y_en;
				break;
			case IIO_MOD_Z:
				ret = dev->nomotion_cfg.z_en;
				break;
			}
		}
		break;
	case IIO_EV_TYPE_THRESH_ADAPTIVE:
		switch (chan->channel2) {
		case IIO_MOD_X:
			ret = dev->high_g_cfg.x_en;
			break;
		case IIO_MOD_Y:
			ret = dev->high_g_cfg.y_en;
			break;
		case IIO_MOD_Z:
			ret = dev->high_g_cfg.z_en;
			break;
		}
		break;
	case IIO_EV_TYPE_THRESH:
		ret = dev->low_g_cfg.enable;
		break;
	case IIO_EV_TYPE_MAG:
		ret = dev->orient_cfg.enable;
		break;
	case IIO_EV_TYPE_CHANGE:
		ret = dev->orient_cfg.ud_en;
		break;
	default:
		break;
	}

	return ret;
}

static int smi230_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir, int state)
{
	int ret;
	uint8_t data;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1))
		data = dev->int_channel_1.int_features;
	else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2))
		data = dev->int_channel_2.int_features;

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				dev->anymotion_cfg.x_en = state;
				break;
			case IIO_MOD_Y:
				dev->anymotion_cfg.y_en = state;
				break;
			case IIO_MOD_Z:
				dev->anymotion_cfg.z_en = state;
				break;
			}

			dev->anymotion_cfg.enable = dev->anymotion_cfg.x_en |
						    dev->anymotion_cfg.y_en |
						    dev->anymotion_cfg.z_en;
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_ANY_MOT,
					       dev->anymotion_cfg.enable);
			ret = smi230_set_anymotion_config(dev);
		} else if (dir == IIO_EV_DIR_FALLING) {
			switch (chan->channel2) {
			case IIO_MOD_X:
				dev->nomotion_cfg.x_en = state;
				break;
			case IIO_MOD_Y:
				dev->nomotion_cfg.y_en = state;
				break;
			case IIO_MOD_Z:
				dev->nomotion_cfg.z_en = state;
				break;
			}

			dev->nomotion_cfg.enable = dev->nomotion_cfg.x_en |
						   dev->nomotion_cfg.y_en |
						   dev->nomotion_cfg.z_en;
			data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_NO_MOT,
					       dev->nomotion_cfg.enable);
			ret = smi230_set_nomotion_config(dev);
		}
		break;
	case IIO_EV_TYPE_THRESH_ADAPTIVE:
		switch (chan->channel2) {
		case IIO_MOD_X:
			dev->high_g_cfg.x_en = state;
			break;
		case IIO_MOD_Y:
			dev->high_g_cfg.y_en = state;
			break;
		case IIO_MOD_Z:
			dev->high_g_cfg.z_en = state;
			break;
		}

		dev->high_g_cfg.enable = dev->high_g_cfg.x_en |
					 dev->high_g_cfg.y_en |
					 dev->high_g_cfg.z_en;
		data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_HIGH_G,
				       dev->high_g_cfg.enable);
		ret = smi230_set_high_g_config(dev);
		break;
	case IIO_EV_TYPE_THRESH:
		dev->low_g_cfg.enable = state;
		data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_LOW_G, state);
		ret = smi230_set_low_g_config(dev);
		break;
	case IIO_EV_TYPE_MAG:
		dev->orient_cfg.enable = state;
		data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_ORIENT,
				       (dev->orient_cfg.enable |
					dev->orient_cfg.ud_en));
		ret = smi230_set_orientation_config(dev);
		break;
	case IIO_EV_TYPE_CHANGE:
		dev->orient_cfg.ud_en = state;
		data = SMI230_SET_BITS(data, SMI230_ACCEL_INT_ORIENT,
				       (dev->orient_cfg.enable |
					dev->orient_cfg.ud_en));
		ret = smi230_set_orientation_config(dev);
		break;
	default:
		break;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1)) {
		dev->int_channel_1.int_features = data;
		ret |= smi230_acc_set_int_feature_config(dev,
							 SMI230_INT_CHANNEL_1);
	} else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2)) {
		dev->int_channel_2.int_features = data;
		ret |= smi230_acc_set_int_feature_config(dev,
							 SMI230_INT_CHANNEL_2);
	}

	if (ret != SMI230_OK)
		ret = -EIO;
	return ret;
}

static int smi230_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			if (info == IIO_EV_INFO_PERIOD)
				*val = dev->anymotion_cfg.duration;
			else if (info == IIO_EV_INFO_VALUE)
				*val = dev->anymotion_cfg.threshold;
		} else if (dir == IIO_EV_DIR_FALLING) {
			if (info == IIO_EV_INFO_PERIOD)
				*val = dev->nomotion_cfg.duration;
			else if (info == IIO_EV_INFO_VALUE)
				*val = dev->nomotion_cfg.threshold;
		}
		break;
	case IIO_EV_TYPE_THRESH_ADAPTIVE:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			*val = dev->high_g_cfg.duration;
			break;
		case IIO_EV_INFO_VALUE:
			*val = dev->high_g_cfg.threshold;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			*val = dev->high_g_cfg.hysteresis;
			break;
		default:
			break;
		}
		break;
	case IIO_EV_TYPE_THRESH:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			*val = dev->low_g_cfg.duration;
			break;
		case IIO_EV_INFO_VALUE:
			*val = dev->low_g_cfg.threshold;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			*val = dev->low_g_cfg.hysteresis;
			break;
		default:
			break;
		}
		break;
	case IIO_EV_TYPE_MAG:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			*val = dev->orient_cfg.mode;
			break;
		case IIO_EV_INFO_VALUE:
			*val = dev->orient_cfg.theta;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			*val = dev->orient_cfg.hysteresis;
			break;
		case IIO_EV_INFO_TIMEOUT:
			*val = dev->orient_cfg.blocking;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return IIO_VAL_INT;
}

static int smi230_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	int ret;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	switch (type) {
	case IIO_EV_TYPE_ROC:
		if (dir == IIO_EV_DIR_RISING) {
			if (info == IIO_EV_INFO_PERIOD)
				dev->anymotion_cfg.duration = val;
			else if (info == IIO_EV_INFO_VALUE)
				dev->anymotion_cfg.threshold = val;

			ret = smi230_set_anymotion_config(dev);
		} else if (dir == IIO_EV_DIR_FALLING) {
			if (info == IIO_EV_INFO_PERIOD)
				dev->nomotion_cfg.duration = val;
			else if (info == IIO_EV_INFO_VALUE)
				dev->nomotion_cfg.threshold = val;

			ret = smi230_set_nomotion_config(dev);
		}
		break;
	case IIO_EV_TYPE_THRESH_ADAPTIVE:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			dev->high_g_cfg.duration = val;
			break;
		case IIO_EV_INFO_VALUE:
			dev->high_g_cfg.threshold = val;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			dev->high_g_cfg.hysteresis = val;
			break;
		default:
			break;
		}

		ret = smi230_set_high_g_config(dev);
		break;
	case IIO_EV_TYPE_THRESH:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			dev->low_g_cfg.duration = val;
			break;
		case IIO_EV_INFO_VALUE:
			dev->low_g_cfg.threshold = val;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			dev->low_g_cfg.hysteresis = val;
			break;
		default:
			break;
		}

		ret = smi230_set_low_g_config(dev);
		break;
	case IIO_EV_TYPE_MAG:
		switch (info) {
		case IIO_EV_INFO_PERIOD:
			dev->orient_cfg.mode = val;
			break;
		case IIO_EV_INFO_VALUE:
			dev->orient_cfg.theta = val;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			dev->orient_cfg.hysteresis = val;
			break;
		case IIO_EV_INFO_TIMEOUT:
			dev->orient_cfg.blocking = val;
			break;
		default:
			break;
		}

		ret = smi230_set_orientation_config(dev);
		break;
	default:
		break;
	}

	if (ret != SMI230_OK)
		ret = -EIO;
	return ret;
}

static const struct iio_info smi230_info = {
	.read_event_config = smi230_read_event_config,
	.read_event_value = smi230_read_event_value,
	.write_event_config = smi230_write_event_config,
	.write_event_value = smi230_write_event_value,
	.read_raw = smi230_read_raw,
	.write_raw = smi230_write_raw,
	.attrs = &smi230_attrs_group,
};

static int8_t smi230_acc_high_g_handler(struct iio_dev *indio_dev,
					int64_t timestamp)
{
	int8_t ret;
	uint8_t data;
	uint8_t dir;
	uint8_t mod;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_regs(SMI230_REG_ORIENT_HIGHG_OUT_REG, &data, 1,
				  dev);
	if (ret != SMI230_OK)
		return ret;

	if (data & SMI230_HIGH_G_AXIS_DIRECTION_MASK)
		dir = IIO_EV_DIR_FALLING;
	else
		dir = IIO_EV_DIR_RISING;

	switch ((data >> 3) & 0x07) {
	case 1:
		mod = IIO_MOD_X;
		break;
	case 2:
		mod = IIO_MOD_Y;
		break;
	case 3:
		mod = IIO_MOD_X_AND_Y;
		break;
	case 4:
		mod = IIO_MOD_Z;
		break;
	case 5:
		mod = IIO_MOD_X_AND_Z;
		break;
	case 6:
		mod = IIO_MOD_Y_AND_Z;
		break;
	case 7:
		mod = IIO_MOD_X_AND_Y_AND_Z;
		break;
	default:
		return ret;
	}

	iio_push_event(indio_dev,
		       IIO_MOD_EVENT_CODE(IIO_ACCEL, SMI230_ACC_X, mod,
					  IIO_EV_TYPE_THRESH_ADAPTIVE, dir),
		       timestamp);

	return ret;
}

static int8_t smi230_acc_orientation_handler(struct iio_dev *indio_dev,
					     int64_t timestamp)
{
	int8_t ret;
	uint8_t data;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_regs(SMI230_REG_ORIENT_HIGHG_OUT_REG, &data, 1,
				  dev);
	if (ret != SMI230_OK)
		return ret;

	if (dev->orient_cfg.ud_en) {
		if (data & SMI230_ORIENT_FACEUP_DOWN_MASK)
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
							    IIO_EV_TYPE_CHANGE,
							    IIO_EV_DIR_FALLING),
				       timestamp);
		else
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
							    IIO_EV_TYPE_CHANGE,
							    IIO_EV_DIR_RISING),
				       timestamp);
	}

	switch (data & SMI230_ORIENT_PORTRAIT_LANDSCAPE_MASK) {
	case SMI230_ORIENT_PORTRAIT_UPRIGHT:
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_RISING),
			       timestamp);
		break;
	case SMI230_ORIENT_PORTRAIT_UPSIDE_DOWN:
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_FALLING),
			       timestamp);
		break;
	case SMI230_ORIENT_LANDSCAPE_LEFT:
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_EITHER),
			       timestamp);
		break;
	case SMI230_ORIENT_LANDSCAPE_RIGHT:
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_ACCEL, 0,
						    IIO_EV_TYPE_MAG,
						    IIO_EV_DIR_NONE),
			       timestamp);
		break;
	}

	return ret;
}

static irqreturn_t smi230_acc_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;

	iio_trigger_poll(indio_dev->trig);
	return IRQ_WAKE_THREAD;
}

static irqreturn_t smi230_acc_irq_thread_handler(int irq, void *private)
{
	int ret;
	uint8_t int_stat;
	int64_t timestamp;
	struct iio_dev *indio_dev = private;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	timestamp = iio_get_time_ns(indio_dev);

	ret = smi230_acc_get_regs(SMI230_ACCEL_INT_STAT_0_REG, &int_stat, 1,
				  dev);
	if (ret != SMI230_OK) {
		dev_err(indio_dev->dev.parent,
			"Failed to read interrupt status register");
		return IRQ_NONE;
	}

	if (int_stat & SMI230_ACCEL_ANY_MOT_INT_ENABLE)
		iio_push_event(
			indio_dev,
			IIO_MOD_EVENT_CODE(IIO_ACCEL, 0, IIO_MOD_X_OR_Y_OR_Z,
					   IIO_EV_TYPE_ROC, IIO_EV_DIR_RISING),
			timestamp);

	if (int_stat & SMI230_ACCEL_NO_MOT_INT_ENABLE)
		iio_push_event(
			indio_dev,
			IIO_MOD_EVENT_CODE(IIO_ACCEL, 0, IIO_MOD_X_AND_Y_AND_Z,
					   IIO_EV_TYPE_ROC, IIO_EV_DIR_FALLING),
			timestamp);

	if (int_stat & SMI230_ACCEL_HIGH_G_INT_ENABLE) {
		ret = smi230_acc_high_g_handler(indio_dev, timestamp);
		if (ret != SMI230_OK)
			dev_err(indio_dev->dev.parent,
				"Failed to read high g out register");
	}

	if (int_stat & SMI230_ACCEL_LOW_G_INT_ENABLE)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(
				       IIO_ACCEL, 0, IIO_MOD_SUM_SQUARED_X_Y_Z,
				       IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING),
			       timestamp);

	if (int_stat & SMI230_ACCEL_ORIENT_INT_ENABLE) {
		ret = smi230_acc_orientation_handler(indio_dev, timestamp);
		if (ret != SMI230_OK)
			dev_err(indio_dev->dev.parent,
				"Failed to read orient out register");
	}

	return IRQ_HANDLED;
}

static irqreturn_t smi230_acc_trigger_handler(int irq, void *p)
{
	int ret;
	uint8_t int_stat[2];
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_acc_dev *dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_regs(SMI230_ACCEL_INT_STAT_0_REG, int_stat, 2,
				  dev);
	if (ret != SMI230_OK) {
		dev_err(indio_dev->dev.parent,
			"Failed to read interrupt status register");
		return IRQ_NONE;
	}

	if (int_stat[1] & (SMI230_ACCEL_FIFO_WM_INT_ENABLE |
			   SMI230_ACCEL_FIFO_FULL_INT_ENABLE)) {
		ret = smi230_fifo_flush(indio_dev, pf->timestamp,
					SMI230_MAX_ACC_FIFO_FRAME);
		if (ret < 0)
			dev_err(indio_dev->dev.parent,
				"Failed to read FIFO data");
	} else if (int_stat[1] & SMI230_ACCEL_DATA_RDY_INT_ENABLE) {
		ret = smi230_data_ready_handler(indio_dev, pf->timestamp);
		if (ret != SMI230_OK)
			dev_err(indio_dev->dev.parent,
				"Failed to read new data");
	} else if (int_stat[0] & SMI230_ACCEL_DATA_SYNC_INT_ENABLE)
		smi230_data_sync_handle(indio_dev, pf->timestamp);
	//odr workaround
	else if (!int_stat[0] && !int_stat[1] && !(++dsync % 4))
		smi230_data_sync_handle(indio_dev, pf->timestamp);

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int smi230_get_irq(struct device *dev, int *irq)
{
	int gpio_pin, ret;

	gpio_pin = of_get_named_gpio_flags(dev->of_node, "gpio_irq", 0, NULL);

	ret = gpio_request_one(gpio_pin, GPIOF_IN, "smi230_acc_interrupt");
	if (ret) {
		dev_err(dev, "Request GPIO pin %d failed", gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(gpio_pin);
	if (ret) {
		dev_err(dev, "Set direction for GPIO pin %d failed", gpio_pin);
		gpio_free(gpio_pin);
		return ret;
	}

	*irq = gpio_to_irq(gpio_pin);

	return ret;
}

static ssize_t hwfifo_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t wm;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	ret = smi230_acc_get_fifo_wm(&wm, smi230_dev);

	if (ret == SMI230_OK)
		return sprintf(buf, "%d\n", wm);
	else
		return sprintf(buf, "0\n");
}

static ssize_t hwfifo_reset_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int err, val;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230_acc_dev *smi230_dev = iio_device_get_drvdata(indio_dev);

	err = kstrtoint(buf, 10, &val);
	if (err) {
		dev_err(dev, "Invalid argument");
		return err;
	}

	if (val) {
		err = smi230_acc_fifo_reset(smi230_dev);
		if (err != SMI230_OK) {
			dev_err(dev, "Failed to reset fifo.");
			return err;
		}
	}
	return count;
}

static IIO_CONST_ATTR(hwfifo_watermark_min, "1");
static IIO_CONST_ATTR(hwfifo_watermark_max,
		      __stringify(SMI230_MAX_ACC_FIFO_BYTES));
static IIO_DEVICE_ATTR_RO(hwfifo_enabled, 0);
static IIO_DEVICE_ATTR_RW(hwfifo_watermark, 0);
static IIO_DEVICE_ATTR_WO(hwfifo_reset, 0);

static const struct attribute *smi230_fifo_attributes[] = {
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	&iio_dev_attr_hwfifo_reset.dev_attr.attr,
	NULL,
};

static int smi230_acc_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	int ret;
	enum smi230_accel_int_types int_type;
	struct smi230_acc_dev *dev = iio_trigger_get_drvdata(trig);

	if (enable) {
		if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO_WM))
			int_type = SMI230_ACCEL_FIFO_WM_INT;
		else if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO_FULL))
			int_type = SMI230_ACCEL_FIFO_FULL_INT;
		else if (IS_ENABLED(CONFIG_SMI230_ACC_DATA_SYNC))
			int_type = SMI230_ACCEL_DATA_SYNC_INT;
		else
			int_type = SMI230_ACCEL_DATA_RDY_INT;

		if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO)) {
			ret = smi230_acc_fifo_reset(dev);
			if (ret) {
				dev_err(trig->dev.parent,
					"Failed to reset fifo.");
				return ret;
			}
		}
	} else {
		int_type = SMI230_ACCEL_NO_INT;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1))
		dev->int_channel_1.int_type = int_type;
	else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2))
		dev->int_channel_2.int_type = int_type;

	ret = smi230_acc_set_int_type_config(dev);
	if (ret)
		dev_err(trig->dev.parent,
			"Failed to set interrupt config, error %d.", ret);

	return ret;
}

static const struct iio_trigger_ops smi230_trigger_ops = {
	.set_trigger_state = &smi230_acc_set_trigger_state,
};

int smi230_acc_probe(struct device *dev, struct smi230_acc_dev *smi230_dev)
{
	int ret = 0;
	int irq;
	unsigned long irqf;
	struct iio_dev *indio_dev;
	const struct attribute **fifo_attrs;

	smi230_dev->config_file_ptr = smi230_config_file;

	ret = smi230_init(smi230_dev);
	if (ret != SMI230_OK) {
		dev_err(dev, "Bosch Sensor %s initialization failed %d",
			SENSOR_ACC_NAME, ret);
		return ret;
	}

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, smi230_dev);
	dev_set_drvdata(dev, indio_dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = smi230_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi230_channels);
	indio_dev->name = MODULE_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->info = &smi230_info;
	indio_dev->trig =
		devm_iio_trigger_alloc(dev, "%s-trigger", indio_dev->name);

	if (indio_dev->trig == NULL)
		return -ENOMEM;

	ret = smi230_get_irq(dev, &irq);
	if (ret) {
		dev_err(dev, "Failed to request GPIO pin");
		return ret;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_ACTIVE_LOW))
		irqf = IRQF_TRIGGER_FALLING;
	else if (IS_ENABLED(CONFIG_SMI230_ACC_ACTIVE_HIGH))
		irqf = IRQF_TRIGGER_RISING;

	ret = devm_request_threaded_irq(dev, irq, smi230_acc_irq_handler,
					smi230_acc_irq_thread_handler, irqf,
					indio_dev->name, indio_dev);

	if (ret) {
		smi230_acc_remove(dev);
		dev_err(dev, "Failed to request irq for pin %d", irq);
		return ret;
	}

	indio_dev->trig->ops = &smi230_trigger_ops;
	iio_trigger_set_drvdata(indio_dev->trig, smi230_dev);

	ret = devm_iio_trigger_register(dev, indio_dev->trig);
	if (ret) {
		dev_err(dev, "Failed to register trigger");
		goto exit_failure;
	}

	if (IS_ENABLED(CONFIG_SMI230_ACC_FIFO))
		fifo_attrs = smi230_fifo_attributes;
	else
		fifo_attrs = NULL;

	ret = devm_iio_triggered_buffer_setup_ext(dev, indio_dev,
						  &iio_pollfunc_store_time,
						  smi230_acc_trigger_handler,
						  NULL, fifo_attrs);
	if (ret) {
		dev_err(dev, "Setup triggered buffer failed");
		goto exit_failure;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Register IIO device failed");
		goto exit_failure;
	}

	return ret;

exit_failure:
	smi230_acc_remove(dev);
	return ret;
}

int smi230_acc_remove(struct device *dev)
{
	int gpio_pin;

	gpio_pin = of_get_named_gpio_flags(dev->of_node, "gpio_irq", 0, NULL);
	dev_info(dev, "free gpio pin %d", gpio_pin);
	gpio_free(gpio_pin);
	return 0;
}
