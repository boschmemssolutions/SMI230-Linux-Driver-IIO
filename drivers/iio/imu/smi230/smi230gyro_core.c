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

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/version.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "smi230.h"

#define SMI230GYRO_DATA_CHANNEL(_type, _axis, _index)                      \
	{                                                                  \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),              \
		.scan_index = _index,                                      \
		.scan_type = {                                             \
			.sign = 's',                                       \
			.realbits = 16,                                    \
			.storagebits = 16,                                 \
			.endianness = IIO_LE,                              \
		},                                                         \
	}

static const struct iio_chan_spec smi230gyro_channels[] = {
	SMI230GYRO_DATA_CHANNEL(IIO_ANGL_VEL, X, SMI230_GYRO_X),
	SMI230GYRO_DATA_CHANNEL(IIO_ANGL_VEL, Y, SMI230_GYRO_Y),
	SMI230GYRO_DATA_CHANNEL(IIO_ANGL_VEL, Z, SMI230_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(SMI230_SCAN_TIMESTAMP),
};

static struct smi230gyro_data *drv_data;

int smi230gyro_read_registers(unsigned int reg, void *val, size_t val_count)
{
	int ret;

	ret = regmap_bulk_read(drv_data->regmap, reg, val, val_count);
	if (ret)
		return ret;
	return 0;
}

int smi230gyro_write_registers(unsigned int reg, const void *val,
			       size_t val_count)
{
	int ret;
	ret = regmap_bulk_write(drv_data->regmap, reg, val, val_count);
	if (ret)
		return ret;
	return 0;
}

static int smi230_gyro_get_data(const struct smi230gyro_data *data,
				struct smi230_sensor_data *gyro)
{
	int ret;
	u8 val[6];

	ret = regmap_bulk_read(data->regmap, SMI230_GYRO_X_LSB_REG, val, 6);
	if (ret)
		return ret;

	gyro->x = (s16)(val[1] << 8) | (val[0]); /* Data in X axis */

	gyro->y = (s16)(val[3] << 8) | (val[2]); /* Data in Y axis */

	gyro->z = (s16)(val[5] << 8) | (val[4]); /* Data in Z axis */

	return 0;
}

static int smi230gyro_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int *val,
			       int *val2, long mask)
{
	int ret;
	struct smi230gyro_data *data = iio_priv(indio_dev);
	struct smi230_sensor_data gyro_val;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ANGL_VEL) {
			ret = smi230_gyro_get_data(data, &gyro_val);
			if (ret)
				return ret;
			switch (chan->channel2) {
			case IIO_MOD_X:
				*val = gyro_val.x;
				break;
			case IIO_MOD_Y:
				*val = gyro_val.y;
				break;
			case IIO_MOD_Z:
				*val = gyro_val.z;
				break;
			}
		}
		return IIO_VAL_INT;
	}
	return 0;
}

static int smi230gyro_set_bw(struct smi230gyro_data *data, u8 bw)
{
	int ret;
	data->cfg.bw_odr.fields.bw = bw;
	ret = regmap_write(data->regmap, SMI230_GYRO_BANDWIDTH_REG,
			   data->cfg.bw_odr.value);
	if (ret)
		return ret;
	return 0;
}

#ifndef CONFIG_SMI230_DATA_SYNC
static ssize_t bw_odr_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_GYRO_BANDWIDTH_REG,
			  &(data->cfg.bw_odr.value));
	if (ret)
		return ret;

	switch (data->cfg.bw_odr.fields.bw) {
	case SMI230_GYRO_BW_523_ODR_2000_HZ:
		return sprintf(buf, "bw523_odr2000\n");
	case SMI230_GYRO_BW_230_ODR_2000_HZ:
		return sprintf(buf, "bw230_odr2000\n");
	case SMI230_GYRO_BW_116_ODR_1000_HZ:
		return sprintf(buf, "bw116_odr1000\n");
	case SMI230_GYRO_BW_47_ODR_400_HZ:
		return sprintf(buf, "bw47_odr400\n");
	case SMI230_GYRO_BW_23_ODR_200_HZ:
		return sprintf(buf, "bw23_odr200\n");
	case SMI230_GYRO_BW_12_ODR_100_HZ:
		return sprintf(buf, "bw12_odr100\n");
	case SMI230_GYRO_BW_64_ODR_200_HZ:
		return sprintf(buf, "bw64_odr200\n");
	case SMI230_GYRO_BW_32_ODR_100_HZ:
		return sprintf(buf, "bw32_odr100\n");
	default:
		return sprintf(buf, "error\n");
	}

	return 0;
}

static ssize_t bw_odr_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "bw523_odr2000", 13) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_523_ODR_2000_HZ;
	else if (strncmp(buf, "bw230_odr2000", 13) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_230_ODR_2000_HZ;
	else if (strncmp(buf, "bw116_odr1000", 13) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_116_ODR_1000_HZ;
	else if (strncmp(buf, "bw47_odr400", 11) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_47_ODR_400_HZ;
	else if (strncmp(buf, "bw23_odr200", 11) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_23_ODR_200_HZ;
	else if (strncmp(buf, "bw12_odr100", 11) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_12_ODR_100_HZ;
	else if (strncmp(buf, "bw64_odr200", 11) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_64_ODR_200_HZ;
	else if (strncmp(buf, "bw32_odr100", 11) == 0)
		data->cfg.bw_odr.fields.bw = SMI230_GYRO_BW_32_ODR_100_HZ;
	else
		return -EINVAL;

	ret = smi230gyro_set_bw(data, data->cfg.bw_odr.fields.bw);
	if (ret)
		return ret;

	return count;
}
#endif

static ssize_t pwr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_GYRO_LPM1_REG,
			  &(data->cfg.pwr_ctrl.value));
	if (ret)
		return ret;

	switch (data->cfg.pwr_ctrl.fields.pwr) {
	case SMI230_GYRO_PM_NORMAL:
		return sprintf(buf, "normal\n");
	case SMI230_GYRO_PM_SUSPEND:
		return sprintf(buf, "suspend\n");
	case SMI230_GYRO_PM_DEEP_SUSPEND:
		return sprintf(buf, "deep_suspend\n");
	default:
		return sprintf(buf, "error\n");
	}

	return 0;
}

static int smi230gyro_set_power(struct smi230gyro_data *data, u8 pwr)
{
	int ret;
	data->cfg.pwr_ctrl.fields.pwr = pwr;
	ret = regmap_write(data->regmap, SMI230_GYRO_LPM1_REG,
			   data->cfg.pwr_ctrl.value);
	if (ret)
		return ret;

	return 0;
}

static ssize_t pwr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "normal", 6) == 0)
		data->cfg.pwr_ctrl.fields.pwr = SMI230_GYRO_PM_NORMAL;
	else if (strncmp(buf, "suspend", 7) == 0)
		data->cfg.pwr_ctrl.fields.pwr = SMI230_GYRO_PM_SUSPEND;
	else if (strncmp(buf, "deep_suspend", 12) == 0)
		data->cfg.pwr_ctrl.fields.pwr = SMI230_GYRO_PM_DEEP_SUSPEND;
	else
		return -EINVAL;

	ret = smi230gyro_set_power(data, data->cfg.pwr_ctrl.fields.pwr);
	if (ret)
		return ret;
	return count;
}

static ssize_t range_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_GYRO_RANGE_REG,
			  &(data->cfg.range.value));
	if (ret)
		return 0;

	switch (data->cfg.range.fields.range) {
	case SMI230_GYRO_RANGE_2000_DPS:
		return sprintf(buf, "2000DPS\n");
	case SMI230_GYRO_RANGE_1000_DPS:
		return sprintf(buf, "1000DPS\n");
	case SMI230_GYRO_RANGE_500_DPS:
		return sprintf(buf, "500DPS\n");
	case SMI230_GYRO_RANGE_250_DPS:
		return sprintf(buf, "250DPS\n");
	case SMI230_GYRO_RANGE_125_DPS:
		return sprintf(buf, "125DPS\n");
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230gyro_set_range(struct smi230gyro_data *data, u8 range)
{
	int ret;
	data->cfg.range.fields.range = range;
	ret = regmap_write(data->regmap, SMI230_GYRO_RANGE_REG,
			   data->cfg.range.value);
	if (ret)
		return ret;
	return 0;
}

static ssize_t range_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "2000DPS", 7) == 0)
		data->cfg.range.fields.range = SMI230_GYRO_RANGE_2000_DPS;
	else if (strncmp(buf, "1000DPS", 7) == 0)
		data->cfg.range.fields.range = SMI230_GYRO_RANGE_1000_DPS;
	else if (strncmp(buf, "500DPS", 6) == 0)
		data->cfg.range.fields.range = SMI230_GYRO_RANGE_500_DPS;
	else if (strncmp(buf, "250DPS", 6) == 0)
		data->cfg.range.fields.range = SMI230_GYRO_RANGE_250_DPS;
	else if (strncmp(buf, "125DPS", 6) == 0)
		data->cfg.range.fields.range = SMI230_GYRO_RANGE_125_DPS;
	else
		return -EINVAL;

	ret = smi230gyro_set_range(data, data->cfg.range.fields.range);
	if (ret)
		return ret;

	msleep(SMI230_GYRO_POWER_MODE_CONFIG_DELAY);

	return count;
}

static ssize_t fifo_wm_frame_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret, fifo_wm_frame;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_GYRO_FIFO_CONFIG_0_ADDR,
			  &fifo_wm_frame);
	if (ret)
		return ret;

	data->cfg.fifo_wm_in_frame = (u16)fifo_wm_frame;
	return sprintf(buf, "%d frames\n", data->cfg.fifo_wm_in_frame);

	return 0;
}

static int smi230gyro_set_fifo_wm(struct smi230gyro_data *data, u16 fifo_wm)
{
	int ret, fifo_wm_in_frame;

	fifo_wm_in_frame = fifo_wm;
	ret = regmap_write(data->regmap, SMI230_GYRO_FIFO_CONFIG_0_ADDR,
			   fifo_wm_in_frame);
	if (ret)
		return ret;
	data->cfg.fifo_wm_in_frame = fifo_wm;

	return 0;
}

static ssize_t fifo_wm_frame_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);
	u16 fifo_wm;

	ret = kstrtou16(buf, 10, &fifo_wm);
	if (ret)
		return -EINVAL;

	if (fifo_wm > SMI230_GYRO_MAX_FIFO_FRAME)
		return -EINVAL;
	ret = smi230gyro_set_fifo_wm(data, fifo_wm);
	if (ret)
		return ret;

	return count;
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int ret, val = 0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	int i;

	for (i = 0; i <= 0x3F; i++) {
		ret = regmap_read(data->regmap, i, &val);
		if (ret)
			return ret;
		printk("0x%x = 0x%x", i, val);
		if (i % 15 == 0)
			printk("\n");
	}

	return sprintf(buf, "check system log\n");

	return 0;
}

static ssize_t self_test_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	int ret = 0, val;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_GYRO_LPM1_REG,
			  &(data->cfg.pwr_ctrl.value));
	if (ret)
		return ret;

	if (data->cfg.pwr_ctrl.value != SMI230_GYRO_PM_NORMAL)
		return snprintf(buf, PAGE_SIZE,
				"gyro disabled, enable it firstly\n");

	ret = regmap_write(data->regmap, SMI230_GYRO_SELF_TEST_REG, 0x01);
	if (ret)
		return ret;

	msleep(80);

	/* Read self-test register */
	ret = regmap_read(data->regmap, SMI230_GYRO_SELF_TEST_REG, &val);

	if ((val & 0x06) != SMI230_GYRO_SELF_TEST_SUCCESS)
		return snprintf(buf, PAGE_SIZE, "gyro self test failed\n");
	else
		return snprintf(buf, PAGE_SIZE, "gyro self test success\n");
}
#ifndef CONFIG_SMI230_DATA_SYNC
static IIO_DEVICE_ATTR_RW(bw_odr, 0);
#endif
static IIO_DEVICE_ATTR_RW(pwr, 0);
static IIO_DEVICE_ATTR_RW(range, 0);
static IIO_DEVICE_ATTR_RW(fifo_wm_frame, 0);
static IIO_DEVICE_ATTR_RO(reg_dump, 0);
static IIO_DEVICE_ATTR_RO(self_test, 0);

static struct attribute *smi230gyro_basic_attrs[] = {
#ifndef CONFIG_SMI230_DATA_SYNC
	&iio_dev_attr_bw_odr.dev_attr.attr,
#endif
	&iio_dev_attr_pwr.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	&iio_dev_attr_fifo_wm_frame.dev_attr.attr,
	&iio_dev_attr_reg_dump.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi230gyro_basic_attrs_group = {
	.name = "smi230gyro_basic",
	.attrs = smi230gyro_basic_attrs,
};

static const struct iio_info smi230gyro_info = {
	.read_raw = smi230gyro_read_raw,
	.attrs = &smi230gyro_basic_attrs_group,
};

static int smi230gyro_data_ready_handler(struct smi230gyro_data *data,
					 struct iio_dev *indio_dev)
{
	int ret, chan;
	int i = 0;
	s16 sample;
	s64 timestamp;
	struct smi230_sensor_data gyro_val;

	if (indio_dev->active_scan_mask == NULL)
		return 0;

	mutex_lock(&data->lock);
	timestamp = iio_get_time_ns(indio_dev);
	ret = smi230_gyro_get_data(data, &gyro_val);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	if (indio_dev->active_scan_mask != NULL) {
		for_each_set_bit(chan, indio_dev->active_scan_mask,
				 indio_dev->masklength) {
			switch (chan) {
			case SMI230_GYRO_X:
				sample = gyro_val.x;
				break;
			case SMI230_GYRO_Y:
				sample = gyro_val.y;
				break;
			case SMI230_GYRO_Z:
				sample = gyro_val.z;
				break;
			default:
				return -EINVAL;
			}
			data->buf[i++] = sample;
		}

		iio_push_to_buffers_with_timestamp(indio_dev, data->buf,
						   timestamp);
	}

	mutex_unlock(&data->lock);

	return 0;
}

static int smi230gyro_extract_sample(u8 *fifo_buf,
				     struct smi230_sensor_data *fifo_sample_buf,
				     u16 fifo_length_in_bytes)
{
	int i = 0, j = 0, sample_count;

	for (i = 0; i < fifo_length_in_bytes;) {
		fifo_sample_buf[j].x = (s16)(fifo_buf[i + 1] << 8) |
				       (fifo_buf[i]);
		fifo_sample_buf[j].y = (s16)(fifo_buf[i + 3] << 8) |
				       (fifo_buf[i + 2]);
		fifo_sample_buf[j].z = (s16)(fifo_buf[i + 5] << 8) |
				       (fifo_buf[i + 4]);
		j += 1;
		i += 6;
	}
	sample_count = j;
	return sample_count;
}

static int smi230gyro_get_fifo_length_in_frames(struct smi230gyro_data *data,
						u16 *fifo_length_in_frames)
{
	int ret, fifo_length;

	ret = regmap_read(data->regmap, SMI230_GYRO_FIFO_STATUS_ADDR,
			  &fifo_length);
	if (ret)
		return ret;

	*fifo_length_in_frames = (u16)(fifo_length & 0x7F);

	return 0;
}

static s64 smi230gyro_calc_sample_time_interval(struct smi230gyro_data *data,
						int sample_count)
{
	s64 interval = 0, peroid;
	if (data->last_timestamp != 0) {
		peroid = data->current_timestamp - data->last_timestamp;

	} else {
		switch (data->cfg.bw_odr.fields.bw) {
		case SMI230_GYRO_BW_32_ODR_100_HZ:
			peroid = 10000000;
			break;
		case SMI230_GYRO_BW_12_ODR_100_HZ:
			peroid = 10000000;
			break;
		case SMI230_GYRO_BW_64_ODR_200_HZ:
			peroid = 5000000;
			break;
		case SMI230_GYRO_BW_23_ODR_200_HZ:
			peroid = 5000000;
			break;
		case SMI230_GYRO_BW_47_ODR_400_HZ:
			peroid = 2500000;
			break;
		case SMI230_GYRO_BW_116_ODR_1000_HZ:
			peroid = 1000000;
			break;
		case SMI230_GYRO_BW_230_ODR_2000_HZ:
			peroid = 500000;
			break;
		case SMI230_GYRO_BW_523_ODR_2000_HZ:
			peroid = 500000;
			break;
		default:
			peroid = 0;
		}
	}
	interval = div_s64(peroid, sample_count);
	return interval;
}

static int smi230gyro_fifo_full_handler(struct smi230gyro_data *data,
					struct iio_dev *indio_dev)
{
	int ret, i, sample_count;
	u16 fifo_length_in_frames, fifo_length_in_bytes;
	s64 interval, timestamp;
	s16 sample;

	if (indio_dev->active_scan_mask == NULL)
		return 0;

	mutex_lock(&data->lock);
	ret = smi230gyro_get_fifo_length_in_frames(data,
						   &fifo_length_in_frames);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	fifo_length_in_bytes =
		fifo_length_in_frames * SMI230_GYRO_FIFO_FRAME_LENGTH;

	ret = regmap_bulk_read(data->regmap, SMI230_GYRO_FIFO_DATA_ADDR,
			       data->fifo_buf, fifo_length_in_bytes);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->current_timestamp = iio_get_time_ns(indio_dev);

	sample_count = smi230gyro_extract_sample(
		data->fifo_buf, data->fifo_sample_buf, fifo_length_in_bytes);

	interval = smi230gyro_calc_sample_time_interval(data, sample_count);

	if (!data->first_irq && (indio_dev->active_scan_mask != NULL)) {
		for (i = 0; i < sample_count; i++) {
			int j = 0;
			int chan = 0;

			for_each_set_bit(chan, indio_dev->active_scan_mask,
					 indio_dev->masklength) {
				switch (chan) {
				case SMI230_GYRO_X:
					sample = data->fifo_sample_buf[i].x;
					break;
				case SMI230_GYRO_Y:
					sample = data->fifo_sample_buf[i].y;
					break;
				case SMI230_GYRO_Z:
					sample = data->fifo_sample_buf[i].z;
					break;
				default:
					mutex_unlock(&data->lock);
					return -EINVAL;
				}
				data->buf[j++] = sample;
			}

			timestamp = data->current_timestamp -
				    (interval * (sample_count - i - 1));

			iio_push_to_buffers_with_timestamp(indio_dev, data->buf,
							   timestamp);
		}
	}

	data->first_irq = false;
	data->last_timestamp = data->current_timestamp;

	mutex_unlock(&data->lock);
	return 0;
}

static int smi230gyro_fifo_wm_handler(struct smi230gyro_data *data,
				      struct iio_dev *indio_dev)
{
	int ret, i, sample_count;
	u16 fifo_length_in_frames, fifo_length_in_bytes;
	s64 interval, timestamp;
	s16 sample;
	bool repeat = false;

	if (indio_dev->active_scan_mask == NULL)
		return 0;

	mutex_lock(&data->lock);

	ret = smi230gyro_get_fifo_length_in_frames(data,
						   &fifo_length_in_frames);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	while (fifo_length_in_frames * 2 >= data->cfg.fifo_wm_in_frame) {
		fifo_length_in_bytes = data->cfg.fifo_wm_in_frame *
				       SMI230_GYRO_FIFO_FRAME_LENGTH;

		ret = regmap_bulk_read(data->regmap, SMI230_GYRO_FIFO_DATA_ADDR,
				       data->fifo_buf, fifo_length_in_bytes);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}

		data->current_timestamp = iio_get_time_ns(indio_dev);

		sample_count = smi230gyro_extract_sample(data->fifo_buf,
							 data->fifo_sample_buf,
							 fifo_length_in_bytes);

		interval = smi230gyro_calc_sample_time_interval(data,
								sample_count);

		//ignore first irq
		//ignore fifo data from next irq
		if ((!data->first_irq) && (!repeat) &&
		    (sample_count >= data->cfg.fifo_wm_in_frame) &&
		    (indio_dev->active_scan_mask != NULL)) {
			for (i = 0; i < sample_count; i++) {
				int j = 0;
				int chan = 0;

				for_each_set_bit(chan,
						 indio_dev->active_scan_mask,
						 indio_dev->masklength) {
					switch (chan) {
					case SMI230_GYRO_X:
						sample =
							data->fifo_sample_buf[i]
								.x;
						break;
					case SMI230_GYRO_Y:
						sample =
							data->fifo_sample_buf[i]
								.y;
						break;
					case SMI230_GYRO_Z:
						sample =
							data->fifo_sample_buf[i]
								.z;
						break;
					default:
						mutex_unlock(&data->lock);
						return -EINVAL;
					}
					data->buf[j++] = sample;
				}

				timestamp = data->current_timestamp -
					    (interval * (sample_count - i - 1));

				iio_push_to_buffers_with_timestamp(
					indio_dev, data->buf, timestamp);
			}
		}

		data->first_irq = false;
		repeat = true;
		data->last_timestamp = data->current_timestamp;
		ret = smi230gyro_get_fifo_length_in_frames(
			data, &fifo_length_in_frames);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}
	}
	mutex_unlock(&data->lock);
	return 0;
}

static irqreturn_t smi230gyro_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230gyro_data *data = iio_priv(indio_dev);

	if (data->int_status1.fields.data_int) {
		smi230gyro_data_ready_handler(data, indio_dev);
	} else if (data->int_status1.fields.fifo_int) {
		if (IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
			smi230gyro_fifo_wm_handler(data, indio_dev);
		} else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL)) {
			smi230gyro_fifo_full_handler(data, indio_dev);
		}
	}

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static irqreturn_t smi230gyro_irq_handler(int irq, void *p)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t smi230gyro_irq_thread_handler(int irq, void *private)
{
	int ret, int_status1;
	struct iio_dev *indio_dev = private;
	struct smi230gyro_data *data = iio_priv(indio_dev);
	ret = regmap_read(data->regmap, SMI230_GYRO_INT_STAT_1_REG,
			  &int_status1);
	if (ret)
		return -EIO;
	data->int_status1.value = int_status1;

	iio_trigger_poll(indio_dev->trig);
	return IRQ_HANDLED;
}

static int smi230gyro_config_interrupt_pin(struct smi230gyro_data *data,
					   struct iio_dev *indio_dev)
{
	int ret;

	if (IS_ENABLED(CONFIG_SMI230_DATA_SYNC)) {
		data->cfg.int_conf.fields.int3_od = SMI230_INT_MODE_PUSH_PULL;
		data->cfg.int_conf.fields.int3_lvl = data->cfg.irq_type;
		ret = regmap_write(data->regmap,
				   SMI230_GYRO_INT3_INT4_IO_CONF_REG,
				   data->cfg.int_conf.value);
		if (ret)
			return ret;
		data->cfg.data_int_mapping.fields.int3_data = 1;
		ret = regmap_write(data->regmap,
				   SMI230_GYRO_INT3_INT4_IO_MAP_REG,
				   data->cfg.data_int_mapping.value);
		if (ret)
			return ret;
		data->cfg.int_ctrl.fields.data_en = 1;
		ret = regmap_write(data->regmap, SMI230_GYRO_INT_CTRL_REG,
				   data->cfg.int_ctrl.value);
		if (ret) {
			return -EIO;
		}
	} else {
		if (IS_ENABLED(CONFIG_SMI230_GYRO_INT3)) {
			data->cfg.int_conf.fields.int3_od =
				SMI230_INT_MODE_PUSH_PULL;
			data->cfg.int_conf.fields.int3_lvl = data->cfg.irq_type;
			ret = regmap_write(data->regmap,
					   SMI230_GYRO_INT3_INT4_IO_CONF_REG,
					   data->cfg.int_conf.value);
			if (ret)
				return ret;

			if (IS_ENABLED(CONFIG_SMI230_DATA_READY))
				data->cfg.data_int_mapping.fields.int3_data = 1;
			else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL))
				data->cfg.data_int_mapping.fields.int3_fifo = 1;
			else if (IS_ENABLED(CONFIG_SMI230_FIFO_WM))
				data->cfg.data_int_mapping.fields.int3_fifo = 1;

		} else if (IS_ENABLED(CONFIG_SMI230_GYRO_INT4)) {
			data->cfg.int_conf.fields.int4_od =
				SMI230_INT_MODE_PUSH_PULL;
			data->cfg.int_conf.fields.int4_lvl = data->cfg.irq_type;
			ret = regmap_write(data->regmap,
					   SMI230_GYRO_INT3_INT4_IO_CONF_REG,
					   data->cfg.int_conf.value);
			if (ret)
				return ret;

			if (IS_ENABLED(CONFIG_SMI230_DATA_READY))
				data->cfg.data_int_mapping.fields.int4_data = 1;
			else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL))
				data->cfg.data_int_mapping.fields.int4_fifo = 1;
			else if (IS_ENABLED(CONFIG_SMI230_FIFO_WM))
				data->cfg.data_int_mapping.fields.int4_fifo = 1;
		}

		ret = regmap_write(data->regmap,
				   SMI230_GYRO_INT3_INT4_IO_MAP_REG,
				   data->cfg.data_int_mapping.value);
		if (ret)
			return ret;
	}

	return 0;
}

static int smi230gyro_request_irq(struct smi230gyro_data *data,
				  struct iio_dev *indio_dev)
{
	struct device_node *dvnode;
	int ret, irq, irq_type;
	struct irq_data *desc;

	dvnode = data->dev->of_node;

	if (!dvnode)
		return -ENODEV;

	irq = of_irq_get_byname(dvnode, "GYRO_INT");
	desc = irq_get_irq_data(irq);

#ifndef CONFIG_SMI230_DATA_SYNC
	if (!desc)
		return dev_err_probe(data->dev, -EINVAL,
				     "GYRO Could not find IRQ %d\n", irq);
#endif

	irq_type = irqd_get_trigger_type(desc);
	data->cfg.irq = irq;
	data->cfg.irq_type = irq_type;

	ret = devm_request_threaded_irq(data->dev, irq, smi230gyro_irq_handler,
					smi230gyro_irq_thread_handler, irq_type,
					indio_dev->name, indio_dev);

	if (ret)
		return dev_err_probe(data->dev, ret,
				     "GYRO Failed to request IRQ\n");

	ret = smi230gyro_config_interrupt_pin(data, indio_dev);
	if (ret)
		return dev_err_probe(data->dev, ret,
				     "GYRO Failed to config interrupt pin\n");

	return 0;
}

static int smi230gyro_config_fifo(struct smi230gyro_data *data)
{
	int ret;

	if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL)) {
		ret = regmap_write(data->regmap, SMI230_GYRO_WM_INT_REG, 0x08);
		if (ret)
			return ret;

	} else if (IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		ret = regmap_write(data->regmap, SMI230_GYRO_WM_INT_REG, 0x88);
		if (ret)
			return ret;
	}

	return 0;
}

static int smi230gyro_load_default_config(struct smi230gyro_data *data)
{
	int ret;

	ret = regmap_read(data->regmap, SMI230_GYRO_BANDWIDTH_REG,
			  &(data->cfg.bw_odr.value));
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, SMI230_GYRO_RANGE_REG,
			  &(data->cfg.range.value));
	if (ret)
		return ret;

	return 0;
}

static int smi230gyro_init(struct smi230gyro_data *data,
			   struct iio_dev *indio_dev)
{
	int ret;

	ret = smi230gyro_load_default_config(data);
	if (ret)
		return ret;
	ret = smi230gyro_set_bw(data, SMI230_GYRO_BW_32_ODR_100_HZ);
	if (ret)
		return ret;

	ret = smi230gyro_request_irq(data, indio_dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL) ||
	    IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		ret = smi230gyro_config_fifo(data);
		if (ret)
			return ret;
	}
	if (IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		ret = smi230gyro_set_fifo_wm(data, 10);
		if (ret)
			return ret;
	}

	ret = smi230gyro_set_power(data, SMI230_GYRO_PM_SUSPEND);
	if (ret)
		return ret;
	return 0;
}

static int smi230gyro_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	int ret, val;

	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230gyro_data *data = iio_priv(indio_dev);

	if (IS_ENABLED(CONFIG_SMI230_DATA_SYNC))
		return 0;

	data->first_irq = enable;
	ret = regmap_read(data->regmap, SMI230_GYRO_INT_CTRL_REG, &val);
	if (ret) {
		return -EIO;
	}
	data->cfg.int_ctrl.value = val;

	if (IS_ENABLED(CONFIG_SMI230_DATA_READY)) {
		data->cfg.int_ctrl.fields.data_en = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_GYRO_INT_CTRL_REG,
				   data->cfg.int_ctrl.value);
		if (ret) {
			return -EIO;
		}
	} else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL) ||
		   IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		data->cfg.int_ctrl.fields.fifo_en = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_GYRO_INT_CTRL_REG,
				   data->cfg.int_ctrl.value);
		if (ret)
			return -EIO;

		// set FIFO mode and clear the FIFO
		data->cfg.fifo_config1.value = enable ? SMI230_GYRO_FIFO_MODE :
							0;

		ret = regmap_write(data->regmap, SMI230_GYRO_FIFO_CONFIG_1_ADDR,
				   data->cfg.fifo_config1.value);
		if (ret)
			return ret;
	}
	return 0;
}
static const struct iio_trigger_ops smi230gyro_trigger_ops = {
	.set_trigger_state = &smi230gyro_set_trigger_state,
};

int smi230gyro_core_probe(struct device *dev, struct regmap *regmap)
{
	int ret, chip_id = 0;
	struct iio_dev *indio_dev;
	struct smi230gyro_data *data;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	drv_data = data;
	data->dev = dev;
	data->regmap = regmap;
	mutex_init(&data->lock);

	ret = regmap_read(data->regmap, SMI230_GYRO_CHIP_ID_REG, &chip_id);
	if (ret)
		return dev_err_probe(dev, ret, "Read GYRO chip id failed\n");

	if (chip_id != SMI230_GYRO_CHIP_ID)
		dev_info(dev, "Unknown GYRO chip id: 0x%04x\n", chip_id);

	indio_dev->channels = smi230gyro_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi230gyro_channels);
	indio_dev->name = "smi230gyro";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->info = &smi230gyro_info;
	data->trig = devm_iio_trigger_alloc(dev, "%s-trigger", indio_dev->name);
	if (data->trig == NULL)
		return -ENOMEM;

	indio_dev->trig = data->trig;
	indio_dev->trig->ops = &smi230gyro_trigger_ops;
	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = smi230gyro_init(data, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Init IIO GYRO device failed\n");

	ret = devm_iio_trigger_register(dev, indio_dev->trig);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to register trigger");
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      smi230gyro_trigger_handler, NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Setup triggered buffer failed\n");

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Register IIO GYRO device failed\n");
	return 0;
}
