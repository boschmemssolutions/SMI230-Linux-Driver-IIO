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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include "smi230.h"

#define SMI230ACC_DATA_CHANNEL(_type, _axis, _index)                       \
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

#define SMI230ACC_TEMP_CHANNEL(_index)                        \
	{                                                     \
		.type = IIO_TEMP, .modified = 1,              \
		.channel2 = IIO_MOD_TEMP_OBJECT,              \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
		.scan_index = _index,                         \
		.scan_type = {                                \
			.sign = 's',                          \
			.realbits = 16,                       \
			.storagebits = 16,                    \
			.endianness = IIO_LE,                 \
		},                                            \
	}

static const struct iio_chan_spec smi230acc_channels[] = {
	SMI230ACC_DATA_CHANNEL(IIO_ACCEL, X, SMI230_ACC_X),
	SMI230ACC_DATA_CHANNEL(IIO_ACCEL, Y, SMI230_ACC_Y),
	SMI230ACC_DATA_CHANNEL(IIO_ACCEL, Z, SMI230_ACC_Z),
	SMI230ACC_TEMP_CHANNEL(SMI230_ACC_TEMP),
	IIO_CHAN_SOFT_TIMESTAMP(SMI230_SCAN_TIMESTAMP),
};

static int smi230_acc_get_data(const struct smi230acc_data *data,
			       struct smi230_sensor_data *accel)
{
	int ret;
	u8 val[6];

	ret = regmap_bulk_read(data->regmap, SMI230_ACC_X_LSB_REG, val, 6);
	if (ret)
		return ret;

	accel->x = (s16)(val[1] << 8) | (val[0]); /* Data in X axis */

	accel->y = (s16)(val[3] << 8) | (val[2]); /* Data in Y axis */

	accel->z = (s16)(val[5] << 8) | (val[4]); /* Data in Z axis */

	return 0;
}

static int smi230_acc_get_temp(const struct smi230acc_data *data, int *val)
{
	s16 temp;
	int ret;
	u8 buf[2];
	u16 msb, lsb;
	u16 msblsb;

	ret = regmap_bulk_read(data->regmap, SMI230_TEMP_MSB_REG, buf, 2);
	if (ret)
		return 0;
	msb = (buf[0] << 3); /* MSB data */
	lsb = (buf[1] >> 5); /* LSB data */
	msblsb = (u16)(msb + lsb);
	if (msblsb > 1023) {
		/* Updating the msblsb */
		temp = (s16)(msblsb - 2048);
	} else {
		temp = (s16)msblsb;
	}

	/* sensor temperature */
	*val = (temp * 125) + 23000;

	return 0;
}

static int smi230acc_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	int ret, temp;

	struct smi230acc_data *data = iio_priv(indio_dev);
	struct smi230_sensor_data acc_val;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ACCEL) {
			ret = smi230_acc_get_data(data, &acc_val);
			if (ret)
				return ret;
			switch (chan->channel2) {
			case IIO_MOD_X:
				*val = acc_val.x;
				break;
			case IIO_MOD_Y:
				*val = acc_val.y;
				break;
			case IIO_MOD_Z:
				*val = acc_val.z;
				break;
			}

		} else if (chan->type == IIO_TEMP) {
			ret = smi230_acc_get_temp(data, &temp);
			if (ret)
				return ret;
			*val = temp;
		}
		return IIO_VAL_INT;
	}
	return 0;
}

static ssize_t bw_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_ACC_CONF_REG,
			  &(data->cfg.acc_conf.value));
	if (ret)
		return ret;

	switch (data->cfg.acc_conf.fields.bw) {
	case SMI230_ACC_BW_NORMAL:
		return sprintf(buf, "normal\n");
	case SMI230_ACC_BW_OSR2:
		return sprintf(buf, "osr2\n");
	case SMI230_ACC_BW_OSR4:
		return sprintf(buf, "osr4\n");
	default:
		return sprintf(buf, "error\n");
	}

	return 0;
}

static int smi230acc_set_bw(struct smi230acc_data *data, u8 bw)
{
	int ret;
	data->cfg.acc_conf.fields.bw = bw;
	ret = regmap_write(data->regmap, SMI230_ACC_CONF_REG,
			   data->cfg.acc_conf.value);
	if (ret)
		return ret;
	return 0;
}

static ssize_t bw_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "normal", 6) == 0)
		data->cfg.acc_conf.fields.bw = SMI230_ACC_BW_NORMAL;
	else if (strncmp(buf, "osr2", 4) == 0)
		data->cfg.acc_conf.fields.bw = SMI230_ACC_BW_OSR2;
	else if (strncmp(buf, "osr4", 4) == 0)
		data->cfg.acc_conf.fields.bw = SMI230_ACC_BW_OSR4;
	else
		return -EINVAL;

	ret = smi230acc_set_bw(data, data->cfg.acc_conf.fields.bw);
	if (ret)
		return ret;

	return count;
}

static ssize_t odr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_ACC_CONF_REG,
			  &(data->cfg.acc_conf.value));
	if (ret)
		return 0;

	switch (data->cfg.acc_conf.fields.odr) {
	case SMI230_ACC_ODR_12_5_HZ:
		return sprintf(buf, "12.5Hz\n");
	case SMI230_ACC_ODR_25_HZ:
		return sprintf(buf, "25Hz\n");
	case SMI230_ACC_ODR_50_HZ:
		return sprintf(buf, "50Hz\n");
	case SMI230_ACC_ODR_100_HZ:
		return sprintf(buf, "100Hz\n");
	case SMI230_ACC_ODR_200_HZ:
		return sprintf(buf, "200Hz\n");
	case SMI230_ACC_ODR_400_HZ:
		return sprintf(buf, "400Hz\n");
	case SMI230_ACC_ODR_800_HZ:
		return sprintf(buf, "800Hz\n");
	case SMI230_ACC_ODR_1600_HZ:
		return sprintf(buf, "1600Hz\n");
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230acc_set_odr(struct smi230acc_data *data, u8 odr)
{
	int ret;
	data->cfg.acc_conf.fields.odr = odr;
	ret = regmap_write(data->regmap, SMI230_ACC_CONF_REG,
			   data->cfg.acc_conf.value);
	if (ret)
		return ret;
	return 0;
}

static ssize_t odr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "12.5Hz", 6) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_12_5_HZ;
	else if (strncmp(buf, "25Hz", 4) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_25_HZ;
	else if (strncmp(buf, "50Hz", 4) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_50_HZ;
	else if (strncmp(buf, "100Hz", 5) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_100_HZ;
	else if (strncmp(buf, "200Hz", 5) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_200_HZ;
	else if (strncmp(buf, "400Hz", 5) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_400_HZ;
	else if (strncmp(buf, "800Hz", 5) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_800_HZ;
	else if (strncmp(buf, "1600Hz", 6) == 0)
		data->cfg.acc_conf.fields.odr = SMI230_ACC_ODR_1600_HZ;
	else
		return -EINVAL;

	ret = smi230acc_set_odr(data, data->cfg.acc_conf.fields.odr);

	if (ret)
		return ret;

	return count;
}

static ssize_t range_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_ACC_RANGE_REG,
			  &(data->cfg.range.value));
	if (ret)
		return 0;

	switch (data->cfg.range.fields.range) {
	case SMI230_ACC_RANGE_2G:
		return sprintf(buf, "2g\n");
	case SMI230_ACC_RANGE_4G:
		return sprintf(buf, "4g\n");
	case SMI230_ACC_RANGE_8G:
		return sprintf(buf, "8g\n");
	case SMI230_ACC_RANGE_16G:
		return sprintf(buf, "16g\n");
	default:
		return -EINVAL;
	}

	return 0;
}

static int smi230acc_set_range(struct smi230acc_data *data, u8 range)
{
	int ret;
	data->cfg.range.fields.range = range;
	ret = regmap_write(data->regmap, SMI230_ACC_RANGE_REG,
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
	struct smi230acc_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "2g", 2) == 0)
		data->cfg.range.fields.range = SMI230_ACC_RANGE_2G;
	else if (strncmp(buf, "4g", 2) == 0)
		data->cfg.range.fields.range = SMI230_ACC_RANGE_4G;
	else if (strncmp(buf, "8g", 2) == 0)
		data->cfg.range.fields.range = SMI230_ACC_RANGE_8G;
	else if (strncmp(buf, "16g", 3) == 0)
		data->cfg.range.fields.range = SMI230_ACC_RANGE_16G;
	else
		return -EINVAL;

	ret = smi230acc_set_range(data, data->cfg.range.fields.range);

	if (ret)
		return ret;

	return count;
}

static ssize_t pwr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	ret = regmap_read(data->regmap, SMI230_ACC_PWR_CONF_REG,
			  &(data->cfg.pwr_conf.value));
	if (ret)
		return ret;

	switch (data->cfg.pwr_conf.fields.pwr_conf) {
	case SMI230_ACC_PM_ACTIVE:
		return sprintf(buf, "normal\n");
	case SMI230_ACC_PM_SUSPEND:
		return sprintf(buf, "suspend\n");
	default:
		return sprintf(buf, "error\n");
	}
	return 0;
}

static int smi230acc_set_power(struct smi230acc_data *data, u8 pwr)
{
	int ret;
	if (pwr == SMI230_ACC_PM_ACTIVE) {
		data->cfg.pwr_conf.value = SMI230_ACC_PM_ACTIVE;
		data->cfg.pwr_ctrl.value = SMI230_ACC_POWER_ENABLE;
	} else if (pwr == SMI230_ACC_PM_SUSPEND) {
		data->cfg.pwr_conf.value = SMI230_ACC_PM_SUSPEND;
		data->cfg.pwr_ctrl.value = SMI230_ACC_POWER_DISABLE;
	}

	ret = regmap_write(data->regmap, SMI230_ACC_PWR_CONF_REG,
			   data->cfg.pwr_conf.value);
	if (ret)
		return ret;

	msleep(SMI230_ACC_POWER_MODE_CONFIG_DELAY);

	ret = regmap_write(data->regmap, SMI230_ACC_PWR_CTRL_REG,
			   data->cfg.pwr_ctrl.value);
	if (ret)
		return ret;

	msleep(SMI230_ACC_POWER_MODE_CONFIG_DELAY);

	return 0;
}

static ssize_t pwr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	if (strncmp(buf, "normal", 6) == 0) {
		ret = smi230acc_set_power(data, SMI230_ACC_PM_ACTIVE);
		if (ret)
			return ret;
	} else if (strncmp(buf, "suspend", 7) == 0) {
		ret = smi230acc_set_power(data, SMI230_ACC_PM_SUSPEND);
		if (ret)
			return ret;
	} else
		return -EINVAL;

	return count;
}

static ssize_t fifo_wm_frame_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	u8 val[2];
	u16 fifo_wm_in_bytes;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	ret = regmap_bulk_read(data->regmap, SMI230_ACC_FIFO_WTM_0_ADDR, val,
			       2);
	if (ret)
		return ret;

	fifo_wm_in_bytes = (val[1] << 8) | (val[0]);

	data->cfg.fifo_wm_in_frame =
		fifo_wm_in_bytes / SMI230_ACC_FIFO_FRAME_LENGTH;

	return sprintf(buf, "%d frames\n", data->cfg.fifo_wm_in_frame);

	return 0;
}

static int smi230acc_set_fifo_wm(struct smi230acc_data *data, u16 fifo_wm)
{
	int ret;
	u16 fifo_wm_in_bytes;
	u8 val[2] = { 0 };

	fifo_wm_in_bytes = fifo_wm * SMI230_ACC_FIFO_FRAME_LENGTH;

	val[0] = (u8)(fifo_wm_in_bytes & 0x00FF);
	val[1] = (u8)((fifo_wm_in_bytes & 0xFF00) >> 8);

	ret = regmap_bulk_write(data->regmap, SMI230_ACC_FIFO_WTM_0_ADDR, val,
				2);
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
	struct smi230acc_data *data = iio_priv(indio_dev);
	u16 fifo_wm;

	ret = kstrtou16(buf, 10, &fifo_wm);
	if (ret)
		return -EINVAL;

	if (fifo_wm > SMI230_ACC_MAX_FIFO_FRAME)
		return -EINVAL;

	ret = smi230acc_set_fifo_wm(data, fifo_wm);
	if (ret)
		return ret;

	return count;
}

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	int ret, val = 0;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);

	int i;

	for (i = 0; i <= 0x7e; i++) {
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

static int smi230acc_self_test(struct smi230acc_data *data)
{
	int ret = 0;
	int int1_io_config, int2_io_config;
	union smi230acc_acc_conf acc_conf;

	struct smi230_sensor_data acc_val_positive = { 0 };
	struct smi230_sensor_data acc_val_negative = { 0 };

	ret = regmap_write(data->regmap, SMI230_ACC_RANGE_REG,
			   SMI230_ACC_RANGE_16G);
	if (ret)
		return ret;

	acc_conf.fields.reserved = 1;
	acc_conf.fields.odr = SMI230_ACC_ODR_1600_HZ;
	acc_conf.fields.bw = SMI230_ACC_BW_NORMAL;
	ret = regmap_write(data->regmap, SMI230_ACC_CONF_REG, acc_conf.value);
	if (ret)
		return ret;

	msleep(3);

	ret = regmap_write(data->regmap, SMI230_ACC_SELF_TEST_REG,
			   SMI230_ACC_POSITIVE_SELF_TEST);
	if (ret)
		return ret;
	msleep(60);
	smi230_acc_get_data(data, &acc_val_positive);

	ret = regmap_write(data->regmap, SMI230_ACC_SELF_TEST_REG,
			   SMI230_ACC_NEGATIVE_SELF_TEST);
	if (ret)
		return ret;
	msleep(60);
	smi230_acc_get_data(data, &acc_val_negative);

	ret = regmap_write(data->regmap, SMI230_ACC_SELF_TEST_REG,
			   SMI230_ACC_SWITCH_OFF_SELF_TEST);
	if (ret)
		return ret;

	msleep(60);

	//restore the regvalue before self test
	ret = regmap_write(data->regmap, SMI230_ACC_RANGE_REG,
			   data->cfg.range.value);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, SMI230_ACC_CONF_REG,
			   data->cfg.acc_conf.value);
	if (ret)
		return ret;

	//disable and re-enable the interrupts
	ret = regmap_read(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
			  &int1_io_config);
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
			  &int2_io_config);
	if (ret)
		return ret;

	ret = regmap_write(data->regmap, SMI230_ACC_INT1_IO_CONF_REG, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, SMI230_ACC_INT2_IO_CONF_REG, 0x00);
	if (ret)
		return ret;

	msleep(60);

	ret = regmap_write(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
			   int1_io_config);
	if (ret)
		return ret;
	ret = regmap_write(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
			   int2_io_config);
	if (ret)
		return ret;

	/*
	dev_info(data->dev, "px%d   nx%d\n", acc_val_positive.x,
		 acc_val_negative.x);
	dev_info(data->dev, "py%d   ny%d\n", acc_val_positive.y,
		 acc_val_negative.y);
	dev_info(data->dev, "pz%d   nz%d\n", acc_val_positive.z,
		 acc_val_negative.z);
	*/

	if ((acc_val_positive.x - acc_val_negative.x) < 2048 ||
	    (acc_val_positive.y - acc_val_negative.y) < 2048 ||
	    (acc_val_positive.z - acc_val_negative.z) < 1024)
		ret = -1;

	return ret;
}

static ssize_t self_test_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi230acc_data *data = iio_priv(indio_dev);
	int ret;

	ret = regmap_read(data->regmap, SMI230_ACC_PWR_CONF_REG,
			  &(data->cfg.pwr_conf.value));
	if (ret)
		return ret;

	if (data->cfg.pwr_conf.value != SMI230_ACC_PM_ACTIVE)
		return snprintf(buf, PAGE_SIZE,
				"acc disabled, enable it firstly\n");

	if (smi230acc_self_test(data))
		return snprintf(buf, PAGE_SIZE, "acc self test failed\n");
	else
		return snprintf(buf, PAGE_SIZE, "acc self test success\n");
}

static IIO_DEVICE_ATTR_RW(bw, 0);
static IIO_DEVICE_ATTR_RW(pwr, 0);
static IIO_DEVICE_ATTR_RW(odr, 0);
static IIO_DEVICE_ATTR_RW(range, 0);
static IIO_DEVICE_ATTR_RW(fifo_wm_frame, 0);
static IIO_DEVICE_ATTR_RO(reg_dump, 0);
static IIO_DEVICE_ATTR_RO(self_test, 0);

static struct attribute *smi230acc_basic_attrs[] = {
	&iio_dev_attr_bw.dev_attr.attr,
	&iio_dev_attr_pwr.dev_attr.attr,
	&iio_dev_attr_odr.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	&iio_dev_attr_fifo_wm_frame.dev_attr.attr,
	&iio_dev_attr_reg_dump.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi230acc_basic_attrs_group = {
	.name = "smi230acc_basic",
	.attrs = smi230acc_basic_attrs,
};

static const struct iio_info smi230acc_info = {
	.read_raw = smi230acc_read_raw,
	.attrs = &smi230acc_basic_attrs_group,
};

static int smi230acc_data_ready_handler(struct smi230acc_data *data,
					struct iio_dev *indio_dev)
{
	int ret, chan;
	int i = 0, temp;
	s16 sample;
	s64 timestamp;
	struct smi230_sensor_data acc_val;

	mutex_lock(&data->lock);
	timestamp = iio_get_time_ns(indio_dev);
	ret = smi230_acc_get_data(data, &acc_val);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	smi230_acc_get_temp(data, &temp);

	for_each_set_bit(chan, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch (chan) {
		case SMI230_ACC_X:
			sample = acc_val.x;
			break;
		case SMI230_ACC_Y:
			sample = acc_val.y;
			break;
		case SMI230_ACC_Z:
			sample = acc_val.z;
			break;
		case SMI230_ACC_TEMP:
			sample = (s16)temp;
			break;
		default:
			return -EINVAL;
		}
		data->buf[i++] = sample;
	}

	ret = iio_push_to_buffers_with_timestamp(indio_dev, data->buf,
						 timestamp);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	mutex_unlock(&data->lock);

	return 0;
}

static int smi230acc_get_fifo_length_in_bytes(struct smi230acc_data *data,
					      u16 *fifo_length_in_bytes)
{
	int ret;
	u8 val[2] = { 0 };

	ret = regmap_bulk_read(data->regmap, SMI230_ACC_FIFO_LENGTH_0_ADDR, val,
			       2);
	if (ret)
		return ret;

	*fifo_length_in_bytes = (u16)(val[1] << 8) | (val[0]);

	return 0;
}

static int smi230acc_extract_sample(u8 *fifo_buf,
				    struct smi230_sensor_data *fifo_sample_buf,
				    u16 fifo_length_in_bytes)
{
	int i = 0, j = 0, sample_count;
	u8 header = 0;

	for (i = 0; i < fifo_length_in_bytes;) {
		header = fifo_buf[i];
		if (header == SMI230_ACC_FIFO_DATA_FRAME_HEADER) {
			fifo_sample_buf[j].x = (s16)(fifo_buf[i + 2] << 8) |
					       (fifo_buf[i + 1]);
			fifo_sample_buf[j].y = (s16)(fifo_buf[i + 4] << 8) |
					       (fifo_buf[i + 3]);
			fifo_sample_buf[j].z = (s16)(fifo_buf[i + 6] << 8) |
					       (fifo_buf[i + 5]);
			j += 1;
			i += 7;
		}
		switch (header) {
		case SMI230_ACC_FIFO_HEADER_SKIP_FRM:
			i += 2;
			break;
		case SMI230_ACC_FIFO_HEADER_SENS_TIME_FRM:
			i += 4;
			break;
		case SMI230_ACC_FIFO_HEADER_INPUT_CFG_FRM:
			i += 2;
			break;
		case SMI230_ACC_FIFO_SAMPLE_DROP_FRM:
			i += 2;
			break;
		}
	}
	sample_count = j;
	return sample_count;
}

static s64 smi230acc_calc_sample_time_interval(struct smi230acc_data *data,
					       int sample_count)
{
	s64 interval = 0, peroid;
	if (data->last_timestamp != 0) {
		peroid = data->current_timestamp - data->last_timestamp;

	} else {
		switch (data->cfg.acc_conf.fields.odr) {
		case SMI230_ACC_ODR_12_5_HZ:
			peroid = 80000000;
			break;
		case SMI230_ACC_ODR_25_HZ:
			peroid = 40000000;
			break;
		case SMI230_ACC_ODR_50_HZ:
			peroid = 20000000;
			break;
		case SMI230_ACC_ODR_100_HZ:
			peroid = 10000000;
			break;
		case SMI230_ACC_ODR_200_HZ:
			peroid = 5000000;
			break;
		case SMI230_ACC_ODR_400_HZ:
			peroid = 2500000;
			break;
		case SMI230_ACC_ODR_800_HZ:
			peroid = 1250000;
			break;
		case SMI230_ACC_ODR_1600_HZ:
			peroid = 625000;
			break;
		default:
			peroid = 0;
		}
	}
	interval = div_s64(peroid, sample_count);
	return interval;
}

static int smi230acc_fifo_full_handler(struct smi230acc_data *data,
				       struct iio_dev *indio_dev)
{
	int ret, i, temp, sample_count;
	s64 interval, timestamp;
	s16 sample;
	u16 fifo_length_in_bytes;

	mutex_lock(&data->lock);
	ret = smi230acc_get_fifo_length_in_bytes(data, &fifo_length_in_bytes);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = regmap_bulk_read(data->regmap, SMI230_ACC_FIFO_DATA_ADDR,
			       data->fifo_buf, fifo_length_in_bytes);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = smi230_acc_get_temp(data, &temp);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->current_timestamp = iio_get_time_ns(indio_dev);

	sample_count = smi230acc_extract_sample(
		data->fifo_buf, data->fifo_sample_buf, fifo_length_in_bytes);

	interval = smi230acc_calc_sample_time_interval(data, sample_count);

	if (!data->first_irq) {
		for (i = 0; i < sample_count; i++) {
			int j = 0;
			int chan = 0;

			for_each_set_bit(chan, indio_dev->active_scan_mask,
					 indio_dev->masklength) {
				switch (chan) {
				case SMI230_ACC_X:
					sample = data->fifo_sample_buf[i].x;
					break;
				case SMI230_ACC_Y:
					sample = data->fifo_sample_buf[i].y;
					break;
				case SMI230_ACC_Z:
					sample = data->fifo_sample_buf[i].z;
					break;
				case SMI230_ACC_TEMP:
					sample = (s16)temp;
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

static int smi230acc_fifo_wm_handler(struct smi230acc_data *data,
				     struct iio_dev *indio_dev)
{
	int ret, i, temp, sample_count;
	s64 interval, timestamp;
	bool repeat = false;
	s16 sample;
	u16 fifo_length_in_bytes;

	mutex_lock(&data->lock);

	ret = smi230acc_get_fifo_length_in_bytes(data, &fifo_length_in_bytes);
	if (ret) {
		mutex_unlock(&data->lock);
		return ret;
	}

	while (fifo_length_in_bytes * 2 >=
	       data->cfg.fifo_wm_in_frame * SMI230_ACC_FIFO_FRAME_LENGTH) {
		ret = regmap_bulk_read(data->regmap, SMI230_ACC_FIFO_DATA_ADDR,
				       data->fifo_buf,
				       data->cfg.fifo_wm_in_frame *
					       SMI230_ACC_FIFO_FRAME_LENGTH);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}

		ret = smi230_acc_get_temp(data, &temp);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}

		data->current_timestamp = iio_get_time_ns(indio_dev);

		sample_count = smi230acc_extract_sample(
			data->fifo_buf, data->fifo_sample_buf,
			data->cfg.fifo_wm_in_frame *
				SMI230_ACC_FIFO_FRAME_LENGTH);

		interval =
			smi230acc_calc_sample_time_interval(data, sample_count);

		//ignore first irq
		//ignore fifo data from next irq
		if ((!data->first_irq) && (!repeat) &&
		    (sample_count >= data->cfg.fifo_wm_in_frame)) {
			for (i = 0; i < sample_count; i++) {
				int j = 0;
				int chan = 0;

				for_each_set_bit(chan,
						 indio_dev->active_scan_mask,
						 indio_dev->masklength) {
					switch (chan) {
					case SMI230_ACC_X:
						sample =
							data->fifo_sample_buf[i]
								.x;
						break;
					case SMI230_ACC_Y:
						sample =
							data->fifo_sample_buf[i]
								.y;
						break;
					case SMI230_ACC_Z:
						sample =
							data->fifo_sample_buf[i]
								.z;
						break;
					case SMI230_ACC_TEMP:
						sample = (s16)temp;
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
		ret = smi230acc_get_fifo_length_in_bytes(data,
							 &fifo_length_in_bytes);
		if (ret) {
			mutex_unlock(&data->lock);
			return ret;
		}
	}

	mutex_unlock(&data->lock);
	return 0;
}

static irqreturn_t smi230acc_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230acc_data *data = iio_priv(indio_dev);

	if (data->int_status1.fields.drdy) {
		smi230acc_data_ready_handler(data, indio_dev);
	} else if (data->int_status1.fields.fwm_int) {
		smi230acc_fifo_wm_handler(data, indio_dev);
	} else if (data->int_status1.fields.fful_int) {
		smi230acc_fifo_full_handler(data, indio_dev);
	}

	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static irqreturn_t smi230acc_irq_handler(int irq, void *p)
{
	return IRQ_WAKE_THREAD;
}

static int smi230acc_toggle_int_pin(struct smi230acc_data *data, bool enable)
{
	int ret;
	union smi230acc_int1_conf int1_conf;
	union smi230acc_int2_conf int2_conf;
	int val = 0;

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1)) {
		ret = regmap_read(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
				  &val);
		if (ret)
			return -EIO;

		int1_conf.value = val;
		int1_conf.fields.int1_out = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
				   int1_conf.value);
		if (ret)
			return -EIO;
	} else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2)) {
		ret = regmap_read(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
				  &val);
		if (ret)
			return -EIO;
		int2_conf.value = val;
		int2_conf.fields.int2_out = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
				   int2_conf.value);
		if (ret)
			return -EIO;
	}

	return 0;
}

static irqreturn_t smi230acc_irq_thread_handler(int irq, void *private)
{
	int ret, int_status1;
	struct iio_dev *indio_dev = private;
	struct smi230acc_data *data = iio_priv(indio_dev);
	if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL) ||
	    IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		smi230acc_toggle_int_pin(data, 0);
	}
	ret = regmap_read(data->regmap, SMI230_ACC_INT_STAT_1_REG,
			  &int_status1);
	if (ret)
		return -EIO;
	data->int_status1.value = int_status1;

	iio_trigger_poll(indio_dev->trig);

	if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL) ||
	    IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		smi230acc_toggle_int_pin(data, 1);
	}
	return IRQ_HANDLED;
}

static int smi230acc_config_interrupt_pin(struct smi230acc_data *data,
					  struct iio_dev *indio_dev)
{
	int ret;

	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1)) {
		data->cfg.int1_conf.fields.int1_od = SMI230_INT_MODE_PUSH_PULL;
		data->cfg.int1_conf.fields.int1_lvl = data->cfg.irq_type;
		ret = regmap_write(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
				   data->cfg.int1_conf.value);
		if (ret)
			return ret;

		if (IS_ENABLED(CONFIG_SMI230_DATA_READY))
			data->cfg.data_int_mapping.fields.int1_drdy = 1;
		else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL))
			data->cfg.data_int_mapping.fields.int1_fful = 1;
		else if (IS_ENABLED(CONFIG_SMI230_FIFO_WM))
			data->cfg.data_int_mapping.fields.int1_fwm = 1;

	} else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2)) {
		data->cfg.int2_conf.fields.int2_od = SMI230_INT_MODE_PUSH_PULL;
		data->cfg.int2_conf.fields.int2_lvl = data->cfg.irq_type;
		ret = regmap_write(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
				   data->cfg.int2_conf.value);
		if (ret)
			return ret;

		if (IS_ENABLED(CONFIG_SMI230_DATA_READY))
			data->cfg.data_int_mapping.fields.int2_drdy = 1;
		else if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL))
			data->cfg.data_int_mapping.fields.int2_fful = 1;
		else if (IS_ENABLED(CONFIG_SMI230_FIFO_WM))
			data->cfg.data_int_mapping.fields.int2_fwm = 1;
	}

	ret = regmap_write(data->regmap, SMI230_ACC_INT1_INT2_MAP_DATA_REG,
			   data->cfg.data_int_mapping.value);
	if (ret)
		return ret;

	return 0;
}

static int smi230acc_request_irq(struct smi230acc_data *data,
				 struct iio_dev *indio_dev)
{
	struct device_node *dvnode;
	int ret, irq, irq_type;
	struct irq_data *desc;

	dvnode = data->dev->of_node;

	if (!dvnode)
		return -ENODEV;

	irq = of_irq_get_byname(dvnode, "ACC_INT");
	desc = irq_get_irq_data(irq);
	if (!desc)
		return dev_err_probe(data->dev, -EINVAL,
				     "ACC Could not find IRQ %d\n", irq);

	irq_type = irqd_get_trigger_type(desc);
	data->cfg.irq = irq;
	data->cfg.irq_type = irq_type;

	ret = devm_request_threaded_irq(data->dev, irq, smi230acc_irq_handler,
					smi230acc_irq_thread_handler, irq_type,
					indio_dev->name, indio_dev);

	if (ret)
		return dev_err_probe(data->dev, ret,
				     "ACC Failed to request IRQ\n");

	ret = smi230acc_config_interrupt_pin(data, indio_dev);
	if (ret)
		return dev_err_probe(data->dev, ret,
				     "ACC Failed to config interrupt pin\n");

	return 0;
}

static int smi230acc_config_fifo(struct smi230acc_data *data)
{
	int ret;

	data->cfg.fifo_config0.fields.be1 = 1;
	data->cfg.fifo_config1.fields.be1 = 1;

	data->cfg.fifo_config0.fields.mode = SMI230_ACC_FIFO_MODE;
	ret = regmap_write(data->regmap, SMI230_ACC_FIFO_CONFIG_0_ADDR,
			   data->cfg.fifo_config0.value);
	if (ret)
		return ret;

	return 0;
}

static int smi230acc_load_default_config(struct smi230acc_data *data)
{
	int ret;

	ret = regmap_read(data->regmap, SMI230_ACC_CONF_REG,
			  &(data->cfg.acc_conf.value));
	if (ret)
		return ret;
	ret = regmap_read(data->regmap, SMI230_ACC_RANGE_REG,
			  &(data->cfg.range.value));
	if (ret)
		return ret;

	return 0;
}

static int smi230acc_init(struct smi230acc_data *data,
			  struct iio_dev *indio_dev)
{
	int ret;

	//1. enable power for acc configuration
	ret = regmap_write(data->regmap, SMI230_ACC_PWR_CONF_REG, 0x0);
	if (ret)
		return ret;

	msleep(SMI230_ACC_POWER_MODE_CONFIG_DELAY);

	//2. disable sampling for acc configuration
	ret = regmap_write(data->regmap, SMI230_ACC_PWR_CTRL_REG, 0x0);
	if (ret)
		return ret;

	ret = smi230acc_load_default_config(data);
	if (ret)
		return ret;

	ret = smi230acc_request_irq(data, indio_dev);
	if (ret)
		return -EINVAL;

	if (IS_ENABLED(CONFIG_SMI230_FIFO_FULL) ||
	    IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		ret = smi230acc_config_fifo(data);
		if (ret)
			return ret;
	}
	if (IS_ENABLED(CONFIG_SMI230_FIFO_WM)) {
		ret = smi230acc_set_fifo_wm(data, 10);
		if (ret)
			return ret;
	}

	ret = smi230acc_set_bw(data, SMI230_ACC_BW_NORMAL);
	if (ret)
		return ret;

	ret = smi230acc_set_odr(data, SMI230_ACC_ODR_100_HZ);
	if (ret)
		return ret;

	ret = smi230acc_set_power(data, SMI230_ACC_PM_SUSPEND);
	if (ret)
		return ret;

	return 0;
}

static int smi230acc_set_trigger_state(struct iio_trigger *trig, bool enable)
{
	int ret, val;

	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230acc_data *data = iio_priv(indio_dev);

	data->first_irq = enable;
	if (IS_ENABLED(CONFIG_SMI230_ACC_INT1)) {
		ret = regmap_read(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
				  &val);
		if (ret) {
			return -EIO;
		}
		data->cfg.int1_conf.value = val;
		data->cfg.int1_conf.fields.int1_out = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_ACC_INT1_IO_CONF_REG,
				   data->cfg.int1_conf.value);
		if (ret) {
			return -EIO;
		}
	} else if (IS_ENABLED(CONFIG_SMI230_ACC_INT2)) {
		ret = regmap_read(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
				  &val);
		if (ret) {
			return -EIO;
		}
		data->cfg.int2_conf.value = val;
		data->cfg.int2_conf.fields.int2_out = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_ACC_INT2_IO_CONF_REG,
				   data->cfg.int2_conf.value);
		if (ret) {
			return -EIO;
		}
	}

	if (IS_ENABLED(CONFIG_SMI230_FIFO_WM) ||
	    IS_ENABLED(CONFIG_SMI230_FIFO_FULL)) {
		data->cfg.fifo_config1.fields.acc_en = enable ? 1 : 0;
		ret = regmap_write(data->regmap, SMI230_ACC_FIFO_CONFIG_1_ADDR,
				   data->cfg.fifo_config1.value);
		if (ret)
			return ret;

		if (enable) {
			// clear FIFO
			ret = regmap_write(data->regmap,
					   SMI230_ACC_SOFTRESET_REG, 0xB0);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct iio_trigger_ops smi230acc_trigger_ops = {
	.set_trigger_state = &smi230acc_set_trigger_state,
};

int smi230acc_core_probe(struct device *dev, struct regmap *regmap)
{
	int ret, chip_id = 0;
	struct iio_dev *indio_dev;
	struct smi230acc_data *data;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->dev = dev;
	data->regmap = regmap;
	mutex_init(&data->lock);

	ret = regmap_read(data->regmap, SMI230_ACC_CHIP_ID_REG, &chip_id);
	if (ret)
		return dev_err_probe(dev, ret, "Read ACC chip id failed\n");

	if (chip_id != SMI230_ACC_CHIP_ID)
		dev_info(dev, "Unknown ACC chip id: 0x%04x\n", chip_id);

	indio_dev->channels = smi230acc_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi230acc_channels);
	indio_dev->name = "smi230acc";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_TRIGGERED;
	indio_dev->info = &smi230acc_info;
	data->trig = devm_iio_trigger_alloc(dev, "%s-trigger", indio_dev->name);
	if (data->trig == NULL)
		return -ENOMEM;
	indio_dev->trig = data->trig;
	indio_dev->trig->ops = &smi230acc_trigger_ops;
	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = smi230acc_init(data, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Init ACC device failed\n");

	ret = devm_iio_trigger_register(dev, indio_dev->trig);
	if (ret) {
		return dev_err_probe(dev, ret, "Failed to register trigger");
	}

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      smi230acc_trigger_handler, NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Setup triggered buffer failed\n");

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Register IIO ACC device failed\n");

	return 0;
}

MODULE_AUTHOR("Jianping Shen <Jianping.Shen@de.bosch.com>");
MODULE_DESCRIPTION("Bosch SMI230 IIO driver");
MODULE_LICENSE("Dual BSD/GPL");