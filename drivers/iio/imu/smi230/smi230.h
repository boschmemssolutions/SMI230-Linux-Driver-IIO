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

#ifndef _SMI230_H
#define _SMI230_H

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/mutex.h>

#define SMI230_ACC_CHIP_ID_REG 0x00
#define SMI230_ACC_ERR_REG 0x02
#define SMI230_ACC_STATUS_REG 0x03
#define SMI230_ACC_X_LSB_REG 0x12
#define SMI230_ACC_X_MSB_REG 0x13
#define SMI230_ACC_Y_LSB_REG 0x14
#define SMI230_ACC_Y_MSB_REG 0x15
#define SMI230_ACC_Z_LSB_REG 0x16
#define SMI230_ACC_Z_MSB_REG 0x17
#define SMI230_ACC_SENSORTIME_0_REG 0x18
#define SMI230_ACC_SENSORTIME_1_REG 0x19
#define SMI230_ACC_SENSORTIME_2_REG 0x1A
#define SMI230_ACC_INT_STAT_0_REG 0x1C
#define SMI230_ACC_INT_STAT_1_REG 0x1D
#define SMI230_SYNC_X_LSB_REG 0x1E
#define SMI230_SYNC_X_MSB_REG 0x1F
#define SMI230_SYNC_Y_LSB_REG 0x20
#define SMI230_SYNC_Y_MSB_REG 0x21
#define SMI230_TEMP_MSB_REG 0x22
#define SMI230_TEMP_LSB_REG 0x23
#define SMI230_SYNC_Z_LSB_REG 0x27
#define SMI230_SYNC_Z_MSB_REG 0x28
#define SMI230_REG_ORIENT_HIGHG_OUT_REG 0x29
#define SMI230_ACC_INTERNAL_STAT_REG 0x2A
#define SMI230_ACC_CONF_REG 0x40
#define SMI230_ACC_RANGE_REG 0x41
#define SMI230_ACC_INT1_IO_CONF_REG 0x53
#define SMI230_ACC_INT2_IO_CONF_REG 0x54
#define SMI230_ACC_INT_LATCH_CONF_REG 0x55
#define SMI230_ACC_INT1_MAP_REG 0x56
#define SMI230_ACC_INT2_MAP_REG 0x57
#define SMI230_ACC_INT1_INT2_MAP_DATA_REG 0x58
#define SMI230_ACC_INIT_CTRL_REG 0x59
#define SMI230_ACC_SELF_TEST_REG 0x6D
#define SMI230_ACC_PWR_CONF_REG 0x7C
#define SMI230_ACC_PWR_CTRL_REG 0x7D
#define SMI230_ACC_SOFTRESET_REG 0x7E

#define SMI230_ACC_CHIP_ID 0x1F
#define SMI230_ACC_I2C_ADDR_PRIMARY 0x18
#define SMI230_ACC_I2C_ADDR_SECONDARY 0x19
#define SMI230_ACC_RESERVED_5B_REG 0x5B
#define SMI230_ACC_RESERVED_5C_REG 0x5C
#define SMI230_ACC_FEATURE_CFG_REG 0x5E
#define SMI230_ACC_DATA_READY_INT 0x80
#define SMI230_ACC_POWER_MODE_CONFIG_DELAY 30
#define SMI230_ACC_BW_OSR4 0x00
#define SMI230_ACC_BW_OSR2 0x01
#define SMI230_ACC_BW_NORMAL 0x02

#define SMI230_ACC_RANGE_2G 0x00
#define SMI230_ACC_RANGE_4G 0x01
#define SMI230_ACC_RANGE_8G 0x02
#define SMI230_ACC_RANGE_16G 0x03

#define SMI230_ACC_ODR_12_5_HZ 0x05
#define SMI230_ACC_ODR_25_HZ 0x06
#define SMI230_ACC_ODR_50_HZ 0x07
#define SMI230_ACC_ODR_100_HZ 0x08
#define SMI230_ACC_ODR_200_HZ 0x09
#define SMI230_ACC_ODR_400_HZ 0x0A
#define SMI230_ACC_ODR_800_HZ 0x0B
#define SMI230_ACC_ODR_1600_HZ 0x0C

#define SMI230_ACC_SWITCH_OFF_SELF_TEST 0x00
#define SMI230_ACC_POSITIVE_SELF_TEST 0x0D
#define SMI230_ACC_NEGATIVE_SELF_TEST 0x09

#define SMI230_ACC_PM_ACTIVE 0x00
#define SMI230_ACC_PM_SUSPEND 0x03

#define SMI230_ACC_POWER_DISABLE 0x00
#define SMI230_ACC_POWER_ENABLE 0x04

#define SMI230_ACC_INTA_DISABLE 0x00
#define SMI230_ACC_INTA_ENABLE 0x01
#define SMI230_ACC_INTB_DISABLE 0x00
#define SMI230_ACC_INTB_ENABLE 0x02

#define SMI230_ACC_FIFO_MODE 0x01
#define SMI230_ACC_MAX_FIFO_BYTES 1024
#define SMI230_ACC_MAX_FIFO_FRAME 147
#define SMI230_ACC_FIFO_FRAME_LENGTH 7
#define SMI230_ACC_FIFO_WTM_0_ADDR 0x46
#define SMI230_ACC_FIFO_DATA_LENGTH 0x02
#define SMI230_ACC_SENSOR_TIME_LENGTH 3
#define SMI230_ACC_FIFO_LENGTH_0_ADDR 0x24
#define SMI230_ACC_FIFO_DATA_ADDR 0x26
#define SMI230_ACC_FIFO_CONFIG_0_ADDR 0x48
#define SMI230_ACC_FIFO_CONFIG_1_ADDR 0x49
#define SMI230_ACC_FIFO_DATA_FRAME_HEADER 0x84

#define SMI230_ACC_FIFO_HEADER_SENS_TIME_FRM 0x44
#define SMI230_ACC_FIFO_HEADER_SKIP_FRM 0x40
#define SMI230_ACC_FIFO_HEADER_INPUT_CFG_FRM 0x48
#define SMI230_ACC_FIFO_SAMPLE_DROP_FRM 0x50
#define SMI230_ACC_SOFT_RESET_CMD 0xB6

#define SMI230_CONFIG_STREAM_SIZE 6144

//###################################### GYRO ##############################################

#define SMI230_GYRO_CHIP_ID_REG 0x00
#define SMI230_GYRO_X_LSB_REG 0x02
#define SMI230_GYRO_X_MSB_REG 0x03
#define SMI230_GYRO_Y_LSB_REG 0x04
#define SMI230_GYRO_Y_MSB_REG 0x05
#define SMI230_GYRO_Z_LSB_REG 0x06
#define SMI230_GYRO_Z_MSB_REG 0x07
#define SMI230_GYRO_INT_STAT_1_REG 0x0A
#define SMI230_GYRO_RANGE_REG 0x0F
#define SMI230_GYRO_BANDWIDTH_REG 0x10
#define SMI230_GYRO_LPM1_REG 0x11
#define SMI230_GYRO_SOFTRESET_REG 0x14
#define SMI230_GYRO_INT_CTRL_REG 0x15
#define SMI230_GYRO_INT3_INT4_IO_CONF_REG 0x16
#define SMI230_GYRO_INT3_INT4_IO_MAP_REG 0x18
#define SMI230_GYRO_WM_INT_REG 0x1E
#define SMI230_GYRO_FIFO_EXT_INT_S_REG 0x34
#define SMI230_GYRO_SELF_TEST_REG 0x3C
#define SMI230_GYRO_CHIP_ID 0x0F
#define SMI230_GYRO_I2C_ADDR_PRIMARY 0x68
#define SMI230_GYRO_I2C_ADDR_SECONDARY 0x69

#define SMI230_GYRO_FIFO_STATUS_ADDR 0x0E
#define SMI230_GYRO_FIFO_CONFIG_0_ADDR 0x3D
#define SMI230_GYRO_FIFO_CONFIG_1_ADDR 0x3E
#define SMI230_GYRO_FIFO_DATA_ADDR 0x3F

#define SMI230_GYRO_INT_STAT_DRDY 0x80
#define SMI230_GYRO_INT_STAT_FIFO 0x10

#define SMI230_GYRO_FIFO_FRAME_LENGTH 6
// 0x8C instead 0x80 to fix FIFO Data Inconsistency
#define SMI230_GYRO_STREAM_MODE 0x8C

// 0x4C instead 0x40 to fix FIFO Data Inconsistency
#define SMI230_GYRO_FIFO_MODE 0x4c

#define SMI230_GYRO_MAX_FIFO_FRAME 100
#define SMI230_GYRO_MAX_FIFO_BYTES \
	(SMI230_GYRO_MAX_FIFO_FRAME * SMI230_GYRO_FIFO_FRAME_LENGTH)

#define SMI230_W_FIFO_EMPTY INT8_C(1)

#define SMI230_GYRO_RANGE_2000_DPS 0x00
#define SMI230_GYRO_RANGE_1000_DPS 0x01
#define SMI230_GYRO_RANGE_500_DPS 0x02
#define SMI230_GYRO_RANGE_250_DPS 0x03
#define SMI230_GYRO_RANGE_125_DPS 0x04
#define SMI230_GYRO_BW_523_ODR_2000_HZ 0x00
#define SMI230_GYRO_BW_230_ODR_2000_HZ 0x01
#define SMI230_GYRO_BW_116_ODR_1000_HZ 0x02
#define SMI230_GYRO_BW_47_ODR_400_HZ 0x03
#define SMI230_GYRO_BW_23_ODR_200_HZ 0x04
#define SMI230_GYRO_BW_12_ODR_100_HZ 0x05
#define SMI230_GYRO_BW_64_ODR_200_HZ 0x06
#define SMI230_GYRO_BW_32_ODR_100_HZ 0x07
#define SMI230_GYRO_ODR_RESET_VAL 0x80
#define SMI230_GYRO_PM_NORMAL 0x00
#define SMI230_GYRO_PM_DEEP_SUSPEND 0x20
#define SMI230_GYRO_PM_SUSPEND 0x80
#define SMI230_GYRO_DRDY_INT_DISABLE_VAL 0x00
#define SMI230_GYRO_DRDY_INT_ENABLE_VAL 0x80
#define SMI230_GYRO_FIFO_INT_DISABLE_VAL 0x00
#define SMI230_GYRO_FIFO_INT_ENABLE_VAL 0x40
#define SMI230_GYRO_MAP_DRDY_TO_INT3 0x01
#define SMI230_GYRO_MAP_DRDY_TO_INT4 0x80
#define SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 0x81
#define SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4 0x24
#define SMI230_GYRO_POWER_MODE_CONFIG_DELAY 30
#define SMI230_GYRO_RANGE_MASK 0x07
#define SMI230_GYRO_BW_MASK 0x0F
#define SMI230_GYRO_POWER_MASK 0xA0
#define SMI230_GYRO_POWER_POS 5
#define SMI230_GYRO_DATA_EN_MASK 0x80
#define SMI230_GYRO_DATA_EN_POS 7
#define SMI230_GYRO_FIFO_EN_MASK 0x40
#define SMI230_GYRO_FIFO_EN_POS 6
#define SMI230_GYRO_INT3_LVL_MASK 0x01
#define SMI230_GYRO_INT3_OD_MASK 0x02
#define SMI230_GYRO_INT4_LVL_MASK 0x04
#define SMI230_GYRO_INT4_OD_MASK 0x08
#define SMI230_GYRO_INT3_OD_POS 1
#define SMI230_GYRO_INT4_LVL_POS 2
#define SMI230_GYRO_INT4_OD_POS 3
#define SMI230_GYRO_INT_EN_MASK 0x80
#define SMI230_GYRO_INT_EN_POS 7
#define SMI230_GYRO_INT3_MAP_MASK 0x01
#define SMI230_GYRO_INT4_MAP_MASK 0x80
#define SMI230_GYRO_FIFO_INT3_MAP_MASK 0x04
#define SMI230_GYRO_FIFO_INT4_MAP_MASK 0x20
#define SMI230_GYRO_INT3_MAP_POS 0
#define SMI230_GYRO_INT4_MAP_POS 7
#define SMI230_GYRO_FIFO_INT3_MAP_POS 2
#define SMI230_GYRO_FIFO_INT4_MAP_POS 5
#define SMI230_GYRO_SELF_TEST_EN_MASK 0x01
#define SMI230_GYRO_SELF_TEST_RDY_MASK 0x02
#define SMI230_GYRO_SELF_TEST_RESULT_MASK 0x04
#define SMI230_GYRO_SELF_TEST_FUNCTION_MASK 0x08
#define SMI230_GYRO_SELF_TEST_RDY_POS 1
#define SMI230_GYRO_SELF_TEST_RESULT_POS 2
#define SMI230_GYRO_SELF_TEST_FUNCTION_POS 3
#define SMI230_GYRO_SELF_TEST_SUCCESS 0x02

#define SMI230_INT_MODE_PUSH_PULL 0
#define SMI230_INT_MODE_OPEN_DRAIN 1
#define SMI230_INT_ACTIVE_LOW 0
#define SMI230_INT_ACTIVE_HIGH 1

#define SMI230_ANYMOTION_ADR 0x00
#define SMI230_DATA_SYNC_ADR 0x02
#define SMI230_HIGH_G_START_ADR 0x03
#define SMI230_LOW_G_START_ADR 0x06
#define SMI230_ORIENT_START_ADR 0x09
#define SMI230_NOMOTION_START_ADR 0x0B

#define SMI230_DATA_SYNC_OFF 0x00
#define SMI230_DATA_SYNC_400HZ 0x01
#define SMI230_DATA_SYNC_1000HZ 0x02
#define SMI230_DATA_SYNC_2000HZ 0x03
#define SMI230_DATA_SYNC_100HZ 0x04
#define SMI230_DATA_SYNC_200HZ 0x05

#define SMI230_CONFIG_STATUS_MASK 0x07
#define SMI230_FEATURE_REG_BASE 0x10
#define SMI230_ANYMOTION_LEN 2
#define SMI230_NOMOTION_LEN 2
#define SMI230_HIGH_G_LEN 3
#define SMI230_LOW_G_LEN 3
#define SMI230_ORIENT_LEN 2

#define SMI230_ANYMOTION_THRESHOLD_MASK 0x07FF
#define SMI230_ANYMOTION_THRESHOLD_SHIFT 0x00
#define SMI230_ANYMOTION_NOMOTION_SEL_MASK 0x0800
#define SMI230_ANYMOTION_NOMOTION_SEL_SHIFT 0x0B
#define SMI230_ANYMOTION_DURATION_MASK 0x1FFF
#define SMI230_ANYMOTION_DURATION_SHIFT 0x00
#define SMI230_ANYMOTION_X_EN_MASK 0x2000
#define SMI230_ANYMOTION_X_EN_SHIFT 0x0D
#define SMI230_ANYMOTION_Y_EN_MASK 0x4000
#define SMI230_ANYMOTION_Y_EN_SHIFT 0x0E
#define SMI230_ANYMOTION_Z_EN_MASK 0x8000
#define SMI230_ANYMOTION_Z_EN_SHIFT 0x0F

#define SMI230_NOMOTION_THRESHOLD_MASK 0x07FF
#define SMI230_NOMOTION_EN_MASK 0x0800
#define SMI230_NOMOTION_DURATION_MASK 0x1FFF
#define SMI230_NOMOTION_X_EN_MASK 0x2000
#define SMI230_NOMOTION_Y_EN_MASK 0x4000
#define SMI230_NOMOTION_Z_EN_MASK 0x8000

#define SMI230_NOMOTION_THRESHOLD_POS 0
#define SMI230_NOMOTION_EN_POS 11
#define SMI230_NOMOTION_DURATION_POS 0
#define SMI230_NOMOTION_X_EN_POS 13
#define SMI230_NOMOTION_Y_EN_POS 14
#define SMI230_NOMOTION_Z_EN_POS 15

struct smi230_sensor_data {
	s16 x;
	s16 y;
	s16 z;
};

enum {
	SMI230_ACC_X,
	SMI230_ACC_Y,
	SMI230_ACC_Z,
#ifdef CONFIG_SMI230_DATA_SYNC
	DSYNC_GYRO_X,
	DSYNC_GYRO_Y,
	DSYNC_GYRO_Z,
#endif
	SMI230_ACC_TEMP,
	SMI230_SCAN_TIMESTAMP,
};

struct smi230_anymotion_cfg {
	/* 11 bit threshold of anymotion detection
	 * (threshold = X mg * 2,048 (5.11 format))
	 */
	u16 threshold;
	u8 enable;

	/* 13 bit set the duration for any- and nomotion
	 * (time = duration * 20ms (@50Hz))
	 */
	u16 duration;
	u8 x_en;
	u8 y_en;
	u8 z_en;
};

struct smi230_nomotion_cfg {
	u16 duration;
	u16 threshold;
	u8 x_en;
	u8 y_en;
	u8 z_en;
	u8 enable;
};

union smi230acc_fifo_config0 {
	struct {
		u8 mode : 1; /*! FIFO mode */
		u8 be1 : 1; /*! must be 1 */
		u8 reserved : 6; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_fifo_config1 {
	struct {
		u8 reserved : 2; /*! Reserved */
		u8 int2_en : 1; /*! enable storing of captured interrupt events at pin INT1 */
		u8 int1_en : 1; /*! enable storing of captured interrupt events at pin INT2 */
		u8 be1 : 2; /*! must be 1 */
		u8 acc_en : 1; /*! enables storing of accelerometer sensor data */
		u8 reserved2 : 1; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_int_status0 {
	struct {
		u8 data_sync_out : 1; /*! data_sync interrupt */
		u8 any_mot_out : 1; /*! any-motion interrupt */
		u8 high_g_out : 1; /*! high-g interrupt */
		u8 low_g_out : 1; /*! low-g interrupt */
		u8 orient_out : 1; /*! orientation interrupt */
		u8 no_mot_out : 1; /*! no-motion interrupt */
		u8 reserved : 2; /*! reserved*/
	} fields;
	int value;
};

union smi230acc_int_status1 {
	struct {
		u8 fful_int : 1; /*! FIFO full interrupt */
		u8 fwm_int : 1; /*! FIFO watermark interrupt */
		u8 reserved : 5; /*! Reserved */
		u8 drdy : 1; /*! new data interrupt */
	} fields;
	int value;
};

union smi230acc_acc_conf {
	struct {
		u8 odr : 4; /*! Output data rate */
		u8 bw : 3; /*! filter setting */
		u8 reserved : 1; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_pwr_conf {
	struct {
		u8 pwr_conf : 3; /*! acc_pwr_save */
		u8 reserved : 5; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_range {
	struct {
		u8 range : 2; /*! acc_range */
		u8 reserved : 6; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_pwr_ctrl {
	struct {
		u8 pwr_ctrl : 8; /*! acc_pwr_ctrl */
	} fields;
	int value;
};

union smi230acc_data_int_mapping {
	struct {
		u8 int1_fful : 1; /*! map fifo full to int1*/
		u8 int1_fwm : 1; /*! map fifo wm to int1*/
		u8 int1_drdy : 1; /*! map data ready to int1*/
		u8 reserved1 : 1; /*! Reserved */
		u8 int2_fful : 1; /*! map fifo full to int2*/
		u8 int2_fwm : 1; /*! map fifo wm to int2*/
		u8 int2_drdy : 1; /*! map data ready to int2*/
		u8 reserved2 : 1; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_feature_int1_mapping {
	struct {
		u8 data_sync_out : 1; /*! map data sync to int1*/
		u8 any_mot_out : 1; /*! map any motion to int1*/
		u8 high_g_out : 1; /*! map high g to int1*/
		u8 low_g_out : 1; /*! map low g to int1 */
		u8 orient_out : 1; /*! map orientation full to int1*/
		u8 no_mot_out : 1; /*! map no motionto int1*/
		u8 reserved : 2; /*! Reserved*/
	} fields;
	int value;
};

union smi230acc_feature_int2_mapping {
	struct {
		u8 data_sync_out : 1; /*! map data sync to int2*/
		u8 any_mot_out : 1; /*! map any motion to int2*/
		u8 high_g_out : 1; /*! map high g to int2*/
		u8 low_g_out : 1; /*! map low g to int2*/
		u8 orient_out : 1; /*! map orientation full to int2*/
		u8 no_mot_out : 1; /*! map no motionto int2*/
		u8 reserved : 2; /*! Reserved*/
	} fields;
	int value;
};

union smi230acc_int1_conf {
	struct {
		u8 reserved1 : 1; /*!Reserved*/
		u8 int1_lvl : 1; /*! configures active state */
		u8 int1_od : 1; /*! configures pin behavior*/
		u8 int1_out : 1; /*! enable INT1 as output pin*/
		u8 int1_in : 1; /*! enable INT1 as input pin*/
		u8 reserved2 : 3; /*! Reserved */
	} fields;
	int value;
};

union smi230acc_int2_conf {
	struct {
		u8 reserved1 : 1; /*!Reserved*/
		u8 int2_lvl : 1; /*! configures active state */
		u8 int2_od : 1; /*! configures pin behavior*/
		u8 int2_out : 1; /*! enable INT1 as output pin*/
		u8 int2_in : 1; /*! enable INT1 as input pin*/
		u8 reserved2 : 3; /*! Reserved */
	} fields;
	int value;
};

struct smi230acc_cfg {
	union smi230acc_pwr_conf pwr_conf;
	union smi230acc_pwr_ctrl pwr_ctrl;
	union smi230acc_acc_conf acc_conf;
	union smi230acc_range range;
	union smi230acc_data_int_mapping data_int_mapping;
	union smi230acc_feature_int1_mapping feature_int1_mapping;
	union smi230acc_feature_int2_mapping feature_int2_mapping;
	union smi230acc_int1_conf int1_conf;
	union smi230acc_int2_conf int2_conf;
	union smi230acc_fifo_config0 fifo_config0;
	union smi230acc_fifo_config1 fifo_config1;
	struct smi230_anymotion_cfg anymotion_cfg;
	struct smi230_nomotion_cfg nomotion_cfg;
	u16 fifo_wm_in_frame;
	int irq;
	int irq_type;
};

struct smi230acc_data {
	struct device *dev;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct smi230acc_cfg cfg;
	union smi230acc_int_status0 int_status0;
	union smi230acc_int_status1 int_status1;
	s64 current_timestamp;
	s64 last_timestamp;
	s64 last_data_sync_timestamp;
	bool first_irq;
	struct mutex lock;
	u8 fifo_buf[SMI230_ACC_MAX_FIFO_BYTES];
	struct smi230_sensor_data fifo_sample_buf[SMI230_GYRO_MAX_FIFO_FRAME];
	/*
	 * Channel size: 2 bytes.
	 * Max length needed:  3 channels + temp channel.
	 * If fewer channels are enabled, less space may be needed. 
	 * aligned to 8 bytes.
	 */
	s16 buf[4] __aligned(8);
};

enum {
	SMI230_GYRO_X,
	SMI230_GYRO_Y,
	SMI230_GYRO_Z,
};

union smi230gyro_fifo_config1 {
	struct {
		u8 reserved : 6; /*! Reserved */
		u8 fixbit : 2; /*! as work around of the FIFO Data Inconsistency, not defined in TCD */
		u8 fifo_mode : 2; /*! fifo_mode */
	} fields;
	int value;
};

union smi230gyro_int_status1 {
	struct {
		u8 reserved1 : 4; /*! Reserved */
		u8 fifo_int : 1; /*! FIFO  interrupt */
		u8 reserved2 : 2; /*! Reserved */
		u8 data_int : 1; /*! new data interrupt */
	} fields;
	int value;
};

union smi230gyro_bw_odr {
	struct {
		u8 bw : 4; /*! bw_odr */
		u8 reserved : 4; /*! Reserved */
	} fields;
	int value;
};

union smi230gyro_lpm1 {
	struct {
		u8 pwr : 8; /*! power mode */
	} fields;
	int value;
};

union smi230gyro_range {
	struct {
		u8 range : 3; /*! range */
		u8 reserved : 5; /*! Reserved */
	} fields;
	int value;
};

union smi230gyro_data_int_mapping {
	struct {
		u8 int3_data : 1; /*! Map new data interrupt to the INT3  */
		u8 reserved1 : 1; /*! Reserved */
		u8 int3_fifo : 1; /*! Map FIFO interrupt to the INT3  */
		u8 reserved2 : 2; /*! Reserved */
		u8 int4_fifo : 1; /*! Map FIFO interrupt to the INT4 */
		u8 reserved3 : 1; /*! Reserved */
		u8 int4_data : 1; /*! Map new data interrupt to the INT4*/
	} fields;
	int value;
};

union smi230gyro_int_conf {
	struct {
		u8 int3_lvl : 1; /*! Active level for INT3 */
		u8 int3_od : 1; /*! Behavior for INT3 pin */
		u8 int4_lvl : 1; /*! Active level for INT4 */
		u8 int4_od : 1; /*! Behavior for INT4 */
		u8 reserved : 4; /*! Reserved */
	} fields;
	int value;
};

union smi230gyro_int_ctrl {
	struct {
		u8 reserved : 6; /*! Reserved */
		u8 fifo_en : 1; /*! enables the FIFO interrupt */
		u8 data_en : 1; /*! enables the new data interrupt */
	} fields;
	int value;
};

struct smi230gyro_cfg {
	union smi230gyro_bw_odr bw_odr;
	union smi230gyro_lpm1 pwr_ctrl;
	union smi230gyro_range range;
	union smi230gyro_data_int_mapping data_int_mapping;
	union smi230gyro_int_conf int_conf;
	union smi230gyro_int_ctrl int_ctrl;
	union smi230gyro_fifo_config1 fifo_config1;
	u16 fifo_wm_in_frame;
	int irq;
	int irq_type;
};

struct smi230gyro_data {
	struct device *dev;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct smi230gyro_cfg cfg;
	s64 current_timestamp;
	s64 last_timestamp;
	bool first_irq;
	union smi230gyro_int_status1 int_status1;
	struct mutex lock;
	u8 fifo_buf[SMI230_GYRO_MAX_FIFO_BYTES];
	struct smi230_sensor_data fifo_sample_buf[SMI230_ACC_MAX_FIFO_FRAME];
	/*
	 * Channel size: 2 bytes.
	 * Max length needed:  3 channels.
	 * If fewer channels are enabled, less space may be needed.
	 * aligned to 8 bytes.
	 */
	s16 buf[3] __aligned(8);
};

int smi230acc_core_probe(struct device *dev, struct regmap *regmap);

int smi230gyro_core_probe(struct device *dev, struct regmap *regmap);

int smi230gyro_read_registers(unsigned int reg, void *val, size_t val_count);

int smi230gyro_write_registers(unsigned int reg, const void *val,
			       size_t val_count);

#endif /* _SMI230_H */
