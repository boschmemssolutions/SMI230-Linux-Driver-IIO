/* SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0 */
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2022 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2022 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
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

#ifndef _SMI230_ACC_H
#define _SMI230_ACC_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#endif

#include <linux/device.h>

#define DRIVER_VERSION	"1.2.0"
#define MODULE_NAME	"SMI230ACC"
#define SENSOR_ACC_NAME "SMI230ACC"

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)  S8_C(x)
#define UINT8_C(x) U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)  S16_C(x)
#define UINT16_C(x) U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)  S32_C(x)
#define UINT32_C(x) U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)  S64_C(x)
#define UINT64_C(x) U64_C(x)
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

#define SMI230_ACCEL_CHIP_ID_REG	    UINT8_C(0x00)
#define SMI230_ACCEL_ERR_REG		    UINT8_C(0x02)
#define SMI230_ACCEL_STATUS_REG		    UINT8_C(0x03)
#define SMI230_ACCEL_X_LSB_REG		    UINT8_C(0x12)
#define SMI230_ACCEL_X_MSB_REG		    UINT8_C(0x13)
#define SMI230_ACCEL_Y_LSB_REG		    UINT8_C(0x14)
#define SMI230_ACCEL_Y_MSB_REG		    UINT8_C(0x15)
#define SMI230_ACCEL_Z_LSB_REG		    UINT8_C(0x16)
#define SMI230_ACCEL_Z_MSB_REG		    UINT8_C(0x17)
#define SMI230_ACCEL_SENSORTIME_0_REG	    UINT8_C(0x18)
#define SMI230_ACCEL_SENSORTIME_1_REG	    UINT8_C(0x19)
#define SMI230_ACCEL_SENSORTIME_2_REG	    UINT8_C(0x1A)
#define SMI230_ACCEL_INT_STAT_0_REG	    UINT8_C(0x1C)
#define SMI230_ACCEL_INT_STAT_1_REG	    UINT8_C(0x1D)
#define SMI230_ACCEL_GP_0_REG		    UINT8_C(0x1E)
#define SMI230_TEMP_MSB_REG		    UINT8_C(0x22)
#define SMI230_TEMP_LSB_REG		    UINT8_C(0x23)
#define SMI230_ACCEL_GP_4_REG		    UINT8_C(0x27)
#define SMI230_REG_ORIENT_HIGHG_OUT_REG	    UINT8_C(0x29)
#define SMI230_ACCEL_INTERNAL_STAT_REG	    UINT8_C(0x2A)
#define SMI230_ACCEL_CONF_REG		    UINT8_C(0x40)
#define SMI230_ACCEL_RANGE_REG		    UINT8_C(0x41)
#define SMI230_ACCEL_INT1_IO_CONF_REG	    UINT8_C(0x53)
#define SMI230_ACCEL_INT2_IO_CONF_REG	    UINT8_C(0x54)
#define SMI230_ACCEL_INT_LATCH_CONF_REG	    UINT8_C(0x55)
#define SMI230_ACCEL_INT1_MAP_REG	    UINT8_C(0x56)
#define SMI230_ACCEL_INT2_MAP_REG	    UINT8_C(0x57)
#define SMI230_ACCEL_INT1_INT2_MAP_DATA_REG UINT8_C(0x58)
#define SMI230_ACCEL_INIT_CTRL_REG	    UINT8_C(0x59)
#define SMI230_ACCEL_SELF_TEST_REG	    UINT8_C(0x6D)
#define SMI230_ACCEL_PWR_CONF_REG	    UINT8_C(0x7C)
#define SMI230_ACCEL_PWR_CTRL_REG	    UINT8_C(0x7D)
#define SMI230_ACCEL_SOFTRESET_REG	    UINT8_C(0x7E)
#define SMI230_ACCEL_CHIP_ID		    UINT8_C(0x1F)
#define SMI230_ACCEL_I2C_ADDR_PRIMARY	    UINT8_C(0x18)
#define SMI230_ACCEL_I2C_ADDR_SECONDARY	    UINT8_C(0x19)
#define SMI230_ACCEL_RESERVED_5B_REG	    UINT8_C(0x5B)
#define SMI230_ACCEL_RESERVED_5C_REG	    UINT8_C(0x5C)
#define SMI230_ACCEL_FEATURE_CFG_REG	    UINT8_C(0x5E)
#define SMI230_ACCEL_DATA_READY_INT	    UINT8_C(0x80)

#define SMI230_ACCEL_BW_OSR4   UINT8_C(0x00)
#define SMI230_ACCEL_BW_OSR2   UINT8_C(0x01)
#define SMI230_ACCEL_BW_NORMAL UINT8_C(0x02)

#define SMI230_ACCEL_RANGE_2G  UINT8_C(0x00)
#define SMI230_ACCEL_RANGE_4G  UINT8_C(0x01)
#define SMI230_ACCEL_RANGE_8G  UINT8_C(0x02)
#define SMI230_ACCEL_RANGE_16G UINT8_C(0x03)

#define SMI230_ACCEL_ODR_12_5_HZ UINT8_C(0x05)
#define SMI230_ACCEL_ODR_25_HZ	 UINT8_C(0x06)
#define SMI230_ACCEL_ODR_50_HZ	 UINT8_C(0x07)
#define SMI230_ACCEL_ODR_100_HZ	 UINT8_C(0x08)
#define SMI230_ACCEL_ODR_200_HZ	 UINT8_C(0x09)
#define SMI230_ACCEL_ODR_400_HZ	 UINT8_C(0x0A)
#define SMI230_ACCEL_ODR_800_HZ	 UINT8_C(0x0B)
#define SMI230_ACCEL_ODR_1600_HZ UINT8_C(0x0C)

#define SMI230_ACCEL_SWITCH_OFF_SELF_TEST UINT8_C(0x00)
#define SMI230_ACCEL_POSITIVE_SELF_TEST	  UINT8_C(0x0D)
#define SMI230_ACCEL_NEGATIVE_SELF_TEST	  UINT8_C(0x09)

#define SMI230_ACCEL_PM_ACTIVE	UINT8_C(0x00)
#define SMI230_ACCEL_PM_SUSPEND UINT8_C(0x03)

#define SMI230_ACCEL_POWER_DISABLE UINT8_C(0x00)
#define SMI230_ACCEL_POWER_ENABLE  UINT8_C(0x04)

#define SMI230_ACCEL_INTA_DISABLE UINT8_C(0x00)
#define SMI230_ACCEL_INTA_ENABLE  UINT8_C(0x01)
#define SMI230_ACCEL_INTB_DISABLE UINT8_C(0x00)
#define SMI230_ACCEL_INTB_ENABLE  UINT8_C(0x02)

#define SMI230_ACCEL_DATA_SYNC_INT_ENABLE UINT8_C(0x01)
#define SMI230_ACCEL_ANY_MOT_INT_ENABLE	  UINT8_C(0x02)
#define SMI230_ACCEL_HIGH_G_INT_ENABLE	  UINT8_C(0x04)
#define SMI230_ACCEL_LOW_G_INT_ENABLE	  UINT8_C(0x08)
#define SMI230_ACCEL_ORIENT_INT_ENABLE	  UINT8_C(0x10)
#define SMI230_ACCEL_NO_MOT_INT_ENABLE	  UINT8_C(0x20)
#define SMI230_ACCEL_ERR_INT_ENABLE	  UINT8_C(0x80)
#define SMI230_ACCEL_FIFO_WM_INT_ENABLE	  UINT8_C(0x02)
#define SMI230_ACCEL_FIFO_FULL_INT_ENABLE UINT8_C(0x01)
#define SMI230_ACCEL_DATA_RDY_INT_ENABLE  UINT8_C(0x80)

#define SMI230_ACCEL_INT_DATA_SYNC_MASK UINT8_C(0x01)
#define SMI230_ACCEL_INT_ANY_MOT_MASK	UINT8_C(0x02)
#define SMI230_ACCEL_INT_HIGH_G_MASK	UINT8_C(0x04)
#define SMI230_ACCEL_INT_LOW_G_MASK	UINT8_C(0x08)
#define SMI230_ACCEL_INT_ORIENT_MASK	UINT8_C(0x10)
#define SMI230_ACCEL_INT_NO_MOT_MASK	UINT8_C(0x20)

#define SMI230_ACCEL_INT_DATA_SYNC_POS UINT8_C(0)
#define SMI230_ACCEL_INT_ANY_MOT_POS   UINT8_C(1)
#define SMI230_ACCEL_INT_HIGH_G_POS    UINT8_C(2)
#define SMI230_ACCEL_INT_LOW_G_POS     UINT8_C(3)
#define SMI230_ACCEL_INT_ORIENT_POS    UINT8_C(4)
#define SMI230_ACCEL_INT_NO_MOT_POS    UINT8_C(5)

#define SMI230_FATAL_ERR_MASK UINT8_C(0x01)
#define SMI230_ERR_CODE_MASK  UINT8_C(0x1C)

#define SMI230_CMD_ERR_POS  UINT8_C(1)
#define SMI230_ERR_CODE_POS UINT8_C(2)

#define SMI230_ACCEL_STATUS_MASK UINT8_C(0x80)
#define SMI230_ACCEL_STATUS_POS	 UINT8_C(7)

#define SMI230_ACCEL_ODR_MASK	UINT8_C(0x0F)
#define SMI230_ACCEL_BW_MASK	UINT8_C(0x70)
#define SMI230_ACCEL_RANGE_MASK UINT8_C(0x03)

#define SMI230_ACCEL_BW_POS UINT8_C(4)

#define SMI230_ACCEL_INT_EDGE_MASK UINT8_C(0x01)
#define SMI230_ACCEL_INT_LVL_MASK  UINT8_C(0x02)
#define SMI230_ACCEL_INT_OD_MASK   UINT8_C(0x04)
#define SMI230_ACCEL_INT_IO_MASK   UINT8_C(0x08)
#define SMI230_ACCEL_INT_IN_MASK   UINT8_C(0x10)

#define SMI230_ACCEL_INT_EDGE_POS UINT8_C(0)
#define SMI230_ACCEL_INT_LVL_POS  UINT8_C(1)
#define SMI230_ACCEL_INT_OD_POS	  UINT8_C(2)
#define SMI230_ACCEL_INT_IO_POS	  UINT8_C(3)
#define SMI230_ACCEL_INT_IN_POS	  UINT8_C(4)

#define SMI230_ACCEL_MAP_INTA_MASK UINT8_C(0x01)
#define SMI230_ACCEL_MAP_INTA_POS  UINT8_C(0x00)

#define SMI230_ACCEL_INT1_DRDY_MASK UINT8_C(0x04)
#define SMI230_ACCEL_INT2_DRDY_MASK UINT8_C(0x40)
#define SMI230_ACCEL_INT1_DRDY_POS  UINT8_C(2)
#define SMI230_ACCEL_INT2_DRDY_POS  UINT8_C(6)
#define SMI230_ACCEL_INT1_FFUL_MASK UINT8_C(0x01)
#define SMI230_ACCEL_INT2_FFUL_MASK UINT8_C(0x10)
#define SMI230_ACCEL_INT1_FFUL_POS  UINT8_C(0)
#define SMI230_ACCEL_INT2_FFUL_POS  UINT8_C(4)
#define SMI230_ACCEL_INT1_FWM_MASK  UINT8_C(0x02)
#define SMI230_ACCEL_INT2_FWM_MASK  UINT8_C(0x20)
#define SMI230_ACCEL_INT1_FWM_POS   UINT8_C(1)
#define SMI230_ACCEL_INT2_FWM_POS   UINT8_C(5)

#define SMI230_ASIC_INITIALIZED UINT8_C(0x01)

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

#define SMI230_W_SELF_TEST_FAIL INT8_C(1)

#define SMI230_SOFT_RESET_CMD UINT8_C(0xB6)
#define SMI230_FIFO_RESET_CMD UINT8_C(0xB0)

#define SMI230_DISABLE UINT8_C(0)
#define SMI230_ENABLE  UINT8_C(1)

#define SMI230_SENSOR_DATA_SYNC_TIME_MS UINT8_C(1)
#define SMI230_DELAY_BETWEEN_WRITES_MS	UINT8_C(1)
#define SMI230_SELF_TEST_DELAY_MS	UINT8_C(75)
#define SMI230_POWER_CONFIG_DELAY	UINT8_C(150)
#define SMI230_ACC_CONF_DELAY_MS	UINT8_C(5)
#define SMI230_ASIC_INIT_TIME_MS	UINT8_C(150)
#define SMI230_SOFTRESET_DELAY_MS	UINT8_C(200)

#define SMI230_CONFIG_STREAM_SIZE UINT16_C(6144)

#define SMI230_SENSOR_TIME_MSB_BYTE  UINT8_C(2)
#define SMI230_SENSOR_TIME_XLSB_BYTE UINT8_C(1)
#define SMI230_SENSOR_TIME_LSB_BYTE  UINT8_C(0)

#define SMI230_INT_ACTIVE_LOW	   UINT8_C(0)
#define SMI230_INT_ACTIVE_HIGH	   UINT8_C(1)
#define SMI230_INT_MODE_PUSH_PULL  UINT8_C(0)
#define SMI230_INT_MODE_OPEN_DRAIN UINT8_C(1)

#define SMI230_16_BIT_RESOLUTION UINT8_C(16)

#ifndef SMI230_ABS
#define SMI230_ABS(a) ((a) > 0 ? (a) : -(a))
#endif

#define SMI230_SET_LOW_BYTE   UINT16_C(0x00FF)
#define SMI230_SET_HIGH_BYTE  UINT16_C(0xFF00)
#define SMI230_SET_LOW_NIBBLE UINT8_C(0x0F)

#define SMI230_ACCEL_ANYMOTION_ADR UINT8_C(0x00)
#define SMI230_ACCEL_DATA_SYNC_ADR UINT8_C(0x02)
#define SMI230_HIGH_G_START_ADR	   UINT8_C(0x03)
#define SMI230_LOW_G_START_ADR	   UINT8_C(0x06)
#define SMI230_ORIENT_START_ADR	   UINT8_C(0x09)
#define SMI230_NOMOTION_START_ADR  UINT8_C(0x0B)

#define SMI230_ACCEL_ANYMOTION_LEN UINT8_C(2)
#define SMI230_NOMOTION_LEN	   UINT8_C(2)
#define SMI230_HIGH_G_LEN	   UINT8_C(3)
#define SMI230_LOW_G_LEN	   UINT8_C(3)
#define SMI230_ORIENT_LEN	   UINT8_C(2)

#define SMI230_ACCEL_ANYMOTION_THRESHOLD_MASK	  UINT16_C(0x07FF)
#define SMI230_ACCEL_ANYMOTION_THRESHOLD_SHIFT	  UINT8_C(0x00)
#define SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_MASK  UINT16_C(0x0800)
#define SMI230_ACCEL_ANYMOTION_NOMOTION_SEL_SHIFT UINT8_C(0x0B)
#define SMI230_ACCEL_ANYMOTION_DURATION_MASK	  UINT16_C(0x1FFF)
#define SMI230_ACCEL_ANYMOTION_DURATION_SHIFT	  UINT8_C(0x00)
#define SMI230_ACCEL_ANYMOTION_X_EN_MASK	  UINT16_C(0x2000)
#define SMI230_ACCEL_ANYMOTION_X_EN_SHIFT	  UINT8_C(0x0D)
#define SMI230_ACCEL_ANYMOTION_Y_EN_MASK	  UINT16_C(0x4000)
#define SMI230_ACCEL_ANYMOTION_Y_EN_SHIFT	  UINT8_C(0x0E)
#define SMI230_ACCEL_ANYMOTION_Z_EN_MASK	  UINT16_C(0x8000)
#define SMI230_ACCEL_ANYMOTION_Z_EN_SHIFT	  UINT8_C(0x0F)

#define SMI230_HIGH_G_THRESHOLD_MASK  UINT16_C(0x7FFF)
#define SMI230_HIGH_G_HYSTERESIS_MASK UINT16_C(0x0FFF)
#define SMI230_HIGH_G_X_EN_MASK	      UINT16_C(0x1000)
#define SMI230_HIGH_G_Y_EN_MASK	      UINT16_C(0x2000)
#define SMI230_HIGH_G_Z_EN_MASK	      UINT16_C(0x4000)
#define SMI230_HIGH_G_ENABLE_MASK     UINT16_C(0x8000)
#define SMI230_HIGH_G_DURATION_MASK   UINT16_C(0x0FFF)

#define SMI230_HIGH_G_THRESHOLD_POS  UINT8_C(0x00)
#define SMI230_HIGH_G_HYSTERESIS_POS UINT8_C(0x00)
#define SMI230_HIGH_G_X_EN_POS	     UINT8_C(0x0C)
#define SMI230_HIGH_G_Y_EN_POS	     UINT8_C(0x0D)
#define SMI230_HIGH_G_Z_EN_POS	     UINT8_C(0x0E)
#define SMI230_HIGH_G_ENABLE_POS     UINT8_C(0x0F)
#define SMI230_HIGH_G_DURATION_POS   UINT8_C(0x00)

#define SMI230_HIGH_G_AXIS_X_MASK	  UINT8_C(0x08)
#define SMI230_HIGH_G_AXIS_Y_MASK	  UINT8_C(0x10)
#define SMI230_HIGH_G_AXIS_Z_MASK	  UINT8_C(0x20)
#define SMI230_HIGH_G_AXIS_DIRECTION_MASK UINT8_C(0x40)

#define SMI230_LOW_G_THRESHOLD_MASK  UINT16_C(0x7FFF)
#define SMI230_LOW_G_HYSTERESIS_MASK UINT16_C(0x0FFF)
#define SMI230_LOW_G_DURATION_MASK   UINT16_C(0x0FFF)
#define SMI230_LOW_G_ENABLE_MASK     UINT16_C(0x1000)

#define SMI230_LOW_G_THRESHOLD_POS  UINT16_C(0x00)
#define SMI230_LOW_G_HYSTERESIS_POS UINT16_C(0x00)
#define SMI230_LOW_G_DURATION_POS   UINT16_C(0x00)
#define SMI230_LOW_G_ENABLE_POS	    UINT16_C(0x0C)

#define SMI230_ORIENT_ENABLE_MASK	      UINT16_C(0x0001)
#define SMI230_ORIENT_UP_DOWN_MASK	      UINT16_C(0x0002)
#define SMI230_ORIENT_SYMM_MODE_MASK	      UINT16_C(0x000C)
#define SMI230_ORIENT_BLOCK_MODE_MASK	      UINT16_C(0x0030)
#define SMI230_ORIENT_THETA_MASK	      UINT16_C(0x0FC0)
#define SMI230_ORIENT_HYST_MASK		      UINT16_C(0x07FF)
#define SMI230_ORIENT_PORTRAIT_LANDSCAPE_MASK UINT8_C(0x03)
#define SMI230_ORIENT_FACEUP_DOWN_MASK	      UINT8_C(0x04)

#define SMI230_ORIENT_ENABLE_POS	     UINT8_C(0x00)
#define SMI230_ORIENT_UP_DOWN_POS	     UINT8_C(0x01)
#define SMI230_ORIENT_SYMM_MODE_POS	     UINT8_C(0x02)
#define SMI230_ORIENT_BLOCK_MODE_POS	     UINT8_C(0x04)
#define SMI230_ORIENT_THETA_POS		     UINT8_C(0x06)
#define SMI230_ORIENT_HYST_POS		     UINT8_C(0x00)
#define SMI230_ORIENT_PORTRAIT_LANDSCAPE_POS UINT8_C(0x00)
#define SMI230_ORIENT_FACEUP_DOWN_POS	     UINT8_C(0x02)

#define SMI230_ORIENT_PORTRAIT_UPRIGHT	   UINT8_C(0x00)
#define SMI230_ORIENT_LANDSCAPE_LEFT	   UINT8_C(0x01)
#define SMI230_ORIENT_PORTRAIT_UPSIDE_DOWN UINT8_C(0x02)
#define SMI230_ORIENT_LANDSCAPE_RIGHT	   UINT8_C(0x03)
#define SMI230_ORIENT_FACE_UP		   UINT8_C(0x00)
#define SMI230_ORIENT_FACE_DOWN		   UINT8_C(0x01)

#define SMI230_NOMOTION_THRESHOLD_MASK UINT16_C(0x07FF)
#define SMI230_NOMOTION_EN_MASK	       UINT16_C(0x0800)
#define SMI230_NOMOTION_DURATION_MASK  UINT16_C(0x1FFF)
#define SMI230_NOMOTION_X_EN_MASK      UINT16_C(0x2000)
#define SMI230_NOMOTION_Y_EN_MASK      UINT16_C(0x4000)
#define SMI230_NOMOTION_Z_EN_MASK      UINT16_C(0x8000)

#define SMI230_NOMOTION_THRESHOLD_POS UINT8_C(0)
#define SMI230_NOMOTION_EN_POS	      UINT8_C(11)
#define SMI230_NOMOTION_DURATION_POS  UINT8_C(0)
#define SMI230_NOMOTION_X_EN_POS      UINT8_C(13)
#define SMI230_NOMOTION_Y_EN_POS      UINT8_C(14)
#define SMI230_NOMOTION_Z_EN_POS      UINT8_C(15)

#define SMI230_ACCEL_DATA_SYNC_LEN	  1
#define SMI230_ACCEL_DATA_SYNC_MODE_MASK  0x0007
#define SMI230_ACCEL_DATA_SYNC_MODE_SHIFT 0

#define SMI230_ACCEL_DATA_SYNC_MODE_OFF	   0x00
#define SMI230_ACCEL_DATA_SYNC_MODE_400HZ  0x01
#define SMI230_ACCEL_DATA_SYNC_MODE_1000HZ 0x02
#define SMI230_ACCEL_DATA_SYNC_MODE_2000HZ 0x03
#define SMI230_ACCEL_DATA_SYNC_MODE_100HZ  0x04
#define SMI230_ACCEL_DATA_SYNC_MODE_200HZ  0x05

#define SMI230_ACCEL_EN_MASK	  UINT8_C(0x40)
#define SMI230_ACCEL_INT1_EN_MASK UINT8_C(0x08)
#define SMI230_ACCEL_INT2_EN_MASK UINT8_C(0x04)

#define SMI230_ACCEL_EN_POS	 UINT8_C(6)
#define SMI230_ACCEL_INT1_EN_POS UINT8_C(3)
#define SMI230_ACCEL_INT2_EN_POS UINT8_C(2)

#define SMI230_ACC_FIFO_MODE	  UINT8_C(0x01)
#define SMI230_MAX_ACC_FIFO_BYTES 1024
#define SMI230_MAX_ACC_FIFO_FRAME 147
#define SMI230_FIFO_ACCEL_LENGTH  UINT8_C(6)
#define SMI230_FIFO_WTM_LENGTH	  UINT8_C(0x02)
#define SMI230_FIFO_WTM_0_ADDR	  UINT8_C(0x46)
#define SMI230_FIFO_DATA_LENGTH	  UINT8_C(0x02)
#define SMI230_SENSOR_TIME_LENGTH UINT8_C(3)
#define SMI230_FIFO_LENGTH_0_ADDR UINT8_C(0x24)
#define SMI230_FIFO_DATA_ADDR	  UINT8_C(0x26)
#define SMI230_FIFO_CONFIG_0_ADDR UINT8_C(0x48)
#define SMI230_FIFO_CONFIG_1_ADDR UINT8_C(0x49)
#define SMI230_ACCEL_EN_MASK	  UINT8_C(0x40)

#define SMI230_W_FIFO_EMPTY   INT8_C(1)
#define SMI230_W_PARTIAL_READ INT8_C(2)

#define SMI230_FIFO_INPUT_CFG_LENGTH	 UINT8_C(1)
#define SMI230_ACC_FIFO_MODE_CONFIG_MASK UINT8_C(0x01)

#define SMI230_FIFO_HEADER_ACC_FRM	 UINT8_C(0x84)
#define SMI230_FIFO_HEADER_ALL_FRM	 UINT8_C(0x9C)
#define SMI230_FIFO_HEADER_SENS_TIME_FRM UINT8_C(0x44)
#define SMI230_FIFO_HEADER_SKIP_FRM	 UINT8_C(0x40)
#define SMI230_FIFO_HEADER_INPUT_CFG_FRM UINT8_C(0x48)
#define SMI230_FIFO_HEAD_OVER_READ_MSB	 UINT8_C(0x80)
#define SMI230_FIFO_SAMPLE_DROP_FRM	 UINT8_C(0x50)

#define SMI230_FIFO_BYTE_COUNTER_MSB_MASK UINT8_C(0x3F)

#define SMI230_CONFIG_STATUS_MASK UINT8_C(0x07)

#define SMI230_SET_BITS(reg_var, bitname, val)                                 \
	((reg_var & ~(bitname##_MASK)) |                                       \
	 ((val << bitname##_POS) & bitname##_MASK))

#define SMI230_GET_BITS(reg_var, bitname)                                      \
	((reg_var & (bitname##_MASK)) >> (bitname##_POS))

#define SMI230_SET_BITS_POS_0(reg_var, bitname, val)                           \
	((reg_var & ~(bitname##_MASK)) | (val & bitname##_MASK))

#define SMI230_GET_BITS_POS_0(reg_var, bitname) (reg_var & (bitname##_MASK))
#define SMI230_SET_BIT_VAL_0(reg_var, bitname)	(reg_var & ~(bitname##_MASK))

#define SMI230_GET_DIFF(x, y) ((x) - (y))

#define SMI230_GET_LSB(var) ((uint8_t)(var & SMI230_SET_LOW_BYTE))
#define SMI230_GET_MSB(var) ((uint8_t)((var & SMI230_SET_HIGH_BYTE) >> 8))

enum smi230_intf { SMI230_I2C_INTF, SMI230_SPI_INTF };

typedef int8_t (*smi230_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr,
				    uint8_t *data, uint16_t len);

typedef void (*smi230_delay_fptr_t)(uint32_t period);

struct smi230_sensor_data {
	int16_t x;
	int16_t y;
	int16_t z;
	uint32_t sensor_time;
};

struct smi230_sensor_data_f {
	float x;
	float y;
	float z;
	double sensor_time;
};

struct smi230_cfg {
	uint8_t power;
	uint8_t range;
	uint8_t bw;
	uint8_t odr;
};

struct smi230_err_reg {
	uint8_t fatal_err;
	uint8_t err_code;
};

struct smi230_anymotion_cfg {
	/* 11 bit threshold of anymotion detection
	 * (threshold = X mg * 2,048 (5.11 format))
	 */
	uint16_t threshold;
	uint8_t enable;

	/* 13 bit set the duration for any- and nomotion
	 * (time = duration * 20ms (@50Hz))
	 */
	uint16_t duration;
	uint8_t x_en;
	uint8_t y_en;
	uint8_t z_en;
};

struct smi230_data_sync_cfg {
	/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */
	uint8_t mode;
};

struct smi230_high_g_cfg {
	uint16_t threshold;
	uint16_t hysteresis;
	uint16_t duration;
	uint8_t x_en;
	uint8_t y_en;
	uint8_t z_en;
	uint8_t enable;
};

struct smi230_low_g_cfg {
	uint16_t threshold;
	uint16_t hysteresis;
	uint16_t duration;
	uint8_t enable;
};

struct smi230_orient_cfg {
	uint8_t ud_en;
	uint8_t mode;
	uint8_t blocking;
	uint8_t theta;
	uint16_t hysteresis;
	uint8_t enable;
};

struct smi230_nomotion_cfg {
	uint16_t duration;
	uint16_t threshold;
	uint8_t x_en;
	uint8_t y_en;
	uint8_t z_en;
	uint8_t enable;
};

enum smi230_accel_int_channel { SMI230_INT_CHANNEL_1, SMI230_INT_CHANNEL_2 };

enum smi230_accel_int_types {
	SMI230_ACCEL_NO_INT,
	SMI230_ACCEL_DATA_RDY_INT,
	SMI230_ACCEL_FIFO_WM_INT,
	SMI230_ACCEL_FIFO_FULL_INT,
	SMI230_ACCEL_SYNC_INPUT,
	SMI230_ACCEL_DATA_SYNC_INT
};

struct smi230_int_pin_cfg {
	/*! interrupt pin level configuration
	 * Assignable macros :
	 * - SMI230_INT_ACTIVE_LOW
	 * - SMI230_INT_ACTIVE_HIGH
	 */
	uint8_t lvl;

	/*! interrupt pin mode configuration
	 * Assignable macros :
	 * - SMI230_INT_MODE_PUSH_PULL
	 * - SMI230_INT_MODE_OPEN_DRAIN
	 */
	uint8_t output_mode;

	/*! Set pint as input
	 * Assignable Macros :
	 * - SMI230_ENABLE
	 * - SMI230_DISABLE
	 */
	uint8_t input_en;

	/*! Set pin as output
	 * Assignable Macros :
	 * - SMI230_ENABLE
	 * - SMI230_DISABLE
	 */
	uint8_t output_en;
};

struct smi230_accel_int_channel_cfg {
	uint8_t int_features;
	enum smi230_accel_int_types int_type;
	struct smi230_int_pin_cfg int_pin_cfg;
};

struct smi230_acc_dev {
	uint8_t accel_chip_id;
	uint8_t accel_id;
	enum smi230_intf intf;
	uint8_t dummy_byte;
	struct smi230_cfg accel_cfg;
	struct smi230_accel_int_channel_cfg int_channel_1;
	struct smi230_accel_int_channel_cfg int_channel_2;
	const uint8_t *config_file_ptr;
	uint8_t read_write_len;
	smi230_com_fptr_t read;
	smi230_com_fptr_t write;
	struct smi230_anymotion_cfg anymotion_cfg;
	struct smi230_nomotion_cfg nomotion_cfg;
	struct smi230_high_g_cfg high_g_cfg;
	struct smi230_low_g_cfg low_g_cfg;
	struct smi230_orient_cfg orient_cfg;
	struct smi230_data_sync_cfg sync_cfg;
	smi230_delay_fptr_t delay_ms;
};

void smi230_delay(uint32_t msec);

int smi230_acc_probe(struct device *dev, struct smi230_acc_dev *smi230_dev);

int smi230_acc_remove(struct device *dev);

int8_t smi230_acc_init(struct smi230_acc_dev *dev);

#endif /* _SMI230_ACC_H */
