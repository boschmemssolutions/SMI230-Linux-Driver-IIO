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
#define INT8_C(x)   S8_C(x)
#define UINT8_C(x)  U8_C(x)
#endif

#ifndef NULL
#ifdef __cplusplus
#define NULL                 0
#else
#define NULL                 ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE                 UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE                UINT8_C(0)
#endif

/* Gyro registers */
#define SMI230_GYRO_CHIP_ID_REG                UINT8_C(0x00)
#define SMI230_GYRO_X_LSB_REG                  UINT8_C(0x02)
#define SMI230_GYRO_X_MSB_REG                  UINT8_C(0x03)
#define SMI230_GYRO_Y_LSB_REG                  UINT8_C(0x04)
#define SMI230_GYRO_Y_MSB_REG                  UINT8_C(0x05)
#define SMI230_GYRO_Z_LSB_REG                  UINT8_C(0x06)
#define SMI230_GYRO_Z_MSB_REG                  UINT8_C(0x07)
#define SMI230_GYRO_INT_STAT_1_REG             UINT8_C(0x0A)
#define SMI230_GYRO_RANGE_REG                  UINT8_C(0x0F)
#define SMI230_GYRO_BANDWIDTH_REG              UINT8_C(0x10)
#define SMI230_GYRO_LPM1_REG                   UINT8_C(0x11)
#define SMI230_GYRO_SOFTRESET_REG              UINT8_C(0x14)
#define SMI230_GYRO_INT_CTRL_REG               UINT8_C(0x15)
#define SMI230_GYRO_INT3_INT4_IO_CONF_REG      UINT8_C(0x16)
#define SMI230_GYRO_INT3_INT4_IO_MAP_REG       UINT8_C(0x18)
#define SMI230_GYRO_WM_INT_REG                 UINT8_C(0x1E)
#define SMI230_GYRO_FIFO_EXT_INT_S_REG		UINT8_C(0x34)
#define SMI230_GYRO_SELF_TEST_REG              UINT8_C(0x3C)
#define SMI230_GYRO_CHIP_ID                    UINT8_C(0x0F)
#define SMI230_GYRO_I2C_ADDR_PRIMARY           UINT8_C(0x68)
#define SMI230_GYRO_I2C_ADDR_SECONDARY         UINT8_C(0x69)
#define SMI230_GYRO_RANGE_2000_DPS             UINT8_C(0x00)
#define SMI230_GYRO_RANGE_1000_DPS             UINT8_C(0x01)
#define SMI230_GYRO_RANGE_500_DPS              UINT8_C(0x02)
#define SMI230_GYRO_RANGE_250_DPS              UINT8_C(0x03)
#define SMI230_GYRO_RANGE_125_DPS              UINT8_C(0x04)
#define SMI230_GYRO_BW_523_ODR_2000_HZ         UINT8_C(0x00)
#define SMI230_GYRO_BW_230_ODR_2000_HZ         UINT8_C(0x01)
#define SMI230_GYRO_BW_116_ODR_1000_HZ         UINT8_C(0x02)
#define SMI230_GYRO_BW_47_ODR_400_HZ           UINT8_C(0x03)
#define SMI230_GYRO_BW_23_ODR_200_HZ           UINT8_C(0x04)
#define SMI230_GYRO_BW_12_ODR_100_HZ           UINT8_C(0x05)
#define SMI230_GYRO_BW_64_ODR_200_HZ           UINT8_C(0x06)
#define SMI230_GYRO_BW_32_ODR_100_HZ           UINT8_C(0x07)
#define SMI230_GYRO_ODR_RESET_VAL              UINT8_C(0x80)
#define SMI230_GYRO_PM_NORMAL                  UINT8_C(0x00)
#define SMI230_GYRO_PM_DEEP_SUSPEND            UINT8_C(0x20)
#define SMI230_GYRO_PM_SUSPEND                 UINT8_C(0x80)
#define SMI230_GYRO_DRDY_INT_DISABLE_VAL       UINT8_C(0x00)
#define SMI230_GYRO_DRDY_INT_ENABLE_VAL        UINT8_C(0x80)
#define SMI230_GYRO_FIFO_INT_DISABLE_VAL       UINT8_C(0x00)
#define SMI230_GYRO_FIFO_INT_ENABLE_VAL        UINT8_C(0x40)
#define SMI230_GYRO_MAP_DRDY_TO_INT3           UINT8_C(0x01)
#define SMI230_GYRO_MAP_DRDY_TO_INT4           UINT8_C(0x80)
#define SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4 UINT8_C(0x81)
#define SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4 UINT8_C(0x24)
#define SMI230_GYRO_SOFTRESET_DELAY            UINT8_C(30)
#define SMI230_GYRO_POWER_MODE_CONFIG_DELAY    UINT8_C(30)
#define SMI230_GYRO_RANGE_MASK                 UINT8_C(0x07)
#define SMI230_GYRO_BW_MASK                    UINT8_C(0x0F)
#define SMI230_GYRO_POWER_MASK                 UINT8_C(0xA0)
#define SMI230_GYRO_POWER_POS                  UINT8_C(5)
#define SMI230_GYRO_DATA_EN_MASK               UINT8_C(0x80)
#define SMI230_GYRO_DATA_EN_POS                UINT8_C(7)
#define SMI230_GYRO_FIFO_EN_MASK               UINT8_C(0x40)
#define SMI230_GYRO_FIFO_EN_POS                UINT8_C(6)
#define SMI230_GYRO_INT3_LVL_MASK              UINT8_C(0x01)
#define SMI230_GYRO_INT3_OD_MASK               UINT8_C(0x02)
#define SMI230_GYRO_INT4_LVL_MASK              UINT8_C(0x04)
#define SMI230_GYRO_INT4_OD_MASK               UINT8_C(0x08)
#define SMI230_GYRO_INT3_OD_POS                UINT8_C(1)
#define SMI230_GYRO_INT4_LVL_POS               UINT8_C(2)
#define SMI230_GYRO_INT4_OD_POS                UINT8_C(3)
#define SMI230_GYRO_INT_EN_MASK                UINT8_C(0x80)
#define SMI230_GYRO_INT_EN_POS                 UINT8_C(7)
#define SMI230_GYRO_INT3_MAP_MASK              UINT8_C(0x01)
#define SMI230_GYRO_INT4_MAP_MASK              UINT8_C(0x80)
#define SMI230_GYRO_FIFO_INT3_MAP_MASK              UINT8_C(0x04)
#define SMI230_GYRO_FIFO_INT4_MAP_MASK              UINT8_C(0x20)
#define SMI230_GYRO_INT3_MAP_POS               UINT8_C(0)
#define SMI230_GYRO_INT4_MAP_POS               UINT8_C(7)
#define SMI230_GYRO_FIFO_INT3_MAP_POS               UINT8_C(2)
#define SMI230_GYRO_FIFO_INT4_MAP_POS               UINT8_C(5)
#define SMI230_GYRO_SELF_TEST_EN_MASK          UINT8_C(0x01)
#define SMI230_GYRO_SELF_TEST_RDY_MASK         UINT8_C(0x02)
#define SMI230_GYRO_SELF_TEST_RESULT_MASK      UINT8_C(0x04)
#define SMI230_GYRO_SELF_TEST_FUNCTION_MASK    UINT8_C(0x08)
#define SMI230_GYRO_SELF_TEST_RDY_POS          UINT8_C(1)
#define SMI230_GYRO_SELF_TEST_RESULT_POS       UINT8_C(2)
#define SMI230_GYRO_SELF_TEST_FUNCTION_POS     UINT8_C(3)

#define SMI230_INT_MODE_PUSH_PULL              UINT8_C(0)
#define SMI230_INT_MODE_OPEN_DRAIN             UINT8_C(1)
#define SMI230_INT_ACTIVE_LOW                  UINT8_C(0)
#define SMI230_INT_ACTIVE_HIGH                 UINT8_C(1)

#define SMI230_SPI_RD_MASK                     UINT8_C(0x80)
#define SMI230_SPI_WR_MASK                     UINT8_C(0x7F)

#define SMI230_OK                              INT8_C(0)
#define SMI230_E_NULL_PTR                      INT8_C(-1)
#define SMI230_E_COM_FAIL                      INT8_C(-2)
#define SMI230_E_DEV_NOT_FOUND                 INT8_C(-3)
#define SMI230_E_OUT_OF_RANGE                  INT8_C(-4)
#define SMI230_E_INVALID_INPUT                 INT8_C(-5)
#define SMI230_E_CONFIG_STREAM_ERROR           INT8_C(-6)
#define SMI230_E_RD_WR_LENGTH_INVALID          INT8_C(-7)
#define SMI230_E_INVALID_CONFIG                INT8_C(-8)
#define SMI230_E_FEATURE_NOT_SUPPORTED         INT8_C(-9)

#define SMI230_DISABLE                         UINT8_C(0)
#define SMI230_ENABLE                          UINT8_C(1)

#define SMI230_SET_BITS(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     ((val << bitname##_POS) & bitname##_MASK))

#define SMI230_GET_BITS(reg_var, bitname)       ((reg_var & (bitname##_MASK)) >> \
                                                 (bitname##_POS))

#define SMI230_SET_BITS_POS_0(reg_var, bitname, val) \
    ((reg_var & ~(bitname##_MASK)) | \
     (val & bitname##_MASK))

#define SMI230_GET_BITS_POS_0(reg_var, bitname) (reg_var & (bitname##_MASK))

#define SMI230_SET_BIT_VAL_0(reg_var, bitname)  (reg_var & ~(bitname##_MASK))





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

enum smi230_gyro_int_channel {
    SMI230_INT_CHANNEL_3,
    SMI230_INT_CHANNEL_4
};

enum smi230_gyro_int_types {
    SMI230_GYRO_DATA_RDY_INT,
    SMI230_GYRO_FIFO_INT
};

struct smi230_int_pin_cfg
{
    uint8_t lvl : 1;
    uint8_t output_mode : 1;
    uint8_t enable_int_pin : 1;
};

struct smi230_gyro_int_channel_cfg
{
    enum smi230_gyro_int_channel int_channel;
    enum smi230_gyro_int_types int_type;
    struct smi230_int_pin_cfg int_pin_cfg;
};

struct smi230_int_cfg
{
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

struct smi230_sensor_data
{
    int16_t x;
    int16_t y;
    int16_t z;
};

static int8_t null_ptr_check(const struct smi230_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_ms == NULL))
    {
        rslt = SMI230_E_NULL_PTR;
    }
    else
    {
        rslt = SMI230_OK;
    }

    return rslt;
}

static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr | SMI230_SPI_RD_MASK);
    }

    /* read a gyro register */
    rslt = dev->read(dev->gyro_id, reg_addr, reg_data, len);

    if (rslt != SMI230_OK)
    {
        /* Updating the error */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct smi230_dev *dev)
{
    int8_t rslt;

    if (dev->intf == SMI230_SPI_INTF)
    {
        /* Configuring reg_addr for SPI Interface */
        reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
    }

    /* write to a gyro register */
    rslt = dev->write(dev->gyro_id, reg_addr, reg_data, len);

    if (rslt != SMI230_OK)
    {
        /* Updating the error */
        rslt = SMI230_E_COM_FAIL;
    }

    return rslt;
}

int8_t smi230_gyro_chip_id_check(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t chip_id = 0;

    rslt = null_ptr_check(dev);

    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_GYRO_CHIP_ID_REG, &chip_id, 1, dev);

        if (rslt == SMI230_OK)
        {
            if (chip_id == SMI230_GYRO_CHIP_ID)
            {
                dev->gyro_chip_id = chip_id;
            }
            else
            {
                rslt = SMI230_E_DEV_NOT_FOUND;
            }
        }
    }

    return rslt;
}

int8_t smi230_gyro_get_meas_conf(struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[2];

    rslt = null_ptr_check(dev);

    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_GYRO_RANGE_REG, data, 2, dev);

        if (rslt == SMI230_OK)
        {
            dev->gyro_cfg.range = data[0];
            dev->gyro_cfg.odr = (data[1] & SMI230_GYRO_BW_MASK);
            dev->gyro_cfg.bw = dev->gyro_cfg.odr;
        }
    }

    return rslt;
}

int8_t smi230_gyro_set_meas_conf(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;
    uint8_t odr, range;
    uint8_t is_range_invalid = FALSE, is_odr_invalid = FALSE;

    rslt = null_ptr_check(dev);

    if (rslt == SMI230_OK)
    {
        odr = dev->gyro_cfg.odr;
        range = dev->gyro_cfg.range;

        if (odr > SMI230_GYRO_BW_32_ODR_100_HZ)
        {
            is_odr_invalid = TRUE;
        }

        if (range > SMI230_GYRO_RANGE_125_DPS)
        {
            is_range_invalid = TRUE;
        }

        if ((!is_odr_invalid) && (!is_range_invalid))
        {
            rslt = get_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1, dev);
            if (rslt == SMI230_OK)
            {
                data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_BW, odr);
                rslt = set_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1, dev);
                if (rslt == SMI230_OK)
                {
                    rslt = get_regs(SMI230_GYRO_RANGE_REG, &data, 1, dev);
                    if (rslt == SMI230_OK)
                    {
                        data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_RANGE, range);
                        rslt = set_regs(SMI230_GYRO_RANGE_REG, &data, 1, dev);
                    }
                }
            }

        }
        else
        {
            rslt = SMI230_E_INVALID_CONFIG;
        }
    }

    return rslt;
}

static int8_t set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data;

    rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);

    if (rslt == SMI230_OK)
    {
        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_3:
                data = SMI230_SET_BITS_POS_0(data, SMI230_GYRO_INT3_LVL, int_config->int_pin_cfg.lvl);
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT3_OD, int_config->int_pin_cfg.output_mode);
                break;

            case SMI230_INT_CHANNEL_4:
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_LVL, int_config->int_pin_cfg.lvl);
                data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_OD, int_config->int_pin_cfg.output_mode);
                break;

            default:
                break;
        }

        rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);
    }

    return rslt;
}

static int8_t set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;

        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_3:
                data[0] = SMI230_SET_BITS_POS_0(data[0], SMI230_GYRO_INT3_MAP, conf);
                break;

            case SMI230_INT_CHANNEL_4:
                data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_INT4_MAP, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            /*condition to check disabling the interrupt in single channel when both
             * interrupts channels are enabled*/
            if (data[0] & SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4)
            {
                data[1] = SMI230_GYRO_DRDY_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;
            }

            rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);
            if (rslt == SMI230_OK)
            {
                rslt = set_int_pin_config(int_config, dev);
                if (rslt == SMI230_OK)
                    rslt = set_regs(SMI230_GYRO_INT_CTRL_REG, &data[1], 1, dev);
            }
        }

    }

    return rslt;
}

static int8_t set_gyro_fifo_int(const struct smi230_gyro_int_channel_cfg *int_config,
                                      const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t conf, data[2] = { 0 };

    rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);
    if (rslt == SMI230_OK)
    {
        conf = int_config->int_pin_cfg.enable_int_pin;
        switch (int_config->int_channel)
        {
            case SMI230_INT_CHANNEL_3:
                data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_FIFO_INT3_MAP, conf);
                break;

            case SMI230_INT_CHANNEL_4:
                data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_FIFO_INT4_MAP, conf);
                break;

            default:
                rslt = SMI230_E_INVALID_INPUT;
                break;
        }

        if (rslt == SMI230_OK)
        {
            if (data[0] & SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4)
            {
                data[1] = SMI230_GYRO_FIFO_INT_ENABLE_VAL;
            }
            else
            {
                data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;
            }

            rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);
            if (rslt == SMI230_OK)
            {
                rslt = set_int_pin_config(int_config, dev);
                if (rslt == SMI230_OK)
                {
                    rslt = set_regs(SMI230_GYRO_INT_CTRL_REG, &data[1], 1, dev);
                }

            }
        }

    }

    return rslt;
}

int8_t smi230_gyro_set_int_config(const struct smi230_gyro_int_channel_cfg *int_config, const struct smi230_dev *dev)
{
    int8_t rslt;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (int_config != NULL))
    {
        switch (int_config->int_type)
        {
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
    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

int8_t smi230_gyro_get_data(struct smi230_sensor_data *gyro, const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t data[6];
    uint8_t lsb, msb;
    uint16_t msblsb;

    rslt = null_ptr_check(dev);
    if ((rslt == SMI230_OK) && (gyro != NULL))
    {
        rslt = get_regs(SMI230_GYRO_X_LSB_REG, data, 6, dev);
        if (rslt == SMI230_OK)
        {
            lsb = data[0];
            msb = data[1];
            msblsb = (msb << 8) | lsb;
            gyro->x = (int16_t)msblsb; /* Data in X axis */

            lsb = data[2];
            msb = data[3];
            msblsb = (msb << 8) | lsb;
            gyro->y = (int16_t)msblsb; /* Data in Y axis */

            lsb = data[4];
            msb = data[5];
            msblsb = (msb << 8) | lsb;
            gyro->z = (int16_t)msblsb; /* Data in Z axis */
        }

    }
    else
    {
        rslt = SMI230_E_NULL_PTR;
    }

    return rslt;
}

int8_t smi230_gyro_set_power_mode(const struct smi230_dev *dev)
{
    int8_t rslt;
    uint8_t power_mode, data;
    uint8_t is_power_switching_mode_valid = TRUE;

    rslt = null_ptr_check(dev);
    if (rslt == SMI230_OK)
    {
        rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);
        if (rslt == SMI230_OK)
        {
            power_mode = dev->gyro_cfg.power;

            /*switching between normal mode and the suspend modes is allowed, it is not possible to switch
             * between suspend and deep suspend and vice versa. Check for invalid power switching (i.e)
             * deep suspend to suspend */
            if ((power_mode == SMI230_GYRO_PM_SUSPEND) && (data == SMI230_GYRO_PM_DEEP_SUSPEND))
                is_power_switching_mode_valid = FALSE;

            /* Check for invalid power switching (i.e) from suspend to deep suspend */
            if ((power_mode == SMI230_GYRO_PM_DEEP_SUSPEND) && (data == SMI230_GYRO_PM_SUSPEND))
                is_power_switching_mode_valid = FALSE;

            /* Check if power switching mode is valid*/
            if (is_power_switching_mode_valid)
            {
                rslt = set_regs(SMI230_GYRO_LPM1_REG, &power_mode, 1, dev);
                if (rslt == SMI230_OK)
                    dev->delay_ms(SMI230_GYRO_POWER_MODE_CONFIG_DELAY);

            }
            else
                rslt = SMI230_E_INVALID_INPUT;
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
	struct smi230_sensor_data data = {0};

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = smi230_gyro_get_data(&data, p_smi230_dev);
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
		ret = smi230_gyro_get_meas_conf(p_smi230_dev);
		if (ret)
			return ret;
		switch(p_smi230_dev->gyro_cfg.odr) {
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
	    struct iio_chan_spec const *chan,
	    int val, int val2, long mask)
{
	int ret;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch(val) {
		case 2000:
			p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_523_ODR_2000_HZ;
			break;
		case 1000:
			p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_116_ODR_1000_HZ;
			break;
		case 400:
			p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_47_ODR_400_HZ;
			break;
		case 200:
			p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_64_ODR_200_HZ;
			break;
		case 100:
			p_smi230_dev->gyro_cfg.odr = SMI230_GYRO_BW_32_ODR_100_HZ;
			break;
		default:
			return -EINVAL;
		}

		ret = smi230_gyro_set_meas_conf(p_smi230_dev);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;

}

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("100 200 400 1000 2000");

static IIO_CONST_ATTR(in_anglvel_scale_available,
                      "0.008 0.004 0.002 0.001 0.0005");

static struct attribute *smi230_attrs[] = {
        &iio_const_attr_sampling_frequency_available.dev_attr.attr,
        &iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
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

static struct smi230_int_cfg int_config;

static int smi230_gyro_init(struct smi230_dev *dev)
{
	int err = 0;

	err = smi230_gyro_chip_id_check(dev);

	dev->gyro_cfg.power = SMI230_GYRO_PM_NORMAL;
	err |= smi230_gyro_set_power_mode(dev);

	int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = SMI230_DISABLE;
	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = SMI230_ENABLE;

	dev->gyro_cfg.odr = SMI230_GYRO_BW_32_ODR_100_HZ;
	dev->gyro_cfg.range = SMI230_GYRO_RANGE_125_DPS;

	err |= smi230_gyro_set_meas_conf(dev);
	smi230_delay(100);

	int_config.gyro_int_config_2.int_channel = SMI230_INT_CHANNEL_4;
	int_config.gyro_int_config_2.int_type = SMI230_GYRO_DATA_RDY_INT;
	int_config.gyro_int_config_2.int_pin_cfg.output_mode = SMI230_INT_MODE_PUSH_PULL;
	int_config.gyro_int_config_2.int_pin_cfg.lvl = SMI230_INT_ACTIVE_HIGH;

	err |= smi230_gyro_set_int_config(&int_config.gyro_int_config_2, dev);

	smi230_delay(100);

	return err;
}

static int smi230_read_channel(struct smi230_sensor_data *data, int i, s16 *sample)
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

static irqreturn_t smi230_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);

	/* hand code the buffer size for now */
	s16 buf[8];
	s16 sample;
	int ret, i, j = 0;

	struct smi230_sensor_data sensor_data = {0};
	ret = smi230_gyro_get_data(&sensor_data, p_smi230_dev);
	if (ret) {
		dev_dbg(indio_dev->dev.parent, "Reading sensor data failed");
		goto done;
	}

	for_each_set_bit(i, indio_dev->active_scan_mask, indio_dev->masklength) {
		ret = smi230_read_channel(&sensor_data, i, &sample);
		if (ret) {
			dev_dbg(indio_dev->dev.parent, "Read channel %d failed", i);
			goto done;
		}
		buf[j++] = sample;
	}


	ret = iio_push_to_buffers_with_timestamp(indio_dev, buf, pf->timestamp);
	if (ret)
		dev_dbg(indio_dev->dev.parent, "Push to buffer failed");
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int smi230_new_data_trigger_set_state(struct iio_trigger *trig, bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct smi230_dev *p_smi230_dev = iio_device_get_drvdata(indio_dev);
	u8 en;

	dev_dbg(indio_dev->dev.parent, "trigger set state %d", enable);
	if (enable)
		en = SMI230_ENABLE;
	else
		en = SMI230_DISABLE;

	int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = en;
	return smi230_gyro_set_int_config(&int_config.gyro_int_config_2, p_smi230_dev);
}

static const struct iio_trigger_ops smi230_trigger_ops = {
	.set_trigger_state = &smi230_new_data_trigger_set_state,
};

static int smi230_get_irq(struct device *dev, int *irq)
{
	int gpio_pin, ret;

	gpio_pin = of_get_named_gpio_flags(dev->of_node,
					"gpio_irq", 0, NULL);

	dev_dbg(dev, "gpio pin %d", gpio_pin);
	ret = gpio_request_one(gpio_pin,
				GPIOF_IN, "smi230_gyro_interrupt");
	if (ret) {
		dev_dbg(dev, "Request GPIO pin %d failed", gpio_pin);
		return ret;
	}

	ret = gpio_direction_input(gpio_pin);
	if (ret)
		return ret;

	*irq = gpio_to_irq(gpio_pin);


	return ret;
}

int smi230_gyro_core_probe(struct device *dev, struct smi230_dev *p_smi230_dev)
{
	int ret = 0;
	int irq;
	struct iio_dev *indio_dev;

	ret = smi230_gyro_init(p_smi230_dev);
        if (ret == SMI230_OK)
		dev_dbg(dev, "Bosch Sensor %s hardware initialized", SENSOR_GYRO_NAME);
	else {
		dev_dbg(dev, "Bosch Sensor %s hardware initialization failed, error %d",
				SENSOR_GYRO_NAME, ret);
	}

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev) {
		dev_dbg(dev, "Bosch Sensor %s iio device alloc failed", SENSOR_GYRO_NAME);
		return -ENOMEM;
	}

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

	dev_dbg(dev, "Bosch Sensor %s device alloced", SENSOR_GYRO_NAME);

	ret = smi230_get_irq(dev, &irq);
	dev_dbg(dev, "irq number %d", irq);
	ret = devm_request_irq(&indio_dev->dev, irq,
				   &iio_trigger_generic_data_rdy_poll,
				   IRQF_TRIGGER_RISING, indio_dev->name,
				   indio_dev->trig);
	if (ret)
		return ret;

	dev_dbg(dev, "Bosch Sensor %s irq alloced", SENSOR_GYRO_NAME);

	indio_dev->trig->dev.parent = dev;
	indio_dev->trig->ops = &smi230_trigger_ops;

	iio_trigger_set_drvdata(indio_dev->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, indio_dev->trig);
	if (ret)
		return ret;

	dev_dbg(dev, "Bosch Sensor %s trigger registered", SENSOR_GYRO_NAME);

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
							  iio_pollfunc_store_time,
							  smi230_trigger_handler,
							  NULL);
	if (ret) {
		dev_dbg(dev, "Setup triggered buffer failed");
		return ret;
	}

	dev_dbg(dev, "Bosch Sensor %s trigger buffer registered", SENSOR_GYRO_NAME);

	return devm_iio_device_register(dev, indio_dev);
}

MODULE_DESCRIPTION("SMI230 GYRO SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
