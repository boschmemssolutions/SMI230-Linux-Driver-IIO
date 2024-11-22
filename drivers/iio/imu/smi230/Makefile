# SPDX-License-Identifier: GPL-2.0-only
#
# SMI230-IIO driver
#

obj-$(CONFIG_SMI230) += smi230.o
smi230-objs += smi230acc_core.o
smi230-objs += smi230gyro_core.o

ifeq ($(CONFIG_SMI230_I2C),y)
    smi230-objs += smi230_i2c.o
else
    smi230-objs += smi230_spi.o
endif
