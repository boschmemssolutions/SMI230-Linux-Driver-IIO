#
# Makefile for Bosch SMI230 Gyro.
#

obj-$(CONFIG_SMI230_GYRO) += smi230_gyro.o
smi230_gyro-objs := smi230_gyro_core.o
ifeq ($(CONFIG_SMI230_GYRO_I2C),y)
        smi230_gyro-objs += smi230_gyro_i2c.o
else        
        smi230_gyro-objs += smi230_gyro_spi.o
endif
