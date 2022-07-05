#
# Makefile for Bosch SMI230 Acc
#

obj-$(CONFIG_IIO_SMI230_ACC) += smi230_acc.o
smi230_acc-objs := smi230_acc_core.o

ifeq ($(CONFIG_IIO_SMI230_ACC_I2C),y)
	smi230_acc-objs += smi230_acc_i2c.o
else
	smi230_acc-objs += smi230_acc_spi.o
endif
