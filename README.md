# SMI230 IIO Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>

SMI230 is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements.
Due to system-in-package approach of SMI230 (two sensors in single package), the gyroscope and acceleration data is acquired in a non-synchronized manner. 
However, synchronization between accelerometer and gyroscope can be achieved:
The software modules in this repository are provided as reference for SMI230 users and shall demonstrate exemplarily the usage of the following features
- data synchronization.
- data collection from FIFO.

_Note: SMI230 was originally implemented as a input deveice driver. The SMI230 input driver is still avaiable in [github](https://github.com/boschmemssolutions/SMI230-Linux-Driver). Since SMI230 is rather an Industrial I/O (IIO) than an input device. We now implement SMI230 as an IIO driver. Therefore the further development will only happen on the IIO driver._

_Note: The sensor driver utilizes sensor api, which is following BMI08x sensor api available on [github](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.4.4)._

_Note: The data synchronization feature utilizes sensor configuration, which is following BMI08x sensor configuration available on [github](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.2.0)._

## License <a name=License></a>
See [LICENSE](drivers/iio/LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* I2C
* SPI

## Architecture <a name=Architecture></a>
```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
                iio-subsystem
	             |
sensor_API <-- smi230_driver --> smi230_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```
## Operation examples <a name=examples></a>
1. Userspace
The driver exposes device file node under /dev/iio_device:0 and /dev/iio_device:1

2. Sysfs
The driver also exposes a set of sysfs nodes under /sys/bus/iio/devices/iio_device:0, iio_device:1, trigger0, trigger1, where users can get information about the sensor and also control the sensor. Eg.:
```
# device number 0 and 1 are dynamically assingn to accel and gyro. 
# to find out which deveice you are working with, you can read the device name
cat /sys/bus/iio/devices/iio:device0/name
cat /sys/bus/iio/devices/iio:device1/name


# read the avaiable acc output data rate 
cat /sys/bus/iio/devices/iio:device1/in_accel_sampling_frequency_available    

# read the current acc output data rate 
cat /sys/bus/iio/devices/iio:device1/in_accel_sampling_frequency

# set the acc output data rate.
sudo su 
echo 200 > /sys/bus/iio/devices/iio:device1/in_accel_sampling_frequency

# read acc x raw value
cat  /sys/bus/iio/devices/iio:device1/in_accel_x_raw


# read the avaiable gyro output data rate 
cat  /sys/bus/iio/devices/iio:device0/sampling_frequency_available    

# read the current gyro output data rate 
cat  /sys/bus/iio/devices/iio:device0/in_anglvel_sampling_frequency

# set the gyro output data rate.
sudo su 
echo 400 >  /sys/bus/iio/devices/iio:device0/in_anglvel_sampling_frequency

# read gyro x raw value
cat  /sys/bus/iio/devices/iio:device0/in_anglvel_x_raw


```
