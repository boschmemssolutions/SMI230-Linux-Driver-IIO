# SMI230 IIO Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>

SMI230 is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements.

_Note: SMI230 was originally implemented as a input deveice driver. The SMI230 input driver is still avaiable in [github](https://github.com/boschmemssolutions/SMI230-Linux-Driver). Since SMI230 is rather an Industrial I/O (IIO) than an input device. We now implement SMI230 as an IIO driver. Therefore the further development will only happen on the IIO driver._

_Note: The sensor driver utilizes sensor api, which is following BMI08x sensor api available on [github](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.4.4)._

_Note: The data synchronization feature utilizes sensor configuration, which is following BMI08x sensor configuration available on [github](https://github.com/BoschSensortec/BMI08x-Sensor-API/releases/tag/bmi08x_v1.2.0)._

## Documentation <a name=Doc></a>

https://boschmemssolutions.github.io/iio/bosch_smi230_IIO.html

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

# read acc power mode 
cat /sys/bus/iio/devices/iio:device0/in_accel_power_mode

# set acc power mode
sudo su  
echo  [normal|suspend] > /sys/bus/iio/devices/iio:device0/in_accel_power_mode

# read the avaiable acc output data rate 
cat /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency_available    

# read the current acc output data rate 
cat /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency

# set the acc output data rate.
sudo su  
echo [12.5|25|50|100|200|400|800|1600] > /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency

# read acc x raw value
cat  /sys/bus/iio/devices/iio:device0/in_accel_x_raw

# read acc xyz and sensor time raw value
cat  /sys/bus/iio/devices/iio:device0/in_accel_raw

# read acc range
cat  /sys/bus/iio/devices/iio:device0/in_accel_range

# set acc range
sudo su  
echo [2|4|8|16] > /sys/bus/iio/devices/iio:device0/in_accel_range

# execute acc self test acc power will be suspened after self test. you need to manually set acc power mode to normal
cat  /sys/bus/iio/devices/iio:device0/self_test

# execute acc soft reset. acc power will be suspened after soft reset. you need to manually set acc power mode to normal
sudo su  
echo 1 >  /sys/bus/iio/devices/iio:device0/soft_reset

# read acc temprature
cat  /sys/bus/iio/devices/iio:device0/in_temp_accel


# read gyro power mode 
cat /sys/bus/iio/devices/iio:device1/pwr_cfg

# set gyro power mode
sudo su  
echo  [0:active|1:suspend|2:deep suspend] > /sys/bus/iio/devices/iio:device1/in_accel_power_mode

# read gyro range 
cat /sys/bus/iio/devices/iio:device1/range

# set the gyro range
sudo su  
echo [125|250|500|1000|2000] > /sys/bus/iio/devices/iio:device1/range

# read the avaiable gyro output data rate 
cat  /sys/bus/iio/devices/iio:device1/sampling_frequency_available    

# read the current gyro output data rate 
cat  /sys/bus/iio/devices/iio:device1/in_anglvel_sampling_frequency

# set the gyro output data rate.
sudo su  
echo [100|200|400|1000|2000] >  /sys/bus/iio/devices/iio:device1/in_anglvel_sampling_frequency

# read gyro x raw value
cat  /sys/bus/iio/devices/iio:device1/in_anglvel_x_raw

# read gyro xyz raw value
cat  /sys/bus/iio/devices/iio:device1/in_anglvel_raw

# execute self test
cat  /sys/bus/iio/devices/iio:device1/selftest

# execute soft reset
cat  /sys/bus/iio/devices/iio:device1/softreset

```
