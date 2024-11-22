# SMI230 Sensor Linux IIO Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)

## Introduction <a name=Intro></a>

The SMI230 is a combined triaxial accelerometer (ACC) and triaxial gyroscope (GYR) for non-safety related applications, e.g. for in-dash navigation in the passenger compartment. Within one package, the SMI230 offers the detection of acceleration and angular rate for the x-, y-, and z-axis. The digital standard serial peripheral interface (SPI) of the SMI230 allows for bi-directional data transmission.

## Documentation <a name=Doc></a>

https://boschmemssolutions.github.io/iio/bosch_smi230_IIO.html

## License <a name=License></a>

See [LICENSE](drivers/iio/imu/smi230/LICENSE.md) file

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
                IMU-subsystem
                      |
                iio-subsystem
                      |
sensor_API <-- smi230_driver --> smi230_SPI/I2C_driver
                                           |
                                      SPI/I2C_bus
                                           |
-------------------------------------------------------
                  Hardware
```