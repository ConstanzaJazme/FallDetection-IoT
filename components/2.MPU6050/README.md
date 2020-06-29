# Important things

## 1. Libraries I2C


To calibrate and manage the "MPU6050" you need to install the libraries "I2Cdev.h" and "MPU6050.h"

>To install the libraries you must clone the following repositories inside the libraries folder inside the arduino root folder

>- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
>- https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev

## 2. MPU6050 Calibration Sketch

The following offsets​ were obtained from the execution of the example "MPU6050_Calibration" on the "MPU6050" library. You will need this values to calibrate your MPU6050 (values may be different for you)

### Values ​​for own case

Sensor readings with offsets:	0	5	16381	-1	1	2

Your offsets:	-1449	558	866	29	-23	35      --> this values are used to calibrate MPU6050

* The first three values ​​belong to accelerometer, on the x, y and z axes respectively. The remaining values ​​correspond to the gyroscope


## 3. About MPU6050

Knowing the ranges with which our MPU6050 is configured, These ranges can be 2g / 4g / 8g / 16g for the accelerometer and 250/500/1000/2000 (°/ s) for the gyroscope.
The default ranges (2g and 250 ° / s)
 
 |Variable|minimum value|central value|maximum value
 |-|-|-|-|
|MPU6050 reading|    -32768 | 0  |           +32767 
| Acceleration  |       -2g   |       0g   |         +2g
 |Angular velocity|  -250°/s|       0°/s |         +250°/s


