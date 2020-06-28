# MPU6050 Calibration Sketch

Sensor readings with offsets:	0	5	16381	-1	1	2
Your offsets:	-1449	558	866	29	-23	35      --> this values are used to calibrate MPU6050

* The first three values ​​belong to accelerometer, on the x, y and z axes respectively. The remaining values ​​correspond to the gyroscope

Data is printed as: acelX acelY acelZ giroX giroY giroZ
Check that your sensor readings are close to 0 0 16384 0 0 0
If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)
