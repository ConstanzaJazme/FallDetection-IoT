// ================================================================
// ===                       LIBRARIES                          ===
// ================================================================

#include "I2Cdev.h"     // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h"    // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "Wire.h"

// ================================================================
// ===                   DEFINES AND CONSTANTS                  ===
// ================================================================

#define CONSTANT_G 9.81

//This offsets​were obtained from the execution of the example "MPU6050_Calibration" on the "MPU6050" library.
const int ACC_OFFSET_X = -1449;
const int ACC_OFFSET_Y = 558;
const int ACC_OFFSET_Z = 866;

const int GYRO_OFFSET_X = 29;
const int GYRO_OFFSET_Y = -23;
const int GYRO_OFFSET_Z = 35;

const int MPU_addr=0x68;  // I2C address of the MPU-6050

// ================================================================
// ===                  GLOBAL VARIABLES                        ===
// ================================================================
 
MPU6050 sensor(MPU_addr);
 
// RAW data from accelerometer and giroscope (x, y and z axis)
int ax, ay, az;
int gx, gy, gz;
 
void setup() {
    Wire.begin();           //Starting I2C  
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing MPU6050...");
    sensor.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //Setting accel/gyro offset values
    sensor.setXGyroOffset(GYRO_OFFSET_X);
    sensor.setYGyroOffset(GYRO_OFFSET_Y);
    sensor.setZGyroOffset(GYRO_OFFSET_Z);
    sensor.setXAccelOffset(ACC_OFFSET_X);
    sensor.setYAccelOffset(ACC_OFFSET_Y);
    sensor.setZAccelOffset(ACC_OFFSET_Z);
}
 
void loop() {
  mpu_read();
  // Leer las aceleraciones y velocidades angulares
  float ax_m_s2 = ax * (CONSTANT_G/16384.0);
  float ay_m_s2 = ay * (CONSTANT_G/16384.0);
  float az_m_s2 = az * (CONSTANT_G/16384.0);
  float gx_deg_s = gx * (250.0/32768.0);
  float gy_deg_s = gy * (250.0/32768.0);
  float gz_deg_s = gz * (250.0/32768.0);
 
  //Mostrar las lecturas separadas por un [tab]
  Serial.print("a[x y z](m/s2) g[x y z](deg/s):\t");
  Serial.print(ax_m_s2); Serial.print("\t");
  Serial.print(ay_m_s2); Serial.print("\t");
  Serial.print(az_m_s2); Serial.print("\t");
  Serial.print(gx_deg_s); Serial.print("\t");
  Serial.print(gy_deg_s); Serial.print("\t");
  Serial.println(gz_deg_s);
 
  delay(100);
}

void mpu_read() {
  //Get Acceleration and Rotation parameters
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
}
