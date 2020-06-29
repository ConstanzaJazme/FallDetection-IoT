// ================================================================
// ===                       LIBRARIES                          ===
// ================================================================

#include "I2Cdev.h"     // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h"    // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "Wire.h"

// ================================================================
// ===                   DEFINES AND CONSTANTS                  ===
// ================================================================

#define CONSTANT_G 9.81   //Gravity constant
//Conversion factor (w_ACCorGYRO=actual_reading*(maximum_absolute_ACCorGYRO/maximum_absolute_values_MPU6050)) = actual_reading*(1/(maximum_absolute_values_MPU6050/maximum_absolute_ACCorGYRO)))
#define ACC_RATIOS 16384.0      //Obtained from (maximum_absolute_values_MPU6050 / maximum_absolute_ACC) = (32768/2)
#define GYRO_RATIOS 131.0
#define RAD_TO_DEG = 57.295779

//This offsets​were obtained from the execution of the example "MPU6050_Calibration" on the "MPU6050" library.
const int ACC_OFFSET_X = -1449;
const int ACC_OFFSET_Y = 558;
const int ACC_OFFSET_Z = 866;

const int GYRO_OFFSET_X = 29;
const int GYRO_OFFSET_Y = -23;
const int GYRO_OFFSET_Z = 35;

const int MPU_addr=0x68;  // I2C address of the MPU-6050

// ================================================================
// ===                       STRUCTS                            ===
// ================================================================

typedef struct {
  float roll;
  float pitch;

} IMU_rotation;

typedef struct {
  float x;
  float y;
  float z;

} IMU_accel;

// ================================================================
// ===                       ENUMS                              ===
// ================================================================

enum States {
  readingMPU6050,
  setTriggers,
  resetTriggers,
  checkSignal,
  breaksLowerTreshold,
  breaksUpperTreshold,
  checkOrientation,
  FallDetected

};

// ================================================================
// ===                  GLOBAL VARIABLES                        ===
// ================================================================

States Signal;
MPU6050 sensor(MPU_addr);
 
// RAW data from accelerometer and giroscope (x, y and z axis)
int16_t AcX, AcY, AcZ, Tmp , GyX, GyY, GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
//Orientation angles
float Acc[2];
float Gy[3];
float Angle[3];
 
void setup() {
    Wire.begin();           //Starting I2C  
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing MPU6050...");
    sensor.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //Setting accel/gyro offset values    //This is optional, if you MPU is calibrated you dont need to uncomment the folowing lines
    //sensor.setXGyroOffset(GYRO_OFFSET_X);
    //sensor.setYGyroOffset(GYRO_OFFSET_Y);
    //sensor.setZGyroOffset(GYRO_OFFSET_Z);
    //sensor.setXAccelOffset(ACC_OFFSET_X);
    //sensor.setYAccelOffset(ACC_OFFSET_Y);
    //sensor.setZAccelOffset(ACC_OFFSET_Z);

    // Start with the readings
    Signal = readingMPU6050;
}
 
void loop() {
    switch (Signal)
    {
        case readingMPU6050:
            mpu_read();
            // Leer las aceleraciones y velocidades angulares
            float ax_m_s2 = AcX * (CONSTANT_G/ACC_RATIOS);
            float ay_m_s2 = AcY * (CONSTANT_G/ACC_RATIOS);
            float az_m_s2 = AcZ * (CONSTANT_G/ACC_RATIOS);
            float gx_deg_s = GyX / GYRO_RATIOS;
            float gy_deg_s = GyY / GYRO_RATIOS;
            float gz_deg_s = GyZ / GYRO_RATIOS;
            
            //Mostrar las lecturas separadas por un [tab]
            Serial.print("a[x y z](m/s2) g[x y z](deg/s):\t");
            Serial.print(ax_m_s2); Serial.print("\t");
            Serial.print(ay_m_s2); Serial.print("\t");
            Serial.print(az_m_s2); Serial.print("\t");
            Serial.print(gx_deg_s); Serial.print("\t");
            Serial.print(gy_deg_s); Serial.print("\t");
            Serial.println(gz_deg_s);
            
            delay(100);
            break;
        
        case FallDetected:
        {
            Serial.println("Fall Detected");
            Signal = resetTriggers;
            break;
        }
    }
}

void mpu_read() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers (two values by register)
    
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
    //Get Acceleration and Rotation parameters
    //   sensor.getAcceleration(&ax, &ay, &az);
    //   sensor.getRotation(&gx, &gy, &gz);
}
