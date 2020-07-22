// ================================================================
// ===                       LIBRARIES                          ===
// ================================================================

#include "I2Cdev.h"     // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
#include "MPU6050.h"    // Find it on https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "Wire.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

// ================================================================
// ===                   DEFINES AND CONSTANTS                  ===
// ================================================================
#define  SAMPLE_FREQUENCY  100      // Frequency to collect data [ms]
#define IMPOSIBLE_LEN 1000      //This is a value to reset the abnormalTreshold
#define BUFFER_TiLEN 100        //For Tilt
#define BUFFER_TrLEN 130        //For Treshold

#define CONSTANT_G 9.81   //Gravity constant
//Conversion factor (w_ACCorGYRO=actual_reading*(maximum_absolute_ACCorGYRO/maximum_absolute_values_MPU6050)) = actual_reading*(1/(maximum_absolute_values_MPU6050/maximum_absolute_ACCorGYRO)))
#define ACC_RATIOS 16384.0      //Obtained from (maximum_absolute_values_MPU6050 / maximum_absolute_ACC) = (32768/2)
#define GYRO_RATIOS 131.0
#define RAD_TO_DEG 57.295779  //57.295779 = 1 / (3.142 / 180) ¡The Arduino asin function is in radians!

<<<<<<< HEAD
#define ACCEL_THRESHOLD  (1.82 * CONSTANT_G)
#define LOWER_TRESHOLD (0.4 * CONSTANT_G)
#define UPPER_TRESHOLD (2 * CONSTANT_G)

#define WAITING_TIME 10000
=======
// Wifi MAC address
byte mac[]= {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
 
WiFiClient espClient;
IPAddress ip;
PubSubClient mqttClient(espClient);
>>>>>>> cdaed6afc2a484d75a1e533e01d2fd8dede8f53b

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
  readingMPU6050,       //Just for reading and process accel magnitudes
  abnormalTreshold,      //when an abnormal event has occurred 
  normalTreshold,       //Transition beetween an abnormal to normal reading
  checkSignal,          //verifying abnormal event    
  checkOrientation,     //verifying abnormal event and orientation change
  FallDetected          //Do something when  fall is detected

};

// ================================================================
// ===                  GLOBAL VARIABLES                        ===
// ================================================================

States Signal;
MPU6050 sensor(MPU_addr);
IMU_rotation current_orientation;
IMU_rotation vector_tilt[BUFFER_TiLEN];
IMU_accel vector_treshold[BUFFER_TrLEN];

uint16_t buffer_index = 0;
int previous_abnormal=0;
int start_abnormal = 0;
int end_abnormal = IMPOSIBLE_LEN;
 
// RAW data from accelerometer and giroscope (x, y and z axis)
int16_t AcX, AcY, AcZ, Tmp , GyX, GyY, GyZ;
float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;

//Orientation angles
float Raw_AM;
float Raw_GM;
float Raw_HM;       //For horizontal magnitude (2 axis, y & z)
float Aangle_roll, Aangle_pitch; 

unsigned  long  mCurrent_time ;
bool waitSignal=false;      //After a fall detected, this program will wait a time to restart

filter_rotY=0.0;
filter_rotZ=0.0;

// ================================================================
// ===                           SETUP                          ===
// ================================================================

void setup() {
    Wire.begin();           //Starting I2C  
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing MPU6050...");
    sensor.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(sensor.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //Setting accel/gyro offset values    //This is optional, if your MPU6050 is calibrated you dont need to uncomment the folowing lines
    //sensor.setXGyroOffset(GYRO_OFFSET_X);
    //sensor.setYGyroOffset(GYRO_OFFSET_Y);
    //sensor.setZGyroOffset(GYRO_OFFSET_Z);
    //sensor.setXAccelOffset(ACC_OFFSET_X);
    //sensor.setYAccelOffset(ACC_OFFSET_Y);
    //sensor.setZAccelOffset(ACC_OFFSET_Z);

    mCurrent_time = millis ();
    // Start with the readings
    Signal = readingMPU6050;
}

// ================================================================
// ===                            LOOP                          ===
// ================================================================
 
void loop() {
    switch (Signal)
    {        
        case readingMPU6050:
            //wait 100 ms to continue
            while (( unsigned  long ) ( millis () - mCurrent_time ) <= SAMPLE_FREQUENCY);
            mCurrent_time = millis();
            //Read Data collected by MPU6050
            mpu_read();    

            //Processing data from Gyroscope and Accelerometer 
            IMU_accel current_accel = setAccel();   //Process data with the respective ratios values
            IMU_rotation current_angles= processAccelAngles(&current_accel);       //get Aceleration Magnitude, Pitch and Roll angles
            
            //Calculate the total Accelerometer Vector or Acceleration Magnitude
            Raw_AM = (float)sqrt(pow(current_accel->x,2)+pow(current_accel->y,2)+pow(current_accel->z,2));    
            Raw_HM = (float)sqrt(pow(current_accel->y,2)+pow(current_accel->z,2));

            // Apply Low Pass IIR Filter to roll/pitch angles
            filter_rotY = FilterLowPass(current_angles.pitch, filter_rotY);
            filter_rotZ = FilterLowPass(current_angles.roll, filter_rotZ);

            // Check if acceleration lower than threshold
            if (Raw_AM <= LOWER_TRESHOLD) {
                Signal = abnormalTreshold;
            }

            // Check the interval recolected after abnormalTreshold
            if (buffer_index == end_abnormal) {
                Signal = checkSignal;
            }   
            
            break;

        case abnormalTreshold:      //when an abnormal event has occurred
            previous_abnormal = GetBufferPosition(buffer_index, -10);   //Equal to 1 second before lower treshold detected
            start_abnormal = buffer_index;
            end_abnormal = GetBufferPosition(buffer_index, 5);      //Equal to 0.5 second after lower treshold detected
            
            //Go back to readingMPU6050
            Signal = readingMPU6050;
            break;
    
        case normalTreshold:       //Transition beetween an abnormal to normal reading
            //index to 0
            mMag_index = 0;
           
            // Continue measuring accel
            mState = stateMeasureAccel;
            break;

        case checkSignal:       //verifying abnormal event
            //Compare min and max amplitudes

            //
            // ***else*** No Fall Detected so reset the trigger points
            mState = stateResetTriggers;    
            break;

        case checkOrientation:      //verifying abnormal event and orientation change
            break;        
        
        case FallDetected:
        {
            Serial.println("Fall Detected");
            waitSignal=true;
            Signal = resetTriggers;
            break;
        }
    }

    // IMPORTANT! only increment buffer when state is measuring data
    if (Signal == stateMeasureAccel) {
        buffer_index = GetBufferPosition(buffer_index, 1);  
    }

    //If a fall is detected, the program will be stopped by a time
    if (waitSignal==true){
        while (( unsigned  long ) ( millis () - mCurrent_time ) <= WAITING_TIME);
        waitSignal=false;
    }
}

// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================

IMU_accel setAccel() {
    Imu_accel accel;
    accel.x = AcX * (CONSTANT_G/ACC_RATIOS);       //This values are measured on [m/s2], to work with g measure change CONSTANT_G to 1
    accel.y = AcY * (CONSTANT_G/ACC_RATIOS);
    accel.z = AcZ * (CONSTANT_G/ACC_RATIOS);

    return accel;
  
}

// void processAccelMagnitudes(IMU_accel* accel){
//     ax = accel->x * (CONSTANT_G/ACC_RATIOS);       //This values are measured on [m/s2], to work with g measure change CONSTANT_G to 1
//     ay = accel->y * (CONSTANT_G/ACC_RATIOS);
//     az = accel->z * (CONSTANT_G/ACC_RATIOS);
//     // netForce = (gForceX+gForceY+gForceZ)/3;
// }

IMU_rotation processAccelAngles(Imu_accel* accel){
    IMU_rotation angles;
    angles.roll= atan(-1 * accel->y / sqrt(pow(accel->x, 2) + pow(accel->z, 2))) * RAD_TO_DEG;       //Calculate the roll angle
    angles.pitch = atan(-1 * accel->z / sqrt(pow(accel->x, 2) + pow(accel->y, 2))) * RAD_TO_DEG;      //Calculate the pitch angle
    
    return angles;
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



void processGyroMagnitudes() {
    gx = GyX / GYRO_RATIOS;
    gy = GyY / GYRO_RATIOS;
    gz = GyZ / GYRO_RATIOS;
    // netRot = sqrt((rotX*rotX)+(rotY*rotY)+(rotZ*rotZ));
}

void processGyroAngles(){
    Raw_GM = sqrt(pow(gx,2)+pow(gy,2)+pow(gz,2));     //Calculate the total Gyroscope Vector or Rotation Magnitude
}

float FilterLowPass(float new_input, float old_output) {
    // IIR Low pass Filter y[n] = 0.98y[n-1] + 0.02x[n]
    return (0.98 * old_output + 0.02 * new_input);

}

int GetBufferPosition(int current_pos, int samples) {       //Get the position given a certain point in the array
  int buffer_pos = current_pos + samples;

  if (buffer_pos > BUFFER_LEN - 1) {
    buffer_pos -= BUFFER_LEN;
  }

  if (buffer_pos < 0) {
    buffer_pos += BUFFER_LEN;
  }
  return (int) buffer_pos;

}