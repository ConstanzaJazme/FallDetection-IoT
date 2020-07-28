// ================================================================
// ===                       LIBRARIES                          ===
// ================================================================

#include "Wire.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <MQTT.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson (use v6.xx)
#include <time.h>
#define emptyString String()

//Follow instructions from https://github.com/debsahu/ESP-MQTT-AWS-IoT-Core/blob/master/doc/README.md
//Enter values in secrets.h ▼
#include "secrets.h"

// ================================================================
// ===                   DEFINES AND CONSTANTS                  ===
// ================================================================
#define SAMPLE_FREQUENCY 100 // Frequency to collect data [miliseconds]
#define IMPOSIBLE_LEN 1000   //This is a value to reset the abnormalTreshold
#define BUFFER_TrLEN 130     //For Treshold
#define MAGNITUDE_LEN 5      //For Treshold

//Conversion factor (w_ACCorGYRO=actual_reading*(maximum_absolute_ACCorGYRO/maximum_absolute_values_MPU6050)) = actual_reading*(1/(maximum_absolute_values_MPU6050/maximum_absolute_ACCorGYRO)))
#define ACC_RATIOS 16384.0 //Obtained from (maximum_absolute_values_MPU6050 / maximum_absolute_ACC) = (32768/2 [g])
#define GYRO_RATIOS 131.0  //Obtained from (maximum_absolute_values_MPU6050 / maximum_absolute_GYRO) = (32768/250 [°/s])

#define CONSTANT_G 9.81      //Gravity constant
#define RAD_TO_DEG 57.295779 //57.295779 = 1 / (3.142 / 180) ¡The asin function is in radians!

#define LOWER_TRESHOLD (0.4 * CONSTANT_G)
#define UPPER_TRESHOLD (2 * CONSTANT_G)
#define ANGLE_THRESHOLD 40.0

#define WAITING_TIME 120000  //Time to wait between Fall events
#define WAIT_POST_BREAKS 100 //Samples requiered post break lower Treshold
#define PREV_BREAKS 30  //Samples before break lower Treshold

//This offsets​were obtained from the execution of the example "MPU6050_Calibration" on the "MPU6050" library.
const int ACC_OFFSET_X = -1449;
const int ACC_OFFSET_Y = 558;
const int ACC_OFFSET_Z = 866;

const int GYRO_OFFSET_X = 29;
const int GYRO_OFFSET_Y = -23;
const int GYRO_OFFSET_Z = 35;

const int MPU_addr = 0x68; // I2C address of the MPU-6050

// ================================================================
// ===                    MQTT CONSTANTS                        ===
// ================================================================

const int MQTT_PORT = 8883;
const char MQTT_SUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";
const char MQTT_PUB_TOPIC[] = "$aws/things/" THINGNAME "/shadow/update";

#ifdef USE_SUMMER_TIME_DST
uint8_t DST = 1;
#else
uint8_t DST = 0;
#endif

WiFiClientSecure net;

BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);

MQTTClient client(516);

unsigned long lastMillis = 0;
time_t now;
time_t nowish = 1510592825;

// ================================================================
// ===                       STRUCTS                            ===
// ================================================================

typedef struct
{
  float roll;
  float pitch;
} IMU_rotation;

typedef struct
{
  float x;
  float y;
  float z;
} IMU_accel;

// ================================================================
// ===                       ENUMS                              ===
// ================================================================

enum States
{
  readingMPU6050,   //Just for reading and process accel magnitudes
  abnormalTreshold, //when an abnormal event has occurred
  normalTreshold,   //Transition beetween an abnormal to normal reading
  checkSignal,      //verifying abnormal event
  checkOrientation, //verifying abnormal event and orientation change
  fallDetected      //Do something when  fall is detected
};

// ================================================================
// ===                  GLOBAL VARIABLES                        ===
// ================================================================

States Signal;
IMU_rotation vector_tilt[BUFFER_TrLEN];
IMU_accel vector_accel[BUFFER_TrLEN];

float vector_magnitudes[MAGNITUDE_LEN] = {0.0};
int mindex = 0;

int buffer_index = 0;
int previous_abnormal = 0;
int start_abnormal = 0;
int end_abnormal = IMPOSIBLE_LEN;
int checking_orientation = IMPOSIBLE_LEN;

// RAW data from accelerometer and giroscope (x, y and z axis)
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

//Orientation angles
float Raw_AM;
float Raw_GM;
float Raw_HM; //For horizontal magnitude (2 axis, y & z)
float Aangle_roll, Aangle_pitch;

unsigned long mCurrent_time;
bool waitSignal = false; //After a fall detected, this program will wait a time to restart

float filter_rotY = 0.0;
float filter_rotZ = 0.0;

// ================================================================
// ===                  FUNCTIONS FOR MQTT                      ===
// ================================================================

void NTPConnect(void)
{
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, DST * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < nowish)
  {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print("Current time: ");
  Serial.print(asctime(&timeinfo));
}

void messageReceived(String &topic, String &payload)
{
  Serial.println("Recieved [" + topic + "]: " + payload);
}

void lwMQTTErr(lwmqtt_err_t reason)
{
  if (reason == lwmqtt_err_t::LWMQTT_SUCCESS)
    Serial.print("Success");
  else if (reason == lwmqtt_err_t::LWMQTT_BUFFER_TOO_SHORT)
    Serial.print("Buffer too short");
  else if (reason == lwmqtt_err_t::LWMQTT_VARNUM_OVERFLOW)
    Serial.print("Varnum overflow");
  else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_CONNECT)
    Serial.print("Network failed connect");
  else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_TIMEOUT)
    Serial.print("Network timeout");
  else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_READ)
    Serial.print("Network failed read");
  else if (reason == lwmqtt_err_t::LWMQTT_NETWORK_FAILED_WRITE)
    Serial.print("Network failed write");
  else if (reason == lwmqtt_err_t::LWMQTT_REMAINING_LENGTH_OVERFLOW)
    Serial.print("Remaining length overflow");
  else if (reason == lwmqtt_err_t::LWMQTT_REMAINING_LENGTH_MISMATCH)
    Serial.print("Remaining length mismatch");
  else if (reason == lwmqtt_err_t::LWMQTT_MISSING_OR_WRONG_PACKET)
    Serial.print("Missing or wrong packet");
  else if (reason == lwmqtt_err_t::LWMQTT_CONNECTION_DENIED)
    Serial.print("Connection denied");
  else if (reason == lwmqtt_err_t::LWMQTT_FAILED_SUBSCRIPTION)
    Serial.print("Failed subscription");
  else if (reason == lwmqtt_err_t::LWMQTT_SUBACK_ARRAY_OVERFLOW)
    Serial.print("Suback array overflow");
  else if (reason == lwmqtt_err_t::LWMQTT_PONG_TIMEOUT)
    Serial.print("Pong timeout");
}

void lwMQTTErrConnection(lwmqtt_return_code_t reason)
{
  if (reason == lwmqtt_return_code_t::LWMQTT_CONNECTION_ACCEPTED)
    Serial.print("Connection Accepted");
  else if (reason == lwmqtt_return_code_t::LWMQTT_UNACCEPTABLE_PROTOCOL)
    Serial.print("Unacceptable Protocol");
  else if (reason == lwmqtt_return_code_t::LWMQTT_IDENTIFIER_REJECTED)
    Serial.print("Identifier Rejected");
  else if (reason == lwmqtt_return_code_t::LWMQTT_SERVER_UNAVAILABLE)
    Serial.print("Server Unavailable");
  else if (reason == lwmqtt_return_code_t::LWMQTT_BAD_USERNAME_OR_PASSWORD)
    Serial.print("Bad UserName/Password");
  else if (reason == lwmqtt_return_code_t::LWMQTT_NOT_AUTHORIZED)
    Serial.print("Not Authorized");
  else if (reason == lwmqtt_return_code_t::LWMQTT_UNKNOWN_RETURN_CODE)
    Serial.print("Unknown Return Code");
}

void connectToMqtt(bool nonBlocking = false)
{
  Serial.print("MQTT connecting ");
  while (!client.connected())
  {
    if (client.connect(THINGNAME))
    {
      Serial.println("connected!");
      if (!client.subscribe(MQTT_SUB_TOPIC))
        lwMQTTErr(client.lastError());
    }
    else
    {
      Serial.print("SSL Error Code: ");
      Serial.println(net.getLastSSLError());
      Serial.print("failed, reason -> ");
      lwMQTTErrConnection(client.returnCode());
      if (!nonBlocking)
      {
        Serial.println(" < try again in 5 seconds");
        delay(5000);
      }
      else
      {
        Serial.println(" <");
      }
    }
    if (nonBlocking)
      break;
  }
}

void connectToWiFi(String init_str)
{
  if (init_str != emptyString)
    Serial.print(init_str);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  if (init_str != emptyString)
    Serial.println("ok!");
}

void checkWiFiThenMQTT(void)
{
  connectToWiFi("Checking WiFi");
  connectToMqtt();
}

unsigned long previousMillis = 0;
const long interval = 5000;

void checkWiFiThenMQTTNonBlocking(void)
{
  connectToWiFi(emptyString);
  if (millis() - previousMillis >= interval && !client.connected())
  {
    previousMillis = millis();
    connectToMqtt(true);
  }
}

void checkWiFiThenReboot(void)
{
  connectToWiFi("Checking WiFi");
  Serial.print("Rebooting");
  ESP.restart();
}

void sendData(void)
{ 
   Serial.println(start_abnormal);
   const size_t capacity = JSON_ARRAY_SIZE(130) + 2*JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(2) + 130*JSON_OBJECT_SIZE(3);
   DynamicJsonDocument jsonBuffer(capacity);
  for (int interval=0; interval<13; interval++){
    
    JsonObject root = jsonBuffer.to<JsonObject>();
    JsonObject state = root.createNestedObject("state");
    JsonObject state_reported = state.createNestedObject("reported");
    JsonArray value = state_reported.createNestedArray("value");
    state_reported["interval"] = interval;
    state_reported["start_interval"] = int(start_abnormal/10);
    state_reported["start_position"] = int(start_abnormal%10);
    int base=interval*10;
    for (int index = 0; index < 10; index++) {    
      value[index].add(vector_accel[base+index].x);
      value[index].add(vector_accel[base+index].y);
      value[index].add(vector_accel[base+index].z);
    }
    Serial.printf("Sending  [%s]: ", MQTT_PUB_TOPIC);
    serializeJson(root, Serial);
    Serial.println();
    char shadow[measureJson(root) + 1];
    serializeJson(root, shadow, sizeof(shadow));
    
    if (!client.publish(MQTT_PUB_TOPIC, shadow, false, 0))
      lwMQTTErr(client.lastError());
  }
}

// ================================================================
// ===                           SETUP                          ===
// ================================================================

void setup()
{
  Serial.begin(115200);
  // initialize MQTT CONNECTION
  Serial.println("Initializing MQTT...");
  delay(5000);
  Serial.println();
  Serial.println();
  WiFi.hostname(THINGNAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  connectToWiFi(String("Attempting to connect to SSID: ") + String(ssid));

  NTPConnect();

  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);
  client.begin(MQTT_HOST, MQTT_PORT, net);
  client.onMessage(messageReceived);
  connectToMqtt();

  // initialize device
  Serial.println("Initializing MPU6050...");

  Wire.begin(0, 2); //Starting I2C
  //Serial.begin(9600);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  mCurrent_time = millis();
  // Start with the readings
  Signal = readingMPU6050;
}

// ================================================================
// ===                            LOOP                          ===
// ================================================================

void loop()
{
  now = time(nullptr);
  if (!client.connected()) //PROBLEMS WITH CONNECTION
  {
    checkWiFiThenMQTT();
  }
  else //CLIENT IS CONNECTED
  {
    client.loop();

    switch (Signal)
    {
    case readingMPU6050:
    {
      //wait 100 ms to continue
      while ((unsigned long)(millis() - mCurrent_time) <= SAMPLE_FREQUENCY)
        ;
      mCurrent_time = millis();
      //Read Data collected by MPU6050
      mpu_read();

      //Processing data from Gyroscope and Accelerometer
      IMU_accel current_accel = setAccel();                             //Process data with the respective ratios values
      IMU_rotation current_angles = processAccelAngles(&current_accel); //get Aceleration Magnitude, Pitch and Roll angles

      //Calculate the total Accelerometer Vector or Acceleration Magnitude
      Raw_AM = (float)sqrt(pow(current_accel.x, 2) + pow(current_accel.y, 2) + pow(current_accel.z, 2));
      Raw_HM = (float)sqrt(pow(current_accel.y, 2) + pow(current_accel.z, 2));

      // Apply Low Pass IIR Filter to roll/pitch angles
      filter_rotY = FilterLowPass(current_angles.pitch, filter_rotY);
      filter_rotZ = FilterLowPass(current_angles.roll, filter_rotZ);

      // Get Current roll/pitch angles filtered
      vector_tilt[buffer_index].roll = filter_rotZ;
      vector_tilt[buffer_index].pitch = filter_rotY;

      // Get Current accelerations angles filtered
      vector_accel[buffer_index] = current_accel;

      // Check if acceleration lower than threshold
      if (Raw_AM <= LOWER_TRESHOLD)
      {
        Serial.println("LOWER TRESHOLD BROKEN");
        Signal = abnormalTreshold;
      }

      // Check the interval recolected after abnormalTreshold
      if (buffer_index == checking_orientation)
      {
        Serial.print("PASSING TO CHECK SIGNAL");
        Signal = checkSignal;
        Serial.println(Signal);
      }

      //saving vector from magnitudes when end_abnormal is settled
      if (end_abnormal != IMPOSIBLE_LEN && mindex < MAGNITUDE_LEN)
      {
        Serial.println("SAVING IN vector_magnitudes...");
        vector_magnitudes[mindex] = Raw_AM;
        mindex++;
      }

      break;
    }
    case abnormalTreshold: //when an abnormal event has occurred
    {
      Serial.println("TRIGGERS SETTLED");
      previous_abnormal = GetBufferPosition(buffer_index, -PREV_BREAKS); //Equal to 3 second before lower treshold detected
      start_abnormal = buffer_index;
      end_abnormal = GetBufferPosition(buffer_index, 5);                        //Equal to 0.5 second after lower treshold detected
      checking_orientation = GetBufferPosition(buffer_index, WAIT_POST_BREAKS); //  Equal to 10 seconds after lower treshold

      //Go back to readingMPU6050
      Signal = readingMPU6050;
      break;
    }
    case normalTreshold: //Transition beetween an abnormal to normal reading, this state is triggered when tresholds are not broken
    {
      end_abnormal = IMPOSIBLE_LEN;
      checking_orientation = IMPOSIBLE_LEN;
      Serial.println("DEACTIVATING TRIGGERS");
      //restarting vector_magnitudes
      mindex = 0;

      // Continue measuring accel
      Signal = readingMPU6050;
      break;
    }
    case checkSignal: //verifying abnormal event
    {
      int difference = getMaxAmplitude();
      if (difference <= UPPER_TRESHOLD)
      { //Compare min and max amplitudes and verify if amplitudes breaks tresholds
        Serial.println("UPPER TRESHOLD BROKEN");
        Signal = checkOrientation;
      }
      else
      {
        Signal = normalTreshold; // No Fall Detected so reset the trigger points
      }
      break;
    }

    case checkOrientation: //verifying abnormal event and orientation change
    {
      int index = GetBufferPosition(buffer_index, -WAIT_POST_BREAKS-10); //getting the last 11 seconds
      float orientation_change = getMaxChange(index, buffer_index);
      Serial.print("CHECKING ORIENTATION -> ");
      Serial.println(orientation_change);

      if (orientation_change > ANGLE_THRESHOLD)
      {
        // Fall has been detected
        Signal = fallDetected;
      }
      else
      {
        // Fall not detected due to low change in orientation
        Signal = normalTreshold;
      }

      break;
    }
    case fallDetected:
    {
      Serial.println("Fall Detected");
      sendData();
      waitSignal = true;
      Signal = normalTreshold;
      break;
    }
    }

    // IMPORTANT! only increment buffer when state is measuring data
    if (Signal == readingMPU6050)
    {
      buffer_index = GetBufferPosition(buffer_index, 1);
    }

    //If a fall is detected, the program will be stopped by a time
    if (waitSignal == true)
    {
      Serial.println("BLOCKED");
      delay(WAITING_TIME);
      waitSignal = false;
      Serial.println("RESTARTING");
    }
  }
}

// ================================================================
// ===               FUNCTIONS FOR ALGORITHM                    ===
// ================================================================

//
// This function convert RAW data from Accelerometer and save this results into a struct. This values are measured on [m/s2], to work with g measure change CONSTANT_G to 1
//
IMU_accel setAccel()
{
  IMU_accel accel;
  accel.x = AcX * (CONSTANT_G / ACC_RATIOS);
  accel.y = AcY * (CONSTANT_G / ACC_RATIOS);
  accel.z = AcZ * (CONSTANT_G / ACC_RATIOS);
  return accel;
}

//
// Function to get roll and pitch angles (measured on Degrees)
//
IMU_rotation processAccelAngles(IMU_accel *accel)
{
  IMU_rotation angles;
  angles.roll = atan(-1 * accel->y / sqrt(pow(accel->x, 2) + pow(accel->z, 2))) * RAD_TO_DEG;  //Calculate the roll angle
  angles.pitch = atan(-1 * accel->z / sqrt(pow(accel->x, 2) + pow(accel->y, 2))) * RAD_TO_DEG; //Calculate the pitch angle

  return angles;
}

//
// Function to read data from MPU6050
//
void mpu_read()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers (two values by register)

  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

//
// This function convert RAW data from gyroscope.
//
void processGyroMagnitudes()
{
  gx = GyX / GYRO_RATIOS;
  gy = GyY / GYRO_RATIOS;
  gz = GyZ / GYRO_RATIOS;
}

//
// Calculate the total Gyroscope Vector or Rotation Magnitude
//
void processGyroAngles()
{
  Raw_GM = sqrt(pow(gx, 2) + pow(gy, 2) + pow(gz, 2));
}

//
// IIR Low pass Filter y[n] = 0.98y[n-1] + 0.02x[n]
//
float FilterLowPass(float new_input, float old_output)
{
  return (0.98 * old_output + 0.02 * new_input);
}

//
// Get next position about a certain point in array. current_pos -> actual position in array, samples -> number of spaces to advance
//
int GetBufferPosition(int current_pos, int samples)
{
  int buffer_pos = current_pos + samples;

  if (buffer_pos > BUFFER_TrLEN - 1)
  {
    buffer_pos -= BUFFER_TrLEN;
  }

  if (buffer_pos < 0)
  {
    buffer_pos += BUFFER_TrLEN;
  }
  return (int)buffer_pos;
}

//
// This function get difference between min and max values from magnitudes array
//
float getMaxAmplitude()
{
  float minVal = vector_magnitudes[0];
  float maxVal = vector_magnitudes[0];

  for (int i = 0; i < MAGNITUDE_LEN; i++)
  {
    if (vector_magnitudes[i] > maxVal)
    {
      maxVal = vector_magnitudes[i];
    }
    if (vector_magnitudes[i] < minVal)
    {
      minVal = vector_magnitudes[i];
    }
  }
  return (maxVal - minVal);
}

//
// This function get difference between min and max values from tilt array
//
float getMaxChange(int start, int finish)
{
  float minValRoll = vector_tilt[start].roll;
  float maxValRoll = vector_tilt[start].roll;

  float minValPitch = vector_tilt[start].pitch;
  float maxValPitch = vector_tilt[start].pitch;
  int next = start;
  Serial.print(start);
  Serial.print(" - ");
  Serial.println(finish);

  while (next != finish)
  {
    if (vector_tilt[next].roll > maxValRoll)
    {
      maxValRoll = vector_tilt[next].roll;
    }
    if (vector_tilt[next].roll < minValRoll)
    {
      minValRoll = vector_tilt[next].roll;
    }
    if (vector_tilt[next].pitch > maxValPitch)
    {
      maxValPitch = vector_tilt[next].pitch;
    }
    if (vector_tilt[next].pitch < minValPitch)
    {
      minValPitch = vector_tilt[next].pitch;
    }

    next = GetBufferPosition(next, 1);
  }

  float difference_pitch = maxValPitch - minValPitch;
  float difference_roll = maxValRoll - minValRoll;

  Serial.print("DIFFERENCE PICH -> ");
  Serial.println(difference_pitch);
  Serial.print("DIFFERENCE ROLL -> ");
  Serial.println(difference_roll);

  if (difference_pitch > difference_roll)
  {
    return difference_pitch;
  }

  return difference_roll;
}

//
// Print RAW data from Accelerometer
//
void print_readings()
{
  Serial.print("a[x y z]:\t");
  Serial.print(AcX);
  Serial.print("\t");
  Serial.print(AcY);
  Serial.print("\t");
  Serial.println(AcZ);
}
