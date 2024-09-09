// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "LowPass.h"
#include "Quaternion.h"
#include "sh2.h"

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// Sensor bitmasks for dynamic calibration
#define ACCELEROMETER_BIT 0x01
#define GYROSCOPE_BIT 0x02

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

// Define constants
const float g = 9.80665; // Gravitational acceleration constant
const float fsr = 8.0; // Full-scale range of accelerometer is +-8g
const uint8_t MY_LED_RED   = 14; // RGB pins on Arduino ESP32
const uint8_t MY_LED_GREEN = 15;
const uint8_t MY_LED_BLUE  = 16;

// Define variables
float imu_linear_accel, filt_linear_accel;
float imu_n, filt_n, filt_enu_az;
float enu_ax = 0, enu_ay = 0, enu_az = 0;
float imu_ax = 0, imu_ay = 0, imu_az = 0;
float heading;

// Define Quaternion objects
const Quaternion enu_q; // Define East-North-Up quaternion object with <1,0,0,0>
Quaternion imu_q; // Initialise IMU frame quaternion object <a,b,c,d> = <qw,qi,qj,qk>
Quaternion enu_accel(enu_ax, enu_ay, enu_az); // Define a pure quaternion describing the acceleration vector in ENU frame
Quaternion imu_accel(imu_ax, imu_ay, imu_az); // Define a pure quaternion describing the acceleration vector in IMU frame

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// RGB blink function notifying user of different stages
void RGB_blink(int red_pin_logic, int green_pin_logic, int blue_pin_logic, float delay_blink_millis){
  // Notify user of current stage using the LED
  digitalWrite(MY_LED_RED, red_pin_logic);
  digitalWrite(MY_LED_GREEN, green_pin_logic);
  digitalWrite(MY_LED_BLUE, blue_pin_logic);
  delay(delay_blink_millis);

  // Switches off the LED
  digitalWrite(MY_LED_RED, HIGH);
  digitalWrite(MY_LED_GREEN, HIGH);
  digitalWrite(MY_LED_BLUE, HIGH);
  delay(delay_blink_millis);
}

// Compute linear acceleration using accel norm and gravity norm
float get_linear_accel(float ax, float ay, float az){
  float linearAccel = sqrt(sq(ax)+sq(ay)+sq(az));
  return linearAccel;
}

// Using quaternion information to obtain heading in [0, 360)
float quaternion_heading(Quaternion &quat) {
    // Determine Euler yaw in radians
    float yaw_rad = atan2(2.0 * (quat.d * quat.a + quat.b * quat.c) , - 1.0 + 2.0 * (quat.a * quat.a + quat.b * quat.b));

    // Convert to Euler yaw to degrees to obtain heading
    float heading = yaw_rad * RAD_TO_DEG;

    // Set heading to be defined for [0, 360)
    if (heading < 0) {
        heading += 360;
    }

    // Handles cases where heading = 360 and just in case if it exceeds 360 for some reason
    if (heading >= 360) {
        heading -= 360;
    }
    return heading;
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

// 4th Order Low-Pass Butterworth filter instance
const int order = 4;
LowPass<order> lp(3,128.2051,false); // f0 = 3 Hz, fs = 128.2051 Hz, No adaptive sampling frequency

void setup(void) {
  Serial.begin(115200);

  //Configure LED pins as output
  pinMode(MY_LED_RED, OUTPUT);
  pinMode(MY_LED_GREEN, OUTPUT);
  pinMode(MY_LED_BLUE, OUTPUT);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  // Enable dynamic calibration for accelerometer
  int result = sh2_setCalConfig(ACCELEROMETER_BIT);
  if (result == SH2_OK) {
    Serial.println("[SUCCESS] Enable dynamic calibration successful! - accelerometer");
  } else {
    Serial.println("[FAIL] Enable dynamic calibration unsuccessful! - accelerometer");
    delay(10);
  }

  // Enable dynamic calibration for gyroscope
  result = sh2_setCalConfig(GYROSCOPE_BIT);
  if (result == SH2_OK) {
    Serial.println("[SUCCESS] Enable dynamic calibration successful! - gyroscope");
  } else {
    Serial.println("[FAIL] Enable dynamic calibration unsuccessful! - gyroscope");
    delay(10);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);

  // Wait until we receive a SH2_GAME_ROTATION_VECTOR sensor event
  while (!bno08x.getSensorEvent(&sensorValue) || sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
    delay(10);
  }

  // Blink RED when taking ENU measurement
  RGB_blink(LOW, HIGH, HIGH, 5000);

  // Store the initial IMU quaternion values equal to ENU frame
  imu_q.a = sensorValue.un.gameRotationVector.real;
  imu_q.b = sensorValue.un.gameRotationVector.i;
  imu_q.c = sensorValue.un.gameRotationVector.j;
  imu_q.d = sensorValue.un.gameRotationVector.k;

  // Indicate to user that ENU is done storing by showing GREEN
  RGB_blink(HIGH, LOW, HIGH, 5000);

  // Indicate to user that executing LOOP by showing BLUE
  digitalWrite(MY_LED_BLUE, LOW);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    // Read sensor data
    imu_accel.b = sensorValue.un.accelerometer.x;
    imu_accel.c = sensorValue.un.accelerometer.y;
    imu_accel.d = sensorValue.un.accelerometer.z;

    // Hamilton product of normalised rotation quaternion and IMU quaternion vector to obtain ENU quaternion
    enu_accel = imu_q * imu_accel * imu_q.conj();

    // Find the [ax,ay,az]' independent of gravity (linear acceleration)
    enu_ax = enu_accel.b;
    enu_ay = enu_accel.c;
    enu_az = enu_accel.d - g;

    // Apply low pass filter to az
    filt_enu_az = lp.filt(enu_az);
    
    // Serial.print("ENU Frame [m/s^2]: ax = ");
    // Serial.print(enu_ax);
    // Serial.print(", ay = ");
    // Serial.print(enu_ay);
    // Serial.print(", az = ");
    // Serial.println(enu_az);

    //Serial.print("Raw ENU [m/s^2]: az = ");
    Serial.print(enu_az, 6);
    Serial.print(",");
    //Serial.print(", Filtered ENU [m/s^2]: az = ");
    Serial.println(filt_enu_az, 6);

    // imu_n = get_linear_accel(ax, ay, az) - g;
    
    // filt_n = lp.filt(imu_n);

    // Serial.print(imu_n, 6);
    // Serial.print(",");
    // Serial.println(filt_n, 6);
    break;
  case SH2_GAME_ROTATION_VECTOR:
    imu_q.a = sensorValue.un.gameRotationVector.real;
    imu_q.b = sensorValue.un.gameRotationVector.i;
    imu_q.c = sensorValue.un.gameRotationVector.j;
    imu_q.d = sensorValue.un.gameRotationVector.k;

    heading = quaternion_heading(imu_q);
    // Serial.print("Heading: ");
    // Serial.println(heading);

    // Serial.print(F("{\"quat_w\":"));
    // Serial.print(imu.a, 3);
    // Serial.print(F(", \"quat_x\":"));
    // Serial.print(imu.b, 3);
    // Serial.print(F(", \"quat_y\":"));
    // Serial.print(imu.c, 3);
    // Serial.print(F(", \"quat_z\":"));
    // Serial.print(imu.d, 3);
    // Serial.println(F("}"));
    break;
  }
}
