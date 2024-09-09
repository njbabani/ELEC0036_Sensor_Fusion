// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <math.h>
#include "LowPass.h"
#include "Quaternion.h"
#include "UserPosition.h"
#include "sh2.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

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
const float imu_ax_offset = 0.026628;   // IMU zero-g ax bias offset
const float imu_ay_offset = 0.026212;   // IMU zero-g ay bias offset
const float imu_az_offset = -0.017193;  // IMU zero-g az bias offset
const float user_x0 = 8.4;              // Initial user x-coordinate
const float user_y0 = 1.25;             // Initial user y-coordinate
const float g = 9.80665;                // Gravitational acceleration constant
const float fsr = 8.0;                  // Full-scale range of accelerometer is +-8g
const int order = 4;                    // Low pass filter order
const int PDR_max_step = 3;             // From step 1 to step 3 (3 valid steps needed)
const int measurement_max = 200;        // Maximum amount of measurements
const float f0 = 3.0;                   // Cutoff frequency
const float f_s = 128.2051;             // Sampling frequency
const float kappa = 0.5309;             // Weinberg coefficient constant for step length
const float delta_a = 1;                // Acceleration threshold from MATLAB
const unsigned long tau_min = 300;      // Time difference threshold from reference

// RGB pins on Arduino ESP32
const uint8_t MY_LED_RED = 14;    // RED
const uint8_t MY_LED_GREEN = 15;  // GREEN
const uint8_t MY_LED_BLUE = 16;   // BLUE

// Define variables
int measurement_count = 0;                       // Global variable to track the number of measurements
float enu_ax = 0.0, enu_ay = 0.0, enu_az = 0.0;  // ENU [ax, ay, az]' acceleration values
float imu_ax = 0, imu_ay = 0, imu_az = 0;        // IMU [ax, ay, az]' acceleration values
float filt_enu_az = 0.0;                         // LPF ENU az value for step detection
float a_max = 0.0;                               // Initialise maximum az variable
float a_min = 0.0;                               // Initialise minimum az variable
double psi;                                      // User heading angle in degrees
float lambda = 0;                                // Step length
char FILE_NAME[] = "/PDR_FC_Test.txt";           // Name of the file written to SD card
unsigned long t_exp = 0;                         // Total experiment time
unsigned long t_meas_start = 0;                  // Measurement start time
unsigned long t_meas_end = 0;                    // Measurement end time

// Define flags
bool step_detect_flag = false;  // Indicates if valid step has been detected, if so then updates user position

// Define Quaternion objects
Quaternion IMUQuat;                           // Initialise IMU frame quaternion object <a,b,c,d> = <qw,qi,qj,qk> for rotation purposes
Quaternion ENUAccel(enu_ax, enu_ay, enu_az);  // Define a pure quaternion describing the acceleration vector in ENU frame
Quaternion IMUAccel(imu_ax, imu_ay, imu_az);  // Define a pure quaternion describing the acceleration vector in IMU frame
// const Quaternion enu_q; -> Define East-North-Up quaternion object with <1,0,0,0> - Not needed as initial sensor orientation is <1,0,0,0>

// 4th Order Low-Pass Butterworth filter instance
LowPass<order> lp(f0, f_s, false);  // f0 = 3 Hz, f_s = 128.2051 Hz, No adaptive sampling frequency

// Create mobile user instance
UserPosition MobileUser;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// RGB blink function notifying user of different stages
void blinkRGB(int red_pin_logic, int green_pin_logic, int blue_pin_logic, float delay_blink_millis) {
  // Notify user of current stage using the LED
  digitalWrite(MY_LED_RED, red_pin_logic);
  digitalWrite(MY_LED_GREEN, green_pin_logic);
  digitalWrite(MY_LED_BLUE, blue_pin_logic);
  delay(delay_blink_millis);

  // Switches off the LED
  digitalWrite(MY_LED_RED, HIGH);
  digitalWrite(MY_LED_GREEN, HIGH);
  digitalWrite(MY_LED_BLUE, HIGH);
}

// Initialise IMU
void initBNO085(void) {
  // Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    // Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    // Serial.print("Part ");
    // Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    // Serial.print(": Version :");
    // Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    // Serial.print(".");
    // Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    // Serial.print(".");
    // Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    // Serial.print(" Build ");
    // Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  // Enable dynamic calibration for accelerometer
  int result = sh2_setCalConfig(ACCELEROMETER_BIT);

  if (result == SH2_OK) {
    // Serial.println("[SUCCESS] Enable dynamic calibration successful! - accelerometer");
  } else {
    // Serial.println("[FAIL] Enable dynamic calibration unsuccessful! - accelerometer");
  }

  // Enable dynamic calibration for gyroscope
  result = sh2_setCalConfig(GYROSCOPE_BIT);

  if (result == SH2_OK) {
    // Serial.println("[SUCCESS] Enable dynamic calibration successful! - gyroscope");
  } else {
    // Serial.println("[FAIL] Enable dynamic calibration unsuccessful! - gyroscope");
  }
}

// Using quaternion information to obtain heading in [0, 360)
double quaternionHeading(Quaternion &quat) {
  // Determine Euler yaw in radians
  double yaw_rad = atan2(2.0 * (quat.d * quat.a + quat.b * quat.c), -1.0 + 2.0 * (quat.a * quat.a + quat.b * quat.b));

  // Convert to Euler yaw to degrees to obtain heading
  double heading = yaw_rad * RAD_TO_DEG;

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

float stepEstimate(float accel_max, float accel_min) {
  // Compute the difference in acceleration
  float accel_diff = accel_max - accel_min;

  // Weinberg formula for step estimation
  return kappa * sqrt(sqrt(accel_diff));  // Avoid pow() for faster computation
}

void stepDetect(float curr_accel) {
  static float prev_accel = curr_accel;       // Initially, there's no 'previous' acceleration
  static float last_max_acc = -1000;          // Start with a very low max to ensure any real max is recorded
  static float last_min_acc = 1000;           // Start with a very high min to ensure any real min is recorded
  static unsigned long last_peak_time = 0;    // Time at which the last peak was recorded
  static unsigned long last_trough_time = 0;  // Time at which the last trough was recorded

  unsigned long curr_time = millis();  // Current time in milliseconds

  // Detecting peaks
  if (curr_accel > last_max_acc) {
    last_max_acc = curr_accel;
  } else if ((curr_accel < prev_accel) && ((curr_time - last_peak_time) > tau_min)) {
    if ((last_max_acc - curr_accel) > delta_a) {  // Confirming that it is indeed a peak
      last_peak_time = curr_time;
      // Calculate the step length using the peak value
      lambda = stepEstimate(last_max_acc, last_min_acc);

      // Indicates a valid step was taken to later update user position
      step_detect_flag = true;

      // Resetting last_min_acc for next trough detection
      last_min_acc = curr_accel;

      // Resetting last_max_acc to be ready for the next peak
      last_max_acc = -1000;
    }
  }

  // Detecting troughs
  if (curr_accel < last_min_acc) {
    last_min_acc = curr_accel;
  } else if ((curr_accel > prev_accel) && ((curr_time - last_trough_time) > tau_min)) {
    if ((curr_accel - last_min_acc) > delta_a) {  // Confirming that it is indeed a trough
      last_trough_time = curr_time;
      // Resetting last_max_acc for next peak detection
      last_max_acc = curr_accel;

      // Resetting last_min_acc to be ready for the next trough
      last_min_acc = 1000;
    }
  }
  // Update the previous acceleration for the next iteration
  prev_accel = curr_accel;
}

// Pedestrian Dead Reckoning (PDR) that updates user position using IMU data
void PDR(void) {
  if (step_detect_flag == true) {
    // Take measurement end time
    t_meas_end = millis();

    // Update the experiment time
    t_exp += t_meas_end - t_meas_start;

    // Update user position
    MobileUser.updatePosition(lambda, psi);

    // Save the user position in SD card
    saveData(MobileUser.getX(), MobileUser.getY(), t_exp);

    // Reset the step detection flag
    step_detect_flag = false;

    // Update measurement start time for next measurement
    t_meas_start = millis();
  }
}

// Save the data onto an SD card
void saveData(double x, double y, unsigned long t) {
  // Now, write to the SD card if we have not reached max measurements yet
  if (measurement_count < measurement_max) {
    File file = SD.open(FILE_NAME, FILE_APPEND);
    if (file) {
      file.printf("Measurement %d: x = %.10f, y = %.10f, t = %lu\n", measurement_count + 1, x, y, t);
      file.close();
      Serial.println("Measurement written to SD card");
      measurement_count++;
    } else {
      Serial.println("Failed to open file for appending");
    }
  }

  // Check if we've reached max measurements and take necessary action
  if (measurement_count >= measurement_max) {
    // Stop scanning
    Serial.println("Reached maximum measurements. Stopping.");
    digitalWrite(MY_LED_GREEN, LOW);  // Finished writing all measurements so indicate to user
    return;
  }
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

void setup(void) {
  Serial.begin(115200);

  // Configure LED pins as output
  pinMode(MY_LED_RED, OUTPUT);
  pinMode(MY_LED_GREEN, OUTPUT);
  pinMode(MY_LED_BLUE, OUTPUT);

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    delay(1);
  }

  Serial.println("SD Card initialized.");
  // Create/open the file on setup if not exists
  File file = SD.open(FILE_NAME, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.close();

  // Write initial position
  {
    File file = SD.open(FILE_NAME, FILE_APPEND);
    file.printf("Measurement %d: x = %.6f, y = %.6f\n", measurement_count + 1, user_x0, user_y0, t_exp);
    measurement_count++;
    file.close();
    Serial.println("Measurement written to SD card");
  }

  initBNO085();

  setReports();

  Serial.println("Reading events");
  delay(100);

  // Wait until we receive a SH2_GAME_ROTATION_VECTOR sensor event
  while (!bno08x.getSensorEvent(&sensorValue) || sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
    delay(10);
  }

  // Blink RED when taking ENU measurement
  blinkRGB(LOW, HIGH, HIGH, 5000);

  // Store the initial IMU quaternion values equal to ENU frame
  IMUQuat.a = sensorValue.un.gameRotationVector.real;
  IMUQuat.b = sensorValue.un.gameRotationVector.i;
  IMUQuat.c = sensorValue.un.gameRotationVector.j;
  IMUQuat.d = sensorValue.un.gameRotationVector.k;

  // Initialise user heading with same direction as IMU quaternion
  psi = quaternionHeading(IMUQuat);

  // Initialise starting user postion
  MobileUser.setX(user_x0);
  MobileUser.setY(user_y0);
  MobileUser.setHeading(psi);

  // Indicate to user that ENU is done storing by showing GREEN
  blinkRGB(HIGH, LOW, HIGH, 5000);

  // Indicate to user that executing LOOP by showing BLUE
  digitalWrite(MY_LED_BLUE, LOW);

  // Update measurement start time
  t_meas_start = millis();
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_ACCELEROMETER) {
    // Read sensor data
    IMUAccel.b = sensorValue.un.accelerometer.x - imu_ax_offset;
    IMUAccel.c = sensorValue.un.accelerometer.y - imu_ay_offset;
    IMUAccel.d = sensorValue.un.accelerometer.z - imu_az_offset;

    // Hamilton product of normalised rotation quaternion and IMU quaternion vector to obtain ENU quaternion
    ENUAccel = IMUQuat * IMUAccel * IMUQuat.conj();

    // Find the [ax,ay,az]' independent of gravity (linear acceleration)
    enu_ax = ENUAccel.b;
    enu_ay = ENUAccel.c;
    enu_az = ENUAccel.d - g;

    // Apply low pass filter to az and store in az array
    filt_enu_az = lp.filt(enu_az);

    // Compute the step length for the acceleration sample
    stepDetect(filt_enu_az);

    // Allows for Pedestrian Dead Reckoning to update user x-y position
    PDR();
  }

  else if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    // Record sensor IMU frame quaternion measurements
    IMUQuat.a = sensorValue.un.gameRotationVector.real;
    IMUQuat.b = sensorValue.un.gameRotationVector.i;
    IMUQuat.c = sensorValue.un.gameRotationVector.j;
    IMUQuat.d = sensorValue.un.gameRotationVector.k;

    // Compute heading angle in degrees using IMU orientation
    psi = quaternionHeading(IMUQuat);
  }
}