// Kalman Filter implementation
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <LinearAlgebra.h>
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

// Define constants (IMU)
const float imu_ax_offset = 0.026628;   // IMU zero-g ax bias offset
const float imu_ay_offset = 0.026212;   // IMU zero-g ay bias offset
const float imu_az_offset = -0.017193;  // IMU zero-g az bias offset
const float g = 9.80665;                // Gravitational acceleration constant
const float fsr = 8.0;                  // Full-scale range of accelerometer is +-8g
const int order = 4;                    // Low pass filter order
const float f0 = 3.0;                   // Cutoff frequency
const float f_s = 128.2051;             // Sampling frequency
const float kappa = 0.5309;             // Weinberg coefficient constant for step length
const float delta_a = 1;                // Acceleration threshold from MATLAB
const unsigned long tau_min = 300;      // Time difference threshold from reference

// Define the constants (BLE)
const String uuid[] = { "1701", "1702", "1703" };         // Targeted UUIDs for the reference nodes
const int node_num = 3;                                   // Number of stationary advertising reference nodes
const uint16_t scan_interval = 40;                        // Scan interval in milliseconds -> Higher values means less frequent scans
const uint16_t scan_window = 20;                          // Scan window in milliseconds -> How long the scanner actively scans within each interval
const uint32_t scan_duration = 1;                         // Duration of the scan -> How long the whole scanning process is conducted
const bool scan_continue = false;                         // Set to false indicating if scan continues
const float weight = 0.20;                                // Weight for the RSSI moving average filter
const float PLE[node_num] = { 1.6, 1.6, 1.6 };            // Path Loss Exponents for the different nodes
const float RSSI_d0[node_num] = { -60.0, -60.0, -61.0 };  // RSSI values at distance of 1 m away from receiver
const float node_x[node_num] = { 0.0, 3.57, 1.79 };       // x-coordinate for reference node
const float node_y[node_num] = { 0.0, 0.0, 6.05 };        // y-coordiante for reference node

// Define variables (KF)
char FILE_NAME[] = "/EKF_data.txt";    // Name of the file written to SD card
const float user_x0 = 0;               // Initial user x-coordinate
const float user_y0 = 0.45;            // Initial user y-coordinate
const int state_size = 4;              // State vector [x, y, λ, ψ]
const int measurement_size = 2;        // Measurement space [x, y]
const int count_max = 200;             // Maximum amount of measurements
const double process_noise = 0.1;      // Omega value for process noise
const double measurement_noise = 100;  // Sigma value for measurement noise

// RGB pins on Arduino ESP32
const uint8_t MY_LED_RED = 14;    // RED
const uint8_t MY_LED_GREEN = 15;  // GREEN
const uint8_t MY_LED_BLUE = 16;   // BLUE

// Define variables (IMU)
float enu_ax = 0.0, enu_ay = 0.0, enu_az = 0.0;  // ENU [ax, ay, az]' acceleration values
float imu_ax = 0, imu_ay = 0, imu_az = 0;        // IMU [ax, ay, az]' acceleration values
float filt_enu_az = 0.0;                         // LPF ENU az value for step detection
float a_max = 0.0;                               // Initialise maximum az variable
float a_min = 0.0;                               // Initialise minimum az variable
float psi;                                       // User heading angle in degrees
float lambda = 0;                                // Step length

unsigned long prev_scan_time = 0;
const unsigned long scan_time_threshold = 5000;  // In milliseconds, adjust as needed

// Define the variables (BLE)
float curr_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Current RSSI values in array for 3 nodes
float filt_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Filtered RSSI values in array for 3 nodes
float distances[node_num] = { 0.0, 0.0, 0.0 };  // Distance between mobile receiver and the reference nodes
float dist_n1_n2, dist_n2_n3, dist_n1_n3;       // Initialise the distance between each pair of nodes
int measurement_count = 0;                      // Global variable to track the number of measurements

// Define flags (IMU)
bool step_detect_flag = false;  // Indicates if valid step has been detected, if so then updates user position

// Define flags (BLE)
bool UUID_flag[node_num] = { false, false, false };  // Flag to check if target UUID is detected

// Define flags (KF)
bool predict_kf_flag = false;
bool update_kf_flag = false;

// Define Quaternion objects
Quaternion IMUQuat;                           // Initialise IMU frame quaternion object <a,b,c,d> = <qw,qi,qj,qk> for rotation purposes
Quaternion ENUAccel(enu_ax, enu_ay, enu_az);  // Define a pure quaternion describing the acceleration vector in ENU frame
Quaternion IMUAccel(imu_ax, imu_ay, imu_az);  // Define a pure quaternion describing the acceleration vector in IMU frame
// const Quaternion enu_q; -> Define East-North-Up quaternion object with <1,0,0,0> - Not needed as initial sensor orientation is <1,0,0,0>

// 4th Order Low-Pass Butterworth filter instance
LowPass<order> lp(f0, f_s, false);  // f0 = 3 Hz, fs = 128.2051 Hz, No adaptive sampling frequency

// Create mobile user instance
UserPosition MobileUser;

// Allows us to conduct the BLE scan for the mobile receiver
BLEScan* scan;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Define the matrices (BLE)
mat A(2, 2);             // A matrix from the trilateration formula
mat At(2, 2);            // Transpose of A
mat AtA(2, 2);           // Transpose of A * A
mat AtA_inv(2, 2);       // Inverse of (A transpose * A)
mat b(2, 1);             // B matrix from the trilateration formula
mat lsq_position(2, 1);  // Store the resulting x-y least squares results in matrix form

// Define the matrices for Kalman Filter (KF)
mat x_kf = mat::zeros(state_size, 1);                 // State vector [x, y, λ, ψ]
mat P_kf = mat::identity(state_size);                 // Error covariance matrix
mat F_kf = mat::identity(state_size);                 // State transition matrix
mat gradf_kf = mat::identity(state_size);             // Jacobian process model matrix
mat Q_kf = mat::identity(state_size);                 // Process noise covariance matrix
mat H_kf = mat::zeros(measurement_size, state_size);  // Observation matrix
mat R_kf = mat::identity(measurement_size);           // Measurement noise covariance matrix
mat I_kf = mat::identity(state_size);                 // Identity matrix
mat S_kf = mat::identity(measurement_size);           // Innovation covariance matrix
mat K_kf = mat::zeros(state_size, measurement_size);  // Kalman gain matrix
mat y_kf = mat::zeros(measurement_size, 1);           // Residual matrix
mat z_kf = mat::zeros(measurement_size, 1);           // Measurement vector in matrix form
mat H_transpose(state_size, measurement_size);
mat PH_transpose(state_size, measurement_size);
mat HPH_transpose(state_size, measurement_size);

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

mat inverse2x2Mat(const mat& M_) {
  mat M = M_;
  double a, b, c, d;
  double det;
  a = M(0, 0);
  b = M(0, 1);
  c = M(1, 0);
  d = M(1, 1);

  det = (a * d - b * c);
  M(0, 0) = (1 / det) * d;
  M(0, 1) = (-1 / det) * b;
  M(1, 0) = (-1 / det) * c;
  M(1, 1) = (1 / det) * a;

  return M;
}

/********************************
********* IMU FUNCTIONS *********
*********************************/

// Initialise the IMU sensor
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
float quaternionHeading(Quaternion& quat) {
  // Determine Euler yaw in radians
  float yaw_rad = atan2(2.0 * (quat.d * quat.a + quat.b * quat.c), -1.0 + 2.0 * (quat.a * quat.a + quat.b * quat.b));

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

// Using the Weinberg model, find lambda using the difference in acceleration values
float stepEstimate(float accel_max, float accel_min) {
  // Compute the difference in acceleration
  float accel_diff = accel_max - accel_min;

  // Weinberg formula for step estimation
  return kappa * sqrt(sqrt(accel_diff));  // Avoid pow() for faster computation
}

// Peak detection logic to see if a valid step was taken
void stepDetect(float curr_accel) {
  static float prev_accel = curr_accel;       // Initially, there's no 'previous' acceleration
  static float last_max_acc = -1000;          // Start with a very low max to ensure any real max is recorded
  static float last_min_acc = 1000;           // Start with a very high min to ensure any real min is recorded
  static unsigned long last_peak_time = 0;    // Time at which the last peak was recorded
  static unsigned long last_trough_time = 0;  // Time at which the last trough was recorded

  // Serial.print("Current acceleration: ");
  // Serial.println(curr_accel);
  // Serial.print("Last max acceleration: ");
  // Serial.println(last_max_acc);
  // Serial.print("Last min acceleration: ");
  // Serial.println(last_min_acc);
  // Serial.print("Step detected flag: ");
  // Serial.println(step_detect_flag ? "true" : "false");

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

      // Print the peak and step length
      // Serial.print("a_max = ");
      // Serial.print(last_max_acc);
      // Serial.print(" a_min = ");
      // Serial.print(last_min_acc);
      // Serial.print(" Step length [m]: ");
      // Serial.println(lambda);

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

// Predict KF using Pedestrian Dead Reckoning (PDR) that updates user position using IMU data
void PDR(void) {
  if (step_detect_flag == true) {
    // Update mobile user's x-y position using the step length and heading values
    MobileUser.updatePosition(lambda, psi);

    // Print x-y position
    // Serial.print("x = ");
    // Serial.print(MobileUser.getX());
    // Serial.print(" y = ");
    // Serial.print(MobileUser.getY());
    // Serial.print(" psi = ");
    // Serial.println(psi);

    // Serial.print(MobileUser.getX());
    // Serial.print(",");
    // Serial.println(MobileUser.getY());

    // Reset the step detection flag
    step_detect_flag = false;
    predict_kf_flag = true;
  }
}

void checkIMUReady(void) {
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

    // Predict Kalman Filter
    predictKF();
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

/********************************
********* BLE FUNCTIONS *********
*********************************/

// Apply a weighter filter based on previous RSSI value and current RSSI
void movingAverageFilter(BLEScanResults results, float alpha) {
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    for (int j = 0; j < node_num; j++) {  // For 3 beacons
      if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID(uuid[j].c_str()))) {
        UUID_flag[j] = true;  // Set flag for valid UUID detection
        if (filt_RSSI[j] == 0.0) {
          // Initialised the filtered RSSI value to be equal to the current RSSI value
          filt_RSSI[j] = device.getRSSI();
        } else {  // Occurs after first value intialised properly
          // Obtain current RSSI value
          curr_RSSI[j] = device.getRSSI();

          // Apply moving average filter equation
          filt_RSSI[j] = alpha * curr_RSSI[j] + (1.0 - alpha) * filt_RSSI[j];
          // Serial.print("Service UUID ");
          // Serial.print(uuid[j]);
          // Serial.print(": RSSI value = ");
          // Serial.println(filt_RSSI[j]);
        }
      }
    }
  }
}

// Applies log-distance path loss model to compute the distance
void RSSIToDistance(float RSSI[], const int size) {
  for (int i = 0; i < size; i++) {
    if (UUID_flag[i]) {  // Only calculate if UUID was detected
      // Compute distances using RSSI path loss model
      distances[i] = pow(10.0, (RSSI_d0[i] - RSSI[i]) / (10 * PLE[i]));
      // Serial.print("Node ");
      // Serial.print(i + 1);
      // Serial.print(": Distance = ");
      // Serial.println(distances[i]);
    }
  }
}

// Logic to check if there are at least 3 intersections for the trilateration
bool checkIntersections(const float d[]) {

  // Check if there are intersections
  bool intersection_n1_n2 = dist_n1_n2 < (d[0] + d[1]);
  bool intersection_n2_n3 = dist_n2_n3 < (d[1] + d[2]);
  bool intersection_n1_n3 = dist_n1_n3 < (d[0] + d[2]);

  return intersection_n1_n2 && intersection_n2_n3 && intersection_n1_n3;
}

// If all UUIDs are detected, all 3 nodes are transmitting
bool allUUIDsDetected(const bool flag[], int size) {
  for (int i = 0; i < size; i++) {
    if (!flag[i]) {
      return false;  // If any flag is false, not all UUIDs are detected
    }
  }
  return true;  // All flags are true, all UUIDs are detected
}

// Perform the trilateration using least squares
void trilateration(void) {
  if (allUUIDsDetected(UUID_flag, node_num)) {
    if (checkIntersections(distances)) {
      // Compute b based on dynamic distances
      b(0, 0) = pow(node_x[0], 2) - pow(node_x[2], 2) + pow(node_y[0], 2) - pow(node_y[2], 2) + pow(distances[2], 2) - pow(distances[0], 2);
      b(1, 0) = pow(node_x[1], 2) - pow(node_x[2], 2) + pow(node_y[1], 2) - pow(node_y[2], 2) + pow(distances[2], 2) - pow(distances[1], 2);

      // Compute the result using precomputed inverse
      lsq_position = AtA_inv * At * b;

      update_kf_flag = true;

      // Print least squares results
      // Serial.print("x = ");
      // Serial.print(lsq_position(0, 0));
      // Serial.print(" y = ");
      // Serial.println(lsq_position(1, 0));

      // saveData(lsq_x, lsq_y);
    }
  }
}

/******************************************
********* SENSOR FUSION FUNCTIONS *********
******************************************/

// Save the data onto an SD card
void saveData(float x, float y) {
  // Now, write to the SD card if we have not reached 50 measurements yet
  if (measurement_count < count_max) {
    File file = SD.open(FILE_NAME, FILE_APPEND);
    if (file) {
      file.printf("Measurement %d: x = %.6f, y = %.6f\n", measurement_count + 1, x, y);
      file.close();
      Serial.println("Measurement written to SD card");
      measurement_count++;
    } else {
      Serial.println("Failed to open file for appending");
    }
  }

  // Check if we've reached 50 measurements and take necessary action
  if (measurement_count >= count_max) {
    // Stop scanning
    Serial.println("Reached 50 measurements. Stopping.");
    digitalWrite(MY_LED_GREEN, LOW);  // Finished writing all measurements so indicate to user
    return;
  }
}

// Initialise the fusion algorithim with appropriate matrices
void setStateSpace(void) {
  x_kf(0, 0) = MobileUser.getX();
  x_kf(1, 0) = MobileUser.getY();
  x_kf(2, 0) = MobileUser.getStepLength();
  x_kf(3, 0) = MobileUser.getHeading() * DEG_TO_RAD;
}

void setMeasurementSpace(void) {
  int i = 1;
  H_kf(0, 0) = i;
  H_kf(1, 1) = i;
}

void setNoiseCovariances(const double omega, const double sigma) {
  Q_kf = Q_kf * omega;  // Multiply by process noise omega
  R_kf = R_kf * sigma;  // Multiply by measurement noise sigma
}

void predictKF(void) {
  if (predict_kf_flag == true) {
    setStateSpace();

    F_kf(0, 2) = cos(psi * DEG_TO_RAD);
    F_kf(1, 2) = sin(psi * DEG_TO_RAD);

    x_kf = F_kf * x_kf;

    gradf_kf(0, 2) = cos(psi * DEG_TO_RAD);
    gradf_kf(0, 3) = -lambda * sin(psi * DEG_TO_RAD);
    gradf_kf(1, 2) = sin(psi * DEG_TO_RAD);
    gradf_kf(1, 3) = lambda * cos(psi * DEG_TO_RAD);

    P_kf = gradf_kf * P_kf * gradf_kf.t() + Q_kf;

    // printMatToSerial(P_kf);
    // printMatToSerial(F_kf);
    // printMatToSerial(Q_kf);

    Serial.print("PREDICTED!!! X = ");
    Serial.print(x_kf(0, 0));
    Serial.print(" Y = ");
    Serial.println(x_kf(1, 0));
  }
}

void calcInnovationCov(void) {
  // Compute the Innovation covariance matrix
  S_kf = HPH_transpose + R_kf;  // Innovation covariance
}

void calcKalmanGain(void) {
  // Compute the Kalman gain
  mat S_inv = inverse2x2Mat(S_kf);
  Serial.print("S_inv after");
  printMatToSerial(S_inv);
  K_kf = PH_transpose * S_inv;  // Kalman gain
}

void updateKF(void) {
  if (update_kf_flag == true) {
    // Set Least Squares solution as measurement input
    z_kf = lsq_position;

    // Measurement residual
    y_kf = z_kf - H_kf * x_kf;

    Serial.print("S before");
    printMatToSerial(S_kf);
    // Compute the Innovation covariance matrix
    calcInnovationCov();
    Serial.print("S after");
    printMatToSerial(S_kf);

    calcKalmanGain();

    // // Update the estimate via measurement from z
    x_kf = x_kf + K_kf * y_kf;           // Updated (a posteriori) state estimate
    P_kf = (I_kf - K_kf * H_kf) * P_kf;  // Updated (a posteriori) error covariance matrix

    Serial.print("UPDATED! X = ");
    Serial.print(x_kf(0, 0));
    Serial.print(" Y = ");
    Serial.println(x_kf(1, 0));
  }
}

// For debugging
void printMatToSerial(mat& matrix) {
  String matrixString;
  matrix.print(matrixString);
  Serial.println(matrixString);
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

  setNoiseCovariances(process_noise, measurement_noise);
  setMeasurementSpace();

  // Initialise BLE scan
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(scan_interval);
  scan->setWindow(scan_window);

  // Compute the distance requirements for the trilateration
  dist_n1_n2 = sqrt(pow(node_x[0] - node_x[1], 2) + pow(node_y[0] - node_y[1], 2));
  dist_n2_n3 = sqrt(pow(node_x[1] - node_x[2], 2) + pow(node_y[1] - node_y[2], 2));
  dist_n1_n3 = sqrt(pow(node_x[0] - node_x[2], 2) + pow(node_y[0] - node_y[2], 2));

  // Compute A based on fixed node positions
  A(0, 0) = 2 * (node_x[0] - node_x[2]);
  A(0, 1) = 2 * (node_y[0] - node_y[2]);
  A(1, 0) = 2 * (node_x[1] - node_x[2]);
  A(1, 1) = 2 * (node_y[1] - node_y[2]);

  // Compute A transpose
  At = A.t();

  // Compute A^T * A
  AtA = At * A;

  // Compute inverse of (A^T * A)
  AtA_inv = inverse2x2Mat(AtA);

  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
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
    file.printf("Measurement %d: x = %.6f, y = %.6f\n", measurement_count + 1, user_x0, user_y0);
    file.close();
    Serial.println("Measurement written to SD card");
  }

  // Initialise IMU
  initBNO085();

  setReports();

  Serial.println("Reading events");
  delay(100);

  // Wait until we receive a SH2_GAME_ROTATION_VECTOR sensor event
  while (!bno08x.getSensorEvent(&sensorValue) || sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
    delay(10);
  }

  // Blink YELLOW when taking ENU measurement
  blinkRGB(LOW, LOW, HIGH, 5000);

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

  setStateSpace();

  H_transpose = H_kf.t();
  PH_transpose = P_kf * H_transpose;
  HPH_transpose = H_kf * PH_transpose;

  // Print initial starting position
  // Serial.print("x0 = ");
  // Serial.print(MobileUser.getX());
  // Serial.print(" y0 = ");
  // Serial.print(MobileUser.getY());
  // Serial.print(" psi0 = ");
  // Serial.println(psi);

  // Indicate to user that ENU is done storing by showing GREEN
  blinkRGB(HIGH, LOW, HIGH, 10000);

  // Indicate to user that executing LOOP by showing BLUE
  digitalWrite(MY_LED_BLUE, LOW);
}

void loop() {

  if (bno08x.wasReset()) {
    Serial.print("Sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  checkIMUReady();

  // Then handle BLE scanning at reduced frequency
  unsigned long scan_curr_time = millis();
  if (scan_curr_time - prev_scan_time > scan_time_threshold) {

    // Reset the UUID flags before scanning
    for (int i = 0; i < node_num; i++) {
      UUID_flag[i] = false;
    }

    // Initiate scan
    BLEScanResults results = scan->start(scan_duration, scan_continue);

    // Apply RSSI filter
    movingAverageFilter(results, weight);

    // Clear scan results
    scan->clearResults();

    // Apply log-distance path loss model
    RSSIToDistance(filt_RSSI, node_num);

    // Compute least squares trilateration
    trilateration();

    // Compute least squares trilateration and update Kalman Filter
    updateKF();

    if (update_kf_flag == true) {
      saveData(x_kf(0, 0), x_kf(1, 0));
      update_kf_flag = false;
    }

    prev_scan_time = scan_curr_time;
  }

  if (predict_kf_flag == true) {
    saveData(x_kf(0, 0), x_kf(1, 0));
    predict_kf_flag = false;
  }
}