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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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

// RGB pins on Arduino ESP32
const uint8_t MY_LED_RED = 14;    // RED
const uint8_t MY_LED_GREEN = 15;  // GREEN
const uint8_t MY_LED_BLUE = 16;   // BLUE

// Time variables
unsigned long t_exp = 0;         // Total experiment time
unsigned long t_meas_start = 0;  // Measurement start time
unsigned long t_meas_end = 0;    // Measurement end time

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
const float RSSI_d0[node_num] = { -61.0, -60.0, -62.0 };  // RSSI values at distance of 1 m away from receiver
const float node_x[node_num] = { 0.0, 3.50, 2.45 };       // x-coordinate for reference node
const float node_y[node_num] = { 0.0, 0.0, 5.30 };        // y-coordiante for reference node
const float condition_num_threshold = 1e3;                // Condition number 10^3
const float outlier_threshold_x = 5.0;                    // Outlier threshold for x for rejection purposes
const float outlier_threshold_y = 6.0;                    // Outlier threshold for x for rejection purposes

// Define variables (IMU)
float enu_ax = 0.0, enu_ay = 0.0, enu_az = 0.0;  // ENU [ax, ay, az]' acceleration values
float imu_ax = 0, imu_ay = 0, imu_az = 0;        // IMU [ax, ay, az]' acceleration values
float filt_enu_az = 0.0;                         // LPF ENU az value for step detection
float a_max = 0.0;                               // Initialise maximum az variable
float a_min = 0.0;                               // Initialise minimum az variable
float psi;                                       // User heading angle in degrees
float lambda = 0;                                // Step length

// Define the variables (BLE)
float curr_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Current RSSI values in array for 3 nodes
float filt_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Filtered RSSI values in array for 3 nodes
float distances[node_num] = { 0.0, 0.0, 0.0 };  // Distance between mobile receiver and the reference nodes
float dist_n1_n2, dist_n2_n3, dist_n1_n3;       // Initialise the distance between each pair of nodes
float lsq_x, lsq_y;                             // Least Squares x-y coordinates
int measurement_count = 0;                      // Global variable to track the number of measurements

// Define variables (KF)
char FILE_NAME[] = "/KF_FC132_Final.txt";  // Name of the file written to SD card
const float user_x0 = 3.50;                // Initial user x0
const float user_y0 = 0.40;                // Initial user y0
const int state_size = 2;                  // State vector [x, y]
const int measurement_size = 2;            // Measurement space [x, y]
const int measurement_max = 200;           // Maximum amount of measurements
const double process_noise = 0.001;        // Omega value for process noise
const double measurement_noise = 200;      // Sigma value for measurement noise
const double initial_error = 0.01;         // Initial error covariance noise

// Define flags (IMU)
bool step_detect_flag = false;   // Indicates if valid step has been detected, if so then updates user position
bool step_detect_reset = false;  // This step detection flag resets all static variables

// Define flags (BLE)
bool UUID_flag[node_num] = { false, false, false };  // Flag to check if target UUID is detected
bool MATRIX_ERROR_FLAG = false;                      // Flag to check if matrix is singular and/or ill-condition

// Define flags (KF)
bool predict_kf_flag = false;
bool update_kf_flag = false;

// Define Quaternion objects
Quaternion IMUQuat;                           // Initialise IMU frame quaternion object <a,b,c,d> = <qw,qi,qj,qk> for rotation purposes
Quaternion ENUAccel(enu_ax, enu_ay, enu_az);  // Define a pure quaternion describing the acceleration vector in ENU frame
Quaternion IMUAccel(imu_ax, imu_ay, imu_az);  // Define a pure quaternion describing the acceleration vector in IMU frame

// 4th Order Low-Pass Butterworth filter instance
LowPass<order> lp(f0, f_s, false);  // f0 = 3 Hz, fs = 128.2051 Hz, No adaptive sampling frequency

// Create mobile user instance
UserPosition MobileUser;

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
mat Q_kf = mat::identity(state_size);                 // Process noise covariance matrix
mat H_kf = mat::identity(measurement_size);           // Observation matrix
mat R_kf = mat::identity(measurement_size);           // Measurement noise covariance matrix
mat I_kf = mat::identity(state_size);                 // Identity matrix
mat S_kf = mat::identity(measurement_size);           // Innovation covariance matrix
mat K_kf = mat::zeros(state_size, measurement_size);  // Kalman gain matrix
mat y_kf = mat::zeros(measurement_size, 1);           // Residual matrix
mat z_kf = mat::zeros(measurement_size, 1);           // Measurement vector in matrix form

/************************************
********* GENERAL FUNCTIONS *********
************************************/

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

// Printing matrices to serial for debugging
void printMatToSerial(mat& matrix) {
  String matrixString;
  matrix.print(matrixString);
  Serial.println(matrixString);
}

/***********************************
********* MATRIX FUNCTIONS *********
***********************************/

// Determinant for 2x2 matrix
double det2x2Mat(const mat& M_) {
  mat M = M_;
  double a, b, c, d;
  double det;
  a = M(0, 0);
  b = M(0, 1);
  c = M(1, 0);
  d = M(1, 1);

  return det = (a * d - b * c);
}

// Inverse for 2x2 matrix
mat inverse2x2Mat(const mat& M_) {
  mat M = M_;
  double a, b, c, d;
  double det;
  double inv_det;
  a = M(0, 0);
  b = M(0, 1);
  c = M(1, 0);
  d = M(1, 1);

  det = (a * d - b * c);
  inv_det = 1.0 / det;
  M(0, 0) = inv_det * d;
  M(0, 1) = -inv_det * b;
  M(1, 0) = -inv_det * c;
  M(1, 1) = inv_det * a;

  return M;
}

// Checks if matrix is singular
void checkMatrixSingular(const mat& M_) {
  mat M = M_;
  if (abs(det2x2Mat(M)) < 1e-3) {
    Serial.println("[ERROR] Matrix is close to singular!");
    MATRIX_ERROR_FLAG = true;
  }

  while (MATRIX_ERROR_FLAG) {
    // Flash RED LED every 1 second signify singular matrix error
    blinkRGB(LOW, HIGH, HIGH, 1000);
  }
}

// Function to calculate the Euclidean norm of a matrix
double euclideanNorm(const mat& M_) {
  mat M = M_;
  double sum = 0.0;
  uint8_t rows = M.get_rows();
  uint8_t cols = M.get_cols();
  for (uint8_t i = 0; i < rows; i++) {
    for (uint8_t j = 0; j < cols; j++) {
      sum += M(i, j) * M(i, j);
    }
  }
  return sqrt(sum);
}

// Checks if matrix has valid condition number
void checkConditionNumber(const mat& M_) {
  mat M = M_;
  mat M_inv = inverse2x2Mat(M);
  double M_norm = euclideanNorm(M);
  double M_inv_norm = euclideanNorm(M_inv);

  // Compute condition number
  double condition_num = M_norm * M_inv_norm;

  if (condition_num > condition_num_threshold) {
    Serial.println("[ERROR] Matrix has high conditon number!");
    MATRIX_ERROR_FLAG = true;
  }

  while (MATRIX_ERROR_FLAG) {
    // Flash RED LED every 1 second signify ill-conditioned matrix error
    blinkRGB(LOW, HIGH, HIGH, 1000);
  }
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
void stepDetect(float curr_accel, bool RESET) {
  static float prev_accel = curr_accel;       // Initially, there's no 'previous' acceleration
  static float last_max_acc = -1000;          // Start with a very low max to ensure any real max is recorded
  static float last_min_acc = 1000;           // Start with a very high min to ensure any real min is recorded
  static unsigned long last_peak_time = 0;    // Time at which the last peak was recorded
  static unsigned long last_trough_time = 0;  // Time at which the last trough was recorded

  unsigned long curr_time = millis();  // Current time in milliseconds

  if (RESET) {
    step_detect_reset = false;     // Deassert the reset flag
    last_max_acc = -1000;          // Reset max acceleration
    last_min_acc = 1000;           // Reset minimum acceleration
    last_peak_time = curr_time;    // Set to current time to avoid immediate false detection
    last_trough_time = curr_time;  // Set to current time to avoid immediate false detection
    prev_accel = curr_accel;       // Reset acceleration detection to current point
    return;                        // Exit the function after reset
  }

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

// User position is now handled inside Kalman filter prediction
void PDR(void) {
  if (step_detect_flag == true) {
    // Reset the step detection flag
    step_detect_flag = false;
    predict_kf_flag = true;
  }
}

/********************************
********* BLE FUNCTIONS *********
*********************************/

// Apply a weighted filter based on previous RSSI value and current RSSI
void movingAverageFilter(BLEAdvertisedDevice device, float weight) {
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
        filt_RSSI[j] = weight * curr_RSSI[j] + (1.0 - weight) * filt_RSSI[j];
        // Serial.print("Service UUID ");
        // Serial.print(uuid[j]);
        // Serial.print(": RSSI value = ");
        // Serial.println(filt_RSSI[j]);
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

      // Result now contains the estimated x and y position
      lsq_x = lsq_position(0, 0);
      lsq_y = lsq_position(1, 0);

      if ((fabs(lsq_x) < outlier_threshold_x) && (fabs(lsq_y) < outlier_threshold_y)) {
        update_kf_flag = true;
      }
    }
  }
}

// Initialise the reference nodes
void initNodes(void) {
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

  // Check if AtA is singular
  checkMatrixSingular(AtA);

  // Compute inverse of (A^T * A)
  AtA_inv = inverse2x2Mat(AtA);

  // Check if valid condition number
  checkConditionNumber(AtA_inv);
}

/*********************
****MULTITHREADING****
*********************/

SemaphoreHandle_t mutex;  // Mutex for thread-safe access

void scanTask(void* pvParameters) {
  BLEDevice::init("");                       // Initialize the device
  BLEScan* pBLEScan = BLEDevice::getScan();  // Correct use to get a scanning object
  pBLEScan->setActiveScan(true);             // Set scan type

  while (1) {
    // Reset the UUID flags before scanning
    for (int i = 0; i < node_num; i++) {
      UUID_flag[i] = false;
    }
    BLEScanResults results = pBLEScan->start(1, false);
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < results.getCount(); i++) {
      BLEAdvertisedDevice device = results.getDevice(i);
      // Compute the moving average filter
      movingAverageFilter(device, 0.20);

      // Apply log-distance path loss model
      RSSIToDistance(filt_RSSI, node_num);

      // Compute least squares trilateration
      trilateration();
    }
    xSemaphoreGive(mutex);
  }
}

/******************************************
********* SENSOR FUSION FUNCTIONS *********
******************************************/

// Sets the state vector elements
void setStateSpace(float x_hat, float y_hat) {
  x_kf(0, 0) = x_hat;
  x_kf(1, 0) = y_hat;
}

// Define the noise covariances
void setNoiseCovariances(const double omega, const double sigma) {
  Q_kf = Q_kf * omega;  // Multiply by process noise omega
  R_kf = R_kf * sigma;  // Multiply by measurement noise sigma
}

void setErrorCovariance(const double p_0) {
  P_kf = P_kf * p_0;
}

// Kalman Filter prediction
void predictKF(void) {
  if (predict_kf_flag == true) {
    // Perform state estimate prediction
    x_kf(0, 0) += lambda * cos(psi * DEG_TO_RAD);
    x_kf(1, 0) += lambda * sin(psi * DEG_TO_RAD);

    // Compute error covariance
    P_kf = F_kf * P_kf * F_kf.t() + Q_kf;
  }
}

// Kalman filter update
void updateKF(void) {
  if (update_kf_flag == true) {
    // Set Least Squares solution as measurement input
    z_kf = lsq_position;

    // Innovation
    y_kf = z_kf - H_kf * x_kf;

    // Innovation covariance
    S_kf = H_kf * P_kf * H_kf.t() + R_kf;

    // Compute Kalman gain
    K_kf = P_kf * H_kf.t() * inverse2x2Mat(S_kf);

    // Update the estimate via measurement from z
    x_kf = x_kf + K_kf * y_kf;           // Updated (a posteriori) state estimate
    P_kf = (I_kf - K_kf * H_kf) * P_kf;  // Updated (a posteriori) error covariance matrix
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

  // Create mutex
  mutex = xSemaphoreCreateMutex();

  // Start multithreaded task
  xTaskCreate(scanTask, "ScanTask", 10000, NULL, 1, NULL);

  // Configure LED pins as output
  pinMode(MY_LED_RED, OUTPUT);
  pinMode(MY_LED_GREEN, OUTPUT);
  pinMode(MY_LED_BLUE, OUTPUT);

  // Initialise process and measurement noise covariances
  setNoiseCovariances(process_noise, measurement_noise);

  // Initialise error covariance with some noise
  setErrorCovariance(initial_error);

  // Initialise the reference nodes
  initNodes();

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
    file.printf("Measurement %d: x = %.10f, y = %.10f, t = %lu\n", measurement_count + 1, user_x0, user_y0, t_exp);
    measurement_count++;
    file.close();
    Serial.println("Measurement written to SD card");
  }

  // Initialise IMU
  initBNO085();

  // Set sensor reports for IMU
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

  // Initialise state space
  setStateSpace(user_x0, user_y0);

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

  // Check IMU sensors for Kalman Filter prediction
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
    stepDetect(filt_enu_az, step_detect_reset);

    // Allows for Pedestrian Dead Reckoning to update user x-y position
    PDR();

    // Predict Kalman Filter
    predictKF();

    // Save prediction data
    if (predict_kf_flag == true) {
      // Take measurement end time
      t_meas_end = millis();

      // Update the experiment time
      t_exp += t_meas_end - t_meas_start;

      // Save data onto SD card
      saveData(x_kf(0, 0), x_kf(1, 0), t_exp);

      // Deassert flag for next measurement
      predict_kf_flag = false;

      // Update measurement start time for next measurement
      t_meas_start = millis();
    }
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

  if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE) {
    // Compute least squares trilateration and update Kalman Filter
    updateKF();

    // Save the data to the SD card
    if (update_kf_flag == true) {
      // Take measurement end time
      t_meas_end = millis();

      // Update the experiment time
      t_exp += t_meas_end - t_meas_start;

      // Save data onto SD card
      saveData(x_kf(0, 0), x_kf(1, 0), t_exp);

      // Deassert flag for next measurement
      update_kf_flag = false;

      // Update measurement start time for next measurement
      t_meas_start = millis();
    }
    xSemaphoreGive(mutex);
  }
}