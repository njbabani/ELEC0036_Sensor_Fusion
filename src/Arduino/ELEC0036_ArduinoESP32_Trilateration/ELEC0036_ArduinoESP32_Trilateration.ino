#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <math.h>
#include <LinearAlgebra.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

BLEScan* scan;

// Define the constants
const String uuid[] = { "1701", "1702", "1703" };         // Targeted UUIDs for the reference nodes
const int node_num = 3;                                   // Number of stationary advertising reference nodes
const int measurement_max = 200;                          // Maximum amount of measurements
const uint16_t scan_interval = 40;                        // Scan interval in milliseconds -> Higher values means less frequent scans
const uint16_t scan_window = 20;                          // Scan window in milliseconds -> How long the scanner actively scans within each interval
const uint32_t scan_duration = 1;                         // Duration of the scan -> How long the whole scanning process is conducted
const bool scan_continue = false;                         // Set to false indicating if scan continues
const float eta = 0.20;                                   // Weight for the RSSI moving average filter
const float user_x0 = 3.50;                               // Initial user x0
const float user_y0 = 0.40;                               // Initial user y0
const float PLE[node_num] = { 1.60, 1.60, 1.60 };         // Path Loss Exponents for the different nodes
const float RSSI_d0[node_num] = { -61.0, -60.0, -62.0 };  // RSSI values at distance of 1 m away from receiver
const float node_x[node_num] = { 0.0, 3.50, 2.45 };       // x-coordinate for reference node
const float node_y[node_num] = { 0.0, 0.0, 5.30 };        // y-coordiante for reference node
const float condition_num_threshold = 1e3;                // Condition number 10^3
const float outlier_threshold_x = 5.0;                    // Outlier threshold for x for rejection purposes
const float outlier_threshold_y = 6.0;                    // Outlier threshold for x for rejection purposes

// RGB pins on Arduino ESP32
const uint8_t MY_LED_RED = 14;    // RED
const uint8_t MY_LED_GREEN = 15;  // GREEN
const uint8_t MY_LED_BLUE = 16;   // BLUE

// Define the variables
float curr_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Current RSSI values in array for 3 nodes
float filt_RSSI[node_num] = { 0.0, 0.0, 0.0 };  // Filtered RSSI values in array for 3 nodes
float distances[node_num] = { 0.0, 0.0, 0.0 };  // Distance between mobile receiver and the reference nodes
float dist_n1_n2, dist_n2_n3, dist_n1_n3;       // Initialise the distance between each pair of nodes
float lsq_x, lsq_y;                             // Least Squares x-y coordinates
char FILE_NAME[] = "/LSQ_FC132_Final.txt";      // Name of the file written to SD card
int measurement_count = 0;                      // Global variable to track the number of measurements
unsigned long t_exp = 0;                        // Total experiment time
unsigned long t_meas_start = 0;                 // Measurement start time
unsigned long t_meas_end = 0;                   // Measurement end time

// Define the matrices
mat A(2, 2);
mat At(2, 2);
mat AtA_inv(2, 2);
mat AtA(2, 2);
mat b(2, 1);
mat lsq_position(2, 1);

// Define flags
bool UUID_flag[node_num] = { false, false, false };  // Flag to check if target UUID is detected
bool MATRIX_ERROR_FLAG = false;                      // Flag to check if matrix is singular and/or ill-condition

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

void movingAverageFilter(BLEScanResults results, float weight) {
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
          filt_RSSI[j] = weight * curr_RSSI[j] + (1.0 - weight) * filt_RSSI[j];
          // Serial.print("Service UUID ");
          // Serial.print(uuid[j]);
          // Serial.print(": RSSI value = ");
          // Serial.println(filt_RSSI[j]);
        }
      }
    }
  }
}

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

bool checkIntersections(const float d[]) {

  // Check if there are intersections
  bool intersection_n1_n2 = dist_n1_n2 < (d[0] + d[1]);
  bool intersection_n2_n3 = dist_n2_n3 < (d[1] + d[2]);
  bool intersection_n1_n3 = dist_n1_n3 < (d[0] + d[2]);

  return intersection_n1_n2 && intersection_n2_n3 && intersection_n1_n3;
}

bool allUUIDsDetected(const bool flag[], int size) {
  for (int i = 0; i < size; i++) {
    if (!flag[i]) {
      return false;  // If any flag is false, not all UUIDs are detected
    }
  }
  return true;  // All flags are true, all UUIDs are detected
}

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

      // Print least squares results
      Serial.print("x = ");
      Serial.print(lsq_x);
      Serial.print(" y = ");
      Serial.println(lsq_y);

      // // Take measurement end time
      // t_meas_end = millis();

      // // Update the experiment time
      // t_exp += t_meas_end - t_meas_start;

      // // Save data onto SD card
      // saveData(lsq_x, lsq_y, t_exp);

      // // Update measurement start time for next measurement
      // t_meas_start = millis();


      if ((fabs(lsq_x) < outlier_threshold_x) && (fabs(lsq_y) < outlier_threshold_y)) {
        // Take measurement end time
        t_meas_end = millis();

        // Update the experiment time
        t_exp += t_meas_end - t_meas_start;

        // Save data onto SD card
        saveData(lsq_x, lsq_y, t_exp);

        // Update measurement start time for next measurement
        t_meas_start = millis();
      }
    }
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

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(scan_interval);
  scan->setWindow(scan_window);

  // Configure LED pins as output
  pinMode(MY_LED_RED, OUTPUT);
  pinMode(MY_LED_GREEN, OUTPUT);
  pinMode(MY_LED_BLUE, OUTPUT);

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

  // Indicate to user that ENU is done storing by showing GREEN
  blinkRGB(HIGH, LOW, HIGH, 5000);

  // Indicate to user that executing LOOP by showing BLUE
  digitalWrite(MY_LED_BLUE, LOW);

  // Update measurement start time
  t_meas_start = millis();
}

void loop() {
  // Reset the UUID flags before scanning
  for (int i = 0; i < node_num; i++) {
    UUID_flag[i] = false;
  }

  // Initiate scan
  BLEScanResults results = scan->start(scan_duration, scan_continue);

  // Apply RSSI filter
  movingAverageFilter(results, eta);

  // Apply log-distance path loss model
  RSSIToDistance(filt_RSSI, node_num);

  // Compute least squares trilateration
  trilateration();

  // Clear scan results
  scan->clearResults();
}