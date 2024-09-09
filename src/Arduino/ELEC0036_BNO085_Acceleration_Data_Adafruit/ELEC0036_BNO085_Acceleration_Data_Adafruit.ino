// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

// Define constants
const float scaleRadToDeg = 180.0 / 3.14159265358; // Scale from radians to degrees for Gyroscope
const float g = 9.80665; // Gravitational acceleration constant
const float fsr = 8.0; // Full-scale range of accelerometer is +-8g

// Define variables
float raw_linear_accel, imu_linear_accel, cal_linear_accel;
float raw_ax, raw_ay, raw_az;
float cal_ax, cal_ay, cal_az;
float imu_ax, imu_ay, imu_az;
float ax, ay, az;

// Define zero-g offset bias for calibration
const float ax_offset = -0.101330;
const float ay_offset = -0.592714;
const float az_offset = -0.103242;
const float imu_ax_offset = 0.026628;
const float imu_ay_offset = 0.026212;
const float imu_az_offset = -0.017193;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

float get_linear_accel (float ax, float ay, float az){
  float linearAccel = sqrt(sq(ax)+sq(ay)+sq(az));
  return linearAccel;
}

void setup(void) {
  Serial.begin(115200);
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

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void loop() {
  //delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

  case SH2_ACCELEROMETER:
    // Serial.print("Accelerometer [m/s^2] - x: ");
    // Serial.print(sensorValue.un.accelerometer.x);
    // Serial.print(" y: ");
    // Serial.print(sensorValue.un.accelerometer.y);
    // Serial.print(" z: ");
    // Serial.println(sensorValue.un.accelerometer.z);

    // ax = sensorValue.un.accelerometer.x;
    // ay = sensorValue.un.accelerometer.y;
    // az = sensorValue.un.accelerometer.z;
    ax = sensorValue.un.accelerometer.x - imu_ax_offset;
    ay = sensorValue.un.accelerometer.y - imu_ay_offset;
    az = sensorValue.un.accelerometer.z - imu_az_offset;

    // Serial.print(ax, 6);
    // Serial.print(",");
    // Serial.print(ay, 6);
    // Serial.print(",");
    // Serial.println(az, 6);

    imu_linear_accel = get_linear_accel(ax, ay, az) - g;
    Serial.println(imu_linear_accel);
    break;
  // case SH2_RAW_ACCELEROMETER:
  //   // Convert ADC to [m/s^2]
  //   raw_ax = sensorValue.un.rawAccelerometer.x * (fsr * g / 32768.0);
  //   raw_ay = sensorValue.un.rawAccelerometer.y * (fsr * g / 32768.0);
  //   raw_az = sensorValue.un.rawAccelerometer.z * (fsr * g / 32768.0);

  //   // For raw and calibration comparison
  //   cal_ax = raw_ax - ax_offset;
  //   cal_ay = raw_ay - ay_offset;
  //   cal_az = raw_az - az_offset;

  //   raw_linear_accel = get_linear_accel(raw_ax, raw_ay, raw_az) - g;
  //   cal_linear_accel = get_linear_accel(cal_ax, cal_ay, cal_az) - g;

  //   Serial.println(raw_linear_accel, 6);
  //   Serial.print(",");
  //   Serial.println(cal_linear_accel, 6);

  //   // // For serial record python script 
  //   // Serial.print(raw_ax, 6);
  //   // Serial.print(",");
  //   // Serial.print(raw_ay, 6);
  //   // Serial.print(",");
  //   // Serial.println(raw_az, 6);
  //   break;
  // case SH2_GAME_ROTATION_VECTOR:
  //   Serial.print("Game Rotation Vector - r: ");
  //   Serial.print(sensorValue.un.gameRotationVector.real);
  //   Serial.print(" i: ");
  //   Serial.print(sensorValue.un.gameRotationVector.i);
  //   Serial.print(" j: ");
  //   Serial.print(sensorValue.un.gameRotationVector.j);
  //   Serial.print(" k: ");
  //   Serial.println(sensorValue.un.gameRotationVector.k);
  //   break;
  }
}
