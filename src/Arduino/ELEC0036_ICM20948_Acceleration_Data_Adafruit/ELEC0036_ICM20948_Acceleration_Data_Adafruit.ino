/************************************************
* Accelerometer readings from Adafruit ICM20948 *
*************************************************/
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Declare icm as an object of Adafruit class to access ICM20948 attributes and methods
Adafruit_ICM20948 icm;

// Declare offsets
float ax_offset = -0.005034;
float ay_offset = -0.193015;
float az_offset = 0.088739;
// float ax_offset = 0;
// float ay_offset = 0;
// float az_offset = 0;

// A function for linear acceleration calculation - maybe use other data type instead of float for less memory?
float getlinearAccel(float ax, float ay, float az){
  float linearAccel = sqrt(sq(ax)+sq(ay)+sq(az)) - 9.8066;
  return linearAccel;
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Will pause MCU until serial console opens

  Serial.println("Adafruit ICM20948 test!"); // Prints message to user

  // Try to initialize!
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip"); // Failed to detect IMU
    while (1) {
      delay(10);
    }
  }

  Serial.println("ICM20948 Found!"); // Sucessfully detects IMU
  
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  // Set the accelerometer data rate to 100 Hz
  uint16_t target_accel_rate = 100;
  uint16_t accel_divisor = 1125 / target_accel_rate - 1;
  icm.setAccelRateDivisor(accel_divisor);
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  // icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();
}

void loop() {
  // Get a new normalized sensor event
  sensors_event_t accel;
  icm.getAccelerometerSensor()->getEvent(&accel);

  // Get raw acceleration values
  float ax_raw = accel.acceleration.x;
  float ay_raw = accel.acceleration.y;
  float az_raw = accel.acceleration.z;

  // Get the true acceleration values
  float ax = ax_raw - ax_offset;
  float ay = ay_raw - ay_offset;
  float az = az_raw - az_offset;

  //Display the results (acceleration is measured in m/s^2)
  // Serial.print(ax, 6);
  // Serial.print(",");
  // Serial.print(ay), 6;
  // Serial.print(",");
  // Serial.println(az, 6);

  // An option for linear acceleration readings
  float linearAccel = getlinearAccel(ax, ay, az);
  // float rawlinearAccel = getlinearAccel(ax_raw, ay_raw, az_raw);
  // Serial.print(rawlinearAccel);
  // Serial.print(",");
  Serial.println(linearAccel);

  // Serial.printf("Linear Acceleration = %f\n", linearAccel);
  // Serial.printf("Raw Linear Acceleration = %f\n", rawlinearAccel);

  delay(20);
}