/********************************************
* Gyroscope readings from Adafruit ICM20948 *
*********************************************/
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Declare icm as an object of Adafruit class to access ICM20948 attributes and methods
Adafruit_ICM20948 icm;

// Declare offsets
float gx_offset = 0.0162;
float gy_offset = 0.0028;
float gz_offset = -0.0080;

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
  sensors_event_t gyro;
  icm.getGyroSensor()->getEvent(&gyro);

  // Get raw gyroscope values
  float gx_raw = gyro.gyro.x;
  float gy_raw = gyro.gyro.y;
  float gz_raw = gyro.gyro.z;

  // Get true gyroscope values
  float gx = gx_raw - gx_offset;
  float gy = gy_raw - gy_offset;
  float gz = gz_raw - gz_offset;

  // Display true results
  Serial.print(gx, 6);
  Serial.print(",");
  Serial.print(gy, 6);
  Serial.print(",");
  Serial.println(gz, 6);

  delay(20);
}