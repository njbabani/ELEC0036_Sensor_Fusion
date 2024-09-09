// Basic demo for all readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Declare icm as an object of Adafruit class to access ICM20948 attributes and methods
Adafruit_ICM20948 icm;

// Mag offsets
float M_B[3]{
  -21.291563, -0.059196, -7.383874
};
float M_Ainv[3][3]{
  {1.380173, -0.014825, 0.000591},
  {-0.014825, 1.273212, -0.017644},
  {0.000591, -0.017644, 1.326894}
};
float Mxyz[3],temp[3];

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause MCU until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");
  //icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
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

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
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

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
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
  sensors_event_t mag;
  icm.getMagnetometerSensor()->getEvent(&mag);

  Mxyz[0] = mag.magnetic.x;
  Mxyz[1] = mag.magnetic.y;
  Mxyz[2] = mag.magnetic.z;

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];


  // Displayed in uT
  // Serial.print(mag.magnetic.x, 6);
  // Serial.print(",");
  // Serial.print(mag.magnetic.y, 6);
  // Serial.print(",");
  // Serial.println(mag.magnetic.z, 6);
  Serial.print(Mxyz[0], 6);
  Serial.print(",");
  Serial.print(Mxyz[1], 6);
  Serial.print(",");
  Serial.println(Mxyz[2], 6);

  // float magNorm = sqrt(sq(mag.magnetic.x)+sq(mag.magnetic.y)+sq(mag.magnetic.z));
  // Serial.println(magNorm);

  delay(100);
}
