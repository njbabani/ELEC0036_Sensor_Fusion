// Basic demo for all readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS_Mahony.h>
#include <Wire.h>
#include <math.h>

// Create an instance of the Adafruit ICM20948 sensor
Adafruit_ICM20948 imu;

// Create an instance of the Mahony AHRS filter
Adafruit_Mahony ahrs;

// Accel offsets
const float ax_offset = -0.005034;
const float ay_offset = -0.193015;
const float az_offset = 0.088739;

// Gyro offsets
const float gx_offset = 0.0162;
const float gy_offset = 0.0028;
const float gz_offset = -0.0080;

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

// Mahony gains
float prop_gain = 0.5;
float int_gain = 0.15;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Will pause MCU until serial console opens

  if (!imu.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ICM20948 Found!");

  // Initialize the Mahony AHRS filter
  ahrs = Adafruit_Mahony(prop_gain, int_gain);
}

void loop() {
  sensors_event_t gyro;
  sensors_event_t accel;
  sensors_event_t mag;

  // Read accelerometer and gyroscope data
  imu.getGyroSensor()->getEvent(&gyro);
  imu.getAccelerometerSensor()->getEvent(&accel);
  imu.getMagnetometerSensor()->getEvent(&mag);

  // Get raw gyroscope values
  float gx_raw = gyro.gyro.x;
  float gy_raw = gyro.gyro.y;
  float gz_raw = gyro.gyro.z;

  // Get true gyroscope values
  float gx = gx_raw - gx_offset;
  float gy = gy_raw - gy_offset;
  float gz = gz_raw - gz_offset;

  //Convert from radians/sec to degree/sec for Mahony filter
  gx *= (180.0 / PI);
  gy *= (180.0 / PI);
  gz *= (180.0 / PI);

  // Get raw acceleration values
  float ax_raw = accel.acceleration.x;
  float ay_raw = accel.acceleration.y;
  float az_raw = accel.acceleration.z;

  // Get the true acceleration values
  float ax = ax_raw - ax_offset;
  float ay = ay_raw - ay_offset;
  float az = az_raw - az_offset;

  Mxyz[0] = mag.magnetic.x;
  Mxyz[1] = mag.magnetic.y;
  Mxyz[2] = mag.magnetic.z;

  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];

  // Set time interval
  float dt = 0.01;

  // Update the Mahony AHRS filter with accelerometer and gyroscope data
  ahrs.update(gx, gy, gz, ax, ay, az, Mxyz[0], Mxyz[1], Mxyz[2], dt);

  // Get Quaternion data
  float quat_w, quat_x, quat_y, quat_z;
  ahrs.getQuaternion(&quat_w, &quat_x, &quat_y, &quat_z);
  float roll = ahrs.getRoll();
  float pitch = ahrs.getPitch();
  float yaw = ahrs.getYaw();

  // Output the Quaternion data
  // Serial.print(F("{\"quat_w\":"));
  // Serial.print(quat_w, 3);
  // Serial.print(F(", \"quat_x\":"));
  // Serial.print(quat_x, 3);
  // Serial.print(F(", \"quat_y\":"));
  // Serial.print(quat_y, 3);
  // Serial.print(F(", \"quat_z\":"));
  // Serial.print(quat_z, 3);
  // Serial.println(F("}"));
 
  // Output Euler angles
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);

  // Add a delay or adjust based on your desired update rate
  delay(10);
}