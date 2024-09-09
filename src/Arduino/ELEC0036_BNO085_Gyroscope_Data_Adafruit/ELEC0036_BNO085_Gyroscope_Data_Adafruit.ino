// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

// Sensor bitmasks for dynamic calibration
#define GYROSCOPE_BIT 0x02

// Define constants
const float scaleRadToDeg = 180.0 / 3.14159265358; //Scale from radians to degrees for Gyroscope

// Define variables
float raw_gx, raw_gy, raw_gz;
float cal_gx, cal_gy, cal_gz;
float imu_gx, imu_gy, imu_gz;
float gx, gy, gz;

// Define zero-rate offset bias for calibration
const float gx_offset = -2.6780;
const float gy_offset = -1.0740;
const float gz_offset = -2.4390;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

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

  // Disable dynamic calibration for gyroscope
  int result = sh2_setCalConfig(GYROSCOPE_BIT);
  if (result == SH2_OK) {
    Serial.println("[SUCCESS] Disable dynamic calibration successful! - gyroscope");
  } else {
    Serial.println("[FAIL] Disable dynamic calibration unsuccessful! - gyroscope");
    delay(10);
  }

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
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

  case SH2_GYROSCOPE_CALIBRATED:
    gx = sensorValue.un.gyroscope.x;
    gy = sensorValue.un.gyroscope.y;
    gz = sensorValue.un.gyroscope.z;

    // For serial record python script 
    Serial.print(gx, 6);
    Serial.print(",");
    Serial.print(gy, 6);
    Serial.print(",");
    Serial.println(gz, 6);
    break;

  // case SH2_RAW_GYROSCOPE:
  //   raw_gx = sensorValue.un.rawGyroscope.x;
  //   raw_gy = sensorValue.un.rawGyroscope.y;
  //   raw_gz = sensorValue.un.rawGyroscope.z;
        
  //   // For raw and calibration comparison
  //   cal_gx = raw_gx - gx_offset;
  //   cal_gy = raw_gy - gy_offset;
  //   cal_gz = raw_gz - gz_offset;

  //   // For serial record python script 
  //   Serial.print(raw_gx, 6);
  //   Serial.print(",");
  //   Serial.print(raw_gy, 6);
  //   Serial.print(",");
  //   Serial.println(raw_gz, 6);
    
  //   // Calibrated values
  //   Serial.print(cal_gx, 6);
  //   Serial.print(",");
  //   Serial.print(cal_gy, 6);
  //   Serial.print(",");
  //   Serial.println(cal_gz, 6);
  //   break;
  }
}
