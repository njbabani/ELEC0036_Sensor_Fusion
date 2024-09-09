#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>

BLEScan* scan;

// Define the target UUID
#define NODE1_UUID "1701"
#define NODE2_UUID "1702"
#define NODE3_UUID "1703"

// Variables for the moving average filter
float eta = 0.20;        // Weight for the current RSSI valueW
float filteredRSSI = 0.0;  // Initialise filteredRSSI as 0
float currentRSSI = 0.0;   // Initialise currentRSSi as 0

void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  scan->setInterval(40);
  scan->setWindow(20);
}

void loop() {
  BLEScanResults results = scan->start(1, false);

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);

    // Check if the device has the target UUID
    if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID(NODE1_UUID))) {
      currentRSSI = device.getRSSI();
      if (filteredRSSI == 0.0) {
        filteredRSSI = currentRSSI;
      } else {
        filteredRSSI = eta * currentRSSI + (1.0 - eta) * filteredRSSI;
        // Serial.print("Found BT-device with UUID ");
        // Serial.print(TARGET_UUID);
        // Serial.print(": ");
        // Serial.print(device.getName().c_str());
        // Serial.print(" Current RSSI: ");
        // Serial.print(currentRSSI);
        // Serial.print(" Filtered RSSI: ");
        // Serial.println(filteredRSSI);
        Serial.print(currentRSSI, 6);
        Serial.print(",");
        Serial.println(filteredRSSI, 6);
      }
    }
  }
  scan->clearResults();
}
