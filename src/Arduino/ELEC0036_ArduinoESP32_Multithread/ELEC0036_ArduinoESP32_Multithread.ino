#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <math.h>
#include <LinearAlgebra.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Constants
const int node_num = 3;             // Example node number for RSSI array
float curr_RSSI[node_num] = { 0 };  // Current RSSI values
float filt_RSSI[node_num] = { 0 };  // Filtered RSSI values

SemaphoreHandle_t mutex;  // Mutex for thread-safe access

void movingAverageFilter(BLEAdvertisedDevice device, float eta) {
  String uuid[node_num] = { "1701", "1702", "1703" };

  for (int j = 0; j < node_num; j++) {
    if (device.haveServiceUUID() && device.isAdvertisingService(BLEUUID(uuid[j].c_str()))) {
      float rssi = device.getRSSI();
      if (filt_RSSI[j] == 0.0) {
        filt_RSSI[j] = rssi;
      } else {
        filt_RSSI[j] = eta * rssi + (1.0 - eta) * filt_RSSI[j];
      }
    }
  }
}

void scanTask(void* pvParameters) {
  BLEDevice::init("");                       // Initialize the device
  BLEScan* pBLEScan = BLEDevice::getScan();  // Correct use to get a scanning object
  pBLEScan->setActiveScan(true);             // Set scan type

  while (1) {
    BLEScanResults results = pBLEScan->start(1, false);
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (int i = 0; i < results.getCount(); i++) {
      BLEAdvertisedDevice device = results.getDevice(i);
      movingAverageFilter(device, 0.20);
    }
    xSemaphoreGive(mutex);
  }
}

void setup() {
  Serial.begin(115200);
  mutex = xSemaphoreCreateMutex();

  xTaskCreate(scanTask, "ScanTask", 10000, NULL, 1, NULL);
}

void loop() {
  if (xSemaphoreTake(mutex, (TickType_t)10) == pdTRUE) {
    for (int i = 0; i < node_num; i++) {
      Serial.printf("Filtered RSSI[%d]: %.2f\n", i, filt_RSSI[i]);
    }
    xSemaphoreGive(mutex);
  }
  Serial.print("Time: ");
  Serial.println(millis());
  delay(20);
}
