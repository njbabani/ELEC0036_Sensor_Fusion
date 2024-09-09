#include <ArduinoBLE.h>

// UUID of the peripheral to search for
String iBeaconService_UUID = "aa10";

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");

  // start scanning for peripherals
  // BLE.scan();
  // BLE.scanForUuid(iBeaconService_UUID);
  BLE.advertise();
}

void loop() {
  // check if a peripheral has been discovered
  // BLEDevice peripheral = BLE.available();
  BLE.poll();

  // String address = BLE.address();
  // Serial.print("Local address is: ");
  // Serial.println(address);

  // if (BLE.advertisedServiceUuid() == iBeaconService_UUID) {
  //   // discovered a peripheral, print out address, local name, and advertised service
  //   Serial.print("Found ");
  //   Serial.print(peripheral.address());
  //   Serial.print(" '");
  //   Serial.print(peripheral.localName());
  //   Serial.print("' ");
  //   Serial.print(peripheral.advertisedServiceUuid());
  //   Serial.print(" RSSI: ");
  //   Serial.println(peripheral.rssi());
  //   }
  // else{
  //   Serial.println("Cannot find targert peripheral! Continuing scan...");
  //   BLE.scanForUuid(iBeaconService_UUID);
  //   delay(5000);
  //}
}