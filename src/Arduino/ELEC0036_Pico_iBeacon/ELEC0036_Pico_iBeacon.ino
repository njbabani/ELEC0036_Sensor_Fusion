#include <BTstackLib.h>
#include <stdio.h>
#include <SPI.h>

/* EXAMPLE_START(iBeacon): iBeacon Simulator

   @section Setup

   @text After BTstack.setup(), iBeaconConfigure() configures BTstack
   to send out iBeacons Advertisements with the provided Major ID,
   Minor ID and UUID.
*/
/* LISTING_START(iBeaconSetup): iBeacon Setup */
UUID uuid("550e8400-e29b-41d4-a716-446655440000");
void setup(void) {
  Serial.begin(115200);
  BTstack.setup();
  BTstack.iBeaconConfigure(&uuid, 4711, 2);
  BTstack.startAdvertising();
}
/* LISTING_END(iBeaconSetup) */

void loop(void) {
  BTstack.loop();
}