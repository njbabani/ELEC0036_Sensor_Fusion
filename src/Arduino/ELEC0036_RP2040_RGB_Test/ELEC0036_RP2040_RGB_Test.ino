#include <Adafruit_NeoPixel.h>

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS  1 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals.
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint8_t colors[3][3] ={ { 15, 0, 0 }, { 0, 10, 0 }, { 0, 0, 20 }};

void setup() {
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

void loop() {
  static int i = 0;
  
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.setPixelColor(0, pixels.Color(colors[i][0], colors[i][1], colors[i][2]));
  pixels.show();   // Send the updated pixel colors to the hardware.

  if (++i == 3) i = 0;

  delay(500);
}