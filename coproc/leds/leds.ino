#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 50

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806, define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 51
#define CLOCK_PIN 52
// 4 5 6 7
// Define the array of leds
CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];

void setup() {
    LEDS.addLeds<APA102, 4, 5, BGR>(leds1, 50);
    LEDS.addLeds<APA102, 6, 7, BGR>(leds2, 50);
    LEDS.setBrightness(255);
  for (int i = 0; i < NUM_LEDS; i++){
    leds1[i] = CRGB::Black;
        leds2[i] = CRGB::Black;
      FastLED.show();
  }
  FastLED.show();
  delay(50);
}

void fadeall() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds1[i].nscale8(230);
        leds2[i].nscale8(230);
  }
}

int repeat = 60;

void loop() {
  for(int i = 0; i < NUM_LEDS; i++) {
    // Set the i'th led to red 
    leds1[i] = CHSV(160, 255, 255);
    leds2[i] = CHSV(160, 255, 255);
    // Show the leds
    FastLED.show(); 
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(5);
  }

  // Now go in the other direction.  
  for(int i = (NUM_LEDS)-1; i >= 0; i--) {
    // Set the i'th led to red 
    leds1[i] = CHSV(32, 255, 255);
    leds2[i] = CHSV(32, 255, 255);
    // Show the leds
    FastLED.show();
    // now that we've shown the leds, reset the i'th led to black
    // leds[i] = CRGB::Black;
    fadeall();
    // Wait a little bit before we loop around and do it again
    delay(5);
  }
}
