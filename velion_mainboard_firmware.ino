#include <Adafruit_NeoPixel.h>

#define RGBLED_DIN 21
#define LED_COUNT 5

// List of LED types to try
uint16_t ledTypes[] = {
  NEO_GRB + NEO_KHZ800,   // WS2812 / SK6812 (most common)
  NEO_RGB + NEO_KHZ800,
  NEO_GRBW + NEO_KHZ800, // SK6812 RGBW
  NEO_GRB + NEO_KHZ400,  // Older WS2811
  NEO_RGB + NEO_KHZ400
};

const int typeCount = sizeof(ledTypes) / sizeof(ledTypes[0]);
int currentType = 0;

Adafruit_NeoPixel strip(
  LED_COUNT,
  RGBLED_DIN,
  ledTypes[2]
);

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Reconfigure strip with new LED type
  strip.updateType(ledTypes[currentType]);
  strip.begin();
  strip.clear();
  strip.setBrightness(128);

  // Set all LEDs to white
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(128, 128, 128));
  }
  strip.show();

  // Debug output
  Serial.print("Testing LED type index: ");
  Serial.println(currentType);

  // Wait 3 seconds
  delay(3000);

  // Move to next LED type
  currentType++;
  if (currentType >= typeCount) {
    currentType = 0;
  }
}
