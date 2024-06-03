/* This example will set the onboard LED current to various values and blink
   the LED */

#include <Basic_AS7343.h>

Basic_AS7343 as7343;


void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

  if (!as7343.begin()){
    Serial.println("Could not find AS7343");
    while (1) { delay(10); }
  }
}

void loop() {
  Serial.println("4 mA LED blink");
  as7343.setLEDCurrent(4); // 4mA
  as7343.enableLED(true);
  delay(100);
  as7343.enableLED(false);
  delay(500);

  Serial.println("32 mA LED blink");
  as7343.setLEDCurrent(32); // 32mA
  as7343.enableLED(true);
  delay(100);
  as7343.enableLED(false);
  delay(500);

  Serial.println("16 mA LED blink");
  as7343.setLEDCurrent(16); // 16mA
  as7343.enableLED(true);
  delay(100);
  as7343.enableLED(false);
  delay(500);

}
