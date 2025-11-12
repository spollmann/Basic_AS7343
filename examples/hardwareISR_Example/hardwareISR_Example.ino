/** This code demonstrates how to use a hardware interrupt with the AS7343
IMPORTANT NOTE, THERE IS A CONSTANT OVERHEAD OF ABOUT 0.375ms (on an Arduino Nano 33 BLE rev2) when clearing the interrupt status bit.
So when you compute a theoretical maximum, vs actual frequency, that needs to be taken into account.
Eg:  ATIME = 2, ASTEP = 499
Integration time: (2+1)*(499+1)*0.00278ms = 4.17ms integration
With constant light, that should max out at 239.8Hz, BUT
in actual reality the integration time + overhead = 4.545ms
or a rate of about 220Hz
*/

#include "Basic_AS7343.h"

Basic_AS7343 as7343;

#define SERIAL_OUT_SECONDS_INTERVAL 5
#define INTERRUPT_PIN 2
volatile uint32_t lightCount = 0;
volatile bool interruptTriggered = false;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.begin(115200);
  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }

  if (!as7343.begin()) {
    Serial.println("Could not find as7343");
    while (1) { delay(10); }
  }

  as7343.enableSpectralMeasurement(false);
  as7343.setAutoChannelReadout(AS7343_6CHANNEL);
  //  as7343.setAutoChannelReadout(AS7343_12CHANNEL);
  //  as7343.setAutoChannelReadout(AS7343_18CHANNEL);

  as7343.setATIME(2);
  as7343.setASTEP(499);
  as7343.setGain(AS7343_GAIN_32X);
  as7343.setLowThreshold(0);
  as7343.setHighThreshold(250);
  Serial.print("Low threshold set to ");
  Serial.println(as7343.getLowThreshold());
  Serial.print("High threshold set to ");
  Serial.println(as7343.getHighThreshold());
  as7343.setAPERS(AS7343_INT_COUNT_1);
  as7343.setSpectralThresholdChannel(AS7343_ADC_CHANNEL_4);  //should be visible light

  Serial.println("Enabling spectral interrupt");
  as7343.enableSpectralInterrupt(true);
  Serial.println("Enabling spectral measurements");
  as7343.enableSpectralMeasurement(true);
  as7343.clearInterruptStatus();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), spectralInterruptHandler, FALLING);

}
char numInterruptsText[60];
uint32_t updateDisplay = 0;
void loop() {
  if (millis() - updateDisplay > (SERIAL_OUT_SECONDS_INTERVAL*1000)) {
    updateDisplay = millis();
    char numInterruptsText[60];
    sprintf(numInterruptsText, "Num. Interrupts in last %d seconds: %d\nFrequency is: %.02fHz\n",SERIAL_OUT_SECONDS_INTERVAL, lightCount, (float)lightCount/SERIAL_OUT_SECONDS_INTERVAL);
    Serial.print(numInterruptsText);
    lightCount = 0;
  }
  if (interruptTriggered) {    
    as7343.clearInterruptStatus();
    interruptTriggered = false;
  } 
}


void spectralInterruptHandler() {
  interruptTriggered = true;
  lightCount++;
}
