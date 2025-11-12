/** This code demonstrates how to read the raw data from all 18 channels of the as7343.
  */
#include "Basic_AS7343.h"

Basic_AS7343 as7343;

#define SERIAL_OUT_MS_INTERVAL 1
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
  if (millis() - updateDisplay > (1000*SERIAL_OUT_MS_INTERVAL)) {
    updateDisplay = millis();
    char numInterruptsText[60];
    sprintf(numInterruptsText, "Num. Interrupts in last %d seconds: %d\nFrequency is: %.02fHz\n",SERIAL_OUT_MS_INTERVAL, lightCount, (float)lightCount/SERIAL_OUT_MS_INTERVAL);
    Serial.print(numInterruptsText);
    lightCount = 0;
  }

  if (interruptTriggered) {    
    uint8_t int_source = as7343.spectralInterruptSource();
    if (int_source & AS7343_SPECTRAL_INT_LOW_MSK) { // Bit 4 = INT_SP_L
      Serial.println(">>> Threshold LOW");
      long ch_value = as7343.readChannel(AS7343_CHANNEL_2X_VIS_1);
      Serial.print("Monitored CH4 Value: ");
      Serial.println(ch_value);

    }
    if (int_source & AS7343_SPECTRAL_INT_HIGH_MSK) { // Bit 5 = INT_SP_H
      Serial.println(">>> Threshold HIGH");
      long ch_value = as7343.readChannel(AS7343_CHANNEL_2X_VIS_1);
      Serial.print("Monitored CH4 Value: ");
      Serial.println(ch_value);
    }
    as7343.clearInterruptStatus();
    interruptTriggered = false;
  } else {
    //    Serial.println("No Interrupt");
    //    long ch_value = as7343.readChannel(AS7343_CHANNEL_2X_VIS_1);
    //    Serial.print("Monitored CH4 Value: ");
    //    Serial.println(ch_value);
  }
}


void spectralInterruptHandler() {
  interruptTriggered = true;
  lightCount++;
}
