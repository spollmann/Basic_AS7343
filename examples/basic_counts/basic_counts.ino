/** This example will read all channels from the as7343.
  * The raw ADC values will then be converted to basic counts
  * according to the sensor's appliation note:
  * 
  * BasicCounts = RawSensorValue / ( Gain x IntegrationTime )
  * 
  * Finally, the converted values are printed out.
  */
#include "Basic_AS7343.h"

Basic_AS7343 as7343;

void setup() {
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) {
    delay(1);
  }
  
  if (!as7343.begin()){
    Serial.println("Could not find as7343");
    while (1) { delay(10); }
  }
//  as7343.setAutoChannelReadout(AS7343_6CHANNEL);
//  as7343.setAutoChannelReadout(AS7343_12CHANNEL);
  as7343.setAutoChannelReadout(AS7343_18CHANNEL);

  as7343.setATIME(100);
  as7343.setASTEP(999);
  as7343.setGain(AS7343_GAIN_256X);
//  as7343.setGain(AS7343_GAIN_2048X);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t readings[18];
  float counts[18];

  if (!as7343.readAllChannels(readings)){
    Serial.println("Error reading all channels!");
    return;
  }

  for(uint8_t i = 0; i < 18; i++) {
    counts[i] = as7343.toBasicCounts(readings[i]);
  }

  Serial.print("FZ 450nm : ");
  Serial.println(counts[0]);
  Serial.print("FY 555nm : ");
  Serial.println(counts[1]);
  Serial.print("FXL 600nm : ");
  Serial.println(counts[2]);
  Serial.print("NIR 855nm : ");
  Serial.println(counts[3]);
  Serial.print("2X VIS 1 : ");
  Serial.println(counts[4]);
  Serial.print("FD 1 : ");
  Serial.println(counts[5]);
  Serial.print("F2 425nm : ");
  Serial.println(counts[6]);
  Serial.print("F3 475nm : ");
  Serial.println(counts[7]);
  Serial.print("F4 515nm : ");
  Serial.println(counts[8]);
  Serial.print("F6 640nm : ");
  Serial.println(counts[9]);
  Serial.print("2X VIS 2 : ");
  Serial.println(counts[10]);
  Serial.print("FD 2 : ");
  Serial.println(counts[11]);
  Serial.print("F1 405nm : ");
  Serial.println(counts[12]);
  Serial.print("F5 550nm : ");
  Serial.println(counts[13]);
  Serial.print("F7 690nm : ");
  Serial.println(counts[14]);
  Serial.print("F8 745nm : ");
  Serial.println(counts[15]);
  Serial.print("2X VIS 3 : ");
  Serial.println(counts[16]);
  Serial.print("FD 3 : ");
  Serial.println(counts[17]);
  Serial.println();

  delay(500);
}
