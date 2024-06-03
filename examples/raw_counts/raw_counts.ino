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

  Serial.print("Raw FZ 450nm : ");
  Serial.println(readings[0]);
  Serial.print("Raw FY 555nm : ");
  Serial.println(readings[1]);
  Serial.print("Raw FXL 600nm : ");
  Serial.println(readings[2]);
  Serial.print("Raw NIR 855nm : ");
  Serial.println(readings[3]);
  Serial.print("Raw 2X VIS 1 : ");
  Serial.println(readings[4]);
  Serial.print("Raw FD 1 : ");
  Serial.println(readings[5]);
  Serial.print("Raw F2 425nm : ");
  Serial.println(readings[6]);
  Serial.print("Raw F3 475nm : ");
  Serial.println(readings[7]);
  Serial.print("Raw F4 515nm : ");
  Serial.println(readings[8]);
  Serial.print("Raw F6 640nm : ");
  Serial.println(readings[9]);
  Serial.print("Raw 2X VIS 2 : ");
  Serial.println(readings[10]);
  Serial.print("Raw FD 2 : ");
  Serial.println(readings[11]);
  Serial.print("Raw F1 405nm : ");
  Serial.println(readings[12]);
  Serial.print("Raw F5 550nm : ");
  Serial.println(readings[13]);
  Serial.print("Raw F7 690nm : ");
  Serial.println(readings[14]);
  Serial.print("Raw F8 745nm : ");
  Serial.println(readings[15]);
  Serial.print("Raw 2X VIS 3 : ");
  Serial.println(readings[16]);
  Serial.print("Raw FD 3 : ");
  Serial.println(readings[17]);
  Serial.println();

  delay(500);
}
