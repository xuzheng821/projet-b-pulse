#include "ADAS1000.h"
#include <RFduinoBLE.h>

void setup() {
  // this is the data we want to appear in the advertisement
  // (if the deviceName and advertisementData are too long to fix into the 31 byte
  // ble advertisement packet, then the advertisementData is truncated first down to
  // a single byte, then it will truncate the deviceName)
  RFduinoBLE.advertisementData = "BPulse";

  // start the BLE stack
  RFduinoBLE.begin();
  pinMode(6, OUTPUT);

  Serial.begin(9600);
  unsigned char adas = ADAS1000_Init(ADAS1000_2KHZ_FRAME_RATE);
  if(adas) Serial.println("Initial success !");
}

void loop() {
  // sample once per second
  unsigned long r;
  ADAS1000_GetRegisterValue(0x0A, &r);
  Serial.println(r, HEX);
  Serial.println("---");
  delay(1000);
  
}
