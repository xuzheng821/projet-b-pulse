#include "Communication.h"
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

  Serial.begin(9600);
}

void loop() {
  // sample once per second
  RFduino_ULPDelay( SECONDS(1) );
  unsigned char adas = ADAS1000_Init(ADAS1000_2KHZ_FRAME_RATE);
  Serial.println(adas);
}

void RFduinoBLE_onReceive(char *data, int len){
  uint8_t m = 0;
  for(int i=0;i<len;i++) {
    uint8_t myByte = data[i];
    if(m<myByte) m=myByte;
  }
  // echo the max value
   RFduinoBLE.sendByte(m);
}
