#include "ADAS1000.h"
#include <RFduinoBLE.h>

#define NUMBER_OF_FRAME 100
unsigned char dataBuffer[NUMBER_OF_FRAME] = {0,};
unsigned short  frame = 0;
unsigned long crcValue = 0;

void setup() {
  // this is the data we want to appear in the advertisement
  // (if the deviceName and advertisementData are too long to fix into the 31 byte
  // ble advertisement packet, then the advertisementData is truncated first down to
  // a single byte, then it will truncate the deviceName)
  RFduinoBLE.advertisementData = "BPulse";

  // start the BLE stack
  RFduinoBLE.begin();
  
  pinMode(6, OUTPUT);

  // interact with computer
  Serial.begin(9600);

  // initialize the communication with the ADAS1000, set the frame rate to 2KHz
  unsigned char adas = ADAS1000_Init(ADAS1000_2KHZ_FRAME_RATE);
  if(adas) Serial.println("Initialization success!");
  else Serial.println("Initialization failed!");
}

void loop() {
  unsigned long regVal;
  // write and read a register
  ADAS1000_SetRegisterValue(ADAS1000_CMREFCTL, 0x85E0000B);
  ADAS1000_GetRegisterValue(ADAS1000_CMREFCTL, &regVal);
  Serial.println("ADAS1000_CMREFCTL: ");
  Serial.println(regVal, HEX);
  

  // start the ADCs Converting and begin streaming ECG data from ADAS1000 at 2kHz rate
  
  // power up device, enable all ECG channels in gain 1.4, low noise mode, 
  // master device using XTAL input source
  ADAS1000_SetRegisterValue(ADAS1000_ECGCTL, 0x81F800AE);
  ADAS1000_GetRegisterValue(ADAS1000_ECGCTL, &regVal);
  Serial.println("ADAS1000_ECGCTL: ");
  Serial.println(regVal, HEX);

  // read some frames of data from the ADAS1000
  ADAS1000_ReadData(dataBuffer, 2,
            1, 1,
            0, 0);
  // compute the CRC of the frame
  crcValue = ADAS1000_ComputeFrameCrc(dataBuffer);

  for(frame = 0; frame < NUMBER_OF_FRAME; frame++) {
    Serial.print(dataBuffer[frame], HEX);
    if ((frame+1)%4 != 0) Serial.print(", ");
    else Serial.println("");
  }

  Serial.println("---");
  delay(10000000);
}

//void RFduinoBLE_onConnect(){
//  RFduinoBLE.send(dataBuffer, NUMBER_OF_FRAME);
//}
