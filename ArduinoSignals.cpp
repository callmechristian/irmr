#include <IRLibDecodeBase.h>  //We need both the coding and 
#include <IRLibSendBase.h>    // sending base classes 
#include <IRLib_P01_NEC.h>    //Lowest numbered protocol 1st 
#include <IRLib_P02_Sony.h>   // Include only protocols you want 
#include <IRLib_P03_RC5.h> 
#include <IRLib_P04_RC6.h> 
#include <IRLib_P05_Panasonic_Old.h> 
#include <IRLib_P07_NECx.h> 
#include <IRLib_HashRaw.h>    //We need this for IRsendRaw 
#include <IRLibCombo.h>       // After all protocols, include this 

// All of the above automatically creates a universal decoder 

// class called "IRdecode" and a universal sender class "IRsend" 

// containing only the protocols you want. 

// Now declare instances of the decoder and the sender. 

IRdecode myDecoder; 

IRsend mySender; 

 

// Include a receiver either this or IRLibRecvPCI or IRLibRecvLoop 

#include <IRLibRecv.h> 

#include <EEPROM.h> 

 

IRrecv myReceiver(2); //pin number for the receiver 

// emitter is connected on pin 3 

 

// Storage for the recorded code 

//uint8_t codeProtocol[5];  // The type of code 

//uint32_t codeValue[5];    // The data bits if type is not raw 

//uint8_t codeBits[5];      // The length of the code in bits 

 

// index of the stored codes 

//unsigned index = 0;// index must be 0 

 

struct AppSettings 

{ 

  unsigned index = 0; 

  uint8_t codeProtocol[5];  // The type of code 

  uint32_t codeValue[5];    // The data bits if type is not raw 

  uint8_t codeBits[5];      // The length of the code in bits 

}; 

AppSettings mySettings; 

 

void SaveSettings() 

{ 

  int eeAddress = 0; 

  EEPROM.put(eeAddress, mySettings); 

} 

 

void RestoreSettings() 

{ 

  int eeAddress = 0; 

  EEPROM.get(eeAddress, mySettings); 

} 

 

void ClearEEPROM() 

{ 

  for (int i = 0; i < EEPROM.length(); i++) 

  { 

    EEPROM.write(i, 0); 

  } 

} 

 

void setup() { 

  RestoreSettings(); 

  if (mySettings.index == 0) 

  { 

    for (unsigned i = 0; i < 5; i++) 

    { 

      mySettings.codeValue[i] = 0; 

      mySettings.codeProtocol[i] = UNKNOWN; 

    } 

  } 

  Serial.begin(9600); 

  delay(2000); while (!Serial); //delay for Leonardo 

  if (mySettings.index == 0) 

    Serial.println(F("Send 5 codes from your remote and we will record it.")); 

  else 

    Serial.println(F("Codes restored from memory")); 

 

  myReceiver.enableIRIn(); // Start the receiver 

} 

 

// Stores the code for later playback 

void storeCode(unsigned i) { 

  mySettings.codeProtocol[i] = myDecoder.protocolNum; 

  Serial.print(F("Received ")); 

  Serial.print(Pnames(mySettings.codeProtocol[i])); 

  if (mySettings.codeProtocol[i] == UNKNOWN) { 

    Serial.println(F(" saving raw data.")); 

    myDecoder.dumpResults(); 

    mySettings.codeValue[i] = myDecoder.value; 

  } 

  else { 

    mySettings.codeValue[i] = myDecoder.value; 

    mySettings.codeBits[i] = myDecoder.bits; 

  } 

  Serial.print(F(" Value:0x")); 

  Serial.println(mySettings.codeValue[i], HEX); 

} 

 

void sendCode(unsigned i) 

{ 

  mySender.send(mySettings.codeProtocol[i],mySettings.codeValue[i], mySettings.codeBits[i]); 

  if (mySettings.codeProtocol[i] == UNKNOWN) return; 

  Serial.print(F("Sent ")); 

  Serial.print(Pnames(mySettings.codeProtocol[i])); 

  Serial.print(F(" Value:0x")); 

  Serial.println(mySettings.codeValue[i], HEX); 

} 

 

void loop() { 

 

 

  while (mySettings.index < 5) 

  { 

    if (myReceiver.getResults()) { 

      myDecoder.decode(); 

 

      if (mySettings.index >= 1) 

      { 

        if (myDecoder.value == mySettings.codeValue[mySettings.index - 1]) 

        { 

          Serial.println("Reject code"); 

        } 

        else 

        { 

          storeCode(mySettings.index); 

          mySettings.index++; 

        } 

      } 

      else 

      { 

        storeCode(mySettings.index); 

        mySettings.index++; 

      } 

      myReceiver.enableIRIn(); // Re-enable receiver 

 

    } 

  } 

 

  if (mySettings.index >= 5) 

    SaveSettings(); 

  while (mySettings.index >= 5) 

  { 

    if (Serial.available()) { 

      uint8_t C = Serial.read(); 

      if (C == 'a') 

      { 

        sendCode(0); 

      } 

      if (C == 's') 

      { 

        sendCode(1); 

      } 

      if (C == 'd') 

      { 

        sendCode(2); 

      } 

      if (C == 'f') 

      { 

        sendCode(3); 

      } 

      if (C == 'g') 

      { 

        sendCode(4); 

      } 

      if (C == 'r') 

      { 

        ClearEEPROM; 

        mySettings.index = 0; 

        for (unsigned i = 0; i < 5; i++) 

        { 

          mySettings.codeValue[i] = 0; 

          mySettings.codeProtocol[i] = UNKNOWN; 

        } 

        myReceiver.enableIRIn(); // Re-enable receiver 

      } 

 

    } 

  } 

} 
