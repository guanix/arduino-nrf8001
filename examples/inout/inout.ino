#include <SPI.h>
#include "services.h"
#include <nRF8001.h>

// This takes up almost 1k of RAM, but it's a hassle to move into PROGMEM
// (The official Nordic driver for Arduino figured it out)
// These two data structures are here because they depend on services.h,
// which we'd rather place here than in the library folder.

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

// we could template nRF8001 to add the number of pipes, but that causes
// problems later
nRF8001 nrf;
nRFPipe<uint8_t> inChar;          // one byte
nRFPipe<uint8_t> outChar;  // string up to 10 characters (zero-terminated? hm)

// TODO: endianness

void setup() {
  Serial.begin(115200);
  Serial.println("nRF8001 test");
  
  // Pin numbers for Nordic's Arduino shield that adapts to their dev kit modules
  // the order is: reset, reqn, rdyn
  nrf.begin(4, 10, 3, NUMBER_OF_PIPES);
  
  inChar.begin(nrf, PIPE_SAMPLE_SERVICE_IN_CHARACTERISTIC_RX_ACK_AUTO);      // 1 is the pipe number
  outChar.begin(nrf, PIPE_SAMPLE_SERVICE_OUT_CHARACTERISTIC_TX);
  
  nrf.setup(NB_SETUP_MESSAGES, setup_msgs);
  
  nrf.connect();
}

void loop() {
  unsigned long lastNotify = millis();
  
  while (nrf.available()) {
    if (inChar.changed()) {
      Serial.print("inChar changed, new value: 0x");
      Serial.println(inChar.read());
    }
    
    if (millis() - lastNotify > 1000) {
      if (!outChar.open()) {
        continue;
      }
      
      Serial.println("sending data on outChar");
      lastNotify = millis();
      uint8_t time = lastNotify/1000;
      outChar.send(time);
    }
  }
}

