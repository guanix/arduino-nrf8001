// Set Serial Monitor to 115200
// Currently this will output large amounts of debugging data

#define NRF_DEBUG 1
#include <SPI.h>
#include <nRF8001.h>

nRF8001 *nrf;

float temperatureC;
uint8_t pipeStatusReceived, dataSent;
unsigned long lastSent;

void temperatureHandler(float tempC)
{
  Serial.println("received temperature");
  temperatureC = tempC;
}

void eventHandler(nRFEvent *event)
{
  Serial.println("event handler");
  
  if (event->event == NRF_PIPESTATUSEVENT) {
    if (event->msg.pipeStatus.pipesOpen & 1<<5) {
      pipeStatusReceived = 1;
    }
  }
  
  nrf->debugEvent(event);
}

void setup() {
  temperatureC = 0.0;
  pipeStatusReceived = 0;
  lastSent = 0;
  
  Serial.begin(115200);
  Serial.println("Hello");
  nrf = new nRF8001(5, 6, 7);
  nrf->setEventHandler(&eventHandler);
  nrf->setTemperatureHandler(&temperatureHandler);
  if ((nrf->setup()) == cmdSuccess) {
    Serial.println("SUCCESS");
  } else {
    Serial.println("FAIL");
    while (1);
  }
  
  nrf->getDeviceAddress();
  nrf->poll();
  nrf->getTemperature();
  nrf->poll();
  
  if (temperatureC > 0.0) {
    Serial.print("Temperature: ");
    Serial.println(temperatureC, 2);
  }
  
  nrf->connect(0, 32);
}

void loop() {
  Serial.println("polling");
  nrf->poll(2000);
  
  if (pipeStatusReceived && (millis() - lastSent) > 1000 && temperatureC > 0.0) {
    Serial.println("ready to send data");
    uint8_t temp[2];
    temp[0] = 0;
    temp[1] = round(temperatureC);
    
    nrf->sendData(5, 2, (uint8_t *)&temp);
    lastSent = millis();
    uint8_t bat = 78;
    nrf->sendData(8, 1, &bat);
    
    // get new temperature
    nrf->getTemperature();
  } else if (nrf->getConnectionStatus() == Disconnected) {
    Serial.println("Reconnecting");
    pipeStatusReceived = 0;
    dataSent = 0;
    nrf->connect(0, 32);
  }
}
