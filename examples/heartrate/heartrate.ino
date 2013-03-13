// Set Serial Monitor to 115200
// Currently this will output large amounts of debugging data

#define NRF_DEBUG 1
#include <SPI.h>
#include <nRF8001.h>

#define HEARTRATE_PIPE 5
#define BATTERY_PIPE 8

// change nRF8001 reset pin to -1 if it's not connected
// Redbear BLE Shield users: to my knowledge reset pin is not connected so use -1!
// NOTE: if you choose -1, youll need to manually reset your device after powerup!!
#define RESET_PIN 7
#define REQN_PIN 9
#define RDYN_PIN 8

nRF8001 *nrf;

float temperatureC;
uint8_t pipeStatusReceived, dataSent;
unsigned long lastSent;

// This function is called when nRF8001 responds with the temperature
void temperatureHandler(float tempC)
{
  Serial.println("received temperature");
  temperatureC = tempC;
}

// Generic event handler, here it's just for debugging all received events
void eventHandler(nRFEvent *event)
{
  Serial.println("event handler");
  nrf->debugEvent(event);
}

void setup() {
  temperatureC = 0.0;
  pipeStatusReceived = 0;
  lastSent = 0;
  
  Serial.begin(115200);
  Serial.println("Hello");
  
  // nRF8001 class initialized with pin numbers
  nrf = new nRF8001(RESET_PIN, REQN_PIN, RDYN_PIN);

  // Register event handles
  nrf->setEventHandler(&eventHandler);
  nrf->setTemperatureHandler(&temperatureHandler);
  if ((nrf->setup()) == cmdSuccess) {
    Serial.println("SUCCESS");
  } else {
    Serial.println("FAIL");
    while (1);
  }
  
  // These functions merely request device address and temperature,
  // actual responses are asynchronous. They'll return error codes
  // if somehow the request itself failed, for example because
  // the device is not ready for these commands.
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
  
  // Polling will block - times out after 2 seconds
  nrf->poll(2000);
  
  // If heart rate pipe is open
  if (nrf->isPipeOpen(HEARTRATE_PIPE) && (millis() - lastSent) > 1000 && temperatureC > 0.0 && nrf->creditsAvailable()) {
    Serial.println("ready to send data");
    uint8_t temp[2];
    temp[0] = 0;
    temp[1] = round(temperatureC);
    
    nrf->sendData(HEARTRATE_PIPE, 2, (uint8_t *)&temp);
    lastSent = millis();
    uint8_t bat = 78;
    
    // If battery pipe is open
    if (nrf->isPipeOpen(BATTERY_PIPE) && nrf->creditsAvailable()) {
      nrf->sendData(BATTERY_PIPE, 1, &bat);
    }
    
    // get new temperature
    nrf->getTemperature();
  } else if (nrf->getConnectionStatus() == Disconnected) {
    Serial.println("Reconnecting");
    dataSent = 0;
    nrf->connect(0, 32);
  }
}
