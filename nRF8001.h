#ifndef _NRF8001_H
#define _NRF8001_H

#include <Arduino.h>

#ifndef NRF_DEBUG
#define NRF_DEBUG 1
#endif

typedef uint8_t nRFLen;
typedef uint8_t nRFPipe;

#include "constants.h"
#include "data.h" // data structures for requests and responses

typedef struct {
    uint8_t status_byte;
    uint8_t buffer[32];
} hal_aci_data_t;

#define NRF_RX_BUFFERS 5

#if NRF_DEBUG
#define nrf_debug(msg) Serial.println(F(msg))
#define nrf_debugnl(msg) Serial.print(F(msg))
#else
#define nrf_debug(msg)
#define nrf_debugnl(msg)
#endif

// event handlers
typedef void (*nRFEventHandler) (nRFEvent *);
typedef void (*nRFCommandResponseHandler) (uint8_t opcode, uint8_t status);
typedef void (*nRFTemperatureHandler) (float tempC);
typedef void (*nRFBatteryLevelHandler) (float voltage);
typedef void (*nRFDeviceVersionHandler) (uint16_t configId,
    uint8_t aciVersion, uint8_t setupFormat, uint8_t configStatus);
typedef void (*nRFDeviceAddressHandler) (uint8_t *address,
    uint8_t addressType);
typedef void (*nRFDynamicDataHandler) (uint8_t seqNo, uint8_t *data);
typedef void (*nRFConnectedHandler) (uint8_t addressType, uint8_t *peerAddress,
    void *connectionData);
typedef void (*nRFDisconnectedHandler) (uint8_t aciStatus, uint8_t btLeStatus);
typedef void (*nRFBondStatusHandler) (void *bondStatusData);
typedef void (*nRFKeyRequestHandler) (uint8_t keyType);
typedef void (*nRFPipeErrorHandler) (nRFPipe servicePipeNo,
    uint8_t errorCode, uint8_t *errorData);
typedef void (*nRFDataReceivedHandler) (nRFPipe servicePipeNo,
    uint8_t *data);
typedef void (*nRFDataAckHandler) (nRFPipe servicePipeNo);

class nRF8001
{
    private:
        uint64_t pipesOpen;
        uint8_t reset_pin;
        uint8_t reqn_pin;
        uint8_t rdyn_pin;
        nRFEventHandler listener;
        uint8_t credits;
        nRFDeviceState deviceState;
        int8_t nextSetupMessage;
        nRFConnectionStatus connectionStatus;

        nRFTxStatus transmitReceive(nRFCommand *txCmd, uint16_t timeout);

        nRFCommandResponseHandler commandResponseHandler;
        nRFTemperatureHandler temperatureHandler;
        nRFBatteryLevelHandler batteryLevelHandler;
        nRFDeviceVersionHandler deviceVersionHandler;
        nRFDeviceAddressHandler deviceAddressHandler;
        nRFDynamicDataHandler dynamicDataHandler;
        nRFConnectedHandler connectedHandler;
        nRFDisconnectedHandler disconnectedHandler;
        nRFBondStatusHandler bondStatusHandler;
        nRFKeyRequestHandler keyRequestHandler;
        nRFPipeErrorHandler pipeErrorHandler;
        nRFDataReceivedHandler dataReceivedHandler;
        nRFDataAckHandler dataAckHandler;

    public:
        void debugEvent(nRFEvent *event);
        void debugAddress(uint8_t *address);

        nRFTxStatus poll(uint16_t timeout);
        nRFTxStatus poll();
        nRFDeviceState getDeviceState();
        nRFCmd setup();

        nRF8001(uint8_t reset_pin,
                   uint8_t reqn_pin,
                   uint8_t rdyn_pin);

        uint8_t creditsAvailable();
        uint8_t isConnected();
        nRFConnectionStatus getConnectionStatus();

        nRFCmd test(uint8_t feature);
        nRFCmd sleep();
        nRFCmd getDeviceVersion();
        nRFCmd echo(nRFLen dataLength, uint8_t *data);
        nRFCmd wakeup();
        nRFCmd getBatteryLevel();
        nRFCmd getTemperature();
        nRFCmd setup(nRFLen dataLength, uint8_t *setupData);
        nRFCmd setTxPower(uint8_t powerLevel);
        nRFCmd getDeviceAddress();
        nRFCmd connect(uint16_t timeout, uint16_t advInterval);
        nRFCmd radioReset();
        nRFCmd bond(uint16_t timeout, uint16_t advInterval);
        nRFCmd disconnect(uint8_t reason);
        nRFCmd changeTimingRequest(uint16_t intervalMin,
                                   uint16_t intervalMax,
                                   uint16_t slaveLatency,
                                   uint16_t timeout);
        nRFCmd openRemotePipe(nRFPipe servicePipeNo);
        nRFCmd closeRemotePipe(nRFPipe servicePipeNo);
        nRFCmd dtmCommand(uint16_t dtmCmd);
        nRFCmd writeDynamicData(uint8_t seqNo,
                                   nRFLen dataLength,
                                   uint8_t *data);
        nRFCmd setApplLatency(uint8_t applLatencyMode,
                                 uint16_t latency);
        nRFCmd setKey(uint8_t keyType, uint8_t *key);
        nRFCmd openAdvPipe(uint64_t advServiceDataPipes);
        nRFCmd broadcast(uint16_t timeout, uint16_t advInterval);
        nRFCmd bondSecurityRequest();
        nRFCmd directedConnect();
        nRFCmd sendData(nRFPipe servicePipeNo,
                           nRFLen dataLength,
                           uint8_t *data);
        nRFCmd requestData(nRFPipe servicePipeNo);
        nRFCmd setLocalData(nRFPipe servicePipeNo,
                               nRFLen dataLength,
                               uint8_t data);
        nRFCmd sendDataAck(nRFPipe servicePipeNo);
        nRFCmd sendDataNack(nRFPipe servicePipeNo,
                               uint8_t errorCode);

        void setEventHandler(nRFEventHandler handler);
        void setCommandResponseHandler(nRFCommandResponseHandler handler);
        void setTemperatureHandler(nRFTemperatureHandler handler);
        void setBatteryLevelHandler(nRFBatteryLevelHandler handler);
        void setDeviceVersionHandler(nRFDeviceVersionHandler handler);
        void setDeviceAddressHandler(nRFDeviceAddressHandler handler);
        void setDynamicDataHandler(nRFDynamicDataHandler handler);
        void setConnectedHandler(nRFConnectedHandler handler);
        void setDisconnectedHandler(nRFDisconnectedHandler handler);
        void setBondStatusHandler(nRFBondStatusHandler handler);
        void setKeyRequestHandler(nRFKeyRequestHandler handler);
        void setPipeErrorHandler(nRFPipeErrorHandler handler);
        void setDataReceivedHandler(nRFDataReceivedHandler handler);
        void setDataAckHandler(nRFDataAckHandler handler);
};

#endif /* _NRF8001_H */
