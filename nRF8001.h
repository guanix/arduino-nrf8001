#ifndef _NRF8001_H
#define _NRF8001_H

#include <Arduino.h>

#ifndef NRF_DEBUG
#define NRF_DEBUG 1
//#define NRF_VERBOSE_DEBUG 1
#endif

typedef uint8_t nRFLen;
typedef uint8_t nRFPipe;

#include "constants.h"
#include "data.h" // data structures for requests and responses

#define NRF_RX_BUFFERS 5

#if NRF_DEBUG
#define nrf_debug(msg) Serial.println(F(msg))
#define nrf_debugnl(msg) Serial.print(F(msg))
#else
#define nrf_debug(msg)
#define nrf_debugnl(msg)
#endif

class nRF8001
{
    private:
        uint64_t pipesOpen;
        uint8_t reset_pin;
        uint8_t reqn_pin;
        uint8_t rdyn_pin;
        uint8_t credits;
        nRFDeviceState deviceState;
        int8_t nextSetupMessage;
        nRFConnectionStatus connectionStatus;

        nRFTxStatus transmitReceive(nRFCommand *txCmd, uint16_t timeout);
        nRFTxStatus transmitCommand(uint8_t command);
        nRFTxStatus transmitPipeCommand(uint8_t command, nRFPipe pipe);

    public:
        void debugEvent(nRFEvent *event);
        void debugAddress(uint8_t *address);
        void addressToString(char *str, uint8_t *address);

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

        uint8_t isPipeOpen(nRFPipe servicePipeNo);

        nRFTxStatus test(uint8_t feature);
        nRFTxStatus sleep();
        nRFTxStatus getDeviceVersion();
        nRFTxStatus echo(nRFLen dataLength, uint8_t *data);
        nRFTxStatus wakeup();
        nRFTxStatus getBatteryLevel();
        nRFTxStatus getTemperature();
        nRFTxStatus setTxPower(uint8_t powerLevel);
        nRFTxStatus getDeviceAddress();
        nRFTxStatus connect(uint16_t timeout, uint16_t advInterval);
        nRFTxStatus radioReset();
        nRFTxStatus bond(uint16_t timeout, uint16_t advInterval);
        nRFTxStatus disconnect(uint8_t reason);
        nRFTxStatus changeTimingRequest(uint16_t intervalMin,
                                   uint16_t intervalMax,
                                   uint16_t slaveLatency,
                                   uint16_t timeout);
        nRFTxStatus openRemotePipe(nRFPipe servicePipeNo);
        nRFTxStatus closeRemotePipe(nRFPipe servicePipeNo);
        nRFTxStatus dtmCommand(uint16_t dtmCmd);
        nRFTxStatus readDynamicData();
        nRFTxStatus writeDynamicData(uint8_t seqNo,
                                   nRFLen dataLength,
                                   uint8_t *data);
        nRFTxStatus setApplLatency(uint8_t applLatencyMode,
                                 uint16_t latency);
        nRFTxStatus setKey(uint8_t keyType, uint8_t *key);
        nRFTxStatus openAdvPipe(uint64_t advServiceDataPipes);
        nRFTxStatus broadcast(uint16_t timeout, uint16_t advInterval);
        nRFTxStatus bondSecurityRequest();
        nRFTxStatus directedConnect();
        nRFTxStatus sendData(nRFPipe servicePipeNo,
                           nRFLen dataLength,
                           uint8_t *data);
        nRFTxStatus requestData(nRFPipe servicePipeNo);
        nRFTxStatus setLocalData(nRFPipe servicePipeNo,
                               nRFLen dataLength,
                               uint8_t *data);
        nRFTxStatus sendDataAck(nRFPipe servicePipeNo);
        nRFTxStatus sendDataNack(nRFPipe servicePipeNo,
                               uint8_t errorCode);
};

#endif /* _NRF8001_H */
