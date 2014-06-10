#ifndef _NRF8001_H
#define _NRF8001_H

#include <Arduino.h>

#ifndef NRF_DEBUG
#define NRF_DEBUG 1
//#define NRF_VERBOSE_DEBUG 1
#endif

typedef uint8_t nRFLen;
typedef uint8_t nRFPipeNo;

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

typedef struct {
    uint8_t status_byte;
    uint8_t buffer[32];
} hal_aci_data_t;

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
        nRFTxStatus transmitPipeCommand(uint8_t command, nRFPipeNo pipe);

    public:
        void debugEvent(nRFEvent *event);
        void debugAddress(uint8_t *address);
        void addressToString(char *str, uint8_t *address);

        nRFTxStatus loop();

        nRFTxStatus poll(uint16_t timeout);
        nRFTxStatus poll();
        nRFDeviceState getDeviceState();
        nRFCmd setup(int setupMessageCount, hal_aci_data_t *setupMessages);

        nRF8001(uint8_t reset_pin,
                   uint8_t reqn_pin,
                   uint8_t rdyn_pin);

        uint8_t creditsAvailable();
        uint8_t isConnected();
        nRFConnectionStatus getConnectionStatus();

        uint8_t isPipeOpen(nRFPipeNo servicePipeNo);

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
        nRFTxStatus connect();
        nRFTxStatus radioReset();
        nRFTxStatus bond(uint16_t timeout, uint16_t advInterval);
        nRFTxStatus disconnect(uint8_t reason);
        nRFTxStatus changeTimingRequest(uint16_t intervalMin,
                                   uint16_t intervalMax,
                                   uint16_t slaveLatency,
                                   uint16_t timeout);
        nRFTxStatus openRemotePipe(nRFPipeNo servicePipeNo);
        nRFTxStatus closeRemotePipe(nRFPipeNo servicePipeNo);
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
        nRFTxStatus sendData(nRFPipeNo servicePipeNo,
                           nRFLen dataLength,
                           uint8_t *data);
        nRFTxStatus requestData(nRFPipeNo servicePipeNo);
        nRFTxStatus setLocalData(nRFPipeNo servicePipeNo,
                               nRFLen dataLength,
                               uint8_t *data);
        nRFTxStatus sendDataAck(nRFPipeNo servicePipeNo);
        nRFTxStatus sendDataNack(nRFPipeNo servicePipeNo,
                               uint8_t errorCode);
};

// A characteristic that we will set up
template<class T>
class nRFPipe {
public:
    nRFPipe(nRF8001 *nrf, nRFPipeNo pipeNo, nRFLen maxLength);
    nRFPipe(nRF8001 *nrf, nRFPipeNo pipeNo);

    bool changed();
    void write(T newval);
    T read(void);
    void set(T newval);

private:
    bool changedAfterLastRead;
    T value;
    nRFLen maxLength;
    nRF8001 *nrfInstance;
    nRFPipeNo pipeNo;
};

#endif /* _NRF8001_H */
