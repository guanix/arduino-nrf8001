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

// This data structure is stored inside the nRF8001 class and
// contains pointers to where we should actually put the data
// for automatically managed pipes.
typedef struct {
    void *valueArray;
    size_t maxLength;
    bool *changedAfterLastRead;
} pipe_data_t;

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

        uint8_t managedPipeCount;
        // This is a double pointer because it will be malloc'ed
        // in begin(). Not the best way to do this. If we had access
        // to services.h here, we could use NUMBER_OF_PIPES.
        pipe_data_t *managedPipes;

        nRFTxStatus transmitReceive(nRFCommand *txCmd, uint16_t timeout);
        nRFTxStatus transmitCommand(uint8_t command);
        nRFTxStatus transmitPipeCommand(uint8_t command, nRFPipeNo pipe);

        bool reconnect;

    public:
        void debugEvent(nRFEvent *event);
        void debugAddress(uint8_t *address);
        void addressToString(char *str, uint8_t *address);

        bool available();
        bool loop();

        nRFTxStatus poll(uint16_t timeout);
        nRFTxStatus poll();
        nRFDeviceState getDeviceState();
        nRFCmd setup(int setupMessageCount, hal_aci_data_t *setupMessages);

        bool registerManagedPipe(nRFPipeNo pipeNo, pipe_data_t *pipe_data);

        void begin(uint8_t reset_pin,
                   uint8_t reqn_pin,
                   uint8_t rdyn_pin,
                   uint8_t pipeCount);

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
        nRFTxStatus connectOnce(uint16_t timeout, uint16_t advInterval);
        nRFTxStatus connect();
        nRFTxStatus connectOnce();
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
template<class T, int howMany = 1>
class nRFPipe {
public:
    void begin(nRF8001 &nrf, nRFPipeNo pipeNo);

    bool changed();
    void write(T newval);
    T read(void);
    void set(T newval);

private:
    bool changedAfterLastRead;
    T valueArray[howMany];
    nRFLen maxLength;
    nRFPipeNo pipeNo;
};

// For obscure C++ reasons this has to be in the header file.
// Probably won't be a big deal because most Arduino sketches
// will only include this file once.
template<class T, int howMany>
void nRFPipe<T, howMany>::begin(nRF8001 &nrf, nRFPipeNo servicePipeNo)
{
    maxLength = sizeof(T)*howMany;
    pipeNo = servicePipeNo;
    changedAfterLastRead = false;
    memset(valueArray, 0, maxLength); // necessary in C++?

    pipe_data_t pipe_data;
    pipe_data.valueArray = valueArray;
    pipe_data.maxLength = sizeof(T)*howMany;
    pipe_data.changedAfterLastRead = &changedAfterLastRead;

    nrf.registerManagedPipe(servicePipeNo, &pipe_data);
}

#endif /* _NRF8001_H */
