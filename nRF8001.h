#ifndef _NRF8001_H
#define _NRF8001_H

#include <Arduino.h>
#include <assert.h>

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

inline uint8_t nRF8001::isConnected() {
    return connectionStatus == Connected;
}

inline uint8_t nRF8001::creditsAvailable()
{
    return credits;
}

inline nRFConnectionStatus nRF8001::getConnectionStatus()
{
    return connectionStatus;
}

inline nRFTxStatus nRF8001::poll(uint16_t timeout)
{
    return transmitReceive(0, timeout);
}

inline nRFTxStatus nRF8001::poll()
{
    return transmitReceive(0, 0);
}

inline uint8_t nRF8001::isPipeOpen(nRFPipeNo servicePipeNo)
{
    return (pipesOpen & ((uint64_t)1) << servicePipeNo) != 0;
}

// A characteristic that we will set up
template<class T, int howMany = 1>
class nRFPipe {
public:
    void begin(nRF8001 &nrf, nRFPipeNo pipeNo);

    bool changed();
    nRFTxStatus send(T newval);
    nRFTxStatus send(T* newval);
    T read(void);
    void read(T* arr);
    nRFTxStatus set(T newval);
    nRFTxStatus ack();
    nRFTxStatus nack();
    bool open();

private:
    bool changedAfterLastRead;
    T valueArray[howMany];
    nRFLen maxLength;
    nRFPipeNo pipeNo;
    nRF8001 *nrfInstance;
};

// For obscure C++ reasons this has to be in the header file.
// Probably won't be a big deal because most Arduino sketches
// will only include this file once.
template<class T, int howMany>
void nRFPipe<T, howMany>::begin(nRF8001 &nrf, nRFPipeNo servicePipeNo)
{
    assert(sizeof(T)*howMany <= NRF_DATA_LENGTH);
    maxLength = sizeof(T)*howMany;
    pipeNo = servicePipeNo;
    changedAfterLastRead = false;
    memset(valueArray, 0, maxLength); // necessary in C++?

    pipe_data_t pipe_data;
    pipe_data.valueArray = valueArray;
    pipe_data.maxLength = sizeof(T)*howMany;
    pipe_data.changedAfterLastRead = &changedAfterLastRead;

    nrfInstance = &nrf;

    nrf.registerManagedPipe(servicePipeNo, &pipe_data);
}

template<class T, int howMany>
inline bool nRFPipe<T, howMany>::changed()
{
    return changedAfterLastRead;
}

template<class T, int howMany>
inline T nRFPipe<T, howMany>::read(void)
{
    changedAfterLastRead = false;
    return valueArray[0];
}

template<class T, int howMany>
inline void nRFPipe<T, howMany>::read(T *arr)
{
    // TODO: endianness
    memcpy(arr, valueArray, sizeof(T)*howMany);
}

template<class T, int howMany>
inline nRFTxStatus nRFPipe<T, howMany>::send(T val)
{
    return nrfInstance->sendData(pipeNo, sizeof(T), &val);
}

template<class T, int howMany>
inline nRFTxStatus nRFPipe<T, howMany>::send(T* val)
{
    return nrfInstance->sendData(pipeNo, sizeof(T)*howMany, val);
}

template<class T, int howMany>
inline bool nRFPipe<T, howMany>::open()
{
    return nrfInstance->isPipeOpen(pipeNo);
}

#endif /* _NRF8001_H */
