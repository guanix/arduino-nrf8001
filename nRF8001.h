#include <Arduino.h>

#ifndef _NRF8001_H
#define _NRF8001_H

typedef uint8_t nrf_cmd_t;
typedef uint8_t nrf_len_t;
typedef uint8_t nrf_pipe_t;

#include "constants.h"
#include "data.h" // data structures for requests and responses
#include "LightweightRingBuff.h"

typedef struct {
    uint8_t status_byte;
    uint8_t buffer[32];
} hal_aci_data_t;

#define NRF_RX_BUFFERS 5

// event handler
typedef void (*nRFEventHandler) (nRFEvent *);

// internal state
typedef enum {
    RXONLY,
    RXTX,
    RXWAIT,
    TXWAIT
} nRFSpiState;

class nRF8001
{
    public:
        static void spiByteISR();

        static void setup(uint8_t reset_pin,
                          uint8_t reqn_pin,
                          uint8_t rdyn_pin,
                          uint8_t rdyn_int,
                          nRFEventHandler eventHandler);

        static uint8_t txReady();
        static uint8_t txDataReady();

        static nrf_cmd_t test(uint8_t feature);
        static nrf_cmd_t sleep();
        static nrf_cmd_t getDeviceVersion();
        static nrf_cmd_t echo(nrf_len_t dataLength, uint8_t *data);
        static nrf_cmd_t wakeup();
        static nrf_cmd_t getBatteryLevel();
        static nrf_cmd_t getTemperature();
        static nrf_cmd_t setup(nrf_len_t dataLength, uint8_t *setupData);
        static nrf_cmd_t setTxPower(uint8_t powerLevel);
        static nrf_cmd_t getDeviceAddress();
        static nrf_cmd_t connect(uint16_t timeout, uint16_t advInterval);
        static nrf_cmd_t radioReset();
        static nrf_cmd_t bond(uint16_t timeout, uint16_t advInterval);
        static nrf_cmd_t disconnect(uint8_t reason);
        static nrf_cmd_t changeTimingRequest(uint16_t intervalMin,
                                             uint16_t intervalMax,
                                             uint16_t slaveLatency,
                                             uint16_t timeout);
        static nrf_cmd_t openRemotePipe(nrf_pipe_t servicePipeNo);
        static nrf_cmd_t closeRemotePipe(nrf_pipe_t servicePipeNo);
        static nrf_cmd_t dtmCommand(uint16_t dtmCmd);
        static nrf_cmd_t writeDynamicData(uint8_t seqNo,
                                          nrf_len_t dataLength,
                                          uint8_t *data);
        static nrf_cmd_t setApplLatency(uint8_t applLatencyMode,
                                        uint16_t latency);
        static nrf_cmd_t setKey(uint8_t keyType, uint8_t *key);
        static nrf_cmd_t openAdvPipe(uint64_t advServiceDataPipes);
        static nrf_cmd_t broadcast(uint16_t timeout, uint16_t advInterval);
        static nrf_cmd_t bondSecurityRequest();
        static nrf_cmd_t directedConnect();
        static nrf_cmd_t sendData(nrf_pipe_t servicePipeNo,
                                  nrf_len_t dataLength,
                                  uint8_t *data);
        static nrf_cmd_t requestData(nrf_pipe_t servicePipeNo);
        static nrf_cmd_t setLocalData(nrf_pipe_t servicePipeNo,
                                      nrf_len_t dataLength,
                                      uint8_t data);
        static nrf_cmd_t sendDataAck(nrf_pipe_t servicePipeNo);
        static nrf_cmd_t sendDataNack(nrf_pipe_t servicePipeNo,
                                      uint8_t errorCode);
};

#endif /* _NRF8001_H */
