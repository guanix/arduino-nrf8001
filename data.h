// nRF8001 data structures

#ifndef _NRF8001_DATA_H
#define _NRF8001_DATA_H

struct nRFCommand {
    uint8_t length;
    uint8_t command;

    union {
        uint8_t rawData[NRF_MAX_PACKET_LENGTH-2];

        uint8_t testFeature;

        uint8_t echoData[NRF_MAX_ECHO_MESSAGE_LENGTH];

        uint8_t setupData[NRF_SETUP_DATA_LENGTH];

        uint8_t radioTxPowerLevel;

        struct {
            uint16_t timeout;
            uint16_t advInterval;
        } connect;

        struct {
            uint16_t timeout;
            uint16_t advInterval;
        } bond;

        uint8_t disconnectReason;

        struct {
            uint16_t intervalMin;
            uint16_t intervalMax;
            uint16_t slaveLatency;
            uint16_t timeout;
        } changeTimingRequest;

        nRFPipe servicePipeNo;

        uint16_t dtmCommand;

        struct {
            uint8_t sequenceNo;
            uint8_t dynamicData[NRF_DYNAMIC_DATA_LENGTH];
        } writeDynamicData;

        struct {
            uint8_t applLatencyMode;
            uint16_t latency;
        } setApplLatency;

        struct {
            uint8_t keyType;
            uint8_t key[NRF_PASSKEY_LENGTH];
        } setKey;

        uint64_t advServiceDataPipes;

        struct {
            uint16_t timeout;
            uint16_t advInterval;
        } broadcast;

        struct {
            uint8_t servicePipeNo;
            uint8_t data[NRF_DATA_LENGTH];
        } data;

        struct {
            uint8_t servicePipeNo;
            uint8_t errorCode;
        } sendDataNack;
    } content;
};

struct nRFEvent {
    uint8_t debug;
    uint8_t length;
    uint8_t event;
    
    union {
        uint8_t rawData[NRF_MAX_PACKET_LENGTH-2];
        
        // Various response types
        struct {
            uint8_t operatingMode;
            uint8_t hwError;
            uint8_t dataCreditAvailable;
        } deviceStarted;
        
        uint8_t echo[NRF_MAX_ECHO_MESSAGE_LENGTH];
        
        struct {
            uint16_t lineNo;
            uint8_t fileName[NRF_FIRMWARE_FILENAME_LENGTH];
        } hardwareError;
        
        struct {
            uint8_t opcode;
            uint8_t status;
            
            union {
                uint8_t rawData[NRF_COMMANDRESPONSEEVENT_LENGTH];

                struct {
                    uint16_t configurationId;
                    uint8_t aciVersion;
                    uint8_t setupFormat;
                    uint32_t setupId;
                    uint8_t configurationStatus;
                } getDeviceVersion;

                uint16_t voltage;

                uint16_t temperature;

                struct {
                    uint8_t deviceAddress[NRF_ADDRESS_LENGTH];
                    uint8_t addressType;
                } getDeviceAddress;

                uint16_t dtmEvent;

                struct {
                    uint8_t sequenceNo;
                    uint8_t dynamicData[NRF_DYNAMIC_DATA_LENGTH];
                } readDynamicData;
            } data;
        } commandResponse;
        
        struct {
            uint8_t addressType;
            uint8_t peerAddress[NRF_ADDRESS_LENGTH];
            uint16_t connectionInterval;
            uint16_t slaveLatency;
            uint16_t supervisionTimeout;
            uint8_t masterClockAccuracy;
        } connected;

        struct {
            uint8_t aciStatus;
            uint8_t btLeStatus;
        } disconnected;

        struct {
            uint8_t bondStatusCode;
            uint8_t bondStatusSource;
            uint8_t bondStatusSecMode1;
            uint8_t bondStatusSecMode2;
            uint8_t bondStatusKeyExchSlave;
            uint8_t bondStatusKeyExchMaster;
        } bondStatus;

        struct {
            uint64_t pipesOpen;
            uint64_t pipesClosed;
        } pipeStatus;

        struct {
            uint16_t connectionInterval;
            uint16_t slaveLatency;
            uint16_t supervisionTimeout;
        } timing;

        uint8_t passkey[NRF_PASSKEY_LENGTH];

        uint8_t keyType;

        uint8_t dataCredits;

        struct {
            nRFPipe servicePipeNo;
            uint8_t errorCode;

            uint8_t rawData[NRF_PIPE_ERROR_LENGTH];
        } pipeError;

        struct {
            nRFPipe servicePipeNo;
            uint8_t data[NRF_DATA_LENGTH];
        } dataReceived;

        nRFPipe servicePipeNo;
    } msg;
};

#endif /* _NRF8001_DATA_H */
