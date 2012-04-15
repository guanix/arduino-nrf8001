#include <Arduino.h>
#include <assert.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include "nRF8001.h"
#include "services.h"
#include "LightweightRingBuff.h"

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

// We keep all this as file static because of problems accessing
// class variables from interrupt handlers
static uint8_t reset_pin;
static uint8_t reqn_pin;
static uint8_t rdyn_pin;
static uint8_t rdyn_int;
static nRFEventHandler listener;
static volatile nRFSpiState spiState;
static volatile uint8_t credits;
static volatile uint8_t nextSetup;
static volatile uint8_t nextByte;
static volatile uint8_t rxLength;
static nRFCommand txBuffer;
static RingBuff_t rxRingBuffer;
static uint8_t deviceState;

// Ring buffer of nRFE
static nRFEvent rxBuffers[NRF_RX_BUFFERS];
static volatile nRFEvent *rxBuffersIn;
static volatile nRFEvent *rxBuffersOut;
static volatile uint8_t rxBuffersCount;

inline void rdynLowISR();

void loadSetupMessage()
{
    memset(&txBuffer, 0, sizeof(nRFEvent));
    memcpy(&txBuffer, setup_msgs[nextSetup++].buffer, NRF_MAX_PACKET_LENGTH);

    spiState = TXWAIT;
}

void nRF8001::setup(uint8_t reset_pin_arg,
                    uint8_t reqn_pin_arg,
                    uint8_t rdyn_pin_arg,
                    uint8_t rdyn_int_arg,
                    nRFEventHandler eventHandler)
{
    // Initialize data structures
    reset_pin = reset_pin_arg;
    reqn_pin = reqn_pin_arg;
    rdyn_pin = rdyn_pin_arg;
    rdyn_int = rdyn_int_arg;
    listener = eventHandler;

    memset(&rxBuffers, 0, sizeof(nRFEvent)*NRF_RX_BUFFERS);
    memset(&rxRingBuffer, 0, sizeof(RingBuff_t));
    memset(&txBuffer, 0, sizeof(nRFCommand));

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rxBuffersIn = rxBuffers;
        rxBuffersOut = rxBuffers;
        rxBuffersCount = 0;
    }

    RingBuffer_InitBuffer(&rxRingBuffer);

    deviceState = NRF_STATE_SETUP;
    spiState = RXONLY;
    nextByte = rxLength = credits = nextSetup = 0;

    // Prepare pins and start SPI
    pinMode(reqn_pin, OUTPUT);
    pinMode(rdyn_pin, INPUT);
    digitalWrite(rdyn_pin, HIGH);
    digitalWrite(reqn_pin, HIGH);

    pinMode(reset_pin, OUTPUT);
    pinMode(reset_pin, HIGH);

    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.begin();

    // Load up the first setup message
    assert(NB_SETUP_MESSAGES > 0);
    loadSetupMessage();

    // Start interrupts
    attachInterrupt(0, rdynLowISR, FALLING);
    interrupts();
}

// Interrupt handlers
inline void rdynLowISR()
{
    // set spiState and send the first byte
    if (spiState == RXWAIT) {
        spiState = RXONLY;
        SPDR = 0;
    } else if (spiState == TXWAIT) {
        spiState = RXTX;
        SPDR = ((uint8_t *)&txBuffer)[0];
    } else {
        // something wrong
        return;
    }
}

ISR(SPI_STC_vect)
{
    RingBuffer_Insert(&rxRingBuffer, SPDR);

    if (nextByte == 1) {
        rxLength = SPDR;
    }
    
    if (spiState == RXTX && nextByte + 1 < txBuffer.length + 1) {
        // there is something to transmit
        SPDR = ((uint8_t *)&txBuffer)[nextByte + 1];
    } else if (rxLength > 0 && nextByte + 1 < rxLength + 2) {
        // receive only
        SPDR = 0;
    } else {
        // we are done!
        noInterrupts();
        digitalWrite(reqn_pin, HIGH);

        // basic housekeeping
        spiState = RXWAIT;
        rxLength = nextByte = 0;
        memset(&txBuffer, 0, sizeof(nRFCommand));
        memset(&rxRingBuffer, 0, sizeof(RingBuff_t));
        RingBuffer_InitBuffer(&rxRingBuffer);
        
        // manage credits TODO

        // special setup stage handling
        if (deviceState == NRF_STATE_SETUP && nextSetup < NB_SETUP_MESSAGES) {
            loadSetupMessage();
        }

        interrupts();
    }

    nextByte++;
}
