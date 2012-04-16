#include <Arduino.h>
#include <assert.h>
#include <SPI.h>
#include <avr/interrupt.h>
#include "nRF8001.h"
#include "services.h"

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

// internal state
typedef enum {
    RXONLY,
    RXTX,
    RXWAIT,
    TXWAIT
} nRFSpiState;

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
static nRFEvent rxBuffer;
static nrf_state_t deviceState;

// Ring buffer of nRFE
static nRFEvent rxRingBuffer[NRF_RX_BUFFERS];
static volatile nRFEvent *rxRingBufferIn;
static volatile nRFEvent *rxRingBufferOut;
static volatile uint8_t rxRingBufferCount;

inline void rdynLowISR();

void sendSetupMessage()
{
    memset(&txBuffer, 0, sizeof(nRFEvent));
    memcpy(&txBuffer, setup_msgs[nextSetup++].buffer, NRF_MAX_PACKET_LENGTH);

    spiState = TXWAIT;
    digitalWrite(reqn_pin, LOW);
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

    memset(&rxRingBuffer, 0, sizeof(nRFEvent)*NRF_RX_BUFFERS);
    memset(&rxBuffer, 0, sizeof(nRFEvent));
    memset(&txBuffer, 0, sizeof(nRFCommand));

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        rxRingBufferIn = rxRingBuffer;
        rxRingBufferOut = rxRingBuffer;
        rxRingBufferCount = 0;
    }

    deviceState = Setup;
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
    sendSetupMessage();

    // Start interrupts
    attachInterrupt(0, rdynLowISR, FALLING);
    interrupts();
}

// Interrupt handlers
inline void rdynLowISR()
{
    deviceState = Active;
    rdynHigh = 0;

    // set spiState and send the first byte
    if (spiState == RXWAIT) {
        spiState = RXONLY;
        digitalWrite(reqn_pin, LOW);
        SPDR = 0;
    } else if (spiState == TXWAIT) {
        spiState = RXTX;
        SPDR = ((uint8_t *)&txBuffer)[0];
    } else {
        // something wrong
        return;
    }
}

nrf_tx_status_t transmit(nRFCommand *txCmd)
{
    // Are we in a state where we can transmit?
    if (deviceState == Setup || deviceState == Active || spiState != RXWAIT) {
        return InvalidState;
    }

    // Enough credits?
    if (txCmd->command == NRF_SENDDATA_OP
     || txCmd->command == NRF_REQUESTDATA_OP
     || txCmd->command == NRF_SETLOCALDATA_OP
     || txCmd->command == NRF_SETDATAACK_OP
     || txCmd->command == NRF_SENDDATANACK_OP) {
        if (credits-- < 1) {
            return InsufficientCredits;
        }
    }

    // Clear and copy to TX buffer
    memset(&txBuffer, 0, sizeof(nRFCommand));
    memcpy(&txBuffer, txCmd, txCmd->length + 1);

    // Bring REQN low
    spiState = TXWAIT;
    digitalWrite(reqn_pin, LOW);

    return Success;
}

ISR(SPI_STC_vect)
{
    nRFEvent *rxEvent;

    ((uint8_t *)&rxBuffer)[nextByte] = SPDR;

    // TODO: check that we are in the right state

    if (nextByte == 1) {
        // I have no idea whether it's a bad idea to double-read SPDR,
        // but assuming that it is, this saves us a byte of memory.
        rxLength = ((uint8_t *)&rxBuffer)[nextByte];
        if (rxLength > 30) {
            // Length too long, there's something wrong
            spiState = RXWAIT;
            digitalWrite(reqn_pin, HIGH);
            return;
        }
    }
    
    // TODO: optimize by precalculating these decisions as a bitmap
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

        // wait until RDYN goes high again
        while (digitalRead(rdyn_pin) == LOW);

        // basic housekeeping
        spiState = RXWAIT;
        rxLength = nextByte = 0;
        
        // manage credits
        if (rxBuffer.event == NRF_DATACREDITEVENT) {
            credits += rxBuffer.msg.dataCredits;
        }

        if (rxBuffer.event == NRF_COMMANDRESPONSEEVENT
&& rxBuffer.msg.commandResponse.opcode == NRF_STATUS_TRANSACTION_CONTINUE) {
            // queue up the next setup message
            sendSetupMessage();
        }

        if (rxBuffer.event == NRF_DEVICESTARTEDEVENT) {
            credits = rxBuffer.msg.deviceStarted.dataCreditAvailable;
            deviceState = Standby;
        }

        if (txBuffer.command == NRF_SLEEP_OP) {
            deviceState = Sleep;
        }

        // copy into ring buffer of nRFEvent objects
        memcpy(&rxRingBufferIn, &rxBuffer, sizeof(nRFEvent));
        rxRingBufferIn += sizeof(nRFEvent);
        if (rxRingBufferIn == rxRingBuffer + NRF_RX_BUFFERS*sizeof(nRFEvent)) {
            rxRingBufferIn = rxRingBuffer;
        }
        rxRingBufferCount++;
        memset(&rxBuffer, 0, sizeof(nRFEvent));
        // save some cycles here by clearing the tx buffer when transmitting

        // Did RDYN go high, then low again? invoke the ISR, we missed an
        // interrupt while we monkeyed around
        if (digitalRead(rdyn_pin) == LOW) {
            // timing is everything
            interrupts();
            rdynLowISR();
        } else {
            interrupts();
        }

    }

    nextByte++;
}
