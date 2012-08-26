#include <SPI.h>
#include <Arduino.h>
#include <assert.h>
#include <avr/interrupt.h>
#include "nRF8001.h"
#include "services.h"

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

nRFDeviceState nRF8001::getDeviceState()
{
    return deviceState;
}

void nRF8001::sendSetupMessages()
{
    /*
#if NRF_DEBUG
    Serial.print("sending setup message number ");
    Serial.println(nextSetup);
    Serial.print("spiState = ");
    Serial.println(spiState);
#endif
    memset(&txBuffer, 0, sizeof(nRFEvent));
    memcpy(&txBuffer, setup_msgs[nextSetup++].buffer, NRF_MAX_PACKET_LENGTH);

    while (spiState != RXWAIT);

    spiState = TXWAIT;
    digitalWrite(reqn_pin, LOW);
    */
}

nRF8001::nRF8001(uint8_t reset_pin_arg,
                 uint8_t reqn_pin_arg,
                 uint8_t rdyn_pin_arg,
                 nRFEventHandler eventHandler)
{
    nrf_debug("Initializing");
    // Initialize data structures
    reset_pin = reset_pin_arg;
    reqn_pin = reqn_pin_arg;
    rdyn_pin = rdyn_pin_arg;
    listener = eventHandler;

    deviceState = Setup;
    credits = 0;

    // Prepare pins and start SPI
    pinMode(reqn_pin, OUTPUT);
    pinMode(rdyn_pin, INPUT);
    digitalWrite(rdyn_pin, HIGH);
    digitalWrite(reqn_pin, HIGH);

    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, LOW);
    delayMicroseconds(1);
    digitalWrite(reset_pin, HIGH);

    // inialize SPI
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SS, OUTPUT);

    digitalWrite(SCK, LOW);
    digitalWrite(MOSI, LOW);
    digitalWrite(SS, HIGH);

    // SPI mode 0; /16 clock divider
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(LSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    SPI.begin();

    // Load up the first setup message and start interrupts
#if NB_SETUP_MESSAGES < 1
#error Make sure you included devices.h from nRFgo Studio, or try services.h.example
#endif

    nrf_debug("Ready to send first setup message");
    sendSetupMessages();
}

// Transmit a command, and simultaneously receive a message from nRF8001 if
// there is one. To just receive without transmitting anything, call this
// function with a NULL argument.
nRFTxStatus nRF8001::transmitReceive(nRFCommand *txCmd)
{
    // Buffer that we will receive into
    uint8_t rxBuffer[sizeof(nRFEvent)];
    nRFEvent *rxEvent = (nRFEvent *)rxBuffer;
    memset(&rxBuffer, 0, sizeof(nRFEvent));

    uint8_t *txBuffer = (uint8_t *)txCmd;

    // Transmit length
    uint8_t txLength, txCommand;
    if (txCmd != NULL) {
        txLength = txCmd->length;
        txCommand = txCmd->command;
    } else {
        txLength = 0;
        txCommand = 0;
    }

    assert(txLength <= NRF_MAX_PACKET_LENGTH);

    // Enough credits?
    if (txLength &&
       (txCmd->command == NRF_SENDDATA_OP
     || txCmd->command == NRF_REQUESTDATA_OP
     || txCmd->command == NRF_SETLOCALDATA_OP
     || txCmd->command == NRF_SETDATAACK_OP
     || txCmd->command == NRF_SENDDATANACK_OP)) {
        if (credits < 1) {
            nrf_debug("transmitReceive fail, not enough credits");
            return InsufficientCredits;
        }

        // Use a credit
        credits--;
    }

    // Bring REQN low
    digitalWrite(reqn_pin, LOW);

    // Wait for RDYN low
    while (digitalRead(rdyn_pin) == HIGH);

    nrf_debug("Ready to transmitReceive full duplex!");

    // Send length and command bytes,
    // receive debug and length bytes
    rxEvent->debug = SPI.transfer(txLength);
    rxEvent->length = SPI.transfer(txCommand);

    assert(rxEvent->length <= NRF_MAX_PACKET_LENGTH);

    // nextByte points to the next byte to be transferred in
    // txBuffer, or the next byte to be received in rxBuffer
    // For TX, packets are data + 1 byte (length)
    // For RX, packets are data + 2 bytes (debug and length)
    for (uint8_t nextByte = 2;
            nextByte < txLength + 1 || nextByte < rxEvent->length + 2;
            nextByte++) {
        uint8_t c;
        if (nextByte < txLength + 1) {
            c = SPI.transfer(txBuffer[nextByte]); // transmit
        } else {
            c = SPI.transfer(0); // receive only
        }

        if (nextByte < rxEvent->length + 2) { // receive
            rxBuffer[nextByte] = c;
        }
    }

    // Bring REQN high
    digitalWrite(reqn_pin, HIGH);
    // Wait for RDYN high
    while (digitalRead(rdyn_pin) == LOW);

    // Return immediately if we didn't receive anything
    if (!rxEvent->length) {
        return Success;
    }

    

    return Success;
}

nRFCmd nRF8001::sleep()
{
    transmitReceive(0);
}
