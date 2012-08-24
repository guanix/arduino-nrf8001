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
    spiState = RXONLY;
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
    SPCR = _BV(SPIE) | _BV(SPE) | _BV(DORD) | _BV(MSTR) | _BV(SPR0);

    // Load up the first setup message and start interrupts
#if NB_SETUP_MESSAGES < 1
#error Make sure you included devices.h from nRFgo Studio
#endif

    nrf_debug("Ready to send first setup message");
    sendSetupMessages();
}

nRFTxStatus nRF8001::transmit(nRFCommand *txCmd)
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
//    memset(&txBuffer, 0, sizeof(nRFCommand));
//    memcpy(&txBuffer, txCmd, txCmd->length + 1);

    // Bring REQN low
    spiState = TXWAIT;
    digitalWrite(reqn_pin, LOW);

    return Success;
}
