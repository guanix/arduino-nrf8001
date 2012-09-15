#include <SPI.h>
#include <Arduino.h>
#include <assert.h>
#include <avr/interrupt.h>
#include "nRF8001.h"
#include "services.h"

#ifdef PROGMEM
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#endif

hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

nRFDeviceState nRF8001::getDeviceState()
{
    return deviceState;
}

void nRF8001::setup()
{
    nrf_debug("sending setup message number 0");

    transmitReceive((nRFCommand *)setup_msgs[nextSetupMessage++].buffer);

    for (;;) {
        nrf_debug("Calling transmitReceive...");
        transmitReceive(0);
    }
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
    nextSetupMessage = 0;

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
}

void nRF8001::debugAddress(uint8_t *address)
{
    Serial.print("0x");
    // Addresses are NRF_ADDRESS_LENGTH long, MSB to LSB
    for (int i = NRF_ADDRESS_LENGTH - 1; i >= 0; i--) {
        uint8_t c = address[i];
        if (c < 0x10) {
            Serial.print("0");
        }
        Serial.print(c, HEX);
    }
}

void nRF8001::debugEvent(nRFEvent *event)
{
    Serial.print(F("EVENT debug="));
    Serial.print(event->debug);
    Serial.print(F(" length="));
    Serial.print(event->length);
    Serial.println(F(" event="));

    switch (event->event) {
        case NRF_DEVICESTARTEDEVENT:
            Serial.println(F("DeviceStartedEvent"));

            Serial.print(F("Operating mode: "));
            switch (event->msg.deviceStarted.operatingMode) {
                case 0x01:
                    Serial.println(F("Test"));
                    break;
                case 0x02:
                    Serial.println(F("Setup"));
                    break;
                case 0x03:
                    Serial.println(F("Standby"));
                    break;
                default:
                    Serial.println(event->msg.deviceStarted.operatingMode, HEX);
                    break;
            }

            Serial.print(F("Hardware error: "));
            switch (event->msg.deviceStarted.hwError) {
                case 0x01:
                    Serial.println(F("No error"));
                    break;
                case 0x02:
                    Serial.println(F("Fatal error"));
                    break;
                default:
                    Serial.println(event->msg.deviceStarted.hwError, HEX);
                    break;
            }

            Serial.print(F("DataCreditAvailable: "));
            Serial.println(event->msg.deviceStarted.dataCreditAvailable);
            break;
        case NRF_ECHOEVENT:
            Serial.println(F("EchoEvent"));
            break;
        case NRF_HARDWAREERROREVENT:
            Serial.println(F("HardwareErrorEvent"));
            break;
        case NRF_COMMANDRESPONSEEVENT:
            Serial.println(F("CommandResponseEvent"));

            Serial.print(F("Status: "));
            switch (event->msg.commandResponse.status) {
                case NRF_STATUS_SUCCESS:
                    Serial.println(F("Success"));
                    break;
                case NRF_STATUS_TRANSACTION_CONTINUE:
                    Serial.println(F("Transaction continuation status"));
                    break;
                case NRF_STATUS_TRANSACTION_COMPLETE:
                    Serial.println(F("Transaction completed"));
                    break;
                case NRF_STATUS_EXTENDED:
                    Serial.println(F("Extended status, further checks needed"));
                    break;
                case NRF_STATUS_ERROR_UNKNOWN:
                    Serial.println(F("Unknown error"));
                    break;
                case NRF_STATUS_ERROR_INTERNAL:
                    Serial.println(F("Internal error"));
                    break;
                case NRF_STATUS_ERROR_CMD_UNKNOWN:
                    Serial.println(F("Unknown command"));
                    break;
                case NRF_STATUS_ERROR_DEVICE_STATE_INVALID:
                    Serial.println(F("Command invalid in the current device state"));
                    break;
                case NRF_STATUS_ERROR_INVALID_LENGTH:
                    Serial.println(F("Invalid length"));
                    break;
                case NRF_STATUS_ERROR_INVALID_PARAMETER:
                    Serial.println(F("Invalid input parameters"));
                    break;
                case NRF_STATUS_ERROR_BUSY:
                    Serial.println(F("Busy"));
                    break;
                case NRF_STATUS_ERROR_INVALID_DATA:
                    Serial.println(F("Invalid data format or contents"));
                    break;
                case NRF_STATUS_ERROR_CRC_MISMATCH:
                    Serial.println(F("CRC mismatch"));
                    break;
                case NRF_STATUS_ERROR_UNSUPPORTED_SETUP_FORMAT:
                    Serial.println(F("Unsupported setup format"));
                    break;
                case NRF_STATUS_ERROR_INVALID_SEQ_NO:
                    Serial.println(F(
                        "Invalid sequence number during "
                        "a write dynamic data sequence"));
                    break;
                case NRF_STATUS_ERROR_SETUP_LOCKED:
                    Serial.println(F(
                        "Setup data is locked and cannot be modified"));
                    break;
                case NRF_STATUS_ERROR_LOCK_FAILED:
                    Serial.println(F(
                        "Setup error due to lock verification failure"));
                    break;
                case NRF_STATUS_ERROR_BOND_REQUIRED:
                    Serial.println(F(
                        "Bond required: Local service pipes need "
                        "bonded/trusted peer"));
                    break;
                case NRF_STATUS_ERROR_REJECTED:
                    Serial.println(F(
                        "Command rejected as a transaction is still pending"));
                    break;
                case NRF_STATUS_ERROR_DATA_SIZE:
                    Serial.println(F(
                        "Pipe Error Event: Data size exceeds size specified "
                        "for pipe, Transmit failed"));
                    break;
                case NRF_STATUS_ERROR_PIPE_INVALID:
                    Serial.println(F(
                        "Pipe Error Event: Transmit failed, "
                        "Invalid or unavailable Pipe number or "
                        "unknown pipe type"));
                    break;
                case NRF_STATUS_ERROR_CREDIT_NOT_AVAILABLE:
                    Serial.println(F(
                        "Pipe Error Event: Credit not available"));
                    break;
                case NRF_STATUS_ERROR_PEER_ATT_ERROR:
                    Serial.println(F(
                        "Pipe Error Event: Peer device has sent an error on "
                        "an pipe operation on the remote characteristic"));
                    break;
                case NRF_STATUS_ERROR_ADVT_TIMEOUT:
                    Serial.println(F(
                        "Connection was not established before the BTLE "
                        "advertising was stopped"));
                    break;
                case NRF_STATUS_ERROR_PEER_SMP_ERROR:
                    Serial.println(F(
                        "Remote device triggered a Security Manager Protocol "
                        "error"));
                    break;
                case NRF_STATUS_ERROR_PIPE_TYPE_INVALID:
                    Serial.println(F(
                        "Pipe Error Event: Pipe type invalid for the "
                        "selected operation"));
                    break;
                case NRF_STATUS_ERROR_PIPE_STATE_INVALID:
                    Serial.println(F("Pipe Error Event: Pipe state invalid "
                        "for the selected operation"));
                    break;
                case NRF_STATUS_ERROR_INVALID_KEY_SIZE:
                    Serial.println(F("Invalid key size provided"));
                    break;
                case NRF_STATUS_ERROR_INVALID_KEY_DATA:
                    Serial.println(F("Invalid key data provided"));
                    break;
                default:
                    Serial.println(event->msg.commandResponse.status, HEX);
                    break;
            }

            Serial.print("Op: ");
            switch (event->msg.commandResponse.opcode) {
                case NRF_TEST_OP:
                    Serial.println(F("Test"));
                    break;
                case NRF_GETDEVICEVERSION_OP:
                    Serial.println(F("GetDeviceVersion"));
                    Serial.print(F("ConfigurationID="));
                    Serial.print(event->msg.commandResponse.data
                        .getDeviceVersion.configurationId);
                    Serial.print(F(" ACIVersion="));
                    Serial.print(event->msg.commandResponse.data
                        .getDeviceVersion.aciVersion);
                    Serial.print(F(" SetupFormat="));
                    Serial.print(event->msg.commandResponse.data
                        .getDeviceVersion.setupFormat);
                    Serial.print(F(" SetupID="));
                    Serial.print(event->msg.commandResponse.data
                        .getDeviceVersion.setupId);
                    Serial.print(F(" ConfigurationStatus="));
                    if (event->msg.commandResponse.data.getDeviceVersion
                        .configurationStatus == 1) {
                        Serial.println(F("SetupLocked"));
                    } else {
                        Serial.println(F("SetupOpen"));
                    }
                    break;
                case NRF_WAKEUP_OP:
                    Serial.println(F("Wakeup"));
                    break;
                case NRF_GETBATTERYLEVEL_OP:
                    Serial.println(F("GetBatteryLevel"));
                    Serial.print(event->msg.commandResponse
                        .data.voltage*3.52, 2);
                    Serial.println("mV");
                    break;
                case NRF_GETTEMPERATURE_OP:
                    Serial.println(F("GetTemperature"));
                    Serial.print(event->msg.commandResponse
                        .data.temperature/4.0, 2);
                    Serial.println(" C");
                    break;
                case NRF_SETUP_OP:
                    Serial.println(F("Setup"));
                    break;
                case NRF_SETTXPOWER_OP:
                    Serial.println(F("SetTxPower"));
                    break;
                case NRF_GETDEVICEADDRESS_OP:
                    Serial.println(F("GetDeviceAddress"));
                    Serial.print(F("Address: "));
                    debugAddress(event->msg.commandResponse.data
                        .getDeviceAddress.deviceAddress);
                    Serial.println("");
                    break;
                case NRF_CONNECT_OP:
                    Serial.println(F("Connect"));
                    break;
                case NRF_RADIORESET_OP:
                    Serial.println(F("RadioReset"));
                    break;
                case NRF_BOND_OP:
                    Serial.println(F("Bond"));
                    break;
                case NRF_DISCONNECT_OP:
                    Serial.println(F("Disconnect"));
                    break;
                case NRF_CHANGETIMINGREQUEST_OP:
                    Serial.println(F("ChangeTimingRequest"));
                    break;
                case NRF_OPENREMOTEPIPE_OP:
                    Serial.println(F("OpenRemotePipe"));
                    break;
                case NRF_CLOSEREMOTEPIPE_OP:
                    Serial.println(F("CloseRemotePipe"));
                    break;
                case NRF_DTMCOMMAND_OP:
                    Serial.println(F("DtmCommand"));
                    Serial.print(F("DTM data: "));
                    Serial.println(event->msg.commandResponse.data.dtmEvent,
                        HEX);
                    break;
                case NRF_READDYNAMICDATA_OP:
                    Serial.println(F("ReadDynamicData"));
                    Serial.print(F("Sequence no="));
                    Serial.println(event->msg.commandResponse.data
                        .readDynamicData.sequenceNo);
                    Serial.println(F("TODO: data here"));
                    break;
                case NRF_WRITEDYNAMICDATA_OP:
                    Serial.println(F("WriteDynamicData"));
                    break;
                case NRF_SETAPPLICATIONLATENCY_OP:
                    Serial.println(F("SetApplLatency"));
                    break;
                case NRF_SETKEY_OP:
                    Serial.println(F("SetKey"));
                    break;
                case NRF_OPENADVPIPE_OP:
                    Serial.println(F("OpenAdvPipe"));
                    break;
                case NRF_BROADCAST_OP:
                    Serial.println(F("Broadcast"));
                    break;
                case NRF_BONDSECREQUEST_OP:
                    Serial.println(F("BondSecurityRequest"));
                    break;
                case NRF_DIRECTEDCONNECT_OP:
                    Serial.println(F("DirectedConnect"));
                    break;
                case NRF_SETLOCALDATA_OP:
                    Serial.println(F("SetLocalData"));
                    break;
                default:
                    Serial.print(event->msg.commandResponse.opcode, HEX);
                    break;
            }
            break;
        case NRF_CONNECTEDEVENT:
            Serial.println(F("ConnectedEvent"));
            Serial.print(F("Address type: "));
            switch (event->msg.connected.addressType) {
                case 0x01:
                    Serial.println(F("Public address"));
                    break;
                case 0x02:
                    Serial.println(F("Random static address"));
                    break;
                case 0x03:
                    Serial.println(F("Random private address (resolvable)"));
                    break;
                case 0x04:
                    Serial.println(
                        F("Random private adddress (non-resolvable)"));
                    break;
                default:
                    Serial.println(event->msg.connected.addressType, HEX);
                    break;
            }

            Serial.print(F("Peer address: "));
            debugAddress(event->msg.connected.peerAddress);

            Serial.print(F("Connection interval: "));
            Serial.print(event->msg.connected.connectionInterval/1.25, 2);
            Serial.println("ms");

            Serial.print(F("Slave latency: "));
            Serial.println(event->msg.connected.slaveLatency);

            Serial.print(F("Supervision timeout: "));
            Serial.println(event->msg.connected.supervisionTimeout);

            Serial.print(F("Master clock accuracy: "));
            switch (event->msg.connected.masterClockAccuracy) {
                case 0x00:
                    Serial.println(F("500 ppm"));
                    break;
                case 0x01:
                    Serial.println(F("250 ppm"));
                    break;
                case 0x02:
                    Serial.println(F("150 ppm"));
                    break;
                case 0x03:
                    Serial.println(F("100 ppm"));
                    break;
                case 0x04:
                    Serial.println(F("75 ppm"));
                    break;
                case 0x05:
                    Serial.println(F("50 ppm"));
                    break;
                case 0x06:
                    Serial.println(F("30 ppm"));
                    break;
                case 0x07:
                    Serial.println(F("20 ppm"));
                    break;
                default:
                    Serial.println(event->msg.connected.masterClockAccuracy,
                        HEX);
                    break;
            }
            break;
        case NRF_DISCONNECTEDEVENT: {
            Serial.println(F("DisconnectedEvent"));

            Serial.print(F("ACI status: "));
            switch (event->msg.disconnected.aciStatus) {
                case 0x03:
                    Serial.println(
                        F("Check the Bluetooth low energy status code"));
                    break;
                case 0x93:
                    Serial.println(F(
                        "Timeout while advertising, "
                        "unable to establish connection"));
                    break;
                case 0x8d:
                    Serial.println(F(
                        "Bond required to proceed with connection"));
                    break;
                default:
                    Serial.println(event->msg.disconnected.aciStatus, HEX);
                    break;
            }

            Serial.print(F("BtLeStatus: 0x"));
            uint8_t btLeStatus = event->msg.disconnected.btLeStatus;
            if (btLeStatus < 0x10) {
                Serial.print("0");
            }
            Serial.println(btLeStatus, HEX);
            break;
        } 
        case NRF_BONDSTATUSEVENT:
            Serial.println(F("BondStatusEvent"));
            Serial.println(F("TODO: bond status data"));
            break;
        case NRF_PIPESTATUSEVENT:
            Serial.println(F("PipeStatusEvent"));
            
            Serial.print(F("Open: "));
            for (int i = 0; i < 64; i++) {
                if (event->msg.pipeStatus.pipesOpen & i) {
                    Serial.print(" ");
                    Serial.print(i, DEC);
                }
            }
            Serial.println("");

            Serial.print(F("Closed: "));
            for (int i = 0; i < 64; i++) {
                if (event->msg.pipeStatus.pipesClosed & i) {
                    Serial.print(" ");
                    Serial.print(i, DEC);
                }
            }
            Serial.println("");
            break;
        case NRF_TIMINGEVENT:
            Serial.println(F("TimingEvent"));

            Serial.print(F("Connection interval: "));
            Serial.println(event->msg.timing.connectionInterval/1.25, 2);

            Serial.print(F("Slave latency: "));
            Serial.println(event->msg.timing.slaveLatency, 2);

            Serial.print(F("Supervision timeout: "));
            Serial.print(event->msg.timing.supervisionTimeout*10.0, 2);
            Serial.println("ms");
            break;
        case NRF_DISPLAYKEYEVENT:
            Serial.println(F("DisplayKeyEvent"));
            Serial.println(F("Passkey: "));
            for (int i = 0; i < NRF_PASSKEY_LENGTH; i++) {
                Serial.print(event->msg.passkey[i]);
            }
            Serial.println("");
            break;
        case NRF_KEYREQUESTEVENT:
            Serial.println(F("KeyRequestEvent"));
            break;
        case NRF_DATACREDITEVENT:
            Serial.println(F("DataCreditEvent"));
            Serial.print(event->msg.dataCredits);
            Serial.println(" credits");
            break;
        case NRF_PIPEERROREVENT:
            Serial.println(F("PipeErrorEvent"));
            Serial.print("Pipe: ");
            Serial.println(event->msg.pipeError.servicePipeNo);

            Serial.print("Error code: 0x");
            Serial.println(event->msg.pipeError.errorCode, HEX);
            break;
        case NRF_DATARECEIVEDEVENT:
            Serial.println(F("DataReceivedEvent"));
            Serial.print("Pipe: ");
            Serial.println(event->msg.dataReceived.servicePipeNo);
            break;
        case NRF_DATAACKEVENT:
            Serial.println(F("DataAckEvent"));
            Serial.print("Pipe: ");
            Serial.println(event->msg.servicePipeNo);
            break;
        default:
            Serial.print("Unknown ");
            Serial.println(event->event);
            break;
    }
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

#if NRF_DEBUG
    debugEvent(rxEvent);
#endif

    // Handle response
    switch (rxEvent->event) {
        case NRF_DEVICESTARTEDEVENT:
            credits = rxEvent->msg.deviceStarted.dataCreditAvailable;
            break;
        case NRF_COMMANDRESPONSEEVENT: {
            switch (rxEvent->msg.commandResponse.opcode) {
                // We only do handling of some of these messages

            }

            // Dispatch event
            break;
        }
        case NRF_CONNECTEDEVENT:
            break;
        case NRF_DISCONNECTEDEVENT:
            break;
        case NRF_DATACREDITEVENT:
            credits = rxEvent->msg.dataCredits;
            break;
        default: {
            // Dispatch event
        }
    }

    return Success;
}

nRFCmd nRF8001::sleep()
{
    transmitReceive(0);
}
