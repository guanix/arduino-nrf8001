#ifndef _NRF8001_CONSTANTS_H
#define _NRF8001_CONSTANTS_H

// Don't use enums for this because we use these values in structs that
// data structures transferred over the wire, and we need to be absolutely
// certain of their sizes.

#define NRF_MAX_PACKET_LENGTH               32
#define NRF_MAX_ECHO_MESSAGE_LENGTH         29
#define NRF_FIRMWARE_FILENAME_LENGTH        22
#define NRF_COMMANDRESPONSEEVENT_LENGTH     27
#define NRF_ADDRESS_LENGTH                  6
#define NRF_PASSKEY_LENGTH                  6
#define NRF_PIPE_ERROR_LENGTH               27
#define NRF_DATA_LENGTH                     20
#define NRF_DYNAMIC_DATA_LENGTH             27
#define NRF_SETUP_DATA_LENGTH               30

#define NRF_DEVICESTARTEDEVENT      0x81
#define NRF_ECHOEVENT               0x82
#define NRF_HARDWAREERROREVENT      0x83
#define NRF_COMMANDRESPONSEEVENT    0x84
#define NRF_CONNECTEDEVENT          0x85
#define NRF_DISCONNECTEDEVENT       0x86
#define NRF_BONDSTATUSEVENT         0x87
#define NRF_PIPESTATUSEVENT         0x88
#define NRF_TIMINGEVENT             0x89
#define NRF_DISPLAYKEYEVENT         0x8e
#define NRF_KEYREQUESTEVENT         0x8f
#define NRF_DATACREDITEVENT         0x8a
#define NRF_PIPEERROREVENT          0x8d
#define NRF_DATARECEIVEDEVENT       0x8c
#define NRF_DATAACKEVENT            0x8b

#define NRF_TEST_OP                 0x01
#define NRF_SLEEP_OP                0x04
#define NRF_GETDEVICEVERSION_OP     0x09
#define NRF_ECHO_OP                 0x02
#define NRF_WAKEUP_OP               0x05
#define NRF_GETBATTERYLEVEL_OP      0x0b
#define NRF_GETTEMPERATURE_OP       0x0c
#define NRF_SETUP_OP                0x06
#define NRF_SETTXPOWER_OP           0x12
#define NRF_GETDEVICEADDRESS_OP     0x0a
#define NRF_CONNECT_OP              0x0f
#define NRF_RADIORESET_OP           0x0e
#define NRF_BOND_OP                 0x10
#define NRF_DISCONNECT_OP           0x11
#define NRF_CHANGETIMINGREQUEST_OP  0x13
#define NRF_OPENREMOTEPIPE_OP       0x14
#define NRF_CLOSEREMOTEPIPE_OP      0x1f
#define NRF_DTMCOMMAND_OP           0x03
#define NRF_READDYNAMICDATA_OP      0x07
#define NRF_WRITEDYNAMICDATA_OP     0x08
#define NRF_SETAPPLICATIONLATENCY_OP 0x19
#define NRF_SETKEY_OP               0x1a
#define NRF_OPENADVPIPE_OP          0x1b
#define NRF_BROADCAST_OP            0x1c
#define NRF_BONDSECREQUEST_OP       0x1d
#define NRF_DIRECTEDCONNECT_OP      0x1e

// Data commands
#define NRF_SENDDATA_OP             0x15
#define NRF_REQUESTDATA_OP          0x17
#define NRF_SETLOCALDATA_OP         0x0d
#define NRF_SENDDATAACK_OP           0x16
#define NRF_SENDDATANACK_OP         0x18

#define NRF_STATUS_SUCCESS                  0x00
#define NRF_STATUS_TRANSACTION_CONTINUE     0x01
#define NRF_STATUS_TRANSACTION_COMPLETE     0x02
#define NRF_STATUS_EXTENDED                 0x03
#define NRF_STATUS_ERROR_UNKNOWN            0x80
#define NRF_STATUS_ERROR_INTERNAL           0x81
#define NRF_STATUS_ERROR_CMD_UNKNOWN        0x82
#define NRF_STATUS_ERROR_DEVICE_STATE_INVALID 0x83
#define NRF_STATUS_ERROR_INVALID_LENGTH     0x84
#define NRF_STATUS_ERROR_INVALID_PARAMETER  0x85
#define NRF_STATUS_ERROR_BUSY               0x86
#define NRF_STATUS_ERROR_INVALID_DATA       0x87
#define NRF_STATUS_ERROR_CRC_MISMATCH       0x88
#define NRF_STATUS_ERROR_UNSUPPORTED_SETUP_FORMAT 0x89
#define NRF_STATUS_ERROR_INVALID_SEQ_NO           0x8a
#define NRF_STATUS_ERROR_SETUP_LOCKED             0x8b
#define NRF_STATUS_ERROR_LOCK_FAILED        0x8c
#define NRF_STATUS_ERROR_BOND_REQUIRED      0x8d
#define NRF_STATUS_ERROR_REJECTED           0x8e
#define NRF_STATUS_ERROR_DATA_SIZE          0x8f
#define NRF_STATUS_ERROR_PIPE_INVALID       0x90
#define NRF_STATUS_ERROR_CREDIT_NOT_AVAILABLE 0x91
#define NRF_STATUS_ERROR_PEER_ATT_ERROR     0x92
#define NRF_STATUS_ERROR_ADVT_TIMEOUT       0x93
#define NRF_STATUS_ERROR_PEER_SMP_ERROR     0x94
#define NRF_STATUS_ERROR_PIPE_TYPE_INVALID  0x95
#define NRF_STATUS_ERROR_PIPE_STATE_INVALID 0x96
#define NRF_STATUS_ERROR_INVALID_KEY_SIZE   0x97
#define NRF_STATUS_ERROR_INVALID_KEY_DATA   0x98

enum nRFDeviceState {
    Initial,
    PreSetup,
    Setup,
    Standby,
    Active,
    Test,
    Sleep,
    Invalid
};

enum nRFConnectionStatus {
    Disconnected,
    Connected,
    Connecting
};

enum nRFCmd {
    cmdSuccess = 0,
    cmdNotStandby = 1,
    cmdNotConnected = 2,
    cmdSetupError = 3,
    cmdPipeNotOpen = 4,
    cmdInsufficientCredits = 5,
    cmdDataTooLong = 6,
    cmdTimeout = 7
};

enum nRFTxStatus {
    Success             = 0,
    InvalidState        = 1,
    InsufficientCredits = 2,
    Timeout             = 3,
    NotConnected        = 4,
    PipeNotOpen         = 5,
    DataTooLong         = 6,
    InvalidParameter    = 7
};

// internal state
enum nRFSpiState {
    RXONLY,
    RXTX,
    RXWAIT,
    TXWAIT
};

#endif /* _NRF8001_CONSTANTS_H */
