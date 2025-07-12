/*******************************************************************************
    Filename:       hci.h
    Revised:
    Revision:

    Description:    This file contains the Host Controller Interface (HCI) API.
                  It provides the defines, types, and functions for all
                  supported Bluetooth Low Energy (BLE) commands.

                  All Bluetooth and BLE commands are based on:
                  Bluetooth Core Specification, V4.0.0, Vol. 2, Part E.

	SDK_LICENSE

*******************************************************************************/

#ifndef HCI_H
#define HCI_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
    INCLUDES
*/
//#include "bcomdef.h"
//#include "OSAL.h"
//#include "ll.h"
//#include "hal_assert.h"
//#include <types.h>
#include <osal/OSAL.h>
#include "bcomdef.h"

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

/*
** HCI Status
**
** Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
*/
#define HCI_SUCCESS                                                         0x00
#define HCI_ERROR_CODE_UNKNOWN_HCI_CMD                                      0x01
#define HCI_ERROR_CODE_UNKNOWN_CONN_ID                                      0x02
#define HCI_ERROR_CODE_HW_FAILURE                                           0x03
#define HCI_ERROR_CODE_PAGE_TIMEOUT                                         0x04
#define HCI_ERROR_CODE_AUTH_FAILURE                                         0x05
#define HCI_ERROR_CODE_PIN_KEY_MISSING                                      0x06
#define HCI_ERROR_CODE_MEM_CAP_EXCEEDED                                     0x07
#define HCI_ERROR_CODE_CONN_TIMEOUT                                         0x08
#define HCI_ERROR_CODE_CONN_LIMIT_EXCEEDED                                  0x09
#define HCI_ERROR_CODE_SYNCH_CONN_LIMIT_EXCEEDED                            0x0A
#define HCI_ERROR_CODE_ACL_CONN_ALREADY_EXISTS                              0x0B
#define HCI_ERROR_CODE_CMD_DISALLOWED                                       0x0C
#define HCI_ERROR_CODE_CONN_REJ_LIMITED_RESOURCES                           0x0D
#define HCI_ERROR_CODE_CONN_REJECTED_SECURITY_REASONS                       0x0E
#define HCI_ERROR_CODE_CONN_REJECTED_UNACCEPTABLE_BDADDR                    0x0F
#define HCI_ERROR_CODE_CONN_ACCEPT_TIMEOUT_EXCEEDED                         0x10
#define HCI_ERROR_CODE_UNSUPPORTED_FEATURE_PARAM_VALUE                      0x11
#define HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS                               0x12
#define HCI_ERROR_CODE_REMOTE_USER_TERM_CONN                                0x13
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES                0x14
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF                    0x15
#define HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST                              0x16
#define HCI_ERROR_CODE_REPEATED_ATTEMPTS                                    0x17
#define HCI_ERROR_CODE_PAIRING_NOT_ALLOWED                                  0x18
#define HCI_ERROR_CODE_UNKNOWN_LMP_PDU                                      0x19
#define HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE                           0x1A
#define HCI_ERROR_CODE_SCO_OFFSET_REJ                                       0x1B
#define HCI_ERROR_CODE_SCO_INTERVAL_REJ                                     0x1C
#define HCI_ERROR_CODE_SCO_AIR_MODE_REJ                                     0x1D
#define HCI_ERROR_CODE_INVALID_LMP_PARAMS                                   0x1E
#define HCI_ERROR_CODE_UNSPECIFIED_ERROR                                    0x1F
#define HCI_ERROR_CODE_UNSUPPORTED_LMP_PARAM_VAL                            0x20
#define HCI_ERROR_CODE_ROLE_CHANGE_NOT_ALLOWED                              0x21
#define HCI_ERROR_CODE_LMP_LL_RESP_TIMEOUT                                  0x22
#define HCI_ERROR_CODE_LMP_ERR_TRANSACTION_COLLISION                        0x23
#define HCI_ERROR_CODE_LMP_PDU_NOT_ALLOWED                                  0x24
#define HCI_ERROR_CODE_ENCRYPT_MODE_NOT_ACCEPTABLE                          0x25
#define HCI_ERROR_CODE_LINK_KEY_CAN_NOT_BE_CHANGED                          0x26
#define HCI_ERROR_CODE_REQ_QOS_NOT_SUPPORTED                                0x27
#define HCI_ERROR_CODE_INSTANT_PASSED                                       0x28
#define HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                  0x29
#define HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION                      0x2A
#define HCI_ERROR_CODE_RESERVED1                                            0x2B
#define HCI_ERROR_CODE_QOS_UNACCEPTABLE_PARAM                               0x2C
#define HCI_ERROR_CODE_QOS_REJ                                              0x2D
#define HCI_ERROR_CODE_CHAN_ASSESSMENT_NOT_SUPPORTED                        0x2E
#define HCI_ERROR_CODE_INSUFFICIENT_SECURITY                                0x2F
#define HCI_ERROR_CODE_PARAM_OUT_OF_MANDATORY_RANGE                         0x30
#define HCI_ERROR_CODE_RESERVED2                                            0x31
#define HCI_ERROR_CODE_ROLE_SWITCH_PENDING                                  0x32
#define HCI_ERROR_CODE_RESERVED3                                            0x33
#define HCI_ERROR_CODE_RESERVED_SLOT_VIOLATION                              0x34
#define HCI_ERROR_CODE_ROLE_SWITCH_FAILED                                   0x35
#define HCI_ERROR_CODE_EXTENDED_INQUIRY_RESP_TOO_LARGE                      0x36
#define HCI_ERROR_CODE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST                 0x37
#define HCI_ERROR_CODE_HOST_BUSY_PAIRING                                    0x38
#define HCI_ERROR_CODE_CONN_REJ_NO_SUITABLE_CHAN_FOUND                      0x39
#define HCI_ERROR_CODE_CONTROLLER_BUSY                                      0x3A
#define HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL                           0x3B
#define HCI_ERROR_CODE_DIRECTED_ADV_TIMEOUT                                 0x3C
#define HCI_ERROR_CODE_CONN_TERM_MIC_FAILURE                                0x3D
#define HCI_ERROR_CODE_CONN_FAILED_TO_ESTABLISH                             0x3E
#define HCI_ERROR_CODE_MAC_CONN_FAILED                                      0x3F

/*
** Max Buffers Supported
*/
#define HCI_MAX_NUM_DATA_BUFFERS                       LL_MAX_NUM_DATA_BUFFERS
#define HCI_MAX_NUM_CMD_BUFFERS                        LL_MAX_NUM_CMD_BUFFERS

/*
** HCI Command API Parameters
*/

// Send Data Packet Boundary Flags
#define FIRST_PKT_HOST_TO_CTRL                         LL_DATA_FIRST_PKT_HOST_TO_CTRL
#define CONTINUING_PKT                                 LL_DATA_CONTINUATION_PKT
#define FIRST_PKT_CTRL_TO_HOST                         LL_DATA_FIRST_PKT_CTRL_TO_HOST

// Receive Data Packet
#define HCI_RSSI_NOT_AVAILABLE                         LL_RSSI_NOT_AVAILABLE

// Disconnect Reasons
#define HCI_DISCONNECT_AUTH_FAILURE                    HCI_ERROR_CODE_AUTH_FAILURE
#define HCI_DISCONNECT_REMOTE_USER_TERM                HCI_ERROR_CODE_REMOTE_USER_TERM_CONN
#define HCI_DISCONNECT_REMOTE_DEV_LOW_RESOURCES        HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES
#define HCI_DISCONNECT_REMOTE_DEV_POWER_OFF            HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF
#define HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE
#define HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED       HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED
#define HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL      HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL

// Tx Power Types
#define HCI_READ_CURRENT_TX_POWER_LEVEL                LL_READ_CURRENT_TX_POWER_LEVEL
#define HCI_READ_MAX_TX_POWER_LEVEL                    LL_READ_MAX_TX_POWER_LEVEL

// Host Flow Control
#define HCI_CTRL_TO_HOST_FLOW_CTRL_OFF                 0
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF    1
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON    2
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON     3

// Device Address Type
#define HCI_PUBLIC_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_PUBLIC
#define HCI_RANDOM_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_RANDOM

// Advertiser Events
#define HCI_CONNECTABLE_UNDIRECTED_ADV                 LL_ADV_CONNECTABLE_UNDIRECTED_EVT
#define HCI_CONNECTABLE_DIRECTED_HDC_ADV               LL_ADV_CONNECTABLE_DIRECTED_HDC_EVT
#define HCI_SCANNABLE_UNDIRECTED                       LL_ADV_SCANNABLE_UNDIRECTED_EVT
#define HCI_NONCONNECTABLE_UNDIRECTED_ADV              LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
#define HCI_CONNECTABLE_DIRECTED_LDC_ADV               LL_ADV_CONNECTABLE_DIRECTED_LDC_EVT

// Advertiser Channels
#define HCI_ADV_CHAN_37                                LL_ADV_CHAN_37
#define HCI_ADV_CHAN_38                                LL_ADV_CHAN_38
#define HCI_ADV_CHAN_39                                LL_ADV_CHAN_39
#define HCI_ADV_CHAN_ALL                               (LL_ADV_CHAN_37 | LL_ADV_CHAN_38 | LL_ADV_CHAN_39)

// Advertiser White List Policy
#define HCI_ADV_WL_POLICY_ANY_REQ                      LL_ADV_WL_POLICY_ANY_REQ
#define HCI_ADV_WL_POLICY_WL_SCAN_REQ                  LL_ADV_WL_POLICY_WL_SCAN_REQ
#define HCI_ADV_WL_POLICY_WL_CONNECT_REQ               LL_ADV_WL_POLICY_WL_CONNECT_REQ
#define HCI_ADV_WL_POLICY_WL_ALL_REQ                   LL_ADV_WL_POLICY_WL_ALL_REQ

// Advertiser Commands
#define HCI_ENABLE_ADV                                 LL_ADV_MODE_ON
#define HCI_DISABLE_ADV                                LL_ADV_MODE_OFF

// Scan Types
#define HCI_SCAN_PASSIVE                               LL_SCAN_PASSIVE
#define HCI_SCAN_ACTIVE                                LL_SCAN_ACTIVE

// Scan White List Policy
#define HCI_SCAN_WL_POLICY_ANY_ADV_PKTS                LL_SCAN_WL_POLICY_ANY_ADV_PKTS
#define HCI_SCAN_WL_POLICY_USE_WHITE_LIST              LL_SCAN_WL_POLICY_USE_WHITE_LIST

// Scan Filtering
#define HCI_FILTER_REPORTS_DISABLE                     LL_FILTER_REPORTS_DISABLE
#define HCI_FILTER_REPORTS_ENABLE                      LL_FILTER_REPORTS_ENABLE

// Scan Commands
#define HCI_SCAN_STOP                                  LL_SCAN_STOP
#define HCI_SCAN_START                                 LL_SCAN_START

// Initiator White List Policy
#define HCI_INIT_WL_POLICY_USE_PEER_ADDR               LL_INIT_WL_POLICY_USE_PEER_ADDR
#define HCI_INIT_WL_POLICY_USE_WHITE_LIST              LL_INIT_WL_POLICY_USE_WHITE_LIST

// Encryption Related
#define HCI_ENCRYPTION_OFF                             LL_ENCRYPTION_OFF
#define HCI_ENCRYPTION_ON                              LL_ENCRYPTION_ON

// Direct Test Mode
#define HCI_DTM_NUMBER_RF_CHANS                        LL_DIRECT_TEST_NUM_RF_CHANS
#define HCI_DIRECT_TEST_MAX_PAYLOAD_LEN                LL_DIRECT_TEST_MAX_PAYLOAD_LEN
//
#define HCI_DIRECT_TEST_PAYLOAD_PRBS9                  LL_DIRECT_TEST_PAYLOAD_PRBS9
#define HCI_DIRECT_TEST_PAYLOAD_0x0F                   LL_DIRECT_TEST_PAYLOAD_0x0F
#define HCI_DIRECT_TEST_PAYLOAD_0x55                   LL_DIRECT_TEST_PAYLOAD_0x55
#define HCI_DIRECT_TEST_PAYLOAD_PRBS15                 LL_DIRECT_TEST_PAYLOAD_PRBS15
#define HCI_DIRECT_TEST_PAYLOAD_0xFF                   LL_DIRECT_TEST_PAYLOAD_0xFF
#define HCI_DIRECT_TEST_PAYLOAD_0x00                   LL_DIRECT_TEST_PAYLOAD_0x00
#define HCI_DIRECT_TEST_PAYLOAD_0xF0                   LL_DIRECT_TEST_PAYLOAD_0xF0
#define HCI_DIRECT_TEST_PAYLOAD_0xAA                   LL_DIRECT_TEST_PAYLOAD_0xAA

// Vendor Specific
#define HCI_EXT_RX_GAIN_STD                            LL_EXT_RX_GAIN_STD
#define HCI_EXT_RX_GAIN_HIGH                           LL_EXT_RX_GAIN_HIGH
//
#define HCI_EXT_TX_POWER_MINUS_23_DBM                  LL_EXT_TX_POWER_MINUS_23_DBM
#define HCI_EXT_TX_POWER_MINUS_6_DBM                   LL_EXT_TX_POWER_MINUS_6_DBM
#define HCI_EXT_TX_POWER_0_DBM                         LL_EXT_TX_POWER_0_DBM
#define HCI_EXT_TX_POWER_4_DBM                         LL_EXT_TX_POWER_4_DBM
//
#define HCI_EXT_ENABLE_ONE_PKT_PER_EVT                 LL_EXT_ENABLE_ONE_PKT_PER_EVT
#define HCI_EXT_DISABLE_ONE_PKT_PER_EVT                LL_EXT_DISABLE_ONE_PKT_PER_EVT
//
#define HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT              LL_EXT_ENABLE_CLK_DIVIDE_ON_HALT
#define HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT             LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT
//
#define HCI_EXT_NV_IN_USE                              LL_EXT_NV_IN_USE
#define HCI_EXT_NV_NOT_IN_USE                          LL_EXT_NV_NOT_IN_USE
//
#define HCI_EXT_ENABLE_FAST_TX_RESP_TIME               LL_EXT_ENABLE_FAST_TX_RESP_TIME
#define HCI_EXT_DISABLE_FAST_TX_RESP_TIME              LL_EXT_DISABLE_FAST_TX_RESP_TIME
//
#define HCI_EXT_ENABLE_SL_OVERRIDE                     LL_EXT_ENABLE_SL_OVERRIDE
#define HCI_EXT_DISABLE_SL_OVERRIDE                    LL_EXT_DISABLE_SL_OVERRIDE
//
#define HCI_EXT_TX_MODULATED_CARRIER                   LL_EXT_TX_MODULATED_CARRIER
#define HCI_EXT_TX_UNMODULATED_CARRIER                 LL_EXT_TX_UNMODULATED_CARRIER
//
#define HCI_PTM_SET_FREQ_TUNE_DOWN                     LL_EXT_SET_FREQ_TUNE_DOWN
#define HCI_PTM_SET_FREQ_TUNE_UP                       LL_EXT_SET_FREQ_TUNE_UP
//
#define HCI_EXT_PM_IO_PORT_P0                          LL_EXT_PM_IO_PORT_P0
#define HCI_EXT_PM_IO_PORT_P1                          LL_EXT_PM_IO_PORT_P1
#define HCI_EXT_PM_IO_PORT_P2                          LL_EXT_PM_IO_PORT_P2
#define HCI_EXT_PM_IO_PORT_NONE                        LL_EXT_PM_IO_PORT_NONE
//
#define HCI_EXT_PM_IO_PORT_PIN0                        LL_EXT_PM_IO_PORT_PIN0
#define HCI_EXT_PM_IO_PORT_PIN1                        LL_EXT_PM_IO_PORT_PIN1
#define HCI_EXT_PM_IO_PORT_PIN2                        LL_EXT_PM_IO_PORT_PIN2
#define HCI_EXT_PM_IO_PORT_PIN3                        LL_EXT_PM_IO_PORT_PIN3
#define HCI_EXT_PM_IO_PORT_PIN4                        LL_EXT_PM_IO_PORT_PIN4
#define HCI_EXT_PM_IO_PORT_PIN5                        LL_EXT_PM_IO_PORT_PIN5
#define HCI_EXT_PM_IO_PORT_PIN6                        LL_EXT_PM_IO_PORT_PIN6
#define HCI_EXT_PM_IO_PORT_PIN7                        LL_EXT_PM_IO_PORT_PIN7
//
#define HCI_EXT_PER_RESET                              LL_EXT_PER_RESET
#define HCI_EXT_PER_READ                               LL_EXT_PER_READ
//
#define HCI_EXT_HALT_DURING_RF_DISABLE                 LL_EXT_HALT_DURING_RF_DISABLE
#define HCI_EXT_HALT_DURING_RF_ENABLE                  LL_EXT_HALT_DURING_RF_ENABLE
//
#define HCI_EXT_SET_USER_REVISION                      LL_EXT_SET_USER_REVISION
#define HCI_EXT_READ_BUILD_REVISION                    LL_EXT_READ_BUILD_REVISION
//
#define HCI_EXT_RESET_SYSTEM_HARD                      LL_EXT_RESET_SYSTEM_HARD
#define HCI_EXT_RESET_SYSTEM_SOFT                      LL_EXT_RESET_SYSTEM_SOFT
//
#define HCI_EXT_DISABLE_OVERLAPPED_PROCESSING          LL_EXT_DISABLE_OVERLAPPED_PROCESSING
#define HCI_EXT_ENABLE_OVERLAPPED_PROCESSING           LL_EXT_ENABLE_OVERLAPPED_PROCESSING
//
#define HCI_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT        LL_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT
#define HCI_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT         LL_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT

/*
** HCI Event Parameters
*/

// HCI Link Type for Buffer Overflow
#define HCI_LINK_TYPE_SCO_BUFFER_OVERFLOW              0
#define HCI_LINK_TYPE_ACL_BUFFER_OVERFLOW              1

/*******************************************************************************
    TYPEDEFS
*/

typedef uint8_t hciStatus_t;

/*
** LE Events
*/

// LE Connection Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  status;
    uint16_t connectionHandle;
    uint8_t  role;
    uint8_t  peerAddrType;
    uint8_t  peerAddr[B_ADDR_LEN];
    uint16_t connInterval;
    uint16_t connLatency;
    uint16_t connTimeout;
    uint8_t  clockAccuracy;
} hciEvt_BLEConnComplete_t;

// LE Connection Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  status;
    uint16_t connectionHandle;
    uint8_t  role;
    uint8_t  peerAddrType;
    uint8_t  peerAddr[B_ADDR_LEN];
    uint8_t  localRpaAddr[B_ADDR_LEN];
    uint8_t  peerRpaAddr[B_ADDR_LEN];
    uint16_t connInterval;
    uint16_t connLatency;
    uint16_t connTimeout;
    uint8_t  clockAccuracy;
} hciEvt_BLEEnhConnComplete_t;

// LE Advertising Report Event
typedef struct
{
    uint8_t  eventType;                       // advertisment or scan response event type
    uint8_t  addrType;                        // public or random address type
    uint8_t  addr[B_ADDR_LEN];                // device address
    uint8_t  dataLen;                         // length of report data
    uint8_t  rspData[B_MAX_ADV_LEN];          // report data given by dataLen
    int8_t   rssi;                            // report RSSI
} hciEvt_DevInfo_t;

typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  numDevices;
    hciEvt_DevInfo_t* devInfo;              // pointer to the array of devInfo
} hciEvt_BLEAdvPktReport_t;

// LE Connection Update Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  status;
    uint16_t connectionHandle;
    uint16_t connInterval;
    uint16_t connLatency;
    uint16_t connTimeout;
} hciEvt_BLEConnUpdateComplete_t;

// LE Read Remote Used Features Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  status;
    uint16_t connectionHandle;
    uint8_t  features[8];
} hciEvt_BLEReadRemoteFeatureComplete_t;

// LE Encryption Change Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint16_t connHandle;
    uint8_t  reason;
    uint8_t  encEnable;
} hciEvt_EncryptChange_t;

// LE Long Term Key Requested Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint16_t connHandle;
    uint8_t  random[B_RANDOM_NUM_SIZE];
    uint16_t encryptedDiversifier;
} hciEvt_BLELTKReq_t;

// LE DATE LENGTH CHANGE Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint16_t connHandle;
    uint16_t MaxTxOctets;
    uint16_t MaxTxTime;
    uint16_t MaxRxOctets;
    uint16_t MaxRxTime;
} hciEvt_BLEDataLenChange_t;

// LE PHY UPDATE Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  status;
    uint16_t connHandle;
    uint8_t  txPhy;
    uint8_t  rxPhy;
} hciEvt_BLEPhyUpdateComplete_t;

// LE PHY UPDATE Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
} hciEvt_BLEEvent_Hdr_t;

// Number of Completed Packets Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  numHandles;
    uint16_t* pConnectionHandle;              // pointer to the connection handle array
    uint16_t* pNumCompletedPackets;           // pointer to the number of completed packets array
} hciEvt_NumCompletedPkt_t;

// Command Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  numHciCmdPkt;                    // number of HCI Command Packet
    uint16_t cmdOpcode;
    uint8_t*  pReturnParam;                    // pointer to the return parameter
} hciEvt_CmdComplete_t;

// Command Status Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  cmdStatus;
    uint8_t  numHciCmdPkt;
    uint16_t cmdOpcode;
} hciEvt_CommandStatus_t;

// Hardware Error Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t hardwareCode;
} hciEvt_HardwareError_t;

// Disconnection Complete Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  status;
    uint16_t connHandle;                      // connection handle
    uint8_t  reason;
} hciEvt_DisconnComplete_t;

// Data Buffer Overflow Event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t linkType;                         // synchronous or asynchronous buffer overflow
} hciEvt_BufferOverflow_t;

// Data structure for HCI Command Complete Event Return Parameter
typedef struct
{
    uint8_t  status;
    uint16_t dataPktLen;
    uint8_t  numDataPkts;
} hciRetParam_LeReadBufSize_t;

typedef struct
{
    uint16_t  eventType;                       // advertisment or scan response event type
    uint8_t   addrType;                        // public or random address type
    uint8_t   addr[B_ADDR_LEN];                // device address
    uint8_t   primaryPHY;
    uint8_t   secondaryPHY;
    uint8_t   advertisingSID;
    uint8_t   txPower;
    int8_t    rssi;                            // report RSSI
    uint16_t  periodicAdvertisingInterval;
    uint8_t   directAddrType;
    uint8_t   directAddr[B_ADDR_LEN];
    uint8_t   dataLen;                         // length of report data
    uint8_t   rptData[B_MAX_EXT_ADV_LEN];          // report data given by dataLen
} hciEvt_ExtAdvRptInfo_t;

// Extended adv report
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint8_t  numReports;
    hciEvt_ExtAdvRptInfo_t* rptInfo;              // pointer to the array of devInfo
} hciEvt_BLEExtAdvPktReport_t;


typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t             BLEEventCode;
    uint8_t             status;
    uint16_t            syncHandle;
    uint8_t             advertisingSID;
    uint8_t             advertiserAddressType;
    uint8_t             advertiserAddress[B_ADDR_LEN];
    uint8_t             advertiserPHY;
    uint16_t            periodicAdvertisingInterval;
    uint8_t             advertiserClockAccuracy;
} hciEvt_BLEPrdAdvSyncEstabPkt_t;

typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t             BLEEventCode;
    uint16_t            syncHandle;
} hciEvt_BLEPrdAdvSyncLostPkt_t;

// 2020-4-22 LE Advertising Set Terminated event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t             BLEEventCode;
    uint8_t             status;
    uint8_t             adv_handle;
    uint16_t            connHandle;                      // connection handle
    uint8_t             Num_Completed_Extended_Advertising_Events;
} hciEvt_AdvSetTerminated_t;

// 2020-4-22 LE Channel Selection Algorithm event
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    uint16_t connHandle;                      // connection handle
    uint8_t  chn_sel;
} hciEvt_ChannelSelAlgo_t;

// 2020-01-14 LE IQ report event structure
typedef struct
{
    uint16_t              Handle;     // syncHandle for connectionless handle , connection for connection Handle
    uint8_t               chan_idx;
    int16_t               rssi;
    uint8_t               rssi_antID;
    uint8_t               cte_type;
    uint8_t               slot_duration;
    uint8_t               packet_status;
    uint16_t              EventCnt;   // paEventcounter or connEventCounter
    uint8_t               sampCnt;
    uint8_t               ISample[B_MAX_IQ_LEN];
    uint8_t               QSample[B_MAX_IQ_LEN];
} hciEvt_IQReportPkt_t;

// 2020-01-14 LE Connectionless IQ report event structure
typedef struct
{
    osal_event_hdr_t    hdr;
    uint8_t               BLEEventCode;
    hciEvt_IQReportPkt_t    ConnectionlessIQ;
} hciEvt_BLEConnectionlessIQ_Pkt_t;

// 2020-01-14 LE Connection IQ report event structure
typedef struct
{
    osal_event_hdr_t    hdr;
    uint8_t               BLEEventCode;
    uint8_t               RX_PHY;
    hciEvt_IQReportPkt_t    ConnectionIQ;
} hciEvt_BLEConnectionIQ_Pkt_t;

// 2020-01-14 LE Connection IQ report event structure
typedef struct
{
    osal_event_hdr_t    hdr;
    uint8_t               BLEEventCode;
    uint8_t               status;
    uint16_t              connHandle;
} hciEvt_BLE_CTEReport_Pkt_t;


typedef struct
{
    uint16_t syncHandle;
    uint8_t txPower;
    uint8_t rssi;
    uint8_t cteType;
    uint8_t dataStatus;
    uint8_t dataLength;
    uint8_t data[B_MAX_PERIOD_ADV_LEN];
} hciEvt_PrdAdvRptInfo_t;



// Periodic adv report
typedef struct
{
    osal_event_hdr_t  hdr;
    uint8_t  BLEEventCode;
    hciEvt_PrdAdvRptInfo_t* rptInfo;              // pointer to the array of devInfo
} hciEvt_BLEPrdAdvPktReport_t;

typedef struct
{
    osal_event_hdr_t hdr;
    uint8_t*            pData;
} hciPacket_t;

typedef struct
{
    osal_event_hdr_t hdr;
    uint8_t  pktType;
    uint16_t connHandle;
    uint8_t  pbFlag;
    uint16_t pktLen;
    uint8_t*  pData;
} hciDataPacket_t;

// OSAL HCI_DATA_EVENT message format. This message is used to forward incoming
// data messages up to an application
typedef struct
{
    osal_event_hdr_t hdr;                   // OSAL event header
    uint16_t connHandle;                      // connection handle
    uint8_t  pbFlag;                          // data packet boundary flag
    uint16_t len;                             // length of data packet
    uint8_t*  pData;                          // data packet given by len
} hciDataEvent_t;




/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/

/*
** HCI Support Functions
*/

/*******************************************************************************
    @fn          HCI_bm_alloc API

    @brief       This API is used to allocate memory using buffer management.

                Note: This function should never be called by the application.
                      It is only used by HCI and L2CAP_bm_alloc.

    input parameters

    @param       size - Number of bytes to allocate from the heap.

    output parameters

    @param       None.

    @return      Pointer to buffer, or NULL.
*/
extern void* HCI_bm_alloc( uint16_t size );


/*******************************************************************************
    @fn          HCI_ValidConnTimeParams API

    @brief       This API is used to check that the connection time parameter
                ranges are valid, and that the connection time parameter
                combination is valid.

                Note: Only connIntervalMax is used as part of the time parameter
                      combination check.

    input parameters

    @param       connIntervalMin - Minimum connection interval.
    @param       connIntervalMax - Maximum connection interval.
    @param       connLatency     - Connection slave latency.
    @param       connTimeout     - Connection supervision timeout.

    output parameters

    @param       None.

    @return      TRUE:  Connection time parameter check is valid.
                FALSE: Connection time parameter check is invalid.
*/
extern uint8_t HCI_ValidConnTimeParams( uint16_t connIntervalMin,
                                      uint16_t connIntervalMax,
                                      uint16_t connLatency,
                                      uint16_t connTimeout );


/*******************************************************************************
    @fn          HCI_TestAppTaskRegister

    @brief       HCI vendor specific registration for HCI Test Application.

    input parameters

    @param       taskID - The HCI Test Application OSAL task identifer.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_TestAppTaskRegister( uint8_t taskID );


/*******************************************************************************
    @fn          HCI_GAPTaskRegister

    @brief       HCI vendor specific registration for Host GAP.

    input parameters

    @param       taskID - The Host GAP OSAL task identifer.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_GAPTaskRegister( uint8_t taskID );


/*******************************************************************************

    @fn          HCI_L2CAPTaskRegister

    @brief       HCI vendor specific registration for Host L2CAP.

    input parameters

    @param       taskID - The Host L2CAP OSAL task identifer.

    output parameters

    @param       None.

    @return      None.

*/
extern void HCI_L2CAPTaskRegister( uint8_t taskID );


/*******************************************************************************
    @fn          HCI_SMPTaskRegister

    @brief       HCI vendor specific registration for Host SMP.

    input parameters

    @param       taskID - The Host SMP OSAL task identifer.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_SMPTaskRegister( uint8_t taskID );


/*******************************************************************************
    @fn          HCI_ExtTaskRegister

    @brief       HCI vendor specific registration for Host extended commands.

    input parameters

    @param       taskID - The Host Extended Command OSAL task identifer.

    output parameters

    @param       None.

    @return      None.
*/
extern void HCI_ExtTaskRegister( uint8_t taskID );


/*******************************************************************************
    @fn          HCI_SendDataPkt API

    @brief       This API is used to send a ACL data packet over a connection.

                Note: Empty packets are not sent.

                Related Events: HCI_NumOfCompletedPacketsEvent

    input parameters

    @param       connHandle - Connection ID (handle).
    @param       pbFlag     - Packet Boundary Flag.
    @param       pktLen     - Number of bytes of data to transmit.
    @param       *pData     - Pointer to data buffer to transmit.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_SendDataPkt( uint16_t connHandle,
                                    uint8_t  pbFlag,
                                    uint16_t pktLen,
                                    uint8_t*  pData );



/*
** HCI API
*/

/*******************************************************************************
    @fn          HCI_DisconnectCmd API

    @brief       This BT API is used to terminate a connection.

                Related Events: HCI_CommandStatusEvent,
                                DisconnectEvent

    input parameters

    @param       connHandle - Connection handle.
    @param       reason     - Reason for disconnection:
                             HCI_DISCONNECT_AUTH_FAILURE,
                             HCI_DISCONNECT_REMOTE_USER_TERM,
                             HCI_DISCONNECT_REMOTE_DEV_POWER_OFF,
                             HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE,
                             HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED
                             HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_DisconnectCmd( uint16_t connHandle,
                                      uint8_t  reason );


/*******************************************************************************
    @fn          HCI_ReadRemoteVersionInfoCmd API

    @brief       This BT API is used to request version information from the
                the remote device in a connection.

                Related Events: HCI_CommandStatusEvent,
                                ReadRemoteVersionInfoEvent

    input parameters

    @param       connHandle - Connection handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadRemoteVersionInfoCmd( uint16_t connHandle );



/*******************************************************************************
    @fn          HCI_SetEventMaskCmd API

    @brief       This BT API is used to set the HCI event mask, which is used to
                determine which events are supported.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       pMask - Pointer to an eight byte event mask.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_SetEventMaskCmd( uint8_t* pMask );


/*******************************************************************************
    @fn          HCI_Reset API

    @brief       This BT API is used to reset the Link Layer.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ResetCmd( void );



/*******************************************************************************
    @fn          HCI_ReadTransmitPowerLevelCmd API

    @brief       This BT API is used to read the transmit power level.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       connHandle - Connection handle.
    @param       txPwrType  - HCI_READ_CURRENT_TX_POWER_LEVEL,
                             HCI_READ_MAXIMUM_TX_POWER_LEVEL

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadTransmitPowerLevelCmd( uint16_t connHandle,
                                                  uint8_t  txPwrType );


/*******************************************************************************
    @fn          HCI_SetControllerToHostFlowCtrlCmd API

    @brief       This BT API is used by the Host to turn flow control on or off
                for data sent from the Controller to Host.

                Note: This command is currently not supported.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       flowControlEnable - HCI_CTRL_TO_HOST_FLOW_CTRL_OFF,
                                    HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF,
                                    HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON,
                                    HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_SetControllerToHostFlowCtrlCmd( uint8_t flowControlEnable );


/*******************************************************************************
    @fn          HCI_HostBufferSizeCmd API

    @brief       This BT API is used by the Host to notify the Controller of the
                maximum size ACL buffer size the Controller can send to the
                Host.

                Note: This command is currently ignored by the Controller. It
                      is assumed that the Host can always handle the maximum
                      BLE data packet size.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       hostAclPktLen        - Host ACL data packet length.
    @param       hostSyncPktLen       - Host SCO data packet length .
    @param       hostTotalNumAclPkts  - Host total number of ACL data packets.
    @param       hostTotalNumSyncPkts - Host total number of SCO data packets.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_HostBufferSizeCmd( uint16_t hostAclPktLen,
                                          uint8_t  hostSyncPktLen,
                                          uint16_t hostTotalNumAclPkts,
                                          uint16_t hostTotalNumSyncPkts );


/*******************************************************************************
    @fn          HCI_HostNumCompletedPktCmd API

    @brief       This BT API is used by the Host to notify the Controller of the
                number of HCI data packets that have been completed for each
                connection handle since this command was previously sent to the
                controller.

                The Host_Number_Of_Conpleted_Packets command is a special
                command. No event is normally generated after the command
                has completed. The command should only be issued by the
                Host if flow control in the direction from controller to
                the host is on and there is at least one connection, or
                if the controller is in local loopback mode.

                Note: It is assumed that there will be at most only one handle.
                      Even if more than one handle is provided, the Controller
                      does not track Host buffers as a function of connection
                      handles (and isn't required to do so).

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       numHandles       - Number of connection handles.
    @param       connHandles      - Array of connection handles.
    @param       numCompletedPkts - Array of number of completed packets.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_HostNumCompletedPktCmd( uint8_t  numHandles,
                                               uint16_t* connHandles,
                                               uint16_t* numCompletedPkts );


/*******************************************************************************
    @fn          HCI_ReadLocalVersionInfoCmd API

    @brief       This BT API is used to read the local version information.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadLocalVersionInfoCmd( void );


/*******************************************************************************
    @fn          HCI_ReadLocalSupportedCommandsCmd API

    @brief       This BT API is used to read the locally supported commands.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadLocalSupportedCommandsCmd( void );


/*******************************************************************************
    @fn          HCI_ReadLocalSupportedFeaturesCmd API

    @brief       This BT API is used to read the locally supported features.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
    @fn          HCI_ReadBDADDRCmd API

    @brief       This BT API is used to read this device's BLE address (BDADDR).

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadBDADDRCmd( void );


/*******************************************************************************
    @fn          HCI_ReadRssiCmd API

    @brief       This BT API is used to read the RSSI of the last packet
                received on a connection given by the connection handle. If
                the Receiver Modem test is running (HCI_EXT_ModemTestRx), then
                the RF RSSI for the last received data will be returned. If
                there is no RSSI value, then HCI_RSSI_NOT_AVAILABLE will be
                returned.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       connHandle - Connection handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_ReadRssiCmd( uint16_t connHandle );

/*
** HCI Low Energy Commands
*/

/*******************************************************************************
    @fn          HCI_LE_SetEventMaskCmd API

    @brief       This LE API is used to set the HCI LE event mask, which is used
                to determine which LE events are supported.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param        pEventMask - Pointer to LE event mask of 8 bytes.


    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetEventMaskCmd( uint8_t* pEventMask );


/*******************************************************************************
    @fn          HCI_LE_ReadBufSizeCmd API

    @brief       This LE API is used by the Host to determine the maximum ACL
                data packet size allowed by the Controller.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadBufSizeCmd( void );


/*******************************************************************************
    @fn          HCI_LE_ReadLocalSupportedFeaturesCmd API

    @brief       This LE API is used to read the LE locally supported features.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
    @fn          HCI_LE_SetRandomAddressCmd API

    @brief       This LE API is used to set this device's Random address.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       pRandAddr - Pointer to random address.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetRandomAddressCmd( uint8_t* pRandAddr );



/*******************************************************************************
    @fn          HCI_LE_SetAdvParamCmd API

    @brief       This LE API is used to set the Advertising parameters.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       advIntervalMin  - Minimum allowed advertising interval.
    @param       advIntervalMax  - Maximum allowed advertising interval.
    @param       advType         - HCI_CONNECTABLE_UNDIRECTED_ADV,
                                  HCI_CONNECTABLE_DIRECTED_HDC_ADV,
                                  HCI_SCANNABLE_UNDIRECTED,
                                  HCI_NONCONNECTABLE_UNDIRECTED_ADV
                                  HCI_CONNECTABLE_DIRECTED_LDC_ADV
    @param       ownAddrType     - HCI_PUBLIC_DEVICE_ADDRESS,
                                  HCI_RANDOM_DEVICE_ADDRESS
    @param       directAddrType  - HCI_PUBLIC_DEVICE_ADDRESS,
                                  HCI_RANDOM_DEVICE_ADDRESS
    @param       directAddr      - Pointer to address of device when using
                                  directed advertising.
    @param       advChannelMap   - HCI_ADV_CHAN_37,
                                  HCI_ADV_CHAN_38,
                                  HCI_ADV_CHAN_39,
                                  HCI_ADV_CHAN_37 | HCI_ADV_CHAN_38,
                                  HCI_ADV_CHAN_37 | HCI_ADV_CHAN_39,
                                  HCI_ADV_CHAN_38 | HCI_ADV_CHAN_39,
                                  HCI_ADV_CHAN_ALL
    @param       advFilterPolicy - HCI_ADV_WL_POLICY_ANY_REQ,
                                  HCI_ADV_WL_POLICY_WL_SCAN_REQ,
                                  HCI_ADV_WL_POLICY_WL_CONNECT_REQ,
                                  HCI_ADV_WL_POLICY_WL_ALL_REQ

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetAdvParamCmd( uint16_t advIntervalMin,
                                          uint16_t advIntervalMax,
                                          uint8_t  advType,
                                          uint8_t  ownAddrType,
                                          uint8_t  directAddrType,
                                          uint8_t*  directAddr,
                                          uint8_t  advChannelMap,
                                          uint8_t  advFilterPolicy );


/*******************************************************************************
    @fn          HCI_LE_SetAdvDataCmd API

    @brief       This LE API is used to set the Advertising data.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       dataLen - Length of Advertising data.
    @param       pData   - Pointer to Advertising data.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetAdvDataCmd( uint8_t dataLen,
                                         uint8_t* pData );


/*******************************************************************************
    @fn          HCI_LE_SetScanRspDataCmd API

    @brief       This LE API is used to set the Advertising Scan Response data.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       dataLen - Length of Scan Response data.
    @param       pData   - Pointer to Scan Response data.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetScanRspDataCmd( uint8_t dataLen,
                                             uint8_t* pData );


/*******************************************************************************
    @fn          HCI_LE_SetAdvEnableCmd API

    @brief       This LE API is used to turn Advertising on or off.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       advEnable - HCI_ENABLE_ADV, HCI_DISABLE_ADV

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetAdvEnableCmd( uint8_t advEnable );


/*******************************************************************************
    @fn          HCI_LE_ReadAdvChanTxPowerCmd API

    @brief       This LE API is used to read transmit power when Advertising.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadAdvChanTxPowerCmd( void );


/*******************************************************************************
    @fn          HCI_LE_SetScanParamCmd API

    @brief       This LE API is used to set the Scan parameters.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       scanType     - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE
    @param       scanInterval - Time between scan events.
    @param       scanWindow   - Time of scan before scan event ends.
                               Note: When the scanWindow equals the scanInterval
                                     then scanning is continuous.
    @param       ownAddrType  - This device's address.
    @param       filterPolicy - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetScanParamCmd( uint8_t  scanType,
                                           uint16_t scanInterval,
                                           uint16_t scanWindow,
                                           uint8_t  ownAddrType,
                                           uint8_t  filterPolicy );


/*******************************************************************************
    @fn          HCI_LE_SetScanEnableCmd API

    @brief       This LE API is used to turn Scanning on or off.

                Related Events: HCI_CommandCompleteEvent,
                                AdvReportEvent

    input parameters

    @param       scanEnable       - HCI_SCAN_START, HCI_SCAN_STOP
    @param       filterDuplicates - HCI_FILTER_REPORTS_ENABLE,
                                   HCI_FILTER_REPORTS_DISABLE

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetScanEnableCmd( uint8_t scanEnable,
                                            uint8_t filterDuplicates );


/*******************************************************************************
    @fn          HCI_LE_CreateConnCmd API

    @brief       This LE API is used to create a connection.

                Related Events: HCI_CommandStatusEvent,
                                ConnectionCompleteEvent

    input parameters

    @param       scanInterval     - Time between Init scan events.
    @param       scanWindow       - Time of scan before Init scan event ends.
                                   Note: When the scanWindow equals the
                                         scanInterval then scanning is
                                         continuous.
    @param       initFilterPolicy - HCI_INIT_WL_POLICY_USE_PEER_ADDR,
                                   HCI_INIT_WL_POLICY_USE_WHITE_LIST
    @param       addrTypePeer     - HCI_PUBLIC_DEVICE_ADDRESS,
                                   HCI_RANDOM_DEVICE_ADDRESS
    @param       peerAddr         - Pointer to peer device's address.
    @param       ownAddrType      - HCI_PUBLIC_DEVICE_ADDRESS,
                                   HCI_RANDOM_DEVICE_ADDRESS
    @param       connIntervalMin  - Minimum allowed connection interval.
    @param       connIntervalMax  - Maximum allowed connection interval.
    @param       connLatency      - Number of skipped events (slave latency).
    @param       connTimeout      - Connection supervision timeout.
    @param       minLen           - Info parameter about min length of conn.
    @param       maxLen           - Info parameter about max length of conn.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_CreateConnCmd( uint16_t scanInterval,
                                         uint16_t scanWindow,
                                         uint8_t  initFilterPolicy,
                                         uint8_t  addrTypePeer,
                                         uint8_t*  peerAddr,
                                         uint8_t  ownAddrType,
                                         uint16_t connIntervalMin,
                                         uint16_t connIntervalMax,
                                         uint16_t connLatency,
                                         uint16_t connTimeout,
                                         uint16_t minLen,
                                         uint16_t maxLen );


/*******************************************************************************
    @fn          HCI_LE_CreateConnCancelCmd API

    @brief       This LE API is used to cancel a create connection.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_CreateConnCancelCmd( void );


/*******************************************************************************
    @fn          HCI_LE_ReadWhiteListSizeCmd API

    @brief       This LE API is used to read the white list.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadWhiteListSizeCmd( void );


/*******************************************************************************
    @fn          HCI_LE_ClearWhiteListCmd API

    @brief       This LE API is used to clear the white list.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ClearWhiteListCmd( void );


/*******************************************************************************
    @fn          HCI_LE_AddWhiteListCmd API

    @brief       This LE API is used to add a white list entry.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       addrType - HCI_PUBLIC_DEVICE_ADDRESS, HCI_RANDOM_DEVICE_ADDRESS
    @param       devAddr  - Pointer to address of device to put in white list.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_AddWhiteListCmd( uint8_t addrType,
                                           uint8_t* devAddr );


/*******************************************************************************
    @fn          HCI_LE_RemoveWhiteListCmd API

    @brief       This LE API is used to remove a white list entry.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       addrType - HCI_PUBLIC_DEVICE_ADDRESS, HCI_RANDOM_DEVICE_ADDRESS
    @param       devAddr  - Pointer to address of device to remove from the
                           white list.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_RemoveWhiteListCmd( uint8_t addrType,
                                              uint8_t* devAddr );



/*******************************************************************************
    @fn          HCI_LE_ConnUpdateCmd API

    @brief       This LE API is used to update the connection parameters.

                Related Events: HCI_CommandStatusEvent,
                                ConnectionUpdateCompleteEvent

    input parameters

    @param       connHandle       - Time between Init scan events.
    @param       connIntervalMin  - Minimum allowed connection interval.
    @param       connIntervalMax  - Maximum allowed connection interval.
    @param       connLatency      - Number of skipped events (slave latency).
    @param       connTimeout      - Connection supervision timeout.
    @param       minLen           - Info parameter about min length of conn.
    @param       maxLen           - Info parameter about max length of conn.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ConnUpdateCmd( uint16_t connHandle,
                                         uint16_t connIntervalMin,
                                         uint16_t connIntervalMax,
                                         uint16_t connLatency,
                                         uint16_t connTimeout,
                                         uint16_t minLen,
                                         uint16_t maxLen );


/*******************************************************************************
    @fn          HCI_LE_SetHostChanClassificationCmd API

    @brief       This LE API is used to update the current data channel map.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       chanMap - Pointer to the new channel map.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_SetHostChanClassificationCmd( uint8_t* chanMap );


/*******************************************************************************
    @fn          HCI_LE_ReadChannelMapCmd API

    @brief       This LE API is used to read a connection's data channel map.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       connHandle - Connection handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadChannelMapCmd( uint16_t connHandle );

/*******************************************************************************
    @fn          HCI_LE_ReadRemoteUsedFeaturesCmd API

    @brief       This LE API is used to read the remote device's used features.

                Related Events: HCI_CommandStatusEvent,
                                ReadRemoteUsedFeaturesCompleteEvent

    input parameters

    @param       connHandle - Connection handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadRemoteUsedFeaturesCmd( uint16_t connHandle );


/*******************************************************************************
    @fn          HCI_LE_EncryptCmd API

    @brief       This LE API is used to perform an encryption using AES128.

                Note: Input parameters are ordered MSB..LSB.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       key       - Pointer to 16 byte encryption key.
    @param       plainText - Pointer to 16 byte plaintext data.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_EncryptCmd( uint8_t* key,
                                      uint8_t* plainText );


/*******************************************************************************
    @fn          HCI_LE_RandCmd API

    @brief       This LE API is used to generate a random number.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_RandCmd( void );



/*******************************************************************************
    @fn          HCI_LE_StartEncyptCmd API

    @brief       This LE API is used to start encryption in a connection.

                Related Events: HCI_CommandStatusEvent,
                                EncChangeEvent or
                                EncKeyRefreshEvent

    input parameters

    @param       connHandle - Connection handle.
    @param       random     - Pointer to eight byte Random number.
    @param       encDiv     - Pointer to two byte Encrypted Diversifier.
    @param       ltk        - Pointer to 16 byte Long Term Key.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_StartEncyptCmd( uint16_t connHandle,
                                          uint8_t*  random,
                                          uint8_t*  encDiv,
                                          uint8_t*  ltk );

/*******************************************************************************
    @fn          HCI_LE_LtkReqReplyCmd API

    @brief       This LE API is used by the Host to send to the Controller a
                positive LTK reply.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       connHandle - Connection handle.
    @param       ltk        - Pointer to 16 byte Long Term Key.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_LtkReqReplyCmd( uint16_t connHandle,
                                          uint8_t*  ltk );

/*******************************************************************************
    @fn          HCI_LE_LtkReqNegReplyCmd API

    @brief       This LE API is used by the Host to send to the Controller a
                negative LTK reply.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       connHandle - Connectin handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_LtkReqNegReplyCmd( uint16_t connHandle );


/*******************************************************************************
    @fn          HCI_LE_ReadSupportedStatesCmd API

    @brief       This LE API is used to read the Controller's supported states.

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReadSupportedStatesCmd( void );


/*******************************************************************************
    @fn          HCI_LE_ReceiverTestCmd API

    @brief       This LE API is used to start the receiver Direct Test Mode test.

                Note: A HCI reset should be issued when done using DTM!

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       rxFreq - Rx RF frequency:
                         k=0..HCI_DTM_NUMBER_RF_CHANS-1, where: F=2402+(k*2MHz)

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_ReceiverTestCmd( uint8_t rxFreq );


/*******************************************************************************
    @fn          HCI_LE_TransmitterTestCmd API

    @brief       This LE API is used to start the transmit Direct Test Mode test.

                Note: The BLE device is to transmit at maximum power!

                Note: A HCI reset should be issued when done using DTM!

    input parameters

    @param       txFreq      - Tx RF frequency:
                              k=0..HCI_DTM_NUMBER_RF_CHANS-1, where:
                              F=2402+(k*2MHz)
    @param       dataLen     - Test data length in bytes:
                              0..HCI_DIRECT_TEST_MAX_PAYLOAD_LEN
    @param       payloadType - Type of packet payload, per Direct Test Mode spec:
                              HCI_DIRECT_TEST_PAYLOAD_PRBS9,
                              HCI_DIRECT_TEST_PAYLOAD_0x0F,
                              HCI_DIRECT_TEST_PAYLOAD_0x55,
                              HCI_DIRECT_TEST_PAYLOAD_PRBS15,
                              HCI_DIRECT_TEST_PAYLOAD_0xFF,
                              HCI_DIRECT_TEST_PAYLOAD_0x00,
                              HCI_DIRECT_TEST_PAYLOAD_0xF0,
                              HCI_DIRECT_TEST_PAYLOAD_0xAA

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_TransmitterTestCmd( uint8_t txFreq,
                                              uint8_t dataLen,
                                              uint8_t pktPayload );


/*******************************************************************************
    @fn          HCI_LE_TestEndCmd API

    @brief       This LE API is used to end the Direct Test Mode test.

                Note: A HCI reset should be issued when done using DTM!

                Related Events: HCI_CommandCompleteEvent

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_LE_TestEndCmd( void );


// BBB ROM code add
extern hciStatus_t HCI_LE_AddDevToResolvingListCmd( uint8_t  addrType,
                                                    uint8_t* devAddr,
                                                    uint8_t* peerIrk,
                                                    uint8_t* localIrk);


extern hciStatus_t HCI_LE_RemoveResolvingListCmd( uint8_t addrType,
                                                  uint8_t* devAddr );

extern hciStatus_t HCI_LE_ClearResolvingListCmd( void );

extern hciStatus_t HCI_LE_ReadResolvingListSizeCmd( void );

extern hciStatus_t HCI_LE_SetAddressResolutionEnableCmd( uint8_t enable );

extern hciStatus_t HCI_LE_SetResolvablePrivateAddressTimeoutCmd( uint16_t rpaTimeout );


/*
** HCI for Extended Adv
*/
//
extern hciStatus_t HCI_LE_SetExtAdvSetRandomAddressCmd( uint8_t adv_handle,
                                                        uint8_t* random_address);

extern hciStatus_t HCI_LE_SetExtAdvParamCmd( uint8_t adv_handle,
                                             uint16_t adv_event_properties,
                                             uint32_t primary_advertising_interval_Min,          // 3 octets
                                             uint32_t primary_advertising_interval_Max,          // 3 octets
                                             uint8_t  primary_advertising_channel_map,
                                             uint8_t  own_address_type,
                                             uint8_t  peer_address_type,
                                             uint8_t* peer_address,
                                             uint8_t  advertising_filter_policy,
                                             int8_t   advertising_tx_power,                      // update 2020-04-08
                                             uint8_t  primary_advertising_PHY,
                                             uint8_t  secondary_advertising_max_skip,
                                             uint8_t  secondary_advertising_PHY,
                                             uint8_t  advertising_SID,
                                             uint8_t  scan_request_notification_enable
                                           );

//
extern hciStatus_t HCI_LE_SetExtAdvDataCmd( uint8_t adv_handle,
                                            uint8_t operation,
                                            uint8_t  fragment_preference,
                                            uint8_t  advertising_data_length,
                                            uint8_t* advertising_data
                                          );
//
extern hciStatus_t HCI_LE_SetExtScanRspDataCmd( uint8_t adv_handle,
                                                uint8_t operation,
                                                uint8_t  fragment_preference,
                                                uint8_t  scan_rsp_data_length,
                                                uint8_t* scan_rsp_data
                                              );

//
extern hciStatus_t HCI_LE_SetExtAdvEnableCmd( uint8_t  enable,
                                              uint8_t  number_of_sets,
                                              uint8_t*  advertising_handle,
                                              uint16_t* duration,
                                              uint8_t*  max_extended_advertising_events);

//
extern hciStatus_t HCI_LE_ReadMaximumAdvDataLengthCmd( void );

//
extern hciStatus_t HCI_LE_ReadNumberOfSupportAdvSetCmd( void );

//
extern hciStatus_t HCI_LE_RemoveAdvSetCmd( uint8_t adv_handle);

//
extern hciStatus_t HCI_LE_ClearAdvSetsCmd( void);


extern hciStatus_t HCI_LE_SetExtendedScanParametersCmd(uint8_t own_address_type,
                                                       uint8_t scanning_filter_policy,
                                                       uint8_t scanning_PHYs,
                                                       uint8_t* scan_sype,
                                                       uint16_t* scan_interval,
                                                       uint16_t* scan_window);

extern hciStatus_t HCI_LE_SetExtendedScanEnableCmd(uint8_t enable,
                                                   uint8_t filter_duplicates,
                                                   uint16_t duration,
                                                   uint16_t period);

extern hciStatus_t HCI_LE_ExtendedCreateConnectionCmd(uint8_t initiator_filter_policy,
                                                      uint8_t own_address_type,
                                                      uint8_t peer_address_type,
                                                      uint8_t* peer_address,
                                                      uint8_t initiating_PHYs,
                                                      uint16_t* scan_interval,
                                                      uint16_t* scan_window,
                                                      uint16_t* conn_interval_min,
                                                      uint16_t* conn_interval_max,
                                                      uint16_t* conn_latency,
                                                      uint16_t* supervision_timeout,
                                                      uint16_t* minimum_CE_length,
                                                      uint16_t* maximum_CE_length);

//
extern hciStatus_t HCI_LE_SetPeriodicAdvParameterCmd( uint8_t adv_handle,
                                                      uint16_t   interval_min,
                                                      uint16_t   interval_max,
                                                      uint16_t   adv_event_properties
                                                    );

extern hciStatus_t HCI_LE_SetPeriodicAdvDataCmd( uint8_t adv_handle,
                                                 uint8_t operation,
                                                 uint8_t  advertising_data_length,
                                                 uint8_t* advertising_data
                                               );

extern hciStatus_t HCI_LE_SetPeriodicAdvEnableCmd( uint8_t  enable,
                                                   uint8_t  advertising_handle);


extern hciStatus_t HCI_LE_PeriodicAdvertisingCreateSyncCmd(uint8_t Options,
                                                           uint8_t Advertising_SID,
                                                           uint8_t Advertiser_Address_Type,
                                                           uint8_t* Advertiser_Address,
                                                           uint16_t Skip,
                                                           uint16_t Sync_Timeout,
                                                           uint8_t Sync_CTE_Type);

extern hciStatus_t HCI_LE_PeriodicAdvertisingCreateSyncCancelCmd(void);

extern hciStatus_t HCI_LE_PeriodicAdvertisingTerminateSyncCmd(uint16_t sync_handle);

extern hciStatus_t HCI_LE_AddDevToPeriodicAdvListCmd( uint8_t  addrType,
                                                      uint8_t* devAddr,
                                                      uint8_t sid);
extern hciStatus_t HCI_LE_RemovePeriodicAdvListCmd( uint8_t  addrType,
                                                    uint8_t* devAddr,
                                                    uint8_t sid);
extern hciStatus_t HCI_LE_ClearPeriodicAdvListCmd( void );
extern hciStatus_t HCI_LE_ReadPeriodicAdvListSizeCmd( void );


/******************************************************************************
    fn:  HCI_LE_ConnectionlessCTE_TransmitParamcmd

    brief:   set CTE Parameters in any periodic advertising
                1、CTE Type
                2、CTE Length
                3、CTE antenna switching pattern

    input parameters:
            advertising handle      : Identify advertising set 0x0-0xEF
            CTE_Length              : CTE Length in 8us 0x2-0x14
            CTE_Type                : 0:AOA CTE , 1:AoD CTE with 1us,2:AoD CTE with 2us,
            CTE_Count               : how many CTE packet in each PA event 0x1-0x10
            Switch_Pattern_LEN      : number of Antenna IDs in the pattern
                                    : AOD CTE, AOA shall be ignored
                                    : 0x2-0x4B
            Antenna_IDs[i]          : List of Antenna IDs in the pattern
                                    : AOD CTE, AOA shall be ignored

    output parameters:
                Status              :HCI_SUCCESS or other error codes


    return       hciStatus_t         : HCI_SUCCESS

 ******************************************************************************/
hciStatus_t HCI_LE_ConnectionlessCTE_TransmitParamCmd(              uint8_t advertising_handle,
                                                                    uint8_t len,
                                                                    uint8_t type,
                                                                    uint8_t count,
                                                                    uint8_t Pattern_LEN,
                                                                    uint8_t* AnaIDs);


/******************************************************************************
    fn:  HCI_LE_ConnectionlessCTE_TransmitEnableCmd

    brief:   Controller enable or disable CTE in PA

    input parameters:
            advertising handle      : Identify advertising set in which CTE is enable or disable
                                    : 0x0-0xEF
            enable                  : 0 : disable , 1: enable


    output parameters:
                Status              :HCI_SUCCESS or other error codes


    return       hciStatus_t         : HCI_SUCCESS or other error codes

 ******************************************************************************/
hciStatus_t HCI_LE_ConnectionlessCTE_TransmitEnableCmd(                 uint8_t advertising_handle,
                                                                        uint8_t enable);



/******************************************************************************
    fn:  HCI_LE_ConnectionlessIQ_SampleEnableCmd

    brief:   Controller enable or disable capturing IQ Samples from the CTE of PA pcakets

    input parameters:
                sync_handle     :   periodic advertising handle
                                    Range:0x0 - 0x0EFF
                slot_Duration   :   switching and sampling slot 0x1:1us,0x2:2us,Other:RFU
                enable          :   0x0:IQ Sampling disable, 0x1:IQ Sampling enable
                MaxSampledCTEs  :   max number of CTE in each PA event that the controller
                                    should collect and report
                                    Range   :   0x0-0x10
                                        0x0 :   sample and report all available CTE
                pattern_len     :   number of Antenna IDs in the pattern
                                    Range:0x2 - 0x4B
                AnaIDs          :   list of Antenna IDs in the pattern


    output parameters:
                status          :   HCI_SUCCESS or other error codes
                sync_handle     :   Periodic advertising handle


    return       hciStatus_t     : HCI_SUCCESS

 ******************************************************************************/
hciStatus_t HCI_LE_ConnectionlessIQ_SampleEnableCmd(                uint16_t sync_handle,
                                                                    uint8_t enable,
                                                                    uint8_t slot_Duration,
                                                                    uint8_t MaxSampledCTEs,
                                                                    uint8_t pattern_len,
                                                                    uint8_t* AnaIDs);


/******************************************************************************
    fn:  HCI_LE_ConnectionCTE_ReceiveParamCmd

    brief:   enable or disable sampling received CTE fields on the connection
            set antenna switching pattern
            set switching and sampling slot durations

    input parameters:
            connHandle  :   connection handle Range 0x0 - 0x0EFF
            enable      :   sampling enable 0:disable , 1:enable
            slot_Duration   : switching and sampling slot 0:1us, 1: 2us
            pattern_len :   the number of Antenna IDs in the pattern
                            Range: 0x2-0x4B
            AnaIDs      :   list of Antenna IDs in the pattern


    output parameters:
                Status      :   HCI_SUCCESS or other error codes
                connHandle  :   Connection Handle


    return       hciStatus_t

 ******************************************************************************/
hciStatus_t HCI_LE_Set_ConnectionCTE_ReceiveParamCmd(               uint16_t connHandle,
                                                                    uint8_t enable,
                                                                    uint8_t slot_Duration,
                                                                    uint8_t pattern_len,
                                                                    uint8_t* AnaIDs);



/******************************************************************************
    fn:  HCI_LE_Set_ConnectionCTE_TransmitParamCmd

    brief:   used to set the antenna switching pattern and permitted CTE type

    input parameters:
            connHandle  :   connection Handle, Range: 0x0 - 0x0EFF
            type        :   bit set for CTE type ,  bit 0 : AOA CTE response,
                                                    bit 1 : AOD CTE response with 1us slots
                                                    bit 2 : AOD CTE response with 2us slots
            pattern_len :   the number of Antenna IDs in the pattern
            AnaIDs      :   list of Antenna IDs in the pattern


    output parameters:
            Status      :   0 : success, other error code
            ConnHandle  :   connection handle


    return       hciStatus_t

 ******************************************************************************/
hciStatus_t HCI_LE_Set_ConnectionCTE_TransmitParamCmd(                  uint16_t connHandle,
                                                                        uint8_t type,
                                                                        uint8_t pattern_len,
                                                                        uint8_t* AnaIDs);




/******************************************************************************
    fn:  HCI_LE_Connection_CTE_Request_EnableCmd

    brief:   request Controller to start or stop initiating the CTE request
            procedure on connection

    input parameters:
            connHandle  :   connection Handle
                            Range:0x0 - 0x0EFF
            enable      :   Enable or disable CTE request for the connection
                            0:disable,1:enable
            Interval    :   define whether the CTE request procedure is initiated
                            only once or periodically.
                            Range:0x0 - 0xFFFF
                            0x0 :   Initiate the CTE request procedure once
                            0x1 - 0xFFFF :  Requested interval for initiating the CTE
                                            procedure in number of connection events
                            Range:
            len         :   minimum length of the CTE in 8us units
                            Range: 0x2 - 0x14
            type        :   indicate the type of CTE that the controller shall
                            request from the remote device
                            0x0:AOA CTE
                            0x1:AOD CTE with 1us
                            0x2:AOD CTE with 2us


    output parameters:
            Status      :   0x0 : command succeed , 0x1 - 0xff : other error code
            connHandle  :   connection handle


    return       hciStatus_t

 ******************************************************************************/
hciStatus_t HCI_LE_Connection_CTE_Request_EnableCmd(              uint16_t connHandle,
                                                                  uint8_t enable,
                                                                  uint16_t Interval,
                                                                  uint8_t len,
                                                                  uint8_t type);


/******************************************************************************
    fn:  HCI_LE_Connection_CTE_Response_EnableCmd

    brief:   request the controller to respond to LL_CTE_REQ with LL_CTE_RSP on the
            specified connection

    input parameters:
            connHandle  :   connection Handle
                            Range:0x0 - 0x0EFF
            enable      :   enable or disable CTE response for the connection


    output parameters:
            status      :   0x0 : command succeed , 0x1 - 0xff : other error code
            connHandle  :   connection handle



    return       hciStatus_t

 ******************************************************************************/
hciStatus_t HCI_LE_Connection_CTE_Response_EnableCmd(               uint16_t connHandle,
                                                                    uint8_t enable);


/******************************************************************************
    fn:  HCI_LE_READ_Anatenna_InfoCmd

    brief:   Host read the switching rates, the sampling reate, the number of antennae,
            and the maxumum length of a transmitted CTE supported by the controller

    input parameters:
                None


    output parameters:
                status              :   0x0 : command succeed , 0x1 - 0xff : other error code
                switch_sample_rate  :   bit number indicate supported switching and sampling rate
                                        bit 0 :  1us switching AOD transmission
                                        bit 1 :  1us sampling AOD reception
                                        bit 2 :  1us switching and sampling AOA reception
                Antenna_len         :   number of Antennae supported by the controller
                MAX_Pattern_len     :   MAX length of antenna switching pattern spooorted by the controller
                MAX_CTE_LEN         :   MAX length or a transmitted CTE supported in 8us units


    return       hciStatus_t

 ******************************************************************************/
hciStatus_t HCI_LE_READ_Anatenna_InfoCmd(void);

#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
/*
** HCI Vendor Specific Comamnds: Link Layer Extensions
*/

/*******************************************************************************
    @fn          HCI_EXT_SetRxGainCmd API

    @brief       This HCI Extension API is used to set the receiver gain.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       rxGain - HCI_EXT_RX_GAIN_STD, HCI_EXT_RX_GAIN_HIGH

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetRxGainCmd( uint8_t rxGain );


/*******************************************************************************
    @fn          HCI_EXT_SetTxPowerCmd API

    @brief       This HCI Extension API is used to set the transmit power.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       txPower - LL_EXT_TX_POWER_MINUS_23_DBM,
                          LL_EXT_TX_POWER_MINUS_6_DBM,
                          LL_EXT_TX_POWER_0_DBM,
                          LL_EXT_TX_POWER_4_DBM

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetTxPowerCmd( uint8_t txPower );


/*******************************************************************************
    @fn          HCI_EXT_OnePktPerEvtCmd API

    @brief       This HCI Extension API is used to set whether a connection will
                be limited to one packet per event.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       control - HCI_EXT_ENABLE_ONE_PKT_PER_EVT,
                          HCI_EXT_DISABLE_ONE_PKT_PER_EVT

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_OnePktPerEvtCmd( uint8_t control );


/*******************************************************************************
    @fn          HCI_EXT_ClkDivOnHaltCmd API

    @brief       This HCI Extension API is used to set whether the system clock
                will be divided when the MCU is halted.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       control - HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT,
                          HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ClkDivOnHaltCmd( uint8_t control );


/*******************************************************************************
    @fn          HCI_EXT_DeclareNvUsageCmd API

    @brief       This HCI Extension API is used to indicate to the Controller
                whether or not the Host will be using the NV memory during BLE
                operations.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       mode - HCI_EXT_NV_IN_USE, HCI_EXT_NV_NOT_IN_USE

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_DeclareNvUsageCmd( uint8_t mode );


/*******************************************************************************
    @fn          HCI_EXT_DecryptCmd API

    @brief       This HCI Extension API is used to decrypt encrypted data using
                AES128.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       key     - Pointer to 16 byte encryption key.
    @param       encText - Pointer to 16 byte encrypted data.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_DecryptCmd( uint8_t* key,
                                       uint8_t* encText );


/*******************************************************************************
    @fn          HCI_EXT_SetLocalSupportedFeaturesCmd API

    @brief       This HCI Extension API is used to write this devie's supported
                features.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       localFeatures - Pointer to eight bytes of local features.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetLocalSupportedFeaturesCmd( uint8_t* localFeatures );


/*******************************************************************************
    @fn          HCI_EXT_SetFastTxResponseTimeCmd API

    @brief       This HCI Extension API is used to set whether transmit data is
                sent as soon as possible even when slave latency is used.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       control - HCI_EXT_ENABLE_FAST_TX_RESP_TIME,
                          HCI_EXT_DISABLE_FAST_TX_RESP_TIME

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetFastTxResponseTimeCmd( uint8_t control );

/*******************************************************************************
    @fn          HCI_EXT_SetSlaveLatencyOverrideCmd API

    @brief       This HCI Extension API is used to to enable or disable
                suspending slave latency.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       control - HCI_EXT_ENABLE_SL_OVERRIDE,
                          HCI_EXT_DISABLE_SL_OVERRIDE

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetSlaveLatencyOverrideCmd( uint8_t control );


/*******************************************************************************
    @fn          HCI_EXT_ModemTestTxCmd

    @brief       This API is used start a continuous transmitter modem test,
                using either a modulated or unmodulated carrier wave tone, at
                the frequency that corresponds to the specified RF channel. Use
                HCI_EXT_EndModemTest command to end the test.

                Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
                Note: The BLE device will transmit at maximum power.
                Note: This API can be used to verify this device meets Japan's
                      TELEC regulations.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       cwMode - HCI_EXT_TX_MODULATED_CARRIER,
                         HCI_EXT_TX_UNMODULATED_CARRIER
                txFreq - Transmit RF channel k=0..39, where BLE F=2402+(k*2MHz).

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ModemTestTxCmd( uint8_t cwMode,
                                           uint8_t txFreq );


/*******************************************************************************
    @fn          HCI_EXT_ModemHopTestTxCmd

    @brief       This API is used to start a continuous transmitter direct test
                mode test using a modulated carrier wave and transmitting a
                37 byte packet of Pseudo-Random 9-bit data. A packet is
                transmitted on a different frequency (linearly stepping through
                all RF channels 0..39) every 625us. Use HCI_EXT_EndModemTest
                command to end the test.

                Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
                Note: The BLE device will transmit at maximum power.
                Note: This API can be used to verify this device meets Japan's
                      TELEC regulations.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ModemHopTestTxCmd( void );


/*******************************************************************************
    @fn          HCI_EXT_ModemTestRxCmd

    @brief       This API is used to start a continuous receiver modem test
                using a modulated carrier wave tone, at the frequency that
                corresponds to the specific RF channel. Any received data is
                discarded. Receiver gain may be adjusted using the
                HCI_EXT_SetRxGain command. RSSI may be read during this test
                by using the HCI_ReadRssi command. Use HCI_EXT_EndModemTest
                command to end the test.

                Note: A Controller reset will be issued by HCI_EXT_EndModemTest!
                Note: The BLE device will transmit at maximum power.

    input parameters

    @param       rxFreq - Receiver RF channel k=0..39, where BLE F=2402+(k*2MHz).

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ModemTestRxCmd( uint8_t rxFreq );


/*******************************************************************************
    @fn          HCI_EXT_EndModemTestCmd

    @brief       This API is used to shutdown a modem test. A complete Controller
                reset will take place.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_EndModemTestCmd( void );


/*******************************************************************************
    @fn          HCI_EXT_SetBDADDRCmd

    @brief       This API is used to set this device's BLE address (BDADDR).

                Note: This command is only allowed when the device's state is
                      Standby.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       bdAddr  - A pointer to a buffer to hold this device's address.
                          An invalid address (i.e. all FF's) will restore this
                          device's address to the address set at initialization.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetBDADDRCmd( uint8_t* bdAddr );


/*******************************************************************************
    @fn          HCI_EXT_SetSCACmd

    @brief       This API is used to set this device's Sleep Clock Accuracy.

                Note: For a slave device, this value is directly used, but only
                      if power management is enabled. For a master device, this
                      value is converted into one of eight ordinal values
                      representing a SCA range, as specified in Table 2.2,
                      Vol. 6, Part B, Section 2.3.3.1 of the Core specification.

                Note: This command is only allowed when the device is not in a
                      connection.

                Note: The device's SCA value remains unaffected by a HCI_Reset.

    input parameters

    @param       scaInPPM - A SCA value in PPM from 0..500.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetSCACmd( uint16_t scaInPPM );


/*******************************************************************************
    @fn          HCI_EXT_EnablePTMCmd

    @brief       This HCI Extension API is used to enable Production Test Mode.

                Note: This function can only be directly called from the
                      application and is not available via an external transport
                      interface such as RS232. Also, no vendor specific
                      command complete will be returned.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_EnablePTMCmd( void );


/*******************************************************************************
    @fn          HCI_EXT_SetFreqTuneCmd

    @brief       This HCI Extension API is used to set the frequency tuning up
                or down. Setting the mode up/down decreases/increases the amount
                of capacitance on the external crystal oscillator.

                Note: This is a Production Test Mode only command!

    input parameters

    @param       step - HCI_PTM_SET_FREQ_TUNE_UP, HCI_PTM_SET_FREQ_TUNE_DOWN

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetFreqTuneCmd( uint8_t step );


/*******************************************************************************
    @fn          HCI_EXT_SaveFreqTuneCmd

    @brief       This HCI Extension API is used to save the frequency tuning
                value to flash.

                Note: This is a Production Test Mode only command!

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SaveFreqTuneCmd( void );


/*******************************************************************************
    @fn          HCI_EXT_SetMaxDtmTxPowerCmd API

    @brief       This HCI Extension API is used to set the maximum transmit
                output power for Direct Test Mode.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       txPower - LL_EXT_TX_POWER_MINUS_23_DBM,
                          LL_EXT_TX_POWER_MINUS_6_DBM,
                          LL_EXT_TX_POWER_0_DBM,
                          LL_EXT_TX_POWER_4_DBM

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_SetMaxDtmTxPowerCmd( uint8_t txPower );


/*******************************************************************************

*/
extern llStatus_t HCI_EXT_MapPmIoPortCmd( uint8_t ioPort, uint8_t ioPin );


/*******************************************************************************
    @fn          HCI_EXT_DisconnectImmedCmd API

    @brief       This HCI Extension API is used to disconnect the connection
                immediately.

                Note: The connection (if valid) is immediately terminated
                      without notifying the remote device. The Host is still
                      notified.

    input parameters

    @param       connHandle - Connection handle.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_DisconnectImmedCmd( uint16_t connHandle );


/*******************************************************************************
    @fn          HCI_EXT_PacketErrorRate Vendor Specific API

    @brief       This function is used to Reset or Read the Packet Error Rate
                counters for a connection.

                Note: The counters are only 16 bits. At the shortest connection
                      interval, this provides a bit over 8 minutes of data.

    input parameters

    @param       connHandle - The LL connection ID on which to send this data.
    @param       command    - HCI_EXT_PER_RESET, HCI_EXT_PER_READ

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_PacketErrorRateCmd( uint16_t connHandle, uint8_t command );



/*******************************************************************************
    @fn          HCI_EXT_PERbyChanCmd Vendor Specific API

    @brief       This HCI Extension API is used to start or end Packet Error Rate
                by Channel counter accumulation for a connection. If the
                pointer is not NULL, it is assumed there is sufficient memory
                for the PER data, per the type perByChan_t. If NULL, then
                the operation is considered disabled.

                Note: It is the user's responsibility to make sure there is
                      sufficient memory for the data, and that the counters
                      are cleared prior to first use.

                Note: The counters are only 16 bits. At the shortest connection
                      interval, this provides a bit over 8 minutes of data.

    input parameters

    @param       connHandle - The LL connection ID on which to send this data.
    @param       perByChan  - Pointer to PER by Channel data, or NULL.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_PERbyChanCmd( uint16_t connHandle, perByChan_t* perByChan );


/*******************************************************************************
*/
extern hciStatus_t HCI_EXT_ExtendRfRangeCmd( void );


/*******************************************************************************
    @fn          HCI_EXT_HaltDuringRfCmd API

    @brief       This HCI Extension API is used to enable or disable halting the
                CPU during RF. The system defaults to enabled.

                Related Events: HCI_VendorSpecifcCommandCompleteEvent

    input parameters

    @param       mode - HCI_EXT_HALT_DURING_RF_ENABLE,
                       HCI_EXT_HALT_DURING_RF_DISABLE

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_HaltDuringRfCmd( uint8_t mode );


/*******************************************************************************
    @fn          HCI_EXT_AdvEventNoticeCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable a
                notification to the specified task using the specified task
                event whenever a Adv event ends. A non-zero taskEvent value is
                taken to be "enable", while a zero valued taskEvent is taken
                to be "disable".

    input parameters

    @param       taskID    - User's task ID.
    @param       taskEvent - User's task event.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_AdvEventNoticeCmd( uint8_t taskID, uint16_t taskEvent );


/*******************************************************************************
    @fn          HCI_EXT_ConnEventNoticeCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable a
                notification to the specified task using the specified task
                event whenever a Connection event ends. A non-zero taskEvent
                value is taken to be "enable", while a zero valued taskEvent
                taken to be "disable".

                Note: Currently, only a Slave connection is supported.

    input parameters

    @param       taskID    - User's task ID.
    @param       taskEvent - User's task event.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ConnEventNoticeCmd( uint8_t taskID, uint16_t taskEvent );


/*******************************************************************************
    @fn          HCI_EXT_BuildRevisionCmd Vendor Specific API

    @brief       This HCI Extension API is used set a user revision number or
                read the build revision number.

    input parameters

    @param       mode - HCI_EXT_SET_USER_REVISION | HCI_EXT_READ_BUILD_REVISION

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_BuildRevisionCmd( uint8_t mode, uint16_t userRevNum );


/*******************************************************************************
    @fn          HCI_EXT_DelaySleepCmd Vendor Specific API

    @brief       This HCI Extension API is used set the sleep delay.

    input parameters

    @param       delay - 0..1000, in milliseconds.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_DelaySleepCmd( uint16_t delay );


/*******************************************************************************
    @fn          HCI_EXT_ResetSystemCmd Vendor Specific API

    @brief       This HCI Extension API is used to issue a soft or hard
                system reset.

    input parameters

    @param       mode - HCI_EXT_RESET_SYSTEM_HARD | HCI_EXT_RESET_SYSTEM_SOFT

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_ResetSystemCmd( uint8_t mode );



/*******************************************************************************
    @fn          HCI_EXT_OverlappedProcessingCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable overlapped
                processing.

    input parameters

    @param       mode - HCI_EXT_ENABLE_OVERLAPPED_PROCESSING |
                       HCI_EXT_DISABLE_OVERLAPPED_PROCESSING

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_OverlappedProcessingCmd( uint8_t mode );

/*******************************************************************************
    @fn          HCI_EXT_NumComplPktsLimitCmd Vendor Specific API

    @brief       This HCI Extension API is used to set the minimum number of
                completed packets which must be met before a Number of
                Completed Packets event is returned. If the limit is not
                reach by the end of the connection event, then a Number of
                Completed Packets event will be returned (if non-zero) based
                on the flushOnEvt flag.

    input parameters

    @param       limit      - From 1 to HCI_MAX_NUM_DATA_BUFFERS.
    @param       flushOnEvt - HCI_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT |
                             HCI_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT

    output parameters

    @param       None.

    @return      hciStatus_t
*/
extern hciStatus_t HCI_EXT_NumComplPktsLimitCmd( uint8_t limit,
                                                 uint8_t flushOnEvt );


/*******************************************************************************
    @fn          HCI_PPLUS_AdvEventDoneNoticeCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable a notification to the
                specified task using the specified task event whenever a Adv event ends.
                A non-zero taskEvent value is taken to be "enable", while a zero valued
                taskEvent is taken to be "disable".

    input parameters

    @param       taskID    - User's task ID.
    @param       taskEvent - User's task event.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t HCI_PPLUS_AdvEventDoneNoticeCmd( uint8_t taskID, uint16_t taskEvent );

/*******************************************************************************
    @fn          HCI_PPLUS_ConnEventDoneNoticeCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable a notification to the
                specified task using the specified task event whenever a Connection event
                ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
                taskEvent is taken to be "disable".

    input parameters

    @param       taskID    - User's task ID.
    @param       taskEvent - User's task event.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t HCI_PPLUS_ConnEventDoneNoticeCmd( uint8_t taskID, uint16_t taskEvent );

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_DateLengthChangedNoticeCmd( uint8_t taskID, uint16_t taskEvent );

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_PhyUpdateNoticeCmd( uint8_t taskID, uint16_t taskEvent );


/*******************************************************************************
    @fn          HCI_PPLUS_ExtendTRXCmd Vendor Specific API

    @brief       This HCI Extension API is used to enable or disable Tx/Rx packets limit
                per connection event to 8(default is 4).

    input parameters

    @param       enable    - TRUE: 8Tx/8Rx;  FALSE: 4Tx/4Rx

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t HCI_PPLUS_ExtendTRXCmd( uint8_t enable );

#endif /*#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)*/

/*******************************************************************************
*/
hciStatus_t HCI_LE_SetDataLengthCmd( uint16_t connHandle,
                                     uint16_t TxOctets,
                                     uint16_t TxTime );


/*******************************************************************************
*/
hciStatus_t HCI_LE_ReadMaxDataLengthCmd(void);

/*******************************************************************************
    This LE API is used to read Suggested Default max Data length

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadSuggestedDefaultDataLengthCmd(void);

/*******************************************************************************
    This LE API is used to write Suggested Default Data length

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_WriteSuggestedDefaultDataLengthCmd(uint16_t suggestedMaxTxOctets,uint16_t suggestedMaxTxTime);


/*******************************************************************************
    This LE API is used to set DefaultPhyMode

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetDefaultPhyMode( uint16_t connId,uint8_t allPhy,uint8_t txPhy, uint8_t rxPhy);


/*******************************************************************************
    This LE API is used to Set PHY Mode

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetPhyMode( uint16_t connId,uint8_t allPhy,uint8_t txPhy, uint8_t rxPhy,uint16_t phyOptions);

/*******************************************************************************
    This LE API is used to Read PHY Mode

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadPhyMode( uint16_t connId);



#ifdef __cplusplus
}
#endif

#endif /* HCI_H */
