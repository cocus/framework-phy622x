/*******************************************************************************
  Filename:       ll.h
  Revised:         
  Revision:        

  Description:    This file contains the Link Layer (LL) API for the Bluetooth
                  Low Energy (BLE) Controller. It provides the defines, types,
                  and functions for all supported Bluetooth Low Energy (BLE)
                  commands.

                  This API is based on the Bluetooth Core Specification,
                  V4.0.0, Vol. 6.

  SDK_LICENSE
   
*******************************************************************************/

#ifndef LL_H
#define LL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */

#include "ll_def.h"
#include "ll_sleep.h"

/*******************************************************************************
 * MACROS
 */

// check if connection parameter ranges for CI (min/max), SL, and LSTO are valid
#define LL_INVALID_CONN_TIME_PARAM( ciMin, ciMax, sl, lsto )                   \
  (((ciMin) < LL_CONN_INTERVAL_MIN) ||                                         \
   ((ciMin) > LL_CONN_INTERVAL_MAX) ||                                         \
   ((ciMax) < LL_CONN_INTERVAL_MIN) ||                                         \
   ((ciMax) > LL_CONN_INTERVAL_MAX) ||                                         \
   ((ciMax) < (ciMin))              ||                                         \
   ((sl)    > LL_SLAVE_LATENCY_MAX) ||                                         \
   ((lsto)  < LL_CONN_TIMEOUT_MIN)  ||                                         \
   ((lsto)  > LL_CONN_TIMEOUT_MAX))

// check if the CI/SL/LSTO combination is valid
// based on: LSTO > (1 + Slave Latency) * (Connection Interval * 2)
// Note: The CI * 2 requirement based on ESR05 V1.0, Erratum 3904.
// Note: LSTO time is normalized to units of 1.25ms (i.e. 10ms = 8 * 1.25ms).
#define LL_INVALID_CONN_TIME_PARAM_COMBO( ci, sl, lsto )                       \
  ((uint32_t)((lsto)*8) <= ((uint32_t)(1+(sl)) * (uint32_t)((ci)*2)))

#define   LL_TIME_DELTA(T1, T2)   ((T2 >= T1) ? (T2 - T1) : (BASE_TIME_UNITS - T1 + T2))


/*******************************************************************************
 * CONSTANTS
 */

/*
** LL API Status Codes
**
** Note: These status values map directly to the HCI Error Codes.
**       Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
*/
#define LL_STATUS_SUCCESS                              0x00 // Success
#define LL_STATUS_ERROR_UNKNOWN_CONN_HANDLE            0x02 // Unknown Connection Identifier
#define LL_STATUS_ERROR_INACTIVE_CONNECTION            0x02 // Unknown Connection Identifier for now; may be needed for multiple connections
#define LL_STATUS_ERROR_AUTH_FAILURE                   0x05 // Authentication Failure
#define LL_STATUS_ERROR_PIN_OR_KEY_MISSING             0x06 // Pin or Key Missing
#define LL_STATUS_ERROR_OUT_OF_CONN_RESOURCES          0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_TX_MEM                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_RX_MEM                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_HEAP                    0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_WL_TABLE_FULL                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_RL_TABLE_FULL                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_PAL_TABLE_FULL                 0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_TX_DATA_QUEUE_FULL             0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_TX_DATA_QUEUE_EMPTY            0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_CONNECTION_TIMEOUT             0x08 // Connection Timeout
#define LL_STATUS_ERROR_CONNECTION_LIMIT_EXCEEDED      0x09 // Connection Limit Exceeded
#define LL_STATUS_ERROR_COMMAND_DISALLOWED             0x0C // Command Disallowed
#define LL_STATUS_ERROR_DUE_TO_LIMITED_RESOURCES       0x0D // Command Rejected Due To Limited Resources
#define LL_STATUS_ERROR_DUE_TO_DELAYED_RESOURCES       0x0D // Command Delayed Due To Limited Resources
#define LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED          0x11 // Unsupported Feature or Parameter Value
#define LL_STATUS_ERROR_UNEXPECTED_PARAMETER           0x12 // Invalid HCI Command Parameters
#define LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION      0x12 // Invalid HCI Command Parameters
#define LL_STATUS_ERROR_BAD_PARAMETER                  0x12 // Invalid HCI Command Parameters or 0x30: Parameter Out of Mandatory Range?
#define LL_STATUS_ERROR_UNKNOWN_ADV_EVT_TYPE           0x12 // Invalid HCI Command Parameters or 0x30: Parameter Out of Mandatory Range?
#define LL_STATUS_ERROR_PEER_TERM                      0x13 // Remote User Terminated Connection
#define LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES 0x14 // Remote Device Terminated Connection Due To Low Resources
#define LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF     0x15 // Remote Device Terminated Connection Due To Power Off
#define LL_STATUS_ERROR_HOST_TERM                      0x16 // Connection Terminated By Local Host
#define LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE     0x1A // Unsupported Remote Feature

// 2020-01-23 add error Code
#define LL_STATUS_ERROR_INVALID_LMP_LL_PARAMETER	   0x1E

#define LL_STATUS_ERROR_WL_ENTRY_NOT_FOUND             0x1F // Unspecified Error
#define LL_STATUS_ERROR_WL_TABLE_EMPTY                 0x1F // Unspecified Error
#define LL_STATUS_ERROR_RL_ENTRY_NOT_FOUND             0x1F // Unspecified Error
#define LL_STATUS_ERROR_RL_TABLE_EMPTY                 0x1F // Unspecified Error
#define LL_STATUS_ERROR_RNG_FAILURE                    0x1F // Unspecified Error
#define LL_STATUS_ERROR_DISCONNECT_IMMEDIATE           0x1F // Unspecified Error
#define LL_STATUS_ERROR_DATA_PACKET_QUEUED             0x1F // Unspecified Error

// 2020-01-23 add error Code
#define LL_STATUS_ERROR_UNSUPPORT_LMP_LL_PARAMETER	   0x20

#define LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE          0x21 // Role Change Not Allowed
#define LL_STATUS_ERROR_LL_TIMEOUT                     0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_TIMEOUT_HOST                0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_TIMEOUT_PEER                0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_PROCEDURE_COLLISION         0x23 // Link Layer Procedure collision


#define LL_STATUS_ERROR_INSTANT_PASSED                 0x28 // Instant Passed
#define LL_STATUS_ERROR_INSTANT_PASSED_HOST            0x28 // Instant Passed
#define LL_STATUS_ERROR_INSTANT_PASSED_PEER            0x28 // Instant Passed
#define LL_STATUS_ERROR_KEY_PAIRING_NOT_SUPPORTED      0x29 // Pairing With Unit Key Not Supported
#define LL_STATUS_ERROR_DIFF_TRANSACTION_COLLISION     0x2A // Link Layer collision in same procedure


#define LL_STATUS_ERROR_NO_ADV_CHAN_FOUND              0x30 // Parameter Out Of Mandatory Range
#define LL_STATUS_ERROR_PARAM_OUT_OF_RANGE             0x30 // Parameter Out Of Mandatory Range
#define LL_STATUS_ERROR_UPDATE_CTRL_PROC_PENDING       0x3A // Controller Busy
#define LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE       0x3A // Controller Busy
#define LL_STATUS_ERROR_VER_INFO_REQ_ALREADY_PENDING   0x3A // Controller Busy
#define LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL     0x3B // Unacceptable Connection Interval
#define LL_STATUS_ERROR_DIRECTED_ADV_TIMEOUT           0x3C // Directed Advertising Timeout
#define LL_STATUS_ERROR_CONN_TERM_DUE_TO_MIC_FAILURE   0x3D // Connection Terminated Due To MIC Failure
#define LL_STATUS_ERROR_CONN_FAILED_TO_BE_ESTABLISHED  0x3E // Connection Failed To Be Established
#define LL_STATUS_ERROR_SYNC_FAILED_TO_BE_ESTABLISHED  0x3E // Connection Failed To Be Established
#define LL_STATUS_ERROR_CONN_TIMING_FAILURE            0x3F // MAC Connection Failed
#define LL_STATUS_ERROR_COARSE_CLK_ADJ_REJECT          0x40 // Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging
#define LL_STATUS_ERROR_TYPE0_SUBMAP_NOT_DEF           0x41 // Type0 Submap Not Defined
#define LL_STATUS_ERROR_UNKNOWN_ADV_ID                 0x42 // Unknown Advertising Identifier
#define LL_STATUS_ERROR_LIMIT_REACHED                  0x43 // Limit Reached
#define LL_STATUS_ERROR_OP_CANCEL_BY_HOST              0x44 // Operation Cancelled by Host
#define LL_STATUS_ERROR_PACKET_TOO_LONG                0x45 // Packet Too Long

// Internal
#define LL_STATUS_DISABLE_LATENCY_INACTIVE_CONN        0x81
#define LL_STATUS_DISABLE_LATENCY_DISABLED             0x82
#define LL_STATUS_DISABLE_LATENCY_PENDING              0x83
#define LL_STATUS_DISABLE_LATENCY_MISS_EVT             0x84

#define LL_STATUS_DISABLE_LATENCY_FAIL                 0x8F 
#define LL_STATUS_WARNING_WAITING_LLIRQ                0xFE // only used internally, so value doesn't matter

#define LL_STATUS_WARNING_TX_DISABLED                  0xFF // only used internally, so value doesn't matter
#define LL_STATUS_WARNING_FLAG_UNCHANGED               0xFF // only used internally, so value doesn't matter

// Encryption Key Request Reason Codes
#define LL_ENC_KEY_REQ_ACCEPTED                        LL_STATUS_SUCCESS
#define LL_ENC_KEY_REQ_REJECTED                        LL_STATUS_ERROR_PIN_OR_KEY_MISSING
#define LL_ENC_KEY_REQ_UNSUPPORTED_FEATURE             LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE

// Disconnect Reason Codes
#define LL_SUPERVISION_TIMEOUT_TERM                    LL_STATUS_ERROR_CONNECTION_TIMEOUT
#define LL_PEER_REQUESTED_TERM                         LL_STATUS_ERROR_PEER_TERM
#define LL_PEER_REQUESTED_LOW_RESOURCES_TERM           LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES
#define LL_PEER_REQUESTED_POWER_OFF_TERM               LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF
#define LL_HOST_REQUESTED_TERM                         LL_STATUS_ERROR_HOST_TERM
#define LL_CTRL_PKT_TIMEOUT_TERM                       LL_STATUS_ERROR_LL_TIMEOUT
#define LL_CTRL_PKT_TIMEOUT_HOST_TERM                  LL_STATUS_ERROR_LL_TIMEOUT_HOST
#define LL_CTRL_PKT_TIMEOUT_PEER_TERM                  LL_STATUS_ERROR_LL_TIMEOUT_PEER
#define LL_CTRL_PKT_INSTANT_PASSED_TERM                LL_STATUS_ERROR_INSTANT_PASSED
#define LL_CTRL_PKT_INSTANT_PASSED_HOST_TERM           LL_STATUS_ERROR_INSTANT_PASSED_HOST
#define LL_CTRL_PKT_INSTANT_PASSED_PEER_TERM           LL_STATUS_ERROR_INSTANT_PASSED_PEER
#define LL_UNACCEPTABLE_CONN_INTERVAL_TERM             LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL
#define LL_MIC_FAILURE_TERM                            LL_STATUS_ERROR_CONN_TERM_DUE_TO_MIC_FAILURE
#define LL_CONN_ESTABLISHMENT_FAILED_TERM              LL_STATUS_ERROR_CONN_FAILED_TO_BE_ESTABLISHED

// Disconnect API Parameter
#define LL_DISCONNECT_AUTH_FAILURE                     LL_STATUS_ERROR_AUTH_FAILURE
#define LL_DISCONNECT_REMOTE_USER_TERM                 LL_STATUS_ERROR_PEER_TERM
#define LL_DISCONNECT_REMOTE_DEV_LOW_RESOURCES         LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES
#define LL_DISCONNECT_REMOTE_DEV_POWER_OFF             LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF
#define LL_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE       LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE
#define LL_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED        LL_STATUS_ERROR_KEY_PAIRING_NOT_SUPPORTED
#define LL_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL       LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL


// LL Advertiser Events
#define LL_ADV_CONNECTABLE_UNDIRECTED_EVT              0
#define LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT            1  // High Duty Cycle
#define LL_ADV_SCANNABLE_UNDIRECTED_EVT                2
#define LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT           3
#define LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT            4  // Low Duty Cycle

// LL Address Type
#define LL_DEV_ADDR_TYPE_PUBLIC                        0
#define LL_DEV_ADDR_TYPE_RANDOM                        1
#define LL_DEV_ADDR_TYPE_RPA_PUBLIC                    2        // Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving list contains no matching entry, use public address
#define LL_DEV_ADDR_TYPE_RPA_RANDOM                    3        // Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving list contains no matching entry, use random address from LE_Set_Random_Address.



// Advertiser White List Policy
#define LL_ADV_WL_POLICY_ANY_REQ                       0  // any scan request or connect request
#define LL_ADV_WL_POLICY_WL_SCAN_REQ                   1  // any connect request, white list scan request
#define LL_ADV_WL_POLICY_WL_CONNECT_REQ                2  // any scan request, white list connect request
#define LL_ADV_WL_POLICY_WL_ALL_REQ                    3  // white list scan request and connect request

// Scanner White List Policy
#define LL_SCAN_WL_POLICY_ANY_ADV_PKTS                 0
#define LL_SCAN_WL_POLICY_USE_WHITE_LIST               1
#define LL_SCAN_WL_POLICY_BLE42_1                      2
#define LL_SCAN_WL_POLICY_BLE42_2                      3


// Initiator White List Policy
#define LL_INIT_WL_POLICY_USE_PEER_ADDR                0
#define LL_INIT_WL_POLICY_USE_WHITE_LIST               1

// Black List Control
#define LL_SET_BLACKLIST_DISABLE                       0
#define LL_SET_BLACKLIST_ENABLE                        1

// Advertiser Commands
#define LL_ADV_MODE_OFF                                0
#define LL_ADV_MODE_ON                                 1
#define LL_ADV_MODE_RESERVED                           2

// LL Scan Commands
#define LL_SCAN_STOP                                   0
#define LL_SCAN_START                                  1

// LL Scan Filtering
#define LL_FILTER_REPORTS_DISABLE                      0
#define LL_FILTER_REPORTS_ENABLE                       1

// LL Scan Types
#define LL_SCAN_PASSIVE                                0
#define LL_SCAN_ACTIVE                                 1

// LL Tx Power Types
#define LL_READ_CURRENT_TX_POWER_LEVEL                 0
#define LL_READ_MAX_TX_POWER_LEVEL                     1

// Data Fragmentation Flag
#define LL_DATA_FIRST_PKT_HOST_TO_CTRL                 0
#define LL_DATA_CONTINUATION_PKT                       1
#define LL_DATA_FIRST_PKT_CTRL_TO_HOST                 2

// Connection Complete Role
#define LL_LINK_CONNECT_COMPLETE_MASTER                0
#define LL_LINK_CONNECT_COMPLETE_SLAVE                 1

// Encryption Related
#define LL_ENCRYPTION_OFF                              0
#define LL_ENCRYPTION_ON                               1

// Feature Set Related
#define LL_MAX_FEATURE_SET_SIZE                        8  // in bytes
//
#define LL_FEATURE_RFU                                 0  // all bits in a byte
#define LL_FEATURE_ENCRYPTION                          1  // byte 0, bit 0
#define LL_FEATURE_EXT_REJECT_IND                      4  // byte 0, bit 0
#define LL_FEATURE_DATA_LENGTH_EXTENSION            0x20  // byte 0, bit 0

#define LL_FEATURE_2M_PHY                           0x01  // byte 1, bit 0
#define LL_FEATURE_CODED_PHY                        0x08  // byte 1, bit 3

#define LL_FEATURE_CSA2                             0x40  // byte 1, bit 6

// 2020-01-15 add 
// CODE PHY feature
#define LL_FEATURE_CODE_PHY_IDX						1	  // byte 1
#define LL_FEATURE_CODE_PHY							0x08
// extended advertisingCTE feature 
#define LL_FEATURE_EXT_ADV_IDX						1
#define LL_FEATURE_EXT_ADV							0x10
// periodic advertising 
#define LL_FEATURE_PRD_ADV_IDX						1
#define LL_FEATURE_PRD_ADV							0x20
// Channel selection Algorithm #2
#define LL_FEATURE_CSA2_IDX							1
#define LL_FEATURE_CSA2								0x40

// CTE FEATURE
#define LL_CTE_FEATURE_IDX							2
#define LL_CONN_CTE_REQ								0x02
#define LL_CONN_CTE_RSP								0x04
#define LL_CONNLESS_CTE_TRANSMITER					0x08
#define LL_CONNLESS_CTE_RECEIVER					0x10
#define LL_AOD_SUPPORT								0x20
#define LL_AOA_SUPPORT								0x40


// Receive Flow Control
#define LL_DISABLE_RX_FLOW_CONTROL                     0
#define LL_ENABLE_RX_FLOW_CONTROL                      1

// Direct Test Mode
#define LL_DIRECT_TEST_NUM_RF_CHANS                    40   // PHY_NUM_RF_CHANS
#define LL_DIRECT_TEST_MAX_PAYLOAD_LEN                 37
//
#define LL_DIRECT_TEST_PAYLOAD_PRBS9                   0
#define LL_DIRECT_TEST_PAYLOAD_0x0F                    1
#define LL_DIRECT_TEST_PAYLOAD_0x55                    2
#define LL_DIRECT_TEST_PAYLOAD_PRBS15                  3
#define LL_DIRECT_TEST_PAYLOAD_0xFF                    4
#define LL_DIRECT_TEST_PAYLOAD_0x00                    5
#define LL_DIRECT_TEST_PAYLOAD_0xF0                    6
#define LL_DIRECT_TEST_PAYLOAD_0xAA                    7
#define LL_DIRECT_TEST_PAYLOAD_UNDEFINED               0xFF
//
#define LL_DIRECT_TEST_MODE_TX                         0
#define LL_DIRECT_TEST_MODE_RX                         1
//
#define LL_RF_RSSI_UNDEFINED                           PHY_RSSI_VALUE_INVALID

// Vendor Specific
#define LL_EXT_RX_GAIN_STD                             0
#define LL_EXT_RX_GAIN_HIGH                            1
//
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3


//
#define LL_EXT_DISABLE_ONE_PKT_PER_EVT                 0
#define LL_EXT_ENABLE_ONE_PKT_PER_EVT                  1
//
#define LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT              0
#define LL_EXT_ENABLE_CLK_DIVIDE_ON_HALT               1
//
#define LL_EXT_NV_NOT_IN_USE                           0
#define LL_EXT_NV_IN_USE                               1
//
#define LL_EXT_DISABLE_FAST_TX_RESP_TIME               0
#define LL_EXT_ENABLE_FAST_TX_RESP_TIME                1
//
#define LL_EXT_DISABLE_SL_OVERRIDE                     0
#define LL_EXT_ENABLE_SL_OVERRIDE                      1
//
#define LL_EXT_TX_MODULATED_CARRIER                    0
#define LL_EXT_TX_UNMODULATED_CARRIER                  1
//
#define LL_EXT_SET_FREQ_TUNE_DOWN                      0
#define LL_EXT_SET_FREQ_TUNE_UP                        1

//
#define LL_EXT_PER_RESET                               0
#define LL_EXT_PER_READ                                1
//
#define LL_EXT_HALT_DURING_RF_DISABLE                  0
#define LL_EXT_HALT_DURING_RF_ENABLE                   1
//
#define LL_EXT_SET_USER_REVISION                       0
#define LL_EXT_READ_BUILD_REVISION                     1
//
#define LL_EXT_RESET_SYSTEM_DELAY                      100 // in ms
#define LL_EXT_RESET_SYSTEM_HARD                       0
#define LL_EXT_RESET_SYSTEM_SOFT                       1
//
#define LL_EXT_DISABLE_OVERLAPPED_PROCESSING           0
#define LL_EXT_ENABLE_OVERLAPPED_PROCESSING            1
//
#define LL_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT         0
#define LL_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT          1


/*
** Event Parameters
*/

// Advertising Report Data
#define LL_ADV_RPT_ADV_IND                             LL_ADV_CONNECTABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_ADV_DIRECT_IND                      LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT
#define LL_ADV_RPT_ADV_SCANNABLE_IND                   LL_ADV_SCANNABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_ADV_NONCONN_IND                     LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_SCAN_RSP                            (LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT + 1)
#define LL_ADV_RPT_INVALID                             0xFF
//
#define LL_RSSI_NOT_AVAILABLE                          127

// Sleep Clock Accuracy (SCA)
#define LL_SCA_500_PPM                                 0
#define LL_SCA_250_PPM                                 1
#define LL_SCA_150_PPM                                 2
#define LL_SCA_100_PPM                                 3
#define LL_SCA_75_PPM                                  4
#define LL_SCA_50_PPM                                  5
#define LL_SCA_30_PPM                                  6
#define LL_SCA_20_PPM                                  7


// Default SCA
//#define LL_SCA_MASTER_DEFAULT         5         // 50ppm (ordinal value)
#define LL_SCA_MASTER_DEFAULT         0          // 500ppm (ordinal value)
#define LL_SCA_SLAVE_DEFAULT          500        // 500ppm

// LL Advertiser Channels
#define LL_ADV_CHAN_37                  1
#define LL_ADV_CHAN_38                  2
#define LL_ADV_CHAN_39                  4
#define LL_ADV_CHAN_ALL                                (LL_ADV_CHAN_37 | LL_ADV_CHAN_38 | LL_ADV_CHAN_39)
#define LL_ADV_CHAN_MAP_DEFAULT       LL_ADV_CHAN_ALL

#define LL_ADV_CHAN_FIRST               37
#define LL_ADV_CHAN_LAST                39

// max future number of events for an update to parameters or data channel
#define LL_MAX_UPDATE_COUNT_RANGE                32767

// Extended Header Flags
#define LE_EXT_HDR_ADVA_PRESENT_BITMASK         0x01
#define LE_EXT_HDR_TARGETA_PRESENT_BITMASK      0x02
#define LE_EXT_HDR_CTE_INFO_PRESENT_BITMASK     0x04
#define LE_EXT_HDR_ADI_PRESENT_BITMASK          0x08
#define LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK      0x10
#define LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK    0x20            
#define LE_EXT_HDR_TX_PWR_PRESENT_BITMASK       0x40            
#define LE_EXT_HDR_RFU_PRESENT_BITMASK          0x80            



// extended advertisement Macros
#define LE_ADV_PROP_CONN_BITMASK         0x00000001
#define LE_ADV_PROP_SCAN_BITMASK         0x00000002
#define LE_ADV_PROP_DIRECT_BITMASK       0x00000004
#define LE_ADV_PROP_HI_DC_CONN_BITMASK   0x00000008
#define LE_ADV_PROP_LEGACY_BITMASK       0x00000010
#define LE_ADV_PROP_ANON_BITMASK         0x00000020            // applicable to extended adv only
#define LE_ADV_PROP_TX_POWER_BITMASK     0x00000040            // applicable to extended adv & periodic adv

/*  TODO: update according to spec
	0x00 Intermediate fragment of fragmented extended advertising data
0x01 First fragment of fragmented extended advertising data
0x02 Last fragment of fragmented extended advertising data
0x03 Complete extended advertising data
0x04 Unchanged data (just update the Advertising DID) */

#define BLE_EXT_ADV_OP_INTERM_FRAG        0x00
#define BLE_EXT_ADV_OP_FIRST_FRAG         0x01
#define BLE_EXT_ADV_OP_LAST_FRAG          0x02
#define BLE_EXT_ADV_OP_COMPLETE_DATA      0x03
#define BLE_EXT_ADV_OP_UNCHANGED_DATA     0x04

#define BLE_EXT_ADV_FRAG_ENABLED          0x00
#define BLE_EXT_ADV_FRAG_DISABLED         0x01

#define LL_EXT_ADV_MODE_NOCONN_NOSC       0
#define LL_EXT_ADV_MODE_AUX_CONN_RSP      0
#define LL_EXT_ADV_MODE_CONN              1
#define LL_EXT_ADV_MODE_SC                2
#define LL_EXT_ADV_MODE_RFU               3

// AuxPtr
// channel Idx(6bits)  |   CA(1bit)   | offset Unit(1 bit)   | Aux offset(13bits)  | Aux PHY
#define LL_AUX_PTR_CHN_IDX_SHIFT               0
#define LL_AUX_PTR_CA_SHIFT                    6
#define LL_AUX_PTR_OFFSET_UNIT_SHIFT           7
#define LL_AUX_PTR_AUX_OFFSET_SHIFT            8
#define LL_AUX_PTR_AUX_PHY_SHIFT               21

#define LL_AUX_PTR_CHN_IDX_MASK               0x3F
#define LL_AUX_PTR_CA_MASK                    0x1
#define LL_AUX_PTR_OFFSET_UNIT_MASK           0x1
#define LL_AUX_PTR_AUX_OFFSET_MASK            0x1FFF
#define LL_AUX_PTR_AUX_PHY_MASK               0x7


// for Periodic scanner
#define LL_PERIODIC_ADV_CREATE_SYNC_USING_ADV_LIST_BITMASK        0x00000001
#define LL_PERIODIC_ADV_CREATE_SYNC_INIT_RPT_DISABLE_BITMASK      0x00000002

/*
** Miscellaneous
*/
#define   BLE_PKT40_LEN                   42
#define   BLE_PKT51_LEN                   262

#define   BLE_PKT_VERSION_4_0             0
#define   BLE_PKT_VERSION_5_1             1


//====== add after BBB ROM code release
#define LL_EXT_ADV_PROP_ADV_IND		      0x13        //0b00010011
#define LL_EXT_ADV_PROP_ADV_LDC_ADV		  0x15        //0b00010101
#define LL_EXT_ADV_PROP_ADV_HDC_ADV		  0x1d        //0b00011101
#define LL_EXT_ADV_PROP_ADV_SCAN_IND	  0x12        //0b00010010
#define LL_EXT_ADV_PROP_ADV_NOCONN_IND	  0x10        //0b00010000

#define LL_CHN_SEL_ALGORITHM_1            0
#define LL_CHN_SEL_ALGORITHM_2            1


/*******************************************************************************
 * TYPEDEFS
 */



/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
/*******************************************************************************
 * LL OSAL Functions
 */

/*******************************************************************************
 * @fn          LL_Init
 *
 * @brief       This is the Link Layer task initialization called by OSAL. It
 *              must be called once when the software system is started and
 *              before any other function in the LL API is called.
 *
 * input parameters
 *
 * @param       taskId - Task identifier assigned by OSAL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_Init( uint8_t taskId );


/*******************************************************************************
 * @fn          LL_ProcessEvent
 *
 * @brief       This is the Link Layer process event handler called by OSAL.
 *
 * input parameters
 *
 * @param       taskId - Task identifier assigned by OSAL.
 *              events - Event flags to be processed by this task.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Unprocessed event flags.
 */
extern uint16_t LL_ProcessEvent( uint8_t task_id, uint16_t events );


/*******************************************************************************
 * LL API for HCI
 */

/*******************************************************************************
 * @fn          LL_TX_bm_alloc API
 *
 * @brief       This API is used to allocate memory using buffer management.
 *
 *              Note: This function should never be called by the application.
 *                    It is only used by HCI and L2CAP_bm_alloc.
 *
 * input parameters
 *
 * @param       size - Number of bytes to allocate from the heap.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to buffer, or NULL.
 */
extern void *LL_TX_bm_alloc( uint16_t size );


/*******************************************************************************
 * @fn          LL_RX_bm_alloc API
 *
 * @brief       This API is used to allocate memory using buffer management.
 *
 *              Note: This function should never be called by the application.
 *                    It is only used by HCI and L2CAP_bm_alloc.
 *
 * input parameters
 *
 * @param       size - Number of bytes to allocate from the heap.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to buffer, or NULL.
 */
extern void *LL_RX_bm_alloc( uint16_t size );


/*******************************************************************************
 * @fn          LL_Reset API
 *
 * @brief       This function is used by the HCI to reset and initialize the
 *              LL Controller.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_Reset( void );


/*******************************************************************************
 * @fn          LL_ReadBDADDR API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              own public device address.
 *
 *              Note: The device's address is stored in NV memory.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       bdAddr  - A pointer to a buffer to hold this device's address.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadBDADDR( uint8_t *bdAddr );


/*******************************************************************************
 *
 * @fn          LL_SetRandomAddress API
 *
 * @brief       This function is used to save this device's random address. It
 *              is provided by the Host for devices that are unable to store a
 *              IEEE assigned public address in NV memory.
 *
 * input parameters
 *
 * @param       devAddr - Pointer to a random address (LSO..MSO).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 *
 */
extern llStatus_t LL_SetRandomAddress( uint8_t *devAddr );

/*******************************************************************************
 * @fn          LL_ClearWhiteList API
 *
 * @brief       This API is called by the HCI to clear the White List.
 *
 *              Note: If Scanning is enabled using filtering, and the white
 *                    list policy is "Any", then this command will be
 *                    disallowed.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ClearWhiteList( void );


/*******************************************************************************
 * @fn          LL_AddWhiteListDevice API
 *
 * @brief       This API is called by the HCI to add a device address and its
 *              type to the White List.
 *
 * input parameters
 *
 * @param       devAddr      - Pointer to a 6 byte device address.
 * @param       addrType     - Public or Random device address.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_WL_TABLE_FULL
 */
extern llStatus_t LL_AddWhiteListDevice( uint8_t *devAddr,
                                         uint8_t addrType );

/*******************************************************************************
 * @fn          LL_RemoveWhiteListDevice API
 *
 * @brief       This API is called by the HCI to remove a device address and
 *              it's type from the White List.
 *
 * input parameters
 *
 * @param       devAddr  - Pointer to a 6 byte device address.
 * @param       addrType - Public or Random device address.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_WL_TABLE_EMPTY,
 *              LL_STATUS_ERROR_WL_ENTRY_NOT_FOUND
 */
extern llStatus_t LL_RemoveWhiteListDevice( uint8_t *devAddr,
                                            uint8_t addrType );


/*******************************************************************************
 * @fn          LL_ReadWlSize API
 *
 * @brief       This API is called by the HCI to get the total number of white
 *              list entries that can be stored in the Controller.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       *numEntries - Total number of available White List entries.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadWlSize( uint8_t *numEntries );


/*******************************************************************************
 * @fn          LL_NumEmptyWlEntries API
 *
 * @brief       This API is called by the HCI to get the number of White List
 *              entries that are empty.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       *numEmptyEntries - number of empty entries in the White List.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_NumEmptyWlEntries( uint8_t *numEmptyEntries );


/*******************************************************************************
 * @fn          LL_Encrypt API
 *
 * @brief       This API is called by the HCI to request the LL to encrypt the
 *              data in the command using the key given in the command.
 *
 *              Note: The parameters are byte ordered MSO to LSO.
 *
 * input parameters
 *
 * @param       *key           - A 128 bit key to be used to calculate the
 *                               session key.
 * @param       *plaintextData - A 128 bit block that is to be encrypted.
 *
 * output parameters
 *
 * @param       *encryptedData - A 128 bit block that is encrypted.
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_Encrypt( uint8_t *key,
                              uint8_t *plaintextData,
                              uint8_t *encryptedData );


/*******************************************************************************
 * @fn          LL_Rand API
 *
 * @brief       This API is called by the HCI to request the LL Controller to
 *              provide a data block with random content.
 *
 *              Note: If the radio is in use, then this operation has to be
 *                    delayed until the radio finishes.
 *
 * input parameters
 *
 * @param       *randData - Pointer to buffer to place a random block of data.
 * @param        dataLen  - The length of the random data block, from 1-255.
 *
 * output parameters
 *
 * @param       *randData - Pointer to buffer containing a block of true random
 *                          data.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_DUE_TO_LIMITED_RESOURCES,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED,
 *              LL_STATUS_ERROR_BAD_PARAMETER, LL_STATUS_ERROR_RNG_FAILURE
 */
extern llStatus_t LL_Rand( uint8_t *randData,
                           uint8_t dataLen );


/*******************************************************************************
 * @fn          LL_ReadSupportedStates API
 *
 * @brief       This function is used to provide the HCI with the Link Layer
 *              supported states and supported state/role combinations.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       *states - Eight byte Bit map of supported states/combos.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadSupportedStates( uint8_t *states );


/*******************************************************************************
 * @fn          LL_ReadLocalSupportedFeatures API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              Features Set. The Controller indicates which features it
 *              supports.
 *
 * input parameters
 *
 * @param       featureSet  - A pointer to the Feature Set where each bit:
 *                            0: Feature not supported.
 *                            1: Feature supported by controller.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadLocalSupportedFeatures( uint8_t *featureSet );


/*******************************************************************************
 * @fn          LL_ReadLocalVersionInfo API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              Version information.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       verNum    - Version of the Bluetooth Controller specification.
 * @param       comId     - Company identifier of the manufacturer of the
 *                          Bluetooth Controller.
 * @param       subverNum - A unique value for each implementation or revision
 *                          of an implementation of the Bluetooth Controller.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadLocalVersionInfo( uint8_t  *verNum,
                                           uint16_t *comId,
                                           uint16_t *subverNum );


/*******************************************************************************
 * @fn          LL_CtrlToHostFlowControl API
 *
 * @brief       This function is used to indicate if the LL enable/disable
 *              receive FIFO processing. This function provides support for
 *              Controller to Host flow control.
 *
 * input parameters
 *
 * @param       mode: LL_ENABLE_RX_FLOW_CONTROL, LL_DISABLE_RX_FLOW_CONTROL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_CtrlToHostFlowControl( uint8_t mode );

/*******************************************************************************
 * @fn          LL_ReadRemoteVersionInfo API
 *
 * @brief       This API is called by the HCI to read the peer controller's
 *              Version Information. If the peer's Version Information has
 *              already been received by its request for our Version
 *              Information, then this data is already cached and can be
 *              directly returned to the Host. If the peer's Version Information
 *              is not already cached, then it will be requested from the peer,
 *              and when received, returned to the Host via the
 *              LL_ReadRemoteVersionInfoCback callback.
 *
 *              Note: Only one Version Indication is allowed for a connection.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_VER_IND_ALREADY_SENT
 */
extern llStatus_t LL_ReadRemoteVersionInfo( uint16_t connId );

/*******************************************************************************
 * @fn          LL_ReadTxPowerLevel
 *
 * @brief       This function is used to read a connection's current transmit
 *              power level or the maximum transmit power level.
 *
 * input parameters
 *
 * @param       connId   - The LL connection handle.
 * @param       type     - LL_READ_CURRENT_TX_POWER_LEVEL or
 *                         LL_READ_MAX_TX_POWER_LEVEL
 * @param       *txPower - A signed value from -30..+20, in dBm.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_PARAM_OUT_OF_RANGE,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
llStatus_t LL_ReadTxPowerLevel( uint8_t connId,
                                uint8_t type,
                                int8_t  *txPower );

// A1 ROM metal change add
/*******************************************************************************
 * @fn          LL_SetTxPowerLevel
 *
 * @brief       This function is used to set transmit power level
 *
 * input parameters
 *
 * @param       txPower   - The transmit power level to be set
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
llStatus_t LL_SetTxPowerLevel( int8_t  txPower );

/*******************************************************************************
 * @fn          LL_ReadChanMap API
 *
 * @brief       This API is called by the HCI to read the channel map that the
 *              LL controller is using for the LL connection.
 *
 * input parameters
 *
 * @param       connId  - The LL connection handle.
 *
 * output parameters
 *
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used" and a 0 means
 *                        the channel is "unused".
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ReadChanMap( uint8_t connId,
                                  uint8_t *chanMap );



/*******************************************************************************
 * @fn          LL_ReadRssi API
 *
 * @brief       This API is called by the HCI to request RSSI. If there is an
 *              active connection for the given connection ID, then the RSSI of
 *              the last received data packet in the LL will be returned. If a
 *              receiver Modem Test is running, then the RF RSSI for the last
 *              received data will be returned. If no valid RSSI value is
 *              available, then LL_RSSI_NOT_AVAILABLE will be returned.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to read last RSSI.
 *
 * output parameters
 *
 * @param       *lastRssi - The last data RSSI received.
 *                          Range: -127dBm..+20dBm, 127=Not Available.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ReadRssi( uint16_t connId,
                               int8_t   *lastRssi );
extern llStatus_t LL_ReadFoff( uint16_t connId,
                               uint16_t   *foff );
extern llStatus_t LL_ReadCarrSens( uint16_t connId,
                                uint8_t   *carrSense );

/*******************************************************************************
 * @fn          LL_Disconnect API
 *
 * @brief       This API is called by the HCI to terminate a LL connection.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 * @param       reason - The reason for the Host connection termination.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 *              LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE
 */
extern llStatus_t LL_Disconnect( uint16_t connId,
                                 uint8_t  reason );

/*******************************************************************************
 * @fn          LL_TxData API
 *
 * @brief       This API is called by the HCI to transmit a buffer of data on a
 *              given LL connection. If fragmentation is supported, the HCI must
 *              also indicate whether this is the first Host packet, or a
 *              continuation Host packet. When fragmentation is not supported,
 *              then a start packet should always specified. If the device is in
 *              a connection as a Master and the current connection ID is the
 *              connection for this data, or is in a connection as a Slave, then
 *              the data is written to the TX FIFO (even if the radio is
 *              curerntly active). If this is a Slave connection, and Fast TX is
 *              enabled and Slave Latency is being used, then the amount of time
 *              to the next event is checked. If there's at least a connection
 *              interval plus some overhead, then the next event is re-aligned
 *              to the next event boundary. Otherwise, in all cases, the buffer
 *              pointer will be retained for transmission, and the callback
 *              event LL_TxDataCompleteCback will be generated to the HCI when
 *              the buffer pointer is no longer needed by the LL.
 *
 *              Note: If the return status is LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *                    then the HCI must not release the buffer until it receives
 *                    the LL_TxDataCompleteCback callback, which indicates the
 *                    LL has copied the transmit buffer.
 *
 *              Note: The HCI should not call this routine if a buffer is still
 *                    pending from a previous call. This is fatal!
 *
 *              Note: If the connection should be terminated within the LL
 *                    before the Host knows, attempts by the HCI to send more
 *                    data (after receiving a LL_TxDataCompleteCback) will
 *                    fail (LL_STATUS_ERROR_INACTIVE_CONNECTION).
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which to send this data.
 * @param       *pBuf    - A pointer to the data buffer to transmit.
 * @param       pktLen   - The number of bytes to transmit on this connection.
 * @param       fragFlag - LL_DATA_FIRST_PKT_HOST_TO_CTRL:
 *                           Indicates buffer is the start of a
 *                           Host-to-Controller packet.
 *                         LL_DATA_CONTINUATION_PKT:
 *                           Indicates buffer is a continuation of a
 *                           Host-to-Controller packet.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION,
 *              LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *              LL_STATUS_ERROR_UNEXPECTED_PARAMETER
 */
extern llStatus_t LL_TxData( uint16_t connId,
                             uint8_t  *pBuf,
                             uint8_t  pktLen,
                             uint8_t  fragFlag );


/*******************************************************************************
 * @fn          LL_DirectTestTxTest API
 *
 * @brief       This function is used to initiate a BLE PHY level Transmit Test
 *              in Direct Test Mode where the DUT generates test reference
 *              packets at fixed intervals. This test will make use of the
 *              nanoRisc Raw Data Transmit and Receive task.
 *
 *              Note: The BLE device is to transmit at maximum power.
 *              Note: A LL reset should be issued when done using DTM!
 *
 * input parameters
 *
 * @param       txFreq      - Tx RF frequency k=0..39, where F=2402+(k*2MHz).
 * @param       payloadLen  - Number of bytes (0..37)in payload for each packet.
 * @param       payloadType - The type of pattern to transmit.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_DirectTestTxTest( uint8_t txFreq,
                                       uint8_t payloadLen,
                                       uint8_t payloadType );


/*******************************************************************************
 * @fn          LL_DirectTestRxTest API
 *
 * @brief       This function is used to initiate a BLE PHY level Receive Test
 *              in Direct Test Mode where the DUT receives test reference
 *              packets at fixed intervals. This test will make use of the
 *              nanoRisc Raw Data Transmit and Receive task. The received
 *              packets are verified based on the CRC, and metrics are kept.
 *
 *              Note: A LL reset should be issued when done using DTM!
 *
 * input parameters
 *
 * @param       rxFreq - Rx RF frequency k=0..39, where F=2402+(k*2MHz).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_DirectTestRxTest( uint8_t rxFreq );


/*******************************************************************************
 * @fn          LL_DirectTestEnd API
 *
 * @brief       This function is used to end the Direct Test Transmit or Direct
 *              Test Receive tests executing in Direct Test mode. When the raw
 *              task is ended, the LL_DirectTestEndDoneCback callback is called.
 *              If a Direct Test mode operation is not currently active, an
 *              error is returned.
 *
 *              Note: A LL reset is issued upon completion!
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_DirectTestEnd( void );


/*******************************************************************************
 * @fn          LL_SetAdvParam API
 *
 * @brief       This API is called by the HCI to set the Advertiser's
 *              parameters.
 *
 * input parameters
 * @param       advIntervalMin - The minimum Adv interval.
 * @param       advIntervalMax - The maximum Adv interval.
 * @param       advEvtType     - The type of advertisment event.
 * @param       ownAddrType    - The Adv's address type of public or random.
 * @param       directAddrType - Only used for directed advertising.
 * @param       *directAddr    - Only used for directed advertising (NULL otherwise).
 * @param       advChanMap     - A byte containing 1 bit per advertising
 *                               channel. A bit set to 1 means the channel is
 *                               used. The bit positions define the advertising
 *                               channels as follows:
 *                               Bit 0: 37, Bit 1: 38, Bit 2: 39.
 * @param       advWlPolicy    - The Adv white list filter policy.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_NO_ADV_CHAN_FOUND
 */
extern llStatus_t LL_SetAdvParam( uint16_t advIntervalMin,
                                  uint16_t advIntervalMax,
                                  uint8_t  advEvtType,
                                  uint8_t  ownAddrType,
                                  uint8_t  directAddrType,
                                  uint8_t  *directAddr,
                                  uint8_t  advChanMap,
                                  uint8_t  advWlPolicy );

/*******************************************************************************
 * @fn          LL_SetAdvData API
 *
 * @brief       This API is called by the HCI to set the Advertiser's data.
 *
 *              Note: If the Advertiser is restarted without intervening calls
 *                    to this routine to make updates, then the previously
 *                    defined data will be reused.
 *
 *              Note: If the data happens to be changed while advertising, then
 *                    the new data will be sent on the next advertising event.
 *
 * input parameters
 *
 * @param       advDataLen - The number of scan response bytes: 0..31.
 * @param       advData    - Pointer to the advertiser data, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetAdvData( uint8_t advDataLen,
                                 uint8_t *advData );

/*******************************************************************************
 * @fn          LL_SetScanRspData API
 *
 * @brief       This API is called by the HCI to set the Advertiser's Scan
 *              Response data.
 *
 *              Note: If the Advertiser is restarted without intervening calls
 *                    to this routine to make updates, then the previously
 *                    defined data will be reused.
 *
 * input parameters
 *
 * @param       scanRspLen   - The number of scan response bytes: 0..31.
 * @param       *scanRspData - Pointer to the scan response data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetScanRspData( uint8_t scanRspLen,
                                     uint8_t *scanRspData );

/*******************************************************************************
 * @fn          LL_SetAdvControl API
 *
 * @brief       This API is called by the HCI to request the Controller to start
 *              or stop advertising.
 *
 * input parameters
 *
 * @param       advMode - LL_ADV_MODE_ON or LL_ADV_MODE_OFF.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_PARAMETER,
 *              LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_SetAdvControl( uint8_t advMode );

/*******************************************************************************
 * @fn          LL_ReadAdvChanTxPower
 *
 * @brief       This function is used to read the transmit power level used
 *              for BLE advertising channel packets. Currently, only two
 *              settings are possible, a standard setting of 0 dBm, and a
 *              maximum setting of 4 dBm.
 *
 * input parameters
 *
 * @param       *txPower - A non-null pointer.
 *
 * output parameters
 *
 * @param       *txPower - A signed value from -20..+10, in dBm.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_PARAM_OUT_OF_RANGE
 */
extern llStatus_t LL_ReadAdvChanTxPower( int8_t *txPower );

/*******************************************************************************
 * @fn          LL_SetScanParam API
 *
 * @brief       This API is called by the HCI to set the Scanner's parameters.
 *
 * input parameters
 *
 * @param       scanType     - Passive or Active scan type.
 * @param       scanInterval - Time between scan events.
 * @param       scanWindow   - Duration of a scan. When the same as the scan
 *                             interval, then scan continuously.
 * @param       ownAddrType  - Address type (Public or Random) to use in the
 *                             SCAN_REQ packet.
 * @param       advWlPolicy  - Either allow all Adv packets, or only those that
 *                             are in the white list.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetScanParam( uint8_t  scanType,
                                   uint16_t scanInterval,
                                   uint16_t scanWindow,
                                   uint8_t  ownAddrType,
                                   uint8_t  advWlPolicy );

/*******************************************************************************
 * @fn          LL_SetScanControl API
 *
 * @brief       This API is called by the HCI to start or stop the Scanner. It
 *              also specifies whether the LL will filter duplicate advertising
 *              reports to the Host, or generate a report for each packet
 *              received.
 *
 * input parameters
 *
 * @param       scanMode      - LL_SCAN_START or LL_SCAN_STOP.
 * @param       filterReports - LL_FILTER_REPORTS_DISABLE or
 *                              LL_FILTER_REPORTS_ENABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_PARAMETER,
 *              LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_SetScanControl( uint8_t scanMode,
                                     uint8_t filterReports );

/*******************************************************************************
 * @fn          LL_EncLtkReply API
 *
 * @brief       This API is called by the HCI to provide the controller with
 *              the Long Term Key (LTK) for encryption. This command is
 *              actually a reply to the link layer's LL_EncLtkReqCback, which
 *              provided the random number and encryption diversifier received
 *              from the Master during an encryption setup.
 *
 *              Note: The key parameter is byte ordered LSO to MSO.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 * @param       *key   - A 128 bit key to be used to calculate the session key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EncLtkReply( uint16_t connId,
                                  uint8_t  *key );

/*******************************************************************************
 * @fn          LL_EncLtkNegReply API
 *
 * @brief       This API is called by the HCI to indicate to the controller
 *              that the Long Term Key (LTK) for encryption can not be provided.
 *              This command is actually a reply to the link layer's
 *              LL_EncLtkReqCback, which provided the random number and
 *              encryption diversifier received from the Master during an
 *              encryption setup. How the LL responds to the negative reply
 *              depends on whether this is part of a start encryption or a
 *              re-start encryption after a pause. For the former, an
 *              encryption request rejection is sent to the peer device. For
 *              the latter, the connection is terminated.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EncLtkNegReply( uint16_t connId );

/*******************************************************************************
 * @fn          LL_CreateConn API
 *
 * @brief       This API is called by the HCI to create a connection.
 *
 * input parameters
 *
 * @param       scanInterval    - The scan interval.
 * @param       scanWindow      - The scan window.
 * @param       initWlPolicy    - Filter Adv address directly or using WL.
 * @param       peerAddrType    - Peer address is Public or Random.
 * @param       *peerAddr       - The Adv address, or NULL for WL policy.
 * @param       ownAddrType     - This device's address is Public or Random.
 * @param       connIntervalMin - Defines minimum connection interval value.
 * @param       connIntervalMax - Defines maximum connection interval value.
 * @param       connLatency     - The connection's Slave Latency.
 * @param       connTimeout     - The connection's Supervision Timeout.
 * @param       minLength       - Info parameter about min length of connection.
 * @param       maxLength       - Info parameter about max length of connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE,
 *              LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION,
 *              LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_CreateConn( uint16_t scanInterval,
                                 uint16_t scanWindow,
                                 uint8_t  initWlPolicy,
                                 uint8_t  peerAddrType,
                                 uint8_t  *peerAddr,
                                 uint8_t  ownAddrType,
                                 uint16_t connIntervalMin,
                                 uint16_t connIntervalMax,
                                 uint16_t connLatency,
                                 uint16_t connTimeout,
                                 uint16_t minLength,
                                 uint16_t maxLength );

/*******************************************************************************
 * @fn          LL_CreateConnCancel API
 *
 * @brief       This API is called by the HCI to cancel a previously given LL
 *              connection creation command that is still pending. This command
 *              should only be used after the LL_CreateConn command as been
 *              issued, but before the LL_ConnComplete callback.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_CreateConnCancel( void );

/*******************************************************************************
 * @fn          LL_ConnActive
 *
 * @brief       This API is called by the HCI to check if a connection
 *              given by the connection handle is active.
 *
 * input parameters
 *
 * @param       connId - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ConnActive( uint16_t connId );

/*******************************************************************************
 * @fn          LL_ConnUpdate API
 *
 * @brief       This API is called by the HCI to update the connection
 *              parameters by initiating a connection update control procedure.
 *
 * input parameters
 *
 * @param       connId          - The connection ID on which to send this data.
 * @param       connIntervalMin - Defines minimum connection interval value.
 * @param       connIntervalMax - Defines maximum connection interval value.
 * @param       connLatency     - The connection's Slave Latency.
 * @param       connTimeout     - The connection's Supervision Timeout.
 * @param       minLength       - Info parameter about min length of connection.
 * @param       maxLength       - Info parameter about max length of connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 *              LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE,
 *              LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION
 */
extern llStatus_t LL_ConnUpdate( uint16_t connId,
                                 uint16_t connIntervalMin,
                                 uint16_t connIntervalMax,
                                 uint16_t connLatency,
                                 uint16_t connTimeout,
                                 uint16_t minLength,
                                 uint16_t maxLength );

/*******************************************************************************
 * @fn          LL_ChanMapUpdate API
 *
 * @brief       This API is called by the HCI to update the Host data channels
 *              initiating an Update Data Channel control procedure.
 *
 *              Note: While it isn't specified, it is assumed that the Host
 *                    expects an update channel map on all active connections.
 *
 *              Note: This LL currently only supports one connection.
 *
 * input parameters
 *
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used".
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION
 */
extern llStatus_t LL_ChanMapUpdate( uint8_t *chanMap );

/*******************************************************************************
 * @fn          LL_StartEncrypt API
 *
 * @brief       This API is called by the Master HCI to setup encryption and to
 *              update encryption keys in the LL connection. If the connection
 *              is already in encryption mode, then this command will first
 *              pause the encryption before subsequently running the encryption
 *              setup.
 *
 *              Note: The parameters are byte ordered LSO to MSO.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 * @param       *rand  - Random vector used in device identification.
 * @param       *eDiv  - Encrypted diversifier.
 * @param       *key   - A 128 bit key to be used to calculate the session key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED
 */
extern llStatus_t LL_StartEncrypt( uint16_t connId,
                                   uint8_t  *rand,
                                   uint8_t  *eDiv,
                                   uint8_t  *ltk );

/*******************************************************************************
 * @fn          LL_ReadRemoteUsedFeatures API
 *
 * @brief       This API is called by the Master HCI to initiate a feature
 *              setup control process.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadRemoteUsedFeatures( uint16_t connId );


/*
** Vendor Specific Command API
*/

/*******************************************************************************
 * @fn          LL_EXT_SetRxGain Vendor Specific API
 *
 * @brief       This function is used to to set the RF RX gain.
 *
 * input parameters
 *
 * @param       rxGain - LL_EXT_RX_GAIN_STD, LL_EXT_RX_GAIN_HIGH
 *
 * output parameters
 *
 * @param       cmdComplete - Boolean to indicate the command is still pending.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetRxGain( uint8_t rxGain,
                                    uint8_t *cmdComplete );


/*******************************************************************************
 * @fn          LL_EXT_SetTxPower Vendor Specific API
 *
 * @brief       This function is used to to set the RF TX power.
 *
 * input parameters
 *
 * @param       txPower - LL_EXT_TX_POWER_0_DBM, LL_EXT_TX_POWER_4_DBM
 *
 * output parameters
 *
 * @param       cmdComplete - Boolean to indicate the command is still pending.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetTxPower( uint8_t txPower,
                                     uint8_t *cmdComplete );



/*******************************************************************************
 * @fn          LL_EXT_OnePacketPerEvent Vendor Specific API
 *
 * @brief       This function is used to enable or disable allowing only one
 *              packet per event.
 *
 * input parameters
 *
 * @param       control - LL_EXT_ENABLE_ONE_PKT_PER_EVT,
 *                        LL_EXT_DISABLE_ONE_PKT_PER_EVT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_OnePacketPerEvent( uint8_t control );



/*******************************************************************************
 * @fn          LL_EXT_ClkDivOnHalt Vendor Specific API
 *
 * @brief       This function is used to enable or disable dividing down the
 *              system clock while halted.
 *
 *              Note: This command is disallowed if haltDuringRf is not defined.
 *
 * input parameters
 *
 * @param       control - LL_EXT_ENABLE_CLK_DIVIDE_ON_HALT,
 *                        LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_EXT_ClkDivOnHalt( uint8_t control );


/*******************************************************************************
 * @fn          LL_EXT_DeclareNvUsage Vendor Specific API
 *
 * @brief       This HCI Extension API is used to indicate to the Controller
 *              whether or not the Host will be using the NV memory during BLE
 *              operations.
 *
 * input parameters
 *
 * @param       mode - HCI_EXT_NV_IN_USE, HCI_EXT_NV_NOT_IN_USE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_EXT_DeclareNvUsage( uint8_t mode );


/*******************************************************************************
 * @fn          LL_EXT_Decrypt API
 *
 * @brief       This API is called by the HCI to request the LL to decrypt the
 *              data in the command using the key given in the command.
 *
 *              Note: The parameters are byte ordered MSO to LSO.
 *
 * input parameters
 *
 * @param       *key           - A 128 bit key to be used to calculate the
 *                               session key.
 * @param       *encryptedData - A 128 bit block that is encrypted.
 *
 * output parameters
 *
 * @param       *plaintextData - A 128 bit block that is to be encrypted.
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EXT_Decrypt( uint8_t *key,
                                  uint8_t *encryptedData,
                                  uint8_t *plaintextData );


/*******************************************************************************
 * @fn          LL_EXT_SetLocalSupportedFeatures API
 *
 * @brief       This API is called by the HCI to indicate to the Controller
 *              which features can or can not be used.
 *
 *              Note: Not all features indicated by the Host to the Controller
 *                    are valid. If invalid, they shall be ignored.
 *
 * input parameters
 *
 * @param       featureSet  - A pointer to the Feature Set where each bit:
 *                            0: Feature shall not be used.
 *                            1: Feature can be used.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EXT_SetLocalSupportedFeatures( uint8_t *featureSet );


/*******************************************************************************
 * @fn          LL_EXT_SetFastTxResponseTime API
 *
 * @brief       This API is used to enable or disable the fast TX response
 *              time feature. This can be helpful when a short connection
 *              interval is used in combination with slave latency. In such
 *              a scenario, the response time for sending the TX data packet
 *              can effectively shorten or eliminate slave latency, thereby
 *              increasing power consumption. By disabling, this feature
 *              trades fast response time for less power consumption.
 *
 * input parameters
 *
 * @param       control - LL_EXT_ENABLE_FAST_TX_RESP_TIME,
 *                        LL_EXT_DISABLE_FAST_TX_RESP_TIME
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED,
 *              LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetFastTxResponseTime( uint8_t control );

/*******************************************************************************
 * @fn          LL_EXT_SetSlaveLatencyOverride API
 *
 * @brief       This API is used to enable or disable the suspention of slave
 *              latency. This can be helpful when the Slave application knows
 *              it will soon receive something that needs to be handled without
 *              delay.
 *
 * input parameters
 *
 * @param       control - LL_EXT_DISABLE_SL_OVERRIDE,
 *                        LL_EXT_ENABLE_SL_OVERRIDE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED,
 *              LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetSlaveLatencyOverride( uint8_t control );

/*******************************************************************************
 * @fn          LL_EXT_ModemTestTx
 *
 * @brief       This API is used start a continuous transmitter modem test,
 *              using either a modulated or unmodulated carrier wave tone, at
 *              the frequency that corresponds to the specified RF channel. Use
 *              LL_EXT_EndModemTest command to end the test.
 *
 *              Note: A LL reset will be issued by LL_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *              Note: This API can be used to verify this device meets Japan's
 *                    TELEC regulations.
 *
 * input parameters
 *
 * @param       cwMode - LL_EXT_TX_MODULATED_CARRIER,
 *                       LL_EXT_TX_UNMODULATED_CARRIER
 *              txFreq - Transmit RF channel k=0..39, where BLE F=2402+(k*2MHz).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_EXT_ModemTestTx( uint8_t cwMode,
                                      uint8_t txFreq );


/*******************************************************************************
 * @fn          LL_EXT_ModemHopTestTx
 *
 * @brief       This API is used to start a continuous transmitter direct test
 *              mode test using a modulated carrier wave and transmitting a
 *              37 byte packet of Pseudo-Random 9-bit data. A packet is
 *              transmitted on a different frequency (linearly stepping through
 *              all RF channels 0..39) every 625us. Use LL_EXT_EndModemTest
 *              command to end the test.
 *
 *              Note: A LL reset will be issued by LL_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *              Note: This API can be used to verify this device meets Japan's
 *                    TELEC regulations.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_EXT_ModemHopTestTx( void );


/*******************************************************************************
 * @fn          LL_EXT_ModemTestRx
 *
 * @brief       This API is used to start a continuous receiver modem test
 *              using a modulated carrier wave tone, at the frequency that
 *              corresponds to the specific RF channel. Any received data is
 *              discarded. Receiver gain may be adjusted using the
 *              LL_EXT_SetRxGain command. RSSI may be read during this test by
 *              using the LL_ReadRssi command. Use LL_EXT_EndModemTest command
 *              to end the test.
 *
 *              Note: A LL reset will be issued by LL_EXT_EndModemTest!
 *              Note: The BLE device will transmit at maximum power.
 *
 * input parameters
 *
 * @param       rxFreq - Receiver RF channel k=0..39, where BLE F=2402+(k*2MHz).
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_EXT_ModemTestRx( uint8_t rxFreq );


/*******************************************************************************
 * @fn          LL_EXT_EndModemTest
 *
 * @brief       This API is used to shutdown a modem test. A complete link
 *              layer reset will take place.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_EXT_EndModemTest( void );


/*******************************************************************************
 * @fn          LL_EXT_SetBDADDR
 *
 * @brief       This API is used to set this device's BLE address (BDADDR).
 *
 *              Note: This command is only allowed when the device's state is
 *                    Standby.
 *
 * input parameters
 *
 * @param       bdAddr  - A pointer to a buffer to hold this device's address.
 *                        An invalid address (i.e. all FF's) will restore this
 *                        device's address to the address set at initialization.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_EXT_SetBDADDR( uint8_t *bdAddr );


/*******************************************************************************
 * @fn          LL_EXT_SetSCA
 *
 * @brief       This API is used to set this device's Sleep Clock Accuracy.
 *
 *              Note: For a slave device, this value is directly used, but only
 *                    if power management is enabled. For a master device, this
 *                    value is converted into one of eight ordinal values
 *                    representing a SCA range, as specified in Table 2.2,
 *                    Vol. 6, Part B, Section 2.3.3.1 of the Core specification.
 *
 *              Note: This command is only allowed when the device is not in a
 *                    connection.
 *
 *              Note: The device's SCA value remains unaffected by a HCI_Reset.
 *
 * input parameters
 *
 * @param       scaInPPM - This device's SCA in PPM from 0..500.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_EXT_SetSCA( uint16_t scaInPPM );


/*******************************************************************************
 * @fn          LL_EXT_SetFreqTune
 *
 * @brief       This API is used to set the Frequncy Tuning up or down. If the
 *              current setting is already at the max/min value, then no
 *              update is performed.
 *
 *              Note: This is a Production Test Mode only command!
 *
 * input parameters
 *
 * @param       step - LL_EXT_SET_FREQ_TUNE_UP or LL_EXT_SET_FREQ_TUNE_DOWN
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetFreqTune( uint8_t step );


/*******************************************************************************
 * @fn          LL_EXT_SaveFreqTune
 *
 * @brief       This API is used to save the current Frequency Tuning value to
 *              flash memory. It is restored on reboot or wake from sleep.
 *
 *              Note: This is a Production Test Mode only command!
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_EXT_SaveFreqTune( void );


/*******************************************************************************
 * @fn          LL_EXT_SetMaxDtmTxPower Vendor Specific API
 *
 * @brief       This function is used to set the max RF TX power to be used
 *              when using Direct Test Mode.
 *
 * input parameters
 *
 * @param       txPower - LL_EXT_TX_POWER_MINUS_23_DBM,
 *                        LL_EXT_TX_POWER_MINUS_6_DBM,
 *                        LL_EXT_TX_POWER_0_DBM,
 *                        LL_EXT_TX_POWER_4_DBM
 *
 * output parameters
 *
 * @param       cmdComplete - Boolean to indicate the command is still pending.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_SetMaxDtmTxPower( uint8_t txPower );



/*******************************************************************************
 * @fn          LL_EXT_DisconnectImmed Vendor Specific API
 *
 * @brief       This function is used to disconnect the connection immediately.
 *
 *              Note: The connection (if valid) is immediately terminated
 *                    without notifying the remote device. The Host is still
 *                    notified.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_EXT_DisconnectImmed( uint16_t connId );

/*******************************************************************************
 * @fn          LL_EXT_PacketErrorRate Vendor Specific API
 *
 * @brief       This function is used to Reset or Read the Packet Error Rate
 *              counters for a connection. When Reset, the counters are cleared;
 *              when Read, the total number of packets received, the number of
 *              packets received with a CRC error, the number of events, and the
 *              number of missed events are returned via a callback.
 *
 *              Note: The counters are only 16 bits. At the shortest connection
 *                    interval, this provides a bit over 8 minutes of data.
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID on which to send this data.
 * @param       command - LL_EXT_PER_RESET, LL_EXT_PER_READ
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_EXT_PacketErrorRate( uint16_t connId, uint8_t command );

/*******************************************************************************
 * @fn          LL_EXT_PERbyChan Vendor Specific API
 *
 * @brief       This API is called by the HCI to start or end Packet Error Rate
 *              by Channel counter accumulation for a connection. If the
 *              pointer is not NULL, it is assumed there is sufficient memory
 *              for the PER data, per the type perByChan_t. If NULL, then
 *              the operation is considered disabled.
 *
 *              Note: It is the user's responsibility to make sure there is
 *                    sufficient memory for the data, and that the counters
 *                    are cleared prior to first use.
 *
 *              Note: The counters are only 16 bits. At the shortest connection
 *                    interval, this provides a bit over 8 minutes of data.
 *
 * input parameters
 *
 * @param       connId    - The LL connection ID on which to send this data.
 * @param       perByChan - Pointer to PER by Channel data, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_EXT_PERbyChan( uint16_t connId, perByChan_t *perByChan );



/*******************************************************************************
 * @fn          LL_EXT_HaltDuringRf Vendor Specfic API
 *
 * @brief       This function is used to enable or disable halting the
 *              CPU during RF. The system defaults to enabled.
 *
 * input parameters
 *
 * @param       mode - LL_EXT_HALT_DURING_RF_ENABLE,
 *                     LL_EXT_HALT_DURING_RF_DISABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED,
 *              LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_HaltDuringRf( uint8_t mode );

/*******************************************************************************
 * @fn          LL_EXT_AdvEventNotice Vendor Specific API
 *
 * @brief       This API is called to enable or disable a notification to the
 *              specified task using the specified task event whenever a Adv
 *              event ends. A non-zero taskEvent value is taken to be "enable",
 *              while a zero valued taskEvent is taken to be "disable".
 *
 * input parameters
 *
 * @param       taskID    - User's task ID.
 * @param       taskEvent - User's task event.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_AdvEventNotice( uint8_t taskID, uint16_t taskEvent );

/*******************************************************************************
 * @fn          LL_EXT_ConnEventNotice Vendor Specific API
 *
 * @brief       This API is called to enable or disable a notification to the
 *              specified task using the specified task event whenever a
 *              Connection event ends. A non-zero taskEvent value is taken to
 *              be "enable", while a zero valued taskEvent is taken to be
 *              "disable".
 *
 *              Note: Currently, only a Slave connection is supported.
 *
 * input parameters
 *
 * @param       taskID    - User's task ID.
 * @param       taskEvent - User's task event.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_INACTIVE_CONNECTION,
 *              LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_ConnEventNotice( uint8_t taskID, uint16_t taskEvent );



/*******************************************************************************
 * @fn          LL_EXT_BuildRevision Vendor Specific API
 *
 * @brief       This API is used to to set a user revision number or read the
 *              build revision number.
 *
 * input parameters
 *
 * @param       mode       - LL_EXT_SET_USER_REVISION |
 *                           LL_EXT_READ_BUILD_REVISION
 * @param       userRevNum - A 16 bit value the user can set as their own
 *                           revision number
 *
 * output parameters
 *
 * @param       buildRev   - Pointer to returned build revision, if any.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_BuildRevision( uint8_t mode, uint16_t userRevNum, uint8_t *buildRev );


/*******************************************************************************
 * @fn          LL_EXT_DelaySleep Vendor Specific API
 *
 * @brief       This API is used to to set the sleep delay.
 *
 * input parameters
 *
 * @param       delay - 0 .. 1000, in milliseconds.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_DelaySleep( uint16_t delay );


/*******************************************************************************
 * @fn          LL_EXT_ResetSystem Vendor Specific API
 *
 * @brief       This API is used to to issue a soft or hard system reset.
 *
 * input parameters
 *
 * @param       mode - LL_EXT_RESET_SYSTEM_HARD | LL_EXT_RESET_SYSTEM_SOFT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_EXT_ResetSystem( uint8_t mode );


/*******************************************************************************
 * @fn          LL_EXT_OverlappedProcessing Vendor Specific API
 *
 * @brief       This API is used to enable or disable overlapped processing.
 *
 * input parameters
 *
 * @param       mode - LL_EXT_ENABLE_OVERLAPPED_PROCESSING |
 *                     LL_EXT_DISABLE_OVERLAPPED_PROCESSING
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_ERROR_CODE_INVALID_HCI_CMD_PARAMS
 */
extern llStatus_t LL_EXT_OverlappedProcessing( uint8_t mode );

/*******************************************************************************
 * @fn          LL_EXT_NumComplPktsLimit Vendor Specific API
 *
 * @brief       This API is used to set the minimum number of
 *              completed packets which must be met before a Number of
 *              Completed Packets event is returned. If the limit is not
 *              reach by the end of the connection event, then a Number of
 *              Completed Packets event will be returned (if non-zero) based
 *              on the flushOnEvt flag.
 *
 * input parameters
 *
 * @param       limit      - From 1 to LL_MAX_NUM_DATA_BUFFERS.
 * @param       flushOnEvt - LL_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT |
 *                           LL_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_ERROR_CODE_INVALID_HCI_CMD_PARAMS
 */
extern llStatus_t LL_EXT_NumComplPktsLimit( uint8_t limit,
                                            uint8_t flushOnEvt );


/*
**  LL Callbacks to HCI
*/

/*******************************************************************************
 * @fn          LL_ConnectionCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              a new connection has been created. For the Slave, this means
 *              a CONNECT_REQ message was received from an Initiator. For the
 *              Master, this means a CONNECT_REQ message was sent in response
 *              to a directed or undirected message addressed to the Initiator.
 *
 * input parameters
 *
 * @param       reasonCode    - LL_STATUS_SUCCESS or ?
 * @param       connId        - The LL connection ID for new connection.
 * @param       role          - LL_LINK_CONNECT_COMPLETE_MASTER or
 *                              LL_LINK_CONNECT_COMPLETE_SLAVE.
 * @param       peerAddrType  - Peer address type (public or random).
 * @param       peerAddr      - Peer address.
 * @param       connInterval  - Connection interval.
 * @param       slaveLatency  - The connection's Slave Latency.
 * @param       connTimeout   - The connection's Supervision Timeout.
 * @param       clockAccuracy - The sleep clock accurracy of the Master. Only
 *                              valid on the Slave. Set to 0x00 for the Master.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ConnectionCompleteCback( uint8_t  reasonCode,
                                        uint16_t connId,
                                        uint8_t  role,
                                        uint8_t  peerAddrType,
                                        uint8_t  *peerAddr,
                                        uint16_t connInterval,
                                        uint16_t slaveLatency,
                                        uint16_t connTimeout,
                                        uint8_t  clockAccuracy );

/*******************************************************************************
 * @fn          LL_DisconnectCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the connection has been terminated. The cause is given by the
 *              reason code.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID.
 * @param       reason - The reason the connection was terminated.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_DisconnectCback( uint16_t connId,
                                uint8_t  reason );

/*******************************************************************************
 * @fn          LL_ConnParamUpdateCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the update parameters control procedure has completed. It is
 *              always made to the Master's Host when the update request has
 *              been sent. It is only made to the Slave's Host when the update
 *              results in a change to the connection interval, and/or the
 *              connection latency, and/or the connection timeout.
 *
 * input parameters
 *
 * @param       connId       - The LL connection ID.
 * @param       connInterval - Connection interval.
 * @param       connLatency  - The connection's Slave Latency.
 * @param       connTimeout  - The connection's Supervision Timeout.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ConnParamUpdateCback( uint16_t connId,
                                     uint16_t connInterval,
                                     uint16_t connLatency,
                                     uint16_t connTimeout );

/*******************************************************************************
 * @fn          LL_ReadRemoteVersionInfoCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host the
 *              requested peer's Version information.
 *
 * input parameters
 *
 * @param       status    - Status of callback.
 * @param       connId    - The LL connection ID.
 * @param       verNum    - Version of the Bluetooth Controller specification.
 * @param       comId     - Company identifier of the manufacturer of the
 *                          Bluetooth Controller.
 * @param       subverNum - A unique value for each implementation or revision
 *                          of an implementation of the Bluetooth Controller.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ReadRemoteVersionInfoCback( uint8_t  status,
                                           uint16_t connId,
                                           uint8_t  verNum,
                                           uint16_t comId,
                                           uint16_t subverNum );

/*******************************************************************************
 * @fn          LL_EncChangeCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              an encryption change has taken place. This results when
 *              the host performs a LL_StartEncrypt when encryption is not
 *              already enabled.
 *
 *              Note: If the key request was rejected, then encryption will
 *                    remain off.
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       reason  - LL_ENC_KEY_REQ_ACCEPTED or LL_ENC_KEY_REQ_REJECTED.
 * @param       encEnab - LL_ENCRYPTION_OFF or LL_ENCRYPTION_ON.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncChangeCback( uint16_t connId,
                               uint8_t  reason,
                               uint8_t  encEnab );

/*******************************************************************************
 * @fn          LL_EncKeyRefreshCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              an encryption key change has taken place. This results when
 *              the host performs a LL_StartEncrypt when encryption is already
 *              enabled.
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       reason  - LL_ENC_KEY_REQ_ACCEPTED.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncKeyRefreshCback( uint16_t connId,
                                   uint8_t  reason );

/*******************************************************************************
 * @fn          LL_AdvReportCback Callback
 *
 * @brief       This Callback is used by the LL to provide information about
 *              advertisers from which an advertising packet was received.
 *
 * input parameters
 *
 * @param       eventType   - Type of advertisement packet received by Scanner
 *                            or Initiator, and scan response for Initiator.
 * @param       advAddrType - Advertiser address type (public or random).
 * @param       advAddr     - Advertiser address.
 * @param       dataLen     - Size in bytes of advertisement packet.
 * @param       data        - Advertisement data.
 * @param       rssi        - RSSI value (-127..20dBm), or not available
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_AdvReportCback( uint8_t eventType,
                               uint8_t advAddrType,
                               uint8_t *advAddr,
                               uint8_t dataLen,
                               uint8_t *data,
                               int8_t  rssi );

void LL_AdvSetTerminatedCback(uint8_t          status,
                              uint8_t   adv_handle,
                              uint16_t  connHandle,                      
                              uint8_t   Num_Completed_Extended_Advertising_Events);  

extern void LL_ExtAdvReportCback( uint8_t advEvt,
                        uint8_t advAddrType,
                        uint8_t *advAddr,
                        uint8_t   primaryPHY,
                        uint8_t   secondaryPHY,
                        uint8_t   advertisingSID,
                        uint8_t   txPower,
                        int8_t    rssi,
                        uint16_t  periodicAdvertisingInterval,
                        uint8_t   directAddrType,
                        uint8_t   *directAddr,                
                        uint8_t   dataLen,
                        uint8_t   *rptData);

void LL_PrdAdvReportCback(uint16_t syncHandle,
                                uint8_t txPower,
                                uint8_t rssi,
                                uint8_t cteType,
                                uint8_t dataStatus,
                                uint8_t dataLength,
                                uint8_t *data
                              );

void LL_PrdAdvSyncEstablishedCback(uint8_t        status,
                                           uint16_t  syncHandle,
                                           uint8_t   advertisingSID,
                                           uint8_t   advertiserAddressType,
                                           uint8_t   *advertiserAddress,
                                           uint8_t   advertiserPHY,
                                           uint16_t  periodicAdvertisingInterval,
                                           uint8_t   advertiserClockAccuracy
                                           );

void LL_PrdAdvSyncLostCback(uint16_t  syncHandle);

void LL_ChannelSelectionAlgorithmCback(uint16_t connHandle,
                                                 uint8_t  chnSel
                                                );

void LL_EnhConnectionCompleteCback( uint8_t  reasonCode,
                                 uint16_t connHandle,
                                 uint8_t  role,
                                 uint8_t  peerAddrType,
                                 uint8_t  *peerAddr,
                                 uint8_t  *localRpaAddr,
                                 uint8_t  *peerRpaAddr,
                                 uint16_t connInterval,
                                 uint16_t slaveLatency,
                                 uint16_t connTimeout,
                                 uint8_t  clockAccuracy );
  
/******************************************************************************
 * fn:	LL_ConnectionlessIQReportCback
 * 
 * brief:	1、usd by the controller to report IQ Information from the CTE of the 
 *			received advertising packet
 *			2、report IQ Information from the CTE of a received Test Mode packet
 * 
 * date:2020-01-14
 * 
 * input parameters:
 *			syncHandle		:	Identifying the periodic advertising train
 *			chan_idx		:	the index of the channel on which the packet has received
 *			rssi			:	rssi of the packet , units 0.1 dBm
 *			rssi_antID		:	Antenna ID
 *			cte_type		:	AOA/AOD CTE Type, AOD with 1us or 2us slots
 *			slot_duration	:	switching and sampling slots with 1us or 2us
 *			packet_status	:	indicates whether the received packet had a valid CRC
 *								and if not , whether the controller has determined the 
 *								position and size of the CTE
 *			PE_Cnt			:	the value of paEventCounter
 *			sampCnt			:	total number of sample pairs
 *			ISample			: 	the list of the I Sample of the report packets
 *			QSample			:	the list of the Q Sample of the report packets
 * 
 * 
 * output parameters:
 * 
 * Note:	Controller shall not generate this event for packets that have a bad CRC
 * 
 * return		hciStatus_t
 * 
 ******************************************************************************/
 void LL_ConnectionlessIQReportCback(			uint16_t syncHandle,
												uint8_t  chan_idx,
												int16_t  rssi,
												uint8_t  rssi_antID,
												uint8_t  cte_type,
												uint8_t  slot_duration,
												uint8_t  packet_status,
												uint16_t PE_Cnt,
												uint8_t  sampCnt,
												uint16_t  *ISample,
												uint16_t  *QSample);


/*****************************************************************************************
 * fn:	LL_ConnectionIQReportCback
 * 
 * date:2020-01-14
 * 
 * brief:	used by the controller to report the IQ samples from the CTE of a received packet.
 * 
 * input parameters:
 *			connHandle	:	identifies the connections that corresponds to the reported information
 * 			rx_PHY		:	receiver PHY for the connection 1M or 2M
 *			data_chan_idx:	the index of data channel on which the data physical channel PDU has received
 *			rssi		:	rssi of the packet , units 0.1 dBm
 *			rssi_antID	:	id of the antenna on which the RSSI is measured
 *			cte_type	:	AOA/AOD CTE Type, AOD with 1us or 2us slots
 *			slot_duration:	switching and sampling slots with 1us or 2us
 *			packet_status:	indicates whether the received packet had a valid CRC
 *							and if not , whether the controller has determined the 
 *							position and size of the CTE
 *			connEventCounter:the value of connection event counter
 *			sampCnt		:	total number of sample pairs
 *			ISample		: 	the list of the I Sample of the report packets
 *			QSample		:	the list of the Q Sample of the report packets
 *
 * 
 * output parameters:
 * 
 * 
 * return		hciStatus_t
 * 
 *****************************************************************************************/
void LL_ConnectionIQReportCback(			uint16_t connHandle,
											uint8_t  rx_PHY,
											uint8_t  data_chan_idx,
											int16_t  rssi,
											uint8_t  rssi_antID,
											uint8_t  cte_type,
											uint8_t  slot_duration,
											uint8_t  packet_status,
											uint16_t connEventCounter,
											uint8_t  sampCnt,
											uint16_t  *ISample,
											uint16_t  *QSample);


/*****************************************************************************************
 * fn:	LL_CTE_Report_FailedCback
 * 
 * date:2020-01-14
 * 
 * brief:	used by the controller to report an issue following a request to a peer device
 *			to reply with a packet containing an LL_CTE_RSP PDU and a CTE
 *			
 * 
 * input parameters:
 * 			status		:	received LL_CTE_RSP PDU status
 *			connHandle	:	connection handle
 * 
 * output parameters:
 * 
 * 
 * return		hciStatus_t
 * 
 *****************************************************************************************/
void LL_CTE_Report_FailedCback(	uint8_t status,uint16_t connHandle);

/*******************************************************************************
 * @fn          LL_ReadRemoteUsedFeaturesCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the Read Remote Feature Support command as completed.
 *
 * input parameters
 *
 * @param       status      - SUCCESS or control procedure timeout.
 * @param       connId      - The LL connection ID for new connection.
 * @param       featureSet  - A pointer to the Feature Set.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ReadRemoteUsedFeaturesCompleteCback( uint8_t  status,
                                                    uint16_t connId,
                                                    uint8_t  *featureSet );



/*******************************************************************************
 * @fn          LL_EncLtkReqCback Callback
 *
 * @brief       This Callback is used by the LL to provide to the Host the
 *              Master's random number and encryption diversifier, and to
 *              request the Host's Long Term Key (LTK).
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       randNum - Random vector used in device identification.
 * @param       encDiv  - Encrypted diversifier.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncLtkReqCback( uint16_t connId,
                               uint8_t  *randNum,
                               uint8_t  *encDiv );


/*******************************************************************************
 * @fn          LL_DirectTestEndDone Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the
 *              Direct Test End command has completed.
 *
 *
 * input parameters
 *
 * @param       numPackets - The number of packets received. Zero for transmit.
 * @param       mode       - LL_DIRECT_TEST_MODE_TX or LL_DIRECT_TEST_MODE_RX.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern void LL_DirectTestEndDoneCback( uint16_t numPackets,
                                       uint8_t  mode );
                                       
/*******************************************************************************
 * @fn          LL_DataLengthChange Callback
 *
 *
 */

extern void LL_DataLengthChangeCback(uint16_t connHandle,
                                        uint16_t MaxTxOctets,
                                        uint16_t MaxTxTime,
                                        uint16_t MaxRxOctets,
                                        uint16_t MaxRxTime);
                                        

/*******************************************************************************
 * @fn          LL_TxDataCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the HCI that
 *              the HCI's buffer is free for its own use again.
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which to send this data.
 * @param       *pBuf    - A pointer to the data buffer to transmit, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 ******************************************************************************/
extern void LL_TxDataCompleteCback( uint16_t connId,
                                    uint8_t  *pBuf );

/*******************************************************************************
 * @fn          LL_RxDataCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the HCI that
 *              data has been received and placed in the buffer provided by
 *              the HCI.
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which data was received.
 * @param       *pBuf    - A pointer to the receive data buffer provided by
 *                           the HCI.
 * @param       len      - The number of bytes received on this connection.
 * @param       fragFlag - LL_DATA_FIRST_PKT indicates buffer is the start of
 *                           a Host packet.
 *                         LL_DATA_CONTINUATION_PKT: Indicates buffer is a
 *                           continuation of a Host packet.
 * @param       rssi     - The RSSI of this received packet as a signed byte.
 *                         Range: -127dBm..+20dBm, 127=Not Available.
 *
 * output parameters
 *
 * @param       **pBuf   - A double pointer updated to the next receive data
 *                         buffer, or NULL if no next buffer is available.
 *
 * @return      None.
 */
extern void LL_RxDataCompleteCback( uint16_t connId,
                                    uint8_t  *ppBuf,
                                    uint8_t  len,
                                    uint8_t  fragFlag,
                                    int8_t   rssi );



/*******************************************************************************
 * @fn          LL_RandCback API
 *
 * @brief       This Callback is used by the LL to notify the HCI that the true
 *              random number command has been completed.
 *
 *              Note: The length is always given by B_RANDOM_NUM_SIZE.
 *
 * input parameters
 *
 * @param       *randData - Pointer to buffer to place a random block of data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_RandCback( uint8_t *randData );


/*******************************************************************************
 * @fn          LL_EXT_SetRxGainCback Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the set
 *              RX gain command has been completed.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EXT_SetRxGainCback( void );


/*******************************************************************************
 * @fn          LL_EXT_SetTxPowerCback Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the set
 *              TX power command has been completed.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EXT_SetTxPowerCback( void );


/*******************************************************************************
 * @fn          LL_EXT_PacketErrorRateCback Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the
 *              Packet Error Rate Read command has been completed.
 *
 *              Note: The counters are only 16 bits. At the shortest connection
 *                    interval, this provides a bit over 8 minutes of data.
 *
 * input parameters
 *
 * @param       numPkts   - Number of Packets received.
 * @param       numCrcErr - Number of Packets received with a CRC error.
 * @param       numEvents - Number of Connection Events.
 * @param       numPkts   - Number of Missed Connection Events.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EXT_PacketErrorRateCback( uint16_t numPkts,
                                         uint16_t numCrcErr,
                                         uint16_t numEvents,
                                         uint16_t numMissedEvts );


/*******************************************************************************
 * @fn          LL_EXT_ExtendRfRangeCback Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the
 *              Extend Rf Range command has been completed.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
//extern void LL_EXT_ExtendRfRangeCback( void );

/*******************************************************************************
 * @fn          LL_PLUS_PerStats_Init
 *
 * @brief       Used to init linklayer per stats
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStats_Init(perStatsByChan_t* p_per);
/*******************************************************************************
 * @fn          LL_PLUS_PerStatsReset
 *
 * @brief       Used to reset linklayer per stats
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStatsReset(void);


/*******************************************************************************
 * @fn          LL_PLUS_PerStasReadByChn
 *
 * @brief       read per stats by data channel id
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStasReadByChn(uint8_t chnId,perStats_t * perStats);

extern LL_PLUS_AdvDataFilterCB_t LL_PLUS_AdvDataFilterCBack;
extern void LL_PLUS_SetAdvDataFilterCB(LL_PLUS_AdvDataFilterCB_t AdvDataFilterCBack);
extern uint8_t* LL_PLUS_GetAdvDataExtendData(void);
extern void LL_PLUS_SetScanRequestData(uint8_t dLen,uint8_t* pData);


extern LL_PLUS_ScanRequestFilterCB_t LL_PLUS_ScanRequestFilterCBack;
extern void LL_PLUS_SetScanRequestFilterCB(LL_PLUS_ScanRequestFilterCB_t ScanRequestFilterCBack);
extern uint8_t LL_PLUS_GetScanRequestExtendData(uint8_t* pData);
extern void LL_PLUS_GetScanerAddr(uint8_t* pData);
extern void LL_PLUS_SetScanRsqData(uint8_t dLen,uint8_t* pData);

extern void LL_PLUS_SetScanRsqDataByIndex(uint8_t dIdx,uint8_t data);



//DLE
extern llStatus_t LL_SetDataLengh( uint16_t connId,uint16_t TxOctets,uint16_t TxTime );
//extern uint8_t LL_PLUS_GetLocalPduDataLength(ll_pdu_length_ctrl_t* pduLen);
extern llStatus_t LL_WriteSuggestedDefaultDataLength(uint16_t TxOctets,uint16_t TxTime);
extern void LL_DataLengthChangeCback(   uint16_t connHandle,
                                                    uint16_t MaxTxOctets,
                                                    uint16_t MaxTxTime,
                                                    uint16_t MaxRxOctets,
                                                    uint16_t MaxRxTime);


                                            
//PHY UPDATE

extern llStatus_t LL_SetDefaultPhyMode( uint16_t connId,uint8_t allPhy,uint8_t txPhy, uint8_t rxPhy);
extern llStatus_t LL_SetPhyMode( uint16_t connId,uint8_t allPhy,uint8_t txPhy, uint8_t rxPhy,uint16_t phyOptions);


extern llStatus_t LL_PhyUpdate( uint16_t connId );
extern void LL_PhyUpdateCompleteCback(  uint16_t connHandle,
                                                    uint8_t status,
                                                    uint8_t txPhy,
                                                    uint8_t rxPhy);

// Resolving list
extern llStatus_t LL_AddResolvingListLDevice( uint8_t  addrType,
                                                     uint8_t *devAddr, 
                                                     uint8_t *peerIrk,
                                                     uint8_t *localIrk);
extern llStatus_t LL_RemoveResolvingListDevice( uint8_t *devAddr,
                                         uint8_t addrType );

extern llStatus_t LL_ClearResolvingList( void );

extern llStatus_t LL_ReadPeerResolvableAddress( uint8_t *peerRpa );

extern llStatus_t LL_ReadLocalResolvableAddress( uint8_t *localRpa );

extern llStatus_t LL_ReadResolvingListSize( uint8_t *numEntries );

extern llStatus_t LL_SetAddressResolutionEnable( uint8_t enable );


extern llStatus_t LL_SetResolvablePrivateAddressTimeout( uint16_t rpaTimeout );

extern llStatus_t LL_PLUS_DisableSlaveLatency(uint8_t connId);

extern llStatus_t LL_PLUS_EnableSlaveLatency(uint8_t connId);

// extended advertisement
llStatus_t LL_InitExtendedAdv( extAdvInfo_t *extAdvInfo,
	                                uint8_t         extAdvNumber,
	                                uint16_t        advSetMaxLen);
llStatus_t LL_SetExtAdvSetRandomAddress( uint8_t adv_handle,
                                            uint8_t* random_address);
llStatus_t LL_SetExtAdvParam( uint8_t adv_handle,
                                   uint16_t adv_event_properties,
                                   uint32_t primary_advertising_interval_Min,          // 3 octets
                                   uint32_t primary_advertising_interval_Max,          // 3 octets
                                   uint8_t  primary_advertising_channel_map,
                                   uint8_t  own_address_type,
                                   uint8_t  peer_address_type,
                                   uint8_t *peer_address,
                                   uint8_t  advertising_filter_policy,
                                   int8_t   advertising_tx_power,
                                   uint8_t  primary_advertising_PHY,
                                   uint8_t  secondary_advertising_max_skip,
                                   uint8_t  secondary_advertising_PHY,
                                   uint8_t  advertising_SID,
                                   uint8_t  scan_request_notification_enable,
                                   int8_t  *selectTxPwr);
llStatus_t LL_SetExtAdvData( uint8_t adv_handle,
                                   uint8_t operation,
                                   uint8_t  fragment_preference,
                                   uint8_t  advertising_data_length,
                                   uint8_t *advertising_data);
llStatus_t LL_SetExtScanRspData( uint8_t adv_handle,
                                   uint8_t operation,
                                   uint8_t  fragment_preference,
                                   uint8_t  scan_rsp_data_length,
                                   uint8_t *scan_rsp_data);
llStatus_t LL_SetExtAdvEnable(uint8_t  enable,
                                   uint8_t  number_of_sets,
                                   uint8_t  *advertising_handle,
                                   uint16_t *duration,
                                   uint8_t  *max_extended_advertising_events);
llStatus_t LL_ReadMaximumAdvDataLength( uint16_t *length );
llStatus_t LL_ReadNumberOfSupportAdvSet( uint8_t *number );
llStatus_t LL_RemoveAdvSet( uint8_t adv_handle);
llStatus_t LL_ClearAdvSets(void);

llStatus_t LL_SetExtendedScanParameters(uint8_t own_address_type,
                                                uint8_t scanning_filter_policy,
                                                uint8_t scanning_PHYs,
                                                uint8_t *scan_type,
                                                uint16_t *scan_interval,
                                                uint16_t *scan_window);
llStatus_t LL_SetExtendedScanEnable(uint8_t enable,
                                           uint8_t filter_duplicates,
                                           uint16_t duration,
                                           uint16_t period);
llStatus_t LL_ExtendedCreateConnection(uint8_t initiator_filter_policy,
                                              uint8_t own_address_type,
                                              uint8_t peer_address_type,
                                              uint8_t* peer_address,
                                              uint8_t initiating_PHYs,
                                              uint16_t *scan_interval,
                                              uint16_t *scan_window,
                                              uint16_t *conn_interval_min,
                                              uint16_t *conn_interval_max,
                                              uint16_t *conn_latency,
                                              uint16_t *supervision_timeout,
                                              uint16_t *minimum_CE_length,
                                              uint16_t *maximum_CE_length);
                                           

// extended adv
void llSetupAdvExtIndPDU(extAdvInfo_t  *pAdvInfo, periodicAdvInfo_t *pPrdAdv);

void llSetupAuxAdvIndPDU(extAdvInfo_t  *pAdvInfo, periodicAdvInfo_t *pPrdAdv);

void llSetupAuxChainIndPDU(extAdvInfo_t  *pAdvInfo, periodicAdvInfo_t *pPrdAdv);

void llSetupAuxSyncIndPDU(extAdvInfo_t  *pAdvInfo, periodicAdvInfo_t *pPrdAdv);

void llSetupAuxConnectReqPDU(void);

void llSetupAuxConnectRspPDU(extAdvInfo_t  *pAdvInfo);

void llSetupAuxScanRspPDU(extAdvInfo_t  *pAdvInfo);

uint8_t ll_isLegacyAdv(extAdvInfo_t *pExtAdv);

/*******************************************************************************
 * @fn          LL_InitConnectContext
 *
 * @brief       This function initialize the LL connection-orient context
 *
 * input parameters
 *
 * @param       pConnContext   - connection-orient context, the memory is allocated by application
 *              maxConnNum     - the size of connect-orient context
 *              maxPktPerEventTx/Rx - number of packets transmit/receive per connection event
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
llStatus_t LL_InitConnectContext(llConnState_t    *pConnContext,
                                    uint8_t *pConnBuffer,
	                                uint8_t maxConnNum, 
	                                uint8_t maxPktPerEventTx,
	                                uint8_t maxPktPerEventRx,
	                                uint8_t blePktVersion);

// extended scan
llStatus_t LL_InitExtendedScan(uint8_t *scanDataBuffer,
	                                uint16_t scanDataBufferLength);

llStatus_t LL_InitPeriodicAdv(extAdvInfo_t *extAdvInfo,
                                    periodicAdvInfo_t *periodicAdvInfo,
	                                uint8_t            periodicAdvSetNumber,
	                                uint16_t           advSetMaxLen);


// Periodic Adv    
llStatus_t LL_SetPeriodicAdvParameter(uint8_t adv_handle,
                                              uint16_t interval_min,
                                              uint16_t interval_max,
                                              uint16_t adv_event_properties);


llStatus_t  LL_SetPeriodicAdvData(uint8_t adv_handle,
                                        uint8_t operation,
                                        uint8_t  advertising_data_length,
                                        uint8_t *advertising_data);

   
													   
llStatus_t LL_SetPeriodicAdvEnable(uint8_t   enable,
                                         uint8_t  advertising_handle);


// periodic scan
llStatus_t LL_PeriodicAdvertisingCreateSync(uint8_t options,
                                                     uint8_t advertising_SID,
                                                     uint8_t advertiser_Address_Type,
                                                     uint8_t *advertiser_Address,
                                                     uint16_t skip,
                                                     uint16_t sync_Timeout,
                                                     uint8_t sync_CTE_Type);	

llStatus_t LL_PeriodicAdvertisingCreateSyncCancel(void);			

llStatus_t LL_PeriodicAdvertisingTerminateSync(          uint16_t sync_handle);	

// Periodic advertiser list
extern llStatus_t LL_AddDevToPeriodicAdvList(uint8_t  addrType,
                                                     uint8_t *devAddr, 
                                                     uint8_t sid);
extern llStatus_t LL_RemovePeriodicAdvListDevice(uint8_t  addrType,
                                                          uint8_t *devAddr, 
                                                          uint8_t sid);

extern llStatus_t LL_ClearPeriodicAdvList( void );

extern llStatus_t LL_ReadPeriodicAdvListSize( uint8_t *numEntries );

  
/*****************************************************************************************
 * fn:	LL_ConnectionlessCTE_TransmitParamCmd
 * 
 * date:2020-01-15
 * 
 * brief:	set CTE Parameters in any periodic advertising 
 *				1、CTE Type
 *				2、CTE Length
 *				3、CTE antenna switching pattern
 *
 * input parameters:
 * 			advertising handle		: Identify advertising set 0x0-0xEF
 *			CTE_Length				: CTE Length in 8us 0x2-0x14
 *			CTE_Type				: 0:AOA CTE , 1:AoD CTE with 1us,2:AoD CTE with 2us,
 *			CTE_Count				: how many CTE packet in each PA event 0x1-0x10
 *			Switch_Pattern_LEN		: number of Antenna IDs in the pattern
 *									: AOD CTE, AOA shall be ignored
 *									: 0x2-0x4B
 *			Antenna_IDs[i]			: List of Antenna IDs in the pattern
 *									: AOD CTE, AOA shall be ignored
 * 
 * output parameters:
 *				Status				:LL_STATUS_SUCCESS or other error codes
 * 
 * 
 * return		LL_STATUS_SUCCESS or other error codes
 *  
 *****************************************************************************************/
llStatus_t LL_ConnectionlessCTE_TransmitParam(	 			 uint8_t advertising_handle,
															 uint8_t len,
															 uint8_t type,
															 uint8_t count,
															 uint8_t Pattern_LEN,
															 uint8_t *AnaIDs);

  
/*****************************************************************************************
 * fn:	LL_ConnectionlessCTE_TransmitEnable
 * 
 * date:2020-01-16
 * 
 * brief:	Controller enable or disable CTE in PA
 * 
 * input parameters:
 * 			advertising handle		: Identify advertising set in which CTE is enable or disable 
 *									: 0x0-0xEF
 *			enable					: 0 : disable , 1: enable
 * 
 * 
 * output parameters:
 *				Status				:LL_STATUS_SUCCESS or other error codes
 * 
 * 
 * return		LL_STATUS_SUCCESS or other error codes
 * 
 *****************************************************************************************/
llStatus_t LL_ConnectionlessCTE_TransmitEnable(			  	uint8_t advertising_handle,
														  	uint8_t enable);

  
/*****************************************************************************************
 * fn:	LL_ConnectionlessIQ_SampleEnable
 * 
 * date:2020-01-17
 * 
 * brief:	Controller enable or disable capturing IQ Samples from the CTE of PA pcakets
 * 
 * input parameters:
 *				sync_handle		: 	periodic advertising handle
 *									Range:0x0 - 0x0EFF
 *				slot_Duration	:	switching and sampling slot 0x1:1us,0x2:2us,Other:RFU
 *				enable			:	0x0:IQ Sampling disable, 0x1:IQ Sampling enable
 *				MaxSampledCTEs	:	max number of CTE in each PA event that the controller 
 *									should collect and report
 *									Range	:	0x0-0x10
 *										0x0	:	sample and report all available CTE
 *				pattern_len		:	number of Antenna IDs in the pattern
 *									Range:0x2 - 0x4B
 *				AnaIDs			:	list of Antenna IDs in the pattern
 * 
 * 
 * output parameters:
 *				status			:	LL_STATUS_SUCCESS or other error codes
 *				sync_handle		: 	Periodic advertising handle
 * 
 * 
 * return		LL_STATUS_SUCCESS or other error codes
 * 

 *****************************************************************************************/
llStatus_t LL_ConnectionlessIQ_SampleEnable(			 uint16_t sync_handle,
														 uint8_t enable,
														 uint8_t slot_Duration,
														 uint8_t MaxSampledCTEs,
														 uint8_t pattern_len,
														 uint8_t *AnaIDs);

  
/*****************************************************************************************
 * fn:	LL_Set_ConnectionCTE_ReceiveParam
 * 
 * date:2020-01-19
 * 
 * brief:	enable or disable sampling received CTE fields on the connection
 *			set antenna switching pattern 
 *			set switching and sampling slot durations
 * 
 * input parameters:
 *			connHandle	:	connection handle Range 0x0 - 0x0EFF
 *			enable		:	sampling enable 0:disable , 1:enable
 *			slot_Duration	: switching and sampling slot 0:1us, 1: 2us
 *			pattern_len	:	the number of Antenna IDs in the pattern 
 *							Range: 0x2-0x4B
 *			AnaIDs		:	list of Antenna IDs in the pattern
 * 
 * 
 * output parameters:
 *				Status		:	LL_STATUS_SUCCESS or other error codes
 *				connHandle	:	Connection Handle
 * 
 * 
 * return		llStatus_t

 * 
 *****************************************************************************************/
llStatus_t LL_Set_ConnectionCTE_ReceiveParam(			 	  uint16_t connHandle,
															  uint8_t enable,
															  uint8_t slot_Duration,
															  uint8_t pattern_len,
															  uint8_t *AnaIDs);

  
/*****************************************************************************************
 * fn:	LL_Connection_CTE_Request_Enable
 * 
 * date:2020-01-19
 * 
 * brief:	request Controller to start or stop initiating the CTE request 
 *			procedure on connection
 * 
 * input parameters:
 *			connHandle	:	connection Handle
 *							Range:0x0 - 0x0EFF
 *			enable		:	Enable or disable CTE request for the connection
 *							0:disable,1:enable
 *			Interval	:	define whether the CTE request procedure is initiated 
 *							only once or periodically.
 *							Range:0x0 - 0xFFFF
 *							0x0	:	Initiate the CTE request procedure once
 *							0x1 - 0xFFFF :	Requested interval for initiating the CTE
 *											procedure in number of connection events
 *							Range:
 *			len			:	minimum length of the CTE in 8us units
 *							Range: 0x2 - 0x14
 *			type		:	indicate the type of CTE that the controller shall 
 *							request from the remote device
 *							0x0:AOA CTE
 *							0x1:AOD CTE with 1us
 *							0x2:AOD CTE with 2us
 * 
 * 
 * output parameters:
 *			Status		:	0x0 : command succeed , 0x1 - 0xff : other error code
 *			connHandle	:	connection handle
 * 
 * 
 * return		llStatus_t

 * 
 *****************************************************************************************/
llStatus_t LL_Connection_CTE_Request_Enable(					uint16_t connHandle,
																uint8_t enable,
																uint16_t Interval,
																uint8_t len,
																uint8_t type);


  
/*****************************************************************************************
 * fn:	LL_Set_ConnectionCTE_TransmitParam
 * 
 * date:2000-01-19
 * 
 * brief:	used to set the antenna switching pattern and permitted CTE type
 * 
 * input parameters:
 *			connHandle	:	connection Handle, Range: 0x0 - 0x0EFF
 *			type		:	bit set for CTE type , 	bit 0 : AOA CTE response, 
 *													bit 1 : AOD CTE response with 1us slots
 *													bit 2 : AOD CTE response with 2us slots
 *			pattern_len	:	the number of Antenna IDs in the pattern
 *			AnaIDs		:	list of Antenna IDs in the pattern
 * 
 * 
 * output parameters:
 *			Status		:	0 : success, other error code
 *			ConnHandle	:	connection handle
 * 
 * 
 * return		llStatus_t
 * 

 *****************************************************************************************/
llStatus_t LL_Set_ConnectionCTE_TransmitParam(				 	uint16_t connHandle,
																 uint8_t type,
																 uint8_t pattern_len,
																 uint8_t *AnaIDs);

  
/*****************************************************************************************
 * fn:	LL_Connection_CTE_Response_Enable
 * 
 * date:2020-01-19
 * 
 * brief:	request the controller to respond to LL_CTE_REQ with LL_CTE_RSP on the 
 *			specified connection
 * 
 * input parameters:
 *			connHandle	:	connection Handle
 *							Range:0x0 - 0x0EFF
 *			enable		:	enable or disable CTE response for the connection
 * 
 * 
 * output parameters:
 *			status		: 	0x0 : command succeed , 0x1 - 0xff : other error code
 *			connHandle	:	connection handle
 * 
 * 
 * 
 * return		llStatus_t
 * 

 *****************************************************************************************/
llStatus_t LL_Connection_CTE_Response_Enable(				  uint16_t connHandle,uint8_t enable);

  
/*****************************************************************************************
 * fn:	HCI_LE_READ_Anatenna_InfoCmd
 * 
 * date:2020-01-19
 * 
 * brief:	Host read the switching rates, the sampling reate, the number of antennae,
 *			and the maxumum length of a transmitted CTE supported by the controller
 * 
 * input parameters:	
 *				None
 * 
 * 
 * output parameters:
 *				status				: 	0x0 : command succeed , 0x1 - 0xff : other error code
 *				switch_sample_rate	:	bit number indicate supported switching and sampling rate
 *										bit 0 :  1us switching AOD transmission
 *										bit 1 :	 1us sampling AOD reception
 *										bit 2 :	 1us switching and sampling AOA reception
 *				Antenna_len			:	number of Antennae supported by the controller
 *				MAX_Pattern_len		: 	MAX length of antenna switching pattern spooorted by the controller
 *				MAX_CTE_LEN			: 	MAX length or a transmitted CTE supported in 8us units
 * 
 * 
 * return		llStatus_t
 * 

 *****************************************************************************************/
llStatus_t LL_READ_Anatenna_Info(           uint8_t *param );


// RF path compensation configuration
llStatus_t LL_Read_Rf_Path_Compensation(uint8_t *param);

llStatus_t LL_Write_Rf_Path_Compensation( int16_t tx_compensation, int16_t rx_compensation);

llStatus_t LL_Set_Privacy_Mode(uint8_t  peerIdType,
                                    uint8_t *peerIdAddr, 
                                    uint8_t privacyMode);

llStatus_t LL_Read_Transmit_Power( uint8_t *param);

llStatus_t LL_SetAdvParam0( uint16_t advIntervalMin,
                            uint16_t advIntervalMax,
                            uint8_t  advEvtType,
                            uint8_t  ownAddrType,
                            uint8_t  directAddrType,
                            uint8_t*  directAddr,
                            uint8_t  advChanMap,
                            uint8_t  advWlPolicy );

llStatus_t LL_SetScanParam0( uint8_t  scanType,
                             uint16_t scanInterval,
                             uint16_t scanWindow,
                             uint8_t  ownAddrType,
                             uint8_t  scanWlPolicy );

llStatus_t LL_CreateConn0( uint16_t scanInterval,
                           uint16_t scanWindow,
                           uint8_t  initWlPolicy,
                           uint8_t  peerAddrType,
                           uint8_t*  peerAddr,
                           uint8_t  ownAddrType,
                           uint16_t connIntervalMin,
                           uint16_t connIntervalMax,
                           uint16_t connLatency,
                           uint16_t connTimeout,
                           uint16_t minLength,
                           uint16_t maxLength );

llStatus_t LL_SetDataLengh0( uint16_t connId,uint16_t TxOctets,uint16_t TxTime );

void LL_EXT_Init_IQ_pBuff(uint16_t *ibuf,uint16_t *qbuf);

void LL_set_default_conn_params0(llConnState_t* connPtr);

uint8_t llSetupNextSlaveEvent0( void );

void llProcessTxData0( llConnState_t* connPtr, uint8_t context );

void ll_scheduler0(uint32_t time);

extern uint8_t   ll_processExtAdvIRQ(uint32_t      irq_status);
extern uint8_t   ll_processPrdAdvIRQ(uint32_t      irq_status);
extern uint8_t   ll_processExtScanIRQ(uint32_t      irq_status);
extern uint8_t   ll_processExtInitIRQ(uint32_t      irq_status);
extern uint8_t   ll_processPrdScanIRQ(uint32_t      irq_status);
extern uint8_t   ll_processBasicIRQ(uint32_t      irq_status);


extern uint8_t ll_processMissMasterEvt(uint8_t connId);
extern uint8_t ll_processMissSlaveEvt(uint8_t connId);

void enterSleepProcess0(uint32_t time);


extern struct buf_tx_desc g_tx_adv_buf;
//extern struct buf_tx_desc g_tx_ext_adv_buf;
extern struct buf_tx_desc tx_scanRsp_desc;

extern struct buf_rx_desc g_rx_adv_buf;

//extern chipMAddr_t g_chipMAddr;

extern uint8_t  g_llAdvMode;
extern uint32_t g_llHdcDirAdvTime;

extern uint32_t g_new_master_delta;

extern uint32_t sleep_flag;
extern uint32_t ll_remain_time;

extern volatile uint32_t llWaitingIrq;
extern uint32_t ISR_entry_time;


extern uint8_t  g_llScanMode;
extern uint8_t g_currentPeerAddrType;
extern uint8_t g_currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8_t ownRandomAddr[];
extern uint32_t llCurrentScanChn;
extern uint8_t ownPublicAddr[];
extern uint32_t llScanTime;
extern uint32_t llScanT1;
extern uint8_t    isPeerRpaStore;
extern uint8_t    currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8_t    storeRpaListIndex;
extern uint8_t    g_currentLocalAddrType;
extern uint8_t    g_currentLocalRpa[LL_DEVICE_ADDR_LEN];
extern llPduLenManagment_t g_llPduLen;
extern uint8_t  llSecondaryState;            // secondary state of LL
extern int slave_conn_event_recv_delay;


#ifdef __cplusplus
}
#endif

#endif /* LL_H */


