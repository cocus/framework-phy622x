/*************
 gap_internal.h
 SDK_LICENSE
***************/

#ifndef GAP_INTERNAL_H
#define GAP_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
//#include "bcomdef.h"
//#include "hci.h"
//#include "l2cap.h"
//#include "gap.h"
//#include <stdint.h>
#include <ble/include/bcomdef.h>
#include <ble/include/gap.h>
#include <ble/include/l2cap.h>
#include <ble/include/hci.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// GAP OSAL Events
#define GAP_OSAL_TIMER_SCAN_DURATION_EVT        0x0001
#define GAP_END_ADVERTISING_EVT                 0x0002
#define GAP_CHANGE_RESOLVABLE_PRIVATE_ADDR_EVT  0x0004

#define ADDRTYPE_RANDOM                         1  // Not public

#define GAP_PRIVATE_ADDR_CHANGE_RESOLUTION      0xEA60 // Timer resolution is 1 minute

#define ADV_TOKEN_HDR   2

// Address header bits
#define RANDOM_ADDR_HDR                       0xC0  // Used for LL RANDOM Address
#define STATIC_ADDR_HDR                       0xC0  // Host Static Address, same as RANDOM address
#define PRIVATE_RESOLVE_ADDR_HDR              0x40

#if defined ( TESTMODES )
  // GAP TestModes
  #define GAP_TESTMODE_OFF                        0 // No Test mode
  #define GAP_TESTMODE_NO_RESPONSE                1 // Don't respond to any GAP message
#endif  // TESTMODES

// L2CAP Connection Parameters Update Request event
#define L2CAP_PARAM_UPDATE                      0xFFFF

/*********************************************************************
 * TYPEDEFS
 */

typedef struct gapAdvToken
{
  struct gapAdvToken *pNext;     // Pointer to next item in link list
  gapAdvDataToken_t  *pToken;    // Pointer to data token
} gapAdvToken_t;

/** Advertising and Scan Response Data **/
typedef struct
{
  uint8_t   dataLen;                  // Number of bytes used in "dataField"
  uint8_t   dataField[B_MAX_ADV_LEN]; // Data field of the advertisement or SCAN_RSP
} gapAdvertisingData_t;

typedef struct
{
  uint8_t   dataLen;       // length (in bytes) of "dataField"
  uint8_t   dataField[1];  // This is just a place holder size
                         // The dataField will be allocated bigger
} gapAdvertRecData_t;

// Temporary advertising record
typedef struct
{
  uint8_t  eventType;               // Avertisement or SCAN_RSP
  uint8_t  addrType;                // Advertiser's address type
  uint8_t  addr[B_ADDR_LEN];        // Advertiser's address
  gapAdvertRecData_t *pAdData;    // Advertising data field. This space is allocated.
  gapAdvertRecData_t *pScanData;  // SCAN_RSP data field. This space is allocated.
} gapAdvertRec_t;

typedef enum
{
  GAP_ADSTATE_SET_PARAMS,     // Setting the advertisement parameters
  GAP_ADSTATE_SET_MODE,       // Turning on advertising
  GAP_ADSTATE_ADVERTISING,    // Currently Advertising
  GAP_ADSTATE_ENDING          // Turning off advertising
} gapAdvertStatesIDs_t;

// Advertising State Information
typedef struct
{
  uint8_t taskID;                   // App that started an advertising period
  gapAdvertStatesIDs_t state;     // Make Discoverable state
  gapAdvertisingParams_t params;  // Advertisement parameters
} gapAdvertState_t;

typedef struct
{
  uint8_t                state;            // Authentication states
  uint16_t               connectionHandle; // Connection Handle from controller,
  smLinkSecurityReq_t  secReqs;          // Pairing Control info

  // The following are only used if secReqs.bondable == BOUND, which means that
  // the device is already bound and we should use the security information and
  // keys
  smSecurityInfo_t     *pSecurityInfo;    // BOUND - security information
  smIdentityInfo_t     *pIdentityInfo;    // BOUND - identity information
  smSigningInfo_t      *pSigningInfo;     // Signing information
} gapAuthStateParams_t;

// Callback when an HCI Command Event has been received on the Central.
typedef uint8_t (*gapProcessHCICmdEvt_t)( uint16_t cmdOpcode, hciEvt_CmdComplete_t *pMsg );

// Callback when an Scanning Report has been received on the Central.
typedef void (*gapProcessScanningEvt_t)( hciEvt_BLEAdvPktReport_t *pMsg );

// Callback to cancel a connection initiation on the Central.
typedef bStatus_t (*gapCancelLinkReq_t)( uint8_t taskID, uint16_t connectionHandle );

// Callback when a connection-related event has been received on the Central.
typedef uint8_t(*gapProcessConnEvt_t)( uint16_t cmdOpcode, hciEvt_CommandStatus_t *pMsg );

// Callback when an HCI Command Command Event on the Peripheral.
typedef uint8_t (*gapProcessHCICmdCompleteEvt_t)( hciEvt_CmdComplete_t *pMsg );

// Callback when an Advertising Event has been received on the Peripheral.
typedef void (*gapProcessAdvertisingEvt_t)( uint8_t timeout );

// Callback when a Set Advertising Params has been received on the Peripheral.
typedef bStatus_t (*gapSetAdvParams_t)( void );

// Central callback structure - must be setup by the Central.
typedef struct
{
  gapProcessHCICmdEvt_t   pfnProcessHCICmdEvt;   // When HCI Command Event received
  gapProcessScanningEvt_t pfnProcessScanningEvt; // When Scanning Report received
} gapCentralCBs_t;

// Central connection-related callback structure - must be setup by the Central.
typedef struct
{
  gapCancelLinkReq_t  pfnCancelLinkReq;  // When cancel connection initiation requested
  gapProcessConnEvt_t pfnProcessConnEvt; // When connection-related event received
} gapCentralConnCBs_t;

// Peripheral callback structure - must be setup by the Peripheral.
typedef struct
{
  gapProcessHCICmdCompleteEvt_t pfnProcessHCICmdCompleteEvt; // When HCI Command Complete Event received
  gapProcessAdvertisingEvt_t    pfnProcessAdvertisingEvt;    // When Advertising Event received
  gapSetAdvParams_t             pfnSetAdvParams;             // When Set Advertising Params received
} gapPeripheralCBs_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern uint8_t gapTaskID;
extern uint8_t gapUnwantedTaskID;

extern uint8_t gapAppTaskID;         // default task ID to send events
extern uint8_t gapProfileRole;       // device GAP Profile Role(s)

extern uint8_t gapDeviceAddrMode;   //  ADDRTYPE_PUBLIC, ADDRTYPE_STATIC,
                                  //  ADDRTYPE_PRIVATE_NONRESOLVE
                                  //  or ADDRTYPE_PRIVATE_RESOLVE

// Central Peripheral variables
extern gapDevDiscReq_t *pGapDiscReq;
extern gapEstLinkReq_t *pEstLink;
extern gapCentralConnCBs_t *pfnCentralConnCBs;

// Peripheral variables
extern gapAdvertState_t *pGapAdvertState;
extern gapPeripheralCBs_t *pfnPeripheralCBs;

// Common variables
extern gapAuthStateParams_t* pAuthLink[];
extern uint16_t gapPrivateAddrChangeTimeout;
extern uint8_t gapAutoAdvPrivateAddrChange;

/*********************************************************************
 * FUNCTIONS - API
 */

/*********************************************************************
 * Application Level Functions
 */

  /*
   * gapSetScanParamStatus - Process HCI Command Complete Event status for
   *              the call to HCI_BLESetScanParamCmd().
   */
  extern uint8_t gapSetScanParamStatus( uint8_t status );

  /*
   * gapSetAdvParamsStatus - Process HCI Command Complete Event status for
   *              the call to HCI_BLESetAdvParamCmd().
   */
  extern uint8_t gapSetAdvParamsStatus( uint8_t status );

  /*
   * gapWriteAdvEnableStatus - Process HCI Command Complete Event status for
   *              the call to HCI_BLEWriteAdvEnableCmd().
   */
  extern uint8_t gapWriteAdvEnableStatus( uint8_t status, uint16_t interval );

  /*
   * gapWriteAdvDataStatus - Process HCI Command Complete Event status for
   *              the call to HCI_BLEWriteAdvDataCmd() or
   *              HCI_BLEWriteScanRspDataCmd().
   */
  extern void gapWriteAdvDataStatus( uint8_t adType, uint8_t status );

  /*
   * gapReadBD_ADDRStatus - Process the HCI Command Complete Event for the
   *              call to HCI_ReadBDADDRCmd().
   */
  extern uint8_t gapReadBD_ADDRStatus( uint8_t status, uint8_t *pBdAddr );

  /*
   * gapReadBufSizeCmdStatus - Process the HCI Command Complete Event for the
   *              call to HCI_BLEReadBufSizeCmd().
   */
  extern uint8_t gapReadBufSizeCmdStatus( hciRetParam_LeReadBufSize_t *pCmdStat );

  /*
   * gapProcessConnectionCompleteEvt - Process the HCI Connection Complete
   *              event for the call to HCI_BLECreateLLConnCmd().
   */
  extern void gapProcessConnectionCompleteEvt( hciEvt_BLEConnComplete_t *pPkt );

  /*
   * gapProcessConnUpdateCompleteEvt - Process the HCI Connection Parameters
   *              Update Complete event for the call to HCI_BLEUpdateLLConnCmd().
   */
  extern void gapProcessConnUpdateCompleteEvt( hciEvt_BLEConnUpdateComplete_t *pPkt );

  /*
   * gapProcessDisconnectCompleteEvt - Process the LL Disconnection Complete Event
   *              for the call to HCI_DisconnectCmd().
   */
  extern void gapProcessDisconnectCompleteEvt( hciEvt_DisconnComplete_t *pPkt );

  /*
   * gapProcessCreateLLConnCmdStatus - Process the status for the HCI_BLECreateLLConnCmd().
   */
  extern void gapProcessCreateLLConnCmdStatus( uint8_t status );

  /*
   * gapProcessConnUpdateCmdStatus - Process the status for the HCI_LE_ConnUpdateCmd().
   */
  extern void gapProcessConnUpdateCmdStatus( uint8_t status );

  /*
   * gapProcessNewAddr - Process message SM
   */
  extern bStatus_t gapProcessNewAddr( uint8_t *pNewAddr );

  /*
   * gapAddAddrAdj - Add the top two bits based on the address type.
   */
  extern uint8_t gapAddAddrAdj( uint8_t addrType, uint8_t *pAddr );

  /*
   * gapDetermineAddrType - Convert from LL address type to host address type.
   */
  extern uint8_t gapDetermineAddrType( uint8_t addrType, uint8_t *pAddr );

  /*
   * gapProcessRandomAddrComplete - Process message HCI
   */
  extern void gapProcessRandomAddrComplete( uint8_t status );

  /*
   * gapGetSRK - Get pointer to the SRK
   */
  extern uint8_t *gapGetSRK( void );

  /*
   * gapGetSignCounter - Get the signature counter
   */
  extern uint32_t gapGetSignCounter( void );

  /*
   * gapIncSignCounter - Increment the signature counter
   */
  extern  void gapIncSignCounter( void );

  /*
   * gapUpdateConnSignCounter - Update a connection's signature's counter
   */
  extern  void gapUpdateConnSignCounter( uint16_t connHandle, uint32_t newSignCounter );

  /*
   * gapLinkCheck - linkDB callback function
   */
  extern void gapLinkCheck( uint16_t connectionHandle, uint8_t changeType );

  /*
   * gapGetDevAddressMode - Get the device address mode.
   */
  extern uint8_t gapGetDevAddressMode( void );

  /*
   * gapGetDevAddress - Get the device address.
   *      real - TRUE if you always want BD_ADDR, FALSE will allow random addresses.
   */
  extern uint8_t *gapGetDevAddress( uint8_t real );

  /*
   * gapGetIRK - Get the device's IRK.
   */
  extern uint8_t *gapGetIRK( void );

  /*
   * gapPasskeyNeededCB - Callback function to ask for passkey
   */
  extern void gapPasskeyNeededCB( uint16_t connectionHandle, uint8_t type );

  /*
   * gapPairingCompleteCB - Callback function to inform pairing process complete.
   */
  extern void gapPairingCompleteCB( uint8_t status, uint8_t initiatorRole,
                          uint16_t connectionHandle,
                          uint8_t authState,
                          smSecurityInfo_t *pEncParams,
                          smSecurityInfo_t *pDevEncParams,
                          smIdentityInfo_t *pIdInfo,
                          smSigningInfo_t  *pSigningInfo );

  /*
   * gapTerminateConnComplete - Process command complete for HCI_BLECreateLLConnCancelCmd.
   */
  extern void gapTerminateConnComplete( void );

  /*
   * gapSendSlaveSecurityReqEvent - Generate a Slave Security Request event to the app.
   */
  extern void gapSendSlaveSecurityReqEvent( uint8_t taskID, uint16_t connHandle, uint8_t *pDevAddr, uint8_t authReq );

  /*
   * gapSetAdvParams - Send the advertisement parameters to the LL.
   */
  extern bStatus_t gapSetAdvParams( void );

  /*
   * gapAddAdvToken - Add token to the end of the list.
   */
  extern bStatus_t gapAddAdvToken( gapAdvDataToken_t *pToken );

  /*
   * gapDeleteAdvToken - Remove a token from the list.
   */
  extern gapAdvDataToken_t *gapDeleteAdvToken( uint8_t ADType );

  /*
   * gapFindAdvToken - Find a Advertisement data token from the advertisement type.
   */
  extern gapAdvToken_t *gapFindAdvToken( uint8_t ADType );

  /*
   * gapCalcAdvTokenDataLen - Find a Advertisement data token from the advertisement type.
   */
  extern void gapCalcAdvTokenDataLen( uint8_t *pAdLen, uint8_t *pSrLen );

  /*
   * gapValidADType - Is a Advertisement Data Type valid.
   */
  extern uint8_t gapValidADType( uint8_t adType );

  /*
   * gapBuildADTokens - Is a Advertisement Data Type valid.
   */
  extern bStatus_t gapBuildADTokens( void );

  /*
   * gapSendBondCompleteEvent - Indicate that a bond has occurred.
   */
  extern void gapSendBondCompleteEvent( uint8_t status, uint16_t connectionHandle );

  /*
   * gapSendPairingReqEvent - Indicate that an unexpected Pairing Request was received.
   */
  extern void gapSendPairingReqEvent( uint8_t status, uint16_t connectionHandle,
                             uint8_t ioCap,
                             uint8_t oobDataFlag,
                             uint8_t authReq,
                             uint8_t maxEncKeySize,
                             keyDist_t keyDist );

  /*
   * gapFindADType - Find Advertisement Data Type field in advertising data
   *                 field.
   */
  extern uint8_t *gapFindADType( uint8_t adType, uint8_t *pAdLen,
                               uint8_t dataLen, uint8_t *pDataField );

  /*
   * gapRegisterCentral - Register Central's processing function with GAP task
   */
  extern void gapRegisterCentral( gapCentralCBs_t *pfnCBs );

  /*
   * gapRegisterCentralConn - Register Central's connection-related processing function with GAP task
   */
  extern void gapRegisterCentralConn( gapCentralConnCBs_t *pfnCBs);

  /*
   * gapRegisterPeripheral - Register Peripheral's processing function with GAP task
   */
  extern void gapRegisterPeripheral( gapPeripheralCBs_t *pfnCBs );

  /*
   * gapIsAdvertising - Check if we are currently advertising.
   */
  extern uint8_t gapIsAdvertising( void );

  /*
   * gapIsScanning - Check if we are currently scanning.
   */
  extern uint8_t gapIsScanning( void );

  /*
   * gapCancelLinkReq - Cancel a connection create request.
   */
  extern bStatus_t gapCancelLinkReq( uint8_t taskID, uint16_t connectionHandle );

  /*
   * gapFreeEstLink - Free the establish link memory.
   */
  extern void gapFreeEstLink( void );

  /*
   * sendEstLinkEvent - Build and send the GAP_LINK_ESTABLISHED_EVENT to the app.
   */
  extern void sendEstLinkEvent( uint8_t status, uint8_t taskID, uint8_t devAddrType,
                                uint8_t *pDevAddr, uint16_t connectionHandle,
                                uint16_t connInterval, uint16_t connLatency,
                                uint16_t connTimeout, uint16_t clockAccuracy );

  /*
   * gapSendLinkUpdateEvent - Build and send the GAP_LINK_PARAM_UPDATE_EVENT to the app.
   *
   */
  extern void gapSendLinkUpdateEvent( uint8_t status, uint16_t connectionHandle,
                                      uint16_t connInterval, uint16_t connLatency,
                                      uint16_t connTimeout );

  /*
   * gapProcessL2CAPSignalEvt - Process L2CAP Signaling messages.
   */
  extern void gapProcessL2CAPSignalEvt( l2capSignalEvent_t *pCmd );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GAP_INTERNAL_H */
