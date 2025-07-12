/*************
 sm_internal.h
 SDK_LICENSE
***************/

#ifndef SM_INTERNAL_H
#define SM_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ble/host/smp.h>
#include <ble/include/l2cap.h>
#include <ble/include/gap.h>
#include "linkdb.h"

/*********************************************************************
 * MACROS
 */
  
/*********************************************************************
 * CONSTANTS
 */
  
// Security Manager Task Events
#define SM_TIMEOUT_EVT            0x0001    // Message timeout event
#define SM_PAIRING_STATE_EVT      0x0002    // Event used to progress to the next pairing state
  
// Pairing states
#define SM_PAIRING_STATE_INITIALIZE                       0  // Pairing has started
#define SM_PAIRING_STATE_PAIRING_REQ_SENT                 1  // Initiator: Pairing Request has been sent, Responder: waiting for Pairing Request.
#define SM_PAIRING_STATE_WAIT_CONFIRM                     2  // Waiting for Confirm message
#define SM_PAIRING_STATE_WAIT_PASSKEY                     3  // Waiting for Passkey from app/profile
#define SM_PAIRING_STATE_WAIT_CONFIRM_PASSKEY             4  // Received Initiator Confirm message and waiting for Passkey from app/profile (responder only)
#define SM_PAIRING_STATE_WAIT_RANDOM                      5  // Waiting for Random message
#define SM_PAIRING_STATE_WAIT_STK                         6  // Waiting for STK process to finish
#define SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO       7  // Waiting for Slave Encryption Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO           8  // Waiting for Slave Master Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO         9  // Waiting for Slave Identity Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO    10 // Waiting for Slave Identity Addr Info to be sent
#define SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO          11 // Waiting for Slave Signing Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO      12 // Waiting for Master Encryption Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO          13 // Waiting for Master Master Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO        14 // Waiting for Master Identity Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO   15 // Waiting for Master Identity Addr Info to be sent
#define SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO         16 // Waiting for Master Signing Info to be sent
#define SM_PAIRING_STATE_WAIT_ENCRYPT                     17 // Waiting for LTK process to finish
#define SM_PAIRING_STATE_DONE                             18 // Closing out the pairing process
  
#if defined ( TESTMODES )
  // SM TestModes
  #define SM_TESTMODE_OFF                           0   // No Test mode
  #define SM_TESTMODE_NO_RESPONSE                   1   // Don't respond to any SM message
  #define SM_TESTMODE_SEND_BAD_CONFIRM              2   // Force a bad confirm value in the Confirm Message
  #define SM_TESTMODE_BAD_CONFIRM_VERIFY            3   // Force a bad confirm check of the received Confirm Message
  #define SM_TESTMODE_SEND_CONFIRM                  4   // Force a SMP Confirm message
#endif  // TESTMODES

// Pairing Types
#define SM_PAIRING_TYPE_INIT                        0 // Pairing has been started but the type hasn't been determined yet
#define SM_PAIRING_TYPE_JUST_WORKS                  1 // Pairing is Just Works
#define SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS    2 // Pairing is MITM Passkey with initiator inputs passkey
#define SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS    3 // Pairing is MITM Passkey with responder inputs passkey
#define SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS         4 // Pairing is MITM Passkey with both initiator and responder input passkey
#define SM_PAIRING_TYPE_OOB                         5 // Pairing is MITM OOB

#define SM_PAIRING_STATE_WAIT                       500 // The default wait time between key distribution messages.

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t  confirm[KEYLEN];       // calculated confirm value
  uint8_t  rand[SMP_RANDOM_LEN];  // First MRand or Srand, then RAND
} devPairing_t;

typedef struct
{
  // From the start
  uint8_t                initiator;        // TRUE if initiator
  uint8_t                state;            // Pairing state
  uint8_t                taskID;           // Task ID of the app/profile that requested the pairing

    uint8_t                timerID;           // 2021-03-29 add , timerid for simultaneously SMP for multi-role(the same as single connection )
    uint8_t                stateID;           // 2021-03-29 add , stateid for simultaneously SMP pairing state change idx
  uint16_t               connectionHandle; // Connection Handle from controller,
  smLinkSecurityReq_t  *pSecReqs;        // Pairing Control info
  uint8_t                tk[KEYLEN];       // Holds tk from app
  uint8_t                authState;        // uses SM_AUTH_STATE_AUTHENTICATED & SM_AUTH_STATE_BONDING
  
  // During pairing
  smpPairingReq_t      *pPairDev;        // Info of paired device.
  uint8_t                type;             // ie. SM_PAIRING_TYPE_JUST_WORKS
  
  // device information
  devPairing_t         myComp;          // This device's pairing components
  devPairing_t         devComp;         // The other device's components
  
  // Encrypt Params
  smSecurityInfo_t     *pEncParams;     // Your (device's) encryption parameters
  smSecurityInfo_t     *pDevEncParams;  // Connected device's encryption parameters
  smIdentityInfo_t     *pIdInfo;        // Connected device's identity parameters
  smSigningInfo_t      *pSigningInfo;    // Connected device's signing parameters
  
} smPairingParams_t;

// Callback when an SMP message has been received on the Initiator or Responder.
typedef uint8_t (*smProcessMsg_t)( linkDBItem_t *pLinkItem, uint8_t cmdID, smpMsgs_t *pParsedMsg );

// Callback to send next key message, and sets state for next event on the Initiator or Responder.
typedef void (*smSendNextKeyInfo_t)( uint16_t connectionHandle );

// Callback to send Start Encrypt through HCI on the Initiator.
typedef bStatus_t (*smStartEncryption_t)( uint16_t connHandle, uint8_t *pLTK, uint16_t div,
                                          uint8_t *pRandNum, uint8_t keyLen );

// Callback when an HCI BLE LTK Request has been received on the Responder.
typedef uint8_t (*smProcessLTKReq_t)( uint16_t connectionHandle, uint8_t *pRandom, uint16_t encDiv );

// Initiator callback structure - must be setup by the Initiator.
typedef struct
{
  smProcessMsg_t      pfnProcessMsg;      // When SMP message received
  smSendNextKeyInfo_t pfnSendNextKeyInfo; // When need to send next key message
  smStartEncryption_t pfnStartEncryption; // When Start Encrypt requested
} smInitiatorCBs_t;

// Responder callback structure - must be setup by the Initiator.
typedef struct
{
  smProcessMsg_t      pfnProcessMsg;      // When SMP message received
  smSendNextKeyInfo_t pfnSendNextKeyInfo; // When need to send next key message
  smProcessLTKReq_t   pfnProcessLTKReq;   // When HCI BLE LTK Request received
} smResponderCBs_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
  
// Security Manager's OSAL task ID
extern uint8_t smTaskID;

extern smPairingParams_t* pPairingParams[];

extern smResponderCBs_t *pfnResponderCBs;

/*********************************************************************
 * FUNCTIONS - API
 */

/*********************************************************************
 * Application Level Functions
 */

  /*
   * smLinkCheck - link database callback function.
   */
  extern void smLinkCheck( uint16_t connectionHandle, uint8_t changeType );
  
  /*
   * smProcessRandComplete - Process the HCI Random Complete Event.
   */
  extern uint8_t smProcessRandComplete( uint8_t status, uint8_t *rand );

  /*
   * smTimedOut - Process the SM timeout.
   */
extern void smTimedOut( uint16_t connectionHandle );
  
  /*
   * smStartRspTimer - Start the SM Response Timer.
*/
extern void smStartRspTimer( uint16_t connectionHandle );

/*
    smStopRspTimer - Stop the SM Response Timer.
*/
extern void smStopRspTimer( uint16_t connectionHandle );

  /*
   * smProcessDataMsg - Process incoming L2CAP messages.
   */
  extern void smProcessDataMsg( l2capDataEvent_t *pMsg );
  
  /*
   * smProcessEncryptChange - Process the HCI BLE Encrypt Change Event.
   */
  extern uint8_t smProcessEncryptChange( uint16_t connectionHandle, uint8_t reason );
  
  /*
   * smInProcess - Is SM already processing something?
   */
  extern uint8_t smInProcess( void );

  /*
   * sm_d1 - SM diversifying function d1
   */
  extern bStatus_t sm_d1( uint8_t *pK, uint16_t d, uint8_t *pD1 );

  /*
   * sm_ah - Random address hash function
   */
  extern bStatus_t sm_ah( uint8_t *pK, uint8_t *pR, uint8_t *pAh );

  /*
   * sm_dm - SM DIV Maxk generation function dm
   */
  extern bStatus_t sm_dm( uint8_t *pK, uint8_t *pR, uint16_t *pDm );

/*
    sm_c1 - SM Confirm value generation function c1
*/
extern bStatus_t sm_c1( uint16_t connectionHandle,uint8_t* pK, uint8_t* pR, uint8_t* pC1 );

/*
    sm_c1new - SM Confirm value generation function c1
*/
extern bStatus_t sm_c1new( uint8_t* pK, uint8_t* pR, uint8_t* pRes, uint8_t* pReq,
                           uint8_t iat, uint8_t* pIA, uint8_t rat, uint8_t* pRA, uint8_t* pC1 );
/*
    sm_s1 - SM key generation function s1
*/
extern bStatus_t sm_s1( uint8_t* pK, uint8_t* pR1, uint8_t* pR2, uint8_t* pS1 );

/*
    smGenerateRandBuf - generate a buffer of random numbers
*/
extern void smGenerateRandBuf( uint8_t* pRandNum, uint8_t len );

/*
    smEncLTK - start LTK Encryption
*/
extern void smEncLTK( uint16_t connectionHandle );

/*
    smNextPairingState - trigger next state machine
*/
extern void smNextPairingState( uint16_t connectionHandle );

/*
    smAuthReqToUint8 - conversion function
*/
extern uint8_t smAuthReqToUint8( authReq_t* pAuthReq );

/*
    smUint8ToAuthReq - conversion function
*/
extern void smUint8ToAuthReq( authReq_t* pAuthReq, uint8_t authReqUint8 );

/*
    smpResponderProcessPairingReq - Process an incoming Pairing Request message
*/
extern uint8_t smpResponderProcessPairingReq( uint16_t connectionHandle,smpPairingReq_t* pParsedMsg );

/*
    smSendFailAndEnd - Send the pairing failed message and end existing pairing
*/
extern bStatus_t smSendFailAndEnd( uint16_t connHandle, smpPairingFailed_t* pFailedMsg );

/*
    generateRandMsg - Generate a Pairing Random
*/
extern bStatus_t smGenerateRandMsg( uint16_t connectionHandle);

/*
    smSavePairInfo - Save the Pairing Req or Rsp information
*/
extern bStatus_t smSavePairInfo( uint16_t connectionHandle,smpPairingReq_t* pPair );

/*
    generateConfirm - Generate a Pairing Confirm
*/
extern bStatus_t smGenerateConfirm( uint16_t connectionHandle );

/*
    smEndPairing - Pairing mode has ended.  Yeah. Notify the GAP and free
                  up the memory used.
*/
extern void smEndPairing( uint16_t connectionHandle,uint8_t status );

/*
    determineKeySize - Determine the maximum encryption key size
*/
extern uint8_t smDetermineKeySize( uint16_t connectionHandle );

/*
    smGeneratePairingReqRsp - Generate a pairing req or response
*/
extern bStatus_t smGeneratePairingReqRsp( uint16_t connectionHandle );

  /*
   * smPairingSendEncInfo - Send SM Encryption Information message
   */
  extern void smPairingSendEncInfo( uint16_t connHandle, uint8_t *pLTK );

  /*
   * smPairingSendMasterID - Send SM Master Identification message
   */
  extern void smPairingSendMasterID( uint16_t connHandle, uint16_t ediv, uint8_t *pRand );

  /*
   * smPairingSendIdentityInfo - Send SM Identity Information message
   */
  extern void smPairingSendIdentityInfo( uint16_t connHandle, uint8_t *pIRK );

  /*
   * smPairingSendIdentityAddrInfo - Send SM Identity Addr Information message
   */
  extern void smPairingSendIdentityAddrInfo( uint16_t connHandle, uint8_t addrType, uint8_t *pMACAddr );

  /*
   * smPairingSendSingingInfo - Send SM Signing Information message
   */
  extern void smPairingSendSingingInfo( uint16_t connHandle, uint8_t *pSRK );

  /*
   * smPairingSendEncInfo - Send SM Encryption Information message
   */
  extern void smPairingSendEncInfo( uint16_t connHandle, uint8_t *pLTK );

  /*
   * smProcessPairingReq - Process Pairing Request
   */
  extern void smProcessPairingReq( linkDBItem_t *pLinkItem, gapPairingReq_t *pPairReq );

  /*
   * smStartEncryption - Perform Encrypt through HCI
   */
  extern bStatus_t smStartEncryption( uint16_t connHandle, uint8_t *pLTK, uint16_t div,
                                      uint8_t *pRandNum, uint8_t keyLen );

  /*
   * smRegisterInitiator - egister Initiator's processing function with SM task
   */
  extern void smRegisterInitiator( smInitiatorCBs_t *pfnCBs );

  /*
   * smRegisterResponder - Register Responder's processing function with SM task
   */
  extern void smRegisterResponder( smResponderCBs_t *pfnCBs );

/*
    smp timerout callback for SMP Timeout and pairing state
*/
extern void smTo_timerCB( uint8_t* pData );
extern void smState_timerCB( uint8_t* pData );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SM_INTERNAL_H */
