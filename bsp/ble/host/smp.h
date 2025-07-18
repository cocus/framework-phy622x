/*************
 smp.h
 SDK_LICENSE
***************/

#ifndef SMP_H
#define SMP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
//#include "bcomdef.h"
//#include "sm_internal.h"
#include <ble/include/bcomdef.h>
#include <ble/include/sm.h>

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

// Code field of the SMP Command format
#define SMP_PAIRING_REQ                           0x01
#define SMP_PAIRING_RSP                           0x02
#define SMP_PAIRING_CONFIRM                       0x03
#define SMP_PAIRING_RANDOM                        0x04
#define SMP_PAIRING_FAILED                        0x05
#define SMP_ENCRYPTION_INFORMATION                0x06
#define SMP_MASTER_IDENTIFICATION                 0x07
#define SMP_IDENTITY_INFORMATION                  0x08
#define SMP_IDENTITY_ADDR_INFORMATION             0x09
#define SMP_SIGNING_INFORMATION                   0x0A
#define SMP_SECURITY_REQUEST                      0x0B

// Pairing Request & Response - IO Capabilities
#define SMP_IO_CAP_DISPLAY_ONLY                   0x00
#define SMP_IO_CAP_DISPLAY_YES_NO                 0x01
#define SMP_IO_CAP_KEYBOARD_ONLY                  0x02
#define SMP_IO_CAP_NO_INPUT_NO_OUTPUT             0x03
#define SMP_IO_CAP_KEYBOARD_DISPLAY               0x04

// Pairing Request & Response - Out Of Bound (OOB) data flag values
#define SMP_OOB_AUTH_DATA_NOT_PRESENT             0x00
#define SMP_OOB_AUTH_DATA_REMOTE_DEVICE_PRESENT   0x01

// Pairing Request & Response - authReq field
//   - This field contains 2 sub-fields:
//      bonding flags - bits 1 & 0
#define SMP_AUTHREQ_BONDING                       0x01
//      Man-In-The-Middle (MITM) - bit 2
#define SMP_AUTHREQ_MITM                          0x04

#define SMP_CONFIRM_LEN                           16
#define SMP_RANDOM_LEN                            16

// Pairing Failed - "reason" field
#define SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED   0x01 //!< The user input of the passkey failed, for example, the user cancelled the operation.
#define SMP_PAIRING_FAILED_OOB_NOT_AVAIL          0x02 //!< The OOB data is not available
#define SMP_PAIRING_FAILED_AUTH_REQ               0x03 //!< The pairing procedure can't be performed as authentication requirements can't be met due to IO capabilities of one or both devices
#define SMP_PAIRING_FAILED_CONFIRM_VALUE          0x04 //!< The confirm value doesn't match the calculated compare value
#define SMP_PAIRING_FAILED_NOT_SUPPORTED          0x05 //!< Pairing isn't supported by the device
#define SMP_PAIRING_FAILED_ENC_KEY_SIZE           0x06 //!< The resultant encryption key size is insufficient for the security requirements of this device.
#define SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED      0x07 //!< The SMP command received is not supported on this device.
#define SMP_PAIRING_FAILED_UNSPECIFIED            0x08 //!< Pairing failed due to an unspecified reason
#define SMP_PAIRING_FAILED_REPEATED_ATTEMPTS      0x09 //!< Pairing or authenication procedure is disallowed because too little time has elapsed since the last pairing request or security request.

#define SMP_PAIRING_FAILED_LOCAL_KEY_FAILURE      0x0A    // Local value - not sent over the air

// Message lengths
#define SMP_PAIRING_REQ_LEN                       7
#define SMP_PAIRING_RSP_LEN                       7
#define SMP_PAIRING_CONFIRM_LEN                   17
#define SMP_PAIRING_RANDOM_LEN                    17
#define SMP_PAIRING_FAILED_LEN                    2
#define SMP_ENCRYPTION_INFORMATION_LEN            17
#define SMP_MASTER_IDENTIFICATION_LEN             11
#define SMP_IDENTITY_INFORMATION_LEN              17
#define SMP_IDENTITY_ADDR_INFORMATION_LEN         8
#define SMP_SIGNING_INFORMATION_LEN               17
#define SMP_SECURITY_REQUEST_LEN                  2

// Macros to use the smSendSMMsg() function to send all of the Security Manager Protocol messages
#define smSendPairingReq( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_PAIRING_REQ_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildPairingReq) )

#define smSendPairingRsp( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_PAIRING_RSP_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildPairingRsp) )

#define smSendPairingConfirm( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_PAIRING_CONFIRM_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildPairingConfirm) )

#define smSendPairingRandom( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_PAIRING_RANDOM_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildPairingRandom) )

#define smSendPairingFailed( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_PAIRING_FAILED_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildPairingFailed) )

#define smSendEncInfo( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_ENCRYPTION_INFORMATION_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildEncInfo) )

#define smSendMasterID( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_MASTER_IDENTIFICATION_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildMasterID) )

#define smSendIdentityInfo( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_IDENTITY_INFORMATION_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildIdentityInfo) )

#define smSendIdentityAddrInfo( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_IDENTITY_ADDR_INFORMATION_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildIdentityAddrInfo) )

#define smSendSigningInfo( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_SIGNING_INFORMATION_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildSigningInfo) )

#define smSendSecurityReq( connHandle, msgStruct )  \
    smSendSMMsg( (connHandle), SMP_SECURITY_REQUEST_LEN, \
                 (smpMsgs_t *)(msgStruct), \
                 (pfnSMBuildCmd_t)(smpBuildSecurityReq) )

/*********************************************************************
    TYPEDEFS
*/

// Pairing Request
typedef struct
{
    uint8_t ioCapability;   // ex. SMP_IO_CAP_DISPLAY_YES_NO
    uint8_t oobDataFlag;    // Out of Bound data flag
    authReq_t authReq;    // Authentication fields
    uint8_t maxEncKeySize;  // Encryption Key size max bytes (7 - 16)
    keyDist_t keyDist;    // Key Distribution Field - bit struct
} smpPairingReq_t;

// Pairing Response - same as Pairing Request
typedef smpPairingReq_t smpPairingRsp_t;

// Pairing Confirm
typedef struct
{
    uint8_t confirmValue[SMP_CONFIRM_LEN];
} smpPairingConfirm_t;

// Pairing Random
typedef struct
{
    uint8_t randomValue[SMP_RANDOM_LEN];
} smpPairingRandom_t;

// Pairing Failed
typedef struct
{
    uint8_t reason;
} smpPairingFailed_t;

// Encryption Information
typedef struct
{
    uint8_t  ltk[KEYLEN];
} smpEncInfo_t;

// Master Identification
typedef struct
{
    uint16_t  ediv;
    uint16_t  rand[B_RANDOM_NUM_SIZE];
} smpMasterID_t;

// Identity Information
typedef struct
{
    uint8_t irk[KEYLEN];
} smpIdentityInfo_t;

// Identity Address Information
typedef struct
{
    uint8_t addrType;
    uint8_t bdAddr[B_ADDR_LEN];
} smpIdentityAddrInfo_t;

// Signing Information
typedef struct
{
    uint8_t signature[KEYLEN];
} smpSigningInfo_t;

// Slave Security Request
typedef struct
{
    authReq_t authReq;
} smpSecurityReq_t;

// Union with all of the SM messages.
typedef union
{
    smpPairingReq_t       pairingReq;
    smpPairingReq_t       pairingRsp;
    smpPairingConfirm_t   pairingConfirm;
    smpPairingRandom_t    pairingRandom;
    smpPairingFailed_t    pairingFailed;
    smpEncInfo_t          encInfo;
    smpMasterID_t         masterID;
    smpIdentityInfo_t     idInfo;
    smpIdentityAddrInfo_t idAddrInfo;
    smpSigningInfo_t      signingInfo;
    smpSecurityReq_t      secReq;
} smpMsgs_t;

typedef uint8_t (*pfnSMBuildCmd_t)( smpMsgs_t* pMsgStruct, uint8_t* pBuf );

/*********************************************************************
    GLOBAL VARIABLES
*/
extern smpPairingReq_t pairingReg;

/*********************************************************************
    FUNCTIONS
*/

/*
    smpBuildPairingReq - Build an SM Pairing Request
*/
extern bStatus_t smpBuildPairingReq( smpPairingReq_t* pPairingReq, uint8_t* pBuf );

/*
    smpBuildPairingRsp - Build an SM Pairing Response
*/
extern bStatus_t smpBuildPairingRsp( smpPairingRsp_t* pPairingRsp, uint8_t* pBuf );

/*
    smpBuildPairingReqRsp - Build an SM Pairing Request or Response
*/
extern bStatus_t smpBuildPairingReqRsp( uint8_t opCode, smpPairingReq_t* pPairingReq, uint8_t* pBuf );

/*
    smpParsePairingReq - Parse an SM Pairing Request
*/
extern bStatus_t smpParsePairingReq( uint8_t* pBuf, smpPairingReq_t* pPairingReq );

/*
    smpParsePairingRsp - Parse an SM Pairing Response
*/
#define smpParsePairingRsp( a, b ) smpParsePairingReq( (a), (b) )

/*
    smpBuildPairingConfirm - Build an SM Pairing Confirm
*/
extern bStatus_t smpBuildPairingConfirm( smpPairingConfirm_t* pPairingConfirm,
                                         uint8_t* pBuf );

/*
    smpParsePairingConfirm - Parse an SM Pairing Confirm
*/
extern bStatus_t smpParsePairingConfirm( uint8_t* pBuf,
                                         smpPairingConfirm_t* pPairingConfirm );

/*
    smpBuildPairingRandom - Build an SM Pairing Random
*/
extern bStatus_t smpBuildPairingRandom( smpPairingRandom_t* pPairingRandom,
                                        uint8_t* pBuf );

/*
    smpParsePairingRandom - Parse an SM Pairing Random
*/
extern bStatus_t smpParsePairingRandom( uint8_t* pBuf,
                                        smpPairingRandom_t* pPairingRandom );

/*
    smpBuildPairingFailed - Build an SM Pairing Failed
*/
extern bStatus_t smpBuildPairingFailed( smpPairingFailed_t* pPairingFailed,
                                        uint8_t* pBuf );

/*
    smpParsePairingFailed - Parse an SM Pairing Failed
*/
extern bStatus_t smpParsePairingFailed( uint8_t* pBuf,
                                        smpPairingFailed_t* pPairingFailed );

/*
    smpBuildEncInfo - Build an SM Encryption Information
*/
extern bStatus_t smpBuildEncInfo( smpEncInfo_t* pEncInfo, uint8_t* pBuf );

/*
    smpParseEncInfo - Parse an SM Encryption Information
*/
extern bStatus_t smpParseEncInfo( uint8_t* buf, smpEncInfo_t* encInfo );

/*
    smpBuildMasterID - Build an SM Master Identification
*/
extern bStatus_t smpBuildMasterID( smpMasterID_t* pMasterID, uint8_t* pBuf );

/*
    smpParseMasterID - Parse an SM Master Identification
*/
extern bStatus_t smpParseMasterID( uint8_t* pBuf, smpMasterID_t* pMasterID );

/*
    smpBuildIdentityInfo - Build an SM Identity Information
*/
extern bStatus_t smpBuildIdentityInfo( smpIdentityInfo_t* pIdInfo, uint8_t* pBuf );

/*
    smpBuildIdentityAddrInfo - Build an SM Identity Address Information
*/
extern bStatus_t smpBuildIdentityAddrInfo( smpIdentityAddrInfo_t* pIdInfo, uint8_t* pBuf );

/*
    smpParseIdentityInfo - Parse an SM Identity Information
*/
extern bStatus_t smpParseIdentityInfo( uint8_t* pBuf, smpIdentityInfo_t* pIdInfo );

/*
    smpParseIdentityAddrInfo - Parse an SM Identity Address Information
*/
extern bStatus_t smpParseIdentityAddrInfo( uint8_t* pBuf, smpIdentityAddrInfo_t* pIdInfo );

/*
    smpBuildSigningInfo - Build an SM Signing Information
*/
extern bStatus_t smpBuildSigningInfo( smpSigningInfo_t* pSigningInfo, uint8_t* pBuf );

/*
    smpParseSigningInfo - Parse an SM Signing Information
*/
extern bStatus_t smpParseSigningInfo( uint8_t* pBuf, smpSigningInfo_t* pSigningInfo );

/*
    smpBuildSecurityReq - Build an SM Slave Security Request
*/
extern bStatus_t smpBuildSecurityReq( smpSecurityReq_t* pSecReq, uint8_t* pBuf );

/*
    smpParseSecurityReq - Parse an SM Slave Security Request
*/
extern bStatus_t smpParseSecurityReq( uint8_t* pBuf, smpSecurityReq_t* pSecReq );

/*
    smSendSMMsg - Generic Send L2CAP SM message function
*/
extern bStatus_t smSendSMMsg( uint16_t connHandle, uint8_t bufLen, smpMsgs_t* pMsg, pfnSMBuildCmd_t buildFn );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SMP_H */
