/*******************************************************************************
  Filename:       ll_enc.h
  Revised:         
  Revision:        

  Description:    This file contains the Link Layer (LL) types, contants,
                  API's etc. for the Bluetooth Low Energy (BLE) Controller
                  CCM encryption and decryption.

                  This API is based on ULP BT LE D09R23.

  SDK_LICENSE

*******************************************************************************/

#ifndef LL_ENC_H
#define LL_ENC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include "ll_def.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

#define LL_ENC_TX_DIRECTION_MASTER   1
#define LL_ENC_TX_DIRECTION_SLAVE    0
#define LL_ENC_RX_DIRECTION_MASTER   0
#define LL_ENC_RX_DIRECTION_SLAVE    1
#define LL_ENC_DATA_BANK_MASK 0xFF7F

#define LL_ENC_TRUE_RAND_BUF_SIZE     ((LL_ENC_IV_LEN/2) + (LL_ENC_SKD_LEN/2))

// Generate Session Key using LTK for key and SKD for plaintext.
#define LL_ENC_GenerateSK LL_ENC_AES128_Encrypt

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint8_t dataPkt[2*LL_ENC_BLOCK_LEN];
extern uint8_t cachedTRNGdata[ LL_ENC_TRUE_RAND_BUF_SIZE ];

/*******************************************************************************
 * Functions
 */

// Random Number Generation
extern uint8_t LL_ENC_GeneratePseudoRandNum( void );
extern uint8_t LL_ENC_GenerateTrueRandNum( uint8_t *buf, uint8_t len );

extern void  LL_ENC_AES128_Decrypt( uint8_t *key, uint8_t *ciphertext, uint8_t *plaintext );
extern void  LL_ENC_LoadEmptyIV( void );
extern void  LL_ENC_ReverseBytes( uint8_t *buf, uint8_t len );
extern void  LL_ENC_GenDeviceSKD( uint8_t *SKD );
extern void  LL_ENC_GenDeviceIV( uint8_t *IV );
extern void  LL_ENC_GenerateNonce( uint32_t pktCnt, uint8_t direction, uint8_t *nonce );
extern void  LL_ENC_EncryptMsg( uint8_t *nonce, uint8_t pktLen, uint8_t *pbuf, uint8_t *mic );
extern void  LL_ENC_DecryptMsg( uint8_t *nonce, uint8_t pktLen, uint8_t *pBuf, uint8_t *mic );
extern void  LL_ENC_Encrypt( llConnState_t *connPtr, uint8_t pktHdr, uint8_t pktLen, uint8_t *pBuf );
extern uint8_t LL_ENC_Decrypt( llConnState_t *connPtr, uint8_t pktHdr, uint8_t pktLen, uint8_t *pBuf );
extern void LL_ENC_sm_ah( uint8_t *pK, uint8_t *pR, uint8_t *pAh );

/*******************************************************************************
    @fn          LL_ENC_AES128_Encrypt API

    @brief       This function takes a key, plaintext, and generates ciphertext
                by AES128 encryption. This function is used to generate the
                BLE Session Key (SK) from the Long Term Key (LTK) and Session
                Key Diversifier (SKD).

                Note: The array indexes are MSO..LSO for LTK, SKD, and SK.

    input parameters

    @param       key       - The 128 bit key to be used for encryption.
    @param       plaintext - The 128 bit plain text before encryption.

    output parameters

    @param       ciphertext - The 128 bit cipher text after encryption.

    @return      None.
*/
void LL_ENC_AES128_Encrypt0( uint8_t* key,
                             uint8_t* plaintext,
                             uint8_t* ciphertext );

uint8_t LL_ENC_GenerateTrueRandNum0( uint8_t* buf,
                                   uint8_t len );
void LL_ENC_GenDeviceSKD0( uint8_t* SKD );
void LL_ENC_GenDeviceIV0( uint8_t* IV );
void LL_ENC_GenerateNonce0( uint32_t pktCnt,
                            uint8_t  direction,
                            uint8_t*  nonce );
void LL_ENC_Encrypt0( llConnState_t* connPtr,
                      uint8_t          pktHdr,
                      uint8_t          pktLen,
                      uint8_t*         pBuf );
uint8_t LL_ENC_Decrypt0( llConnState_t* connPtr,
                       uint8_t          pktHdr,
                       uint8_t          pktLen,
                       uint8_t*         pBuf );

extern void  LL_ENC_MoveData( uint8_t *pDst, uint8_t *pSrc, uint16_t len );

extern void LL_ENC_LoadKey( uint8_t* key );

#ifdef __cplusplus
}
#endif

#endif /* LL_ENC_H */
