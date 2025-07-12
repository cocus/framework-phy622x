/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_AES_H
#define _HAL_AES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <phy62xx.h>

#include <stddef.h> /* size_t */

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup AES
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup AES_Exported_Types AES Exported Types
  * @{
  */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AES_Exported_Constants AES Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup AES_Exported_Macros AES Exported Macros
  * @{
  */

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup AES_Exported_Functions
  * @{
  */
    /**
      * @brief  Performs AES-128 encryption or decryption in GCM mode.
      * @param  key: Pointer to the 16-byte AES-128 key.
      * @param  iv: Pointer to the IV (nonce); can be 12 bytes (recommended) or longer.
      * @param  iv_len: Length of the IV in bytes.
      * @param  aad: Pointer to Additional Authenticated Data (AAD).
      * @param  aad_len: Length of the AAD in bytes.
      * @param  input: Pointer to input data (plaintext if encrypting, ciphertext if decrypting).
      * @param  input_len: Length of the input data in bytes.
      * @param  output: Pointer to output buffer (ciphertext or plaintext).
      * @param  tag: Pointer to tag buffer for 16-byte authentication tag.
      * @param  encrypt: Set to 1 for encryption, 0 for decryption.
      * @retval None. You'd need to check the tag in decryption mode for success.
      */
    void aes_gcm_128(
        const uint8_t *key,
        const uint8_t *iv, size_t iv_len,
        const uint8_t *aad, size_t aad_len,
        const uint8_t *input, size_t input_len,  // plaintext or ciphertext
        uint8_t *output,                         // output buffer
        uint8_t *tag,                            // tag output
        int encrypt                              // 1 = encrypt, 0 = decrypt
    );

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup AES_Exported_ROM_Functions AES Exported ROM Functions
  * @{
  */
    /**
      * @brief  Performs a XOR operation on two 16-bytes (128 bits) buffers into a resulting one.
      * @param  a: Source buffer A.
      * @param  b: Source buffer B.
      * @param  out: Destination buffer.
      * @retval None.
      */
    // --- XOR 128-bit blocks ---
    ATTR_ROM_FN void aes_xor_128(const uint8_t *a, const uint8_t *b, uint8_t *out);
    // AES 128 bit ECB encrypt function
    /**
      * @brief  AES-128 bit ECB encryption using the AES HW block.
      * @param  key: Key buffer, 16 bytes.
      * @param  plaintext: Source data buffer, 16 bytes.
      * @param  ciphertext: Output buffer, 16 bytes.
      * @retval None.
      */
    ATTR_ROM_FN void LL_ENC_AES128_Encrypt(const uint8_t *key, const uint8_t *plaintext, uint8_t *ciphertext);

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup AES_Exported_ROM_Variables AES Exported ROM Variables
  * @{
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _HAL_AES_H */
