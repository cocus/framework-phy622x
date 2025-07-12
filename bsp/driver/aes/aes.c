/**
 ******************************************************************************
 * @file    aes.c
 * @author  Santiago Hormazabal
 * @brief   AES BSP module driver.
 *          This file provides firmware functions to encrypt or decrypt using
 *          AES-GCM (based on the HW engine that can only do ECB):
 *           +
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "aes.h"

#include <string.h>

#include <phy_error.h>

/** @addtogroup PHY62XX_BSP_Driver
 * @{
 */

/** @defgroup AES
 * @brief AES BSP module driver
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup AES_Private_Constants AES Private Constants
 * @{
 */

/**
 * @}
 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup AES_Private_Functions AES Private functions
  * @{
  */

// === GF(2^128) multiplication for GHASH ===
static void gcm_gf_mult(const uint8_t *X, const uint8_t *Y, uint8_t *result)
{
    uint8_t Z[16] = {0};
    uint8_t V[16];
    memcpy(V, Y, 16);

    for (int i = 0; i < 128; i++)
    {
        int byte = i / 8;
        int bit = 7 - (i % 8);
        if ((X[byte] >> bit) & 1)
        {
            for (int j = 0; j < 16; j++)
                Z[j] ^= V[j];
        }

        // Shift V right by 1 bit (big-endian)
        uint8_t lsb = V[15] & 1;
        for (int k = 15; k > 0; k--)
        {
            V[k] = (V[k] >> 1) | ((V[k - 1] & 1) << 7);
        }
        V[0] >>= 1;

        if (lsb)
        {
            V[0] ^= 0xe1;
        }
    }
    memcpy(result, Z, 16);
}

// GHASH update: S = (S ^ data_block) * H
static void gcm_ghash_update(uint8_t *S, const uint8_t *data, size_t len, const uint8_t *H)
{
    uint8_t block[16];
    for (size_t offset = 0; offset < len; offset += 16)
    {
        size_t block_len = (len - offset >= 16) ? 16 : (len - offset);
        memset(block, 0, 16);
        memcpy(block, data + offset, block_len);

        aes_xor_128(S, block, S);
        gcm_gf_mult(S, H, S);
    }
}

// Increment 32-bit big-endian counter in last 4 bytes of ctr
static void increment_ctr(uint8_t *ctr)
{
    for (int i = 15; i >= 12; i--)
    {
        if (++ctr[i] != 0)
            break;
    }
}

// Compute J0 for IV != 12 bytes
static void compute_J0(
    const uint8_t *iv, size_t iv_len,
    const uint8_t *H, // hash subkey
    uint8_t *J0       // output 16 bytes
)
{
    uint8_t S[16] = {0};

    // GHASH the IV blocks
    gcm_ghash_update(S, iv, iv_len, H);

    // Prepare length block: 64 bits zeros + 64 bits IV length in bits
    uint8_t len_block[16] = {0};
    uint64_t iv_bits = iv_len * 8;

    for (int i = 0; i < 8; i++)
    {
        len_block[15 - i] = (iv_bits >> (i * 8)) & 0xFF;
    }

    // GHASH length block
    aes_xor_128(S, len_block, S);
    gcm_gf_mult(S, H, S);

    memcpy(J0, S, 16);
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup AES_Exported_Functions AES related exported functions
  * @{
  */

void aes_gcm_128(
    const uint8_t *key,
    const uint8_t *iv, size_t iv_len,
    const uint8_t *aad, size_t aad_len,
    const uint8_t *input, size_t input_len, // plaintext or ciphertext
    uint8_t *output,                        // output buffer
    uint8_t *tag,                            // tag output
    int encrypt                             // 1 = encrypt, 0 = decrypt
)
{
    uint8_t H[16] = {0};
    uint8_t J0[16] = {0};
    uint8_t S[16] = {0};
    uint8_t tmp[16] = {0};
    uint8_t ctr[16] = {0};
    uint8_t block[16] = {0};

    // Step 1: Calculate H = AES_K(0^128)
    LL_ENC_AES128_Encrypt(key, H, H);

    // Step 2: Form J0 = IV || 0x00000001
    if (iv_len == 12)
    {
        memcpy(J0, iv, iv_len);
        J0[15] = 0x01;
    }
    else
    {
        compute_J0(iv, iv_len, H, J0);
    }

    // Step 3: GHASH AAD
    gcm_ghash_update(S, aad, aad_len, H);

    // Step 4: Encrypt input with CTR mode and GHASH ciphertext
    memcpy(ctr, J0, 16);
    increment_ctr(ctr);

    for (size_t offset = 0; offset < input_len; offset += 16)
    {
        size_t block_len = (input_len - offset >= 16) ? 16 : (input_len - offset);

        LL_ENC_AES128_Encrypt(key, ctr, block);
        increment_ctr(ctr);

        memcpy(tmp, input + offset, block_len);
        if (block_len < 16)
        {
            memset(tmp + block_len, 0, 16 - block_len);
        }

        if (encrypt)
        {
            for (size_t i = 0; i < block_len; i++)
            {
                output[offset + i] = input[offset + i] ^ block[i];
            }
            gcm_ghash_update(S, output + offset, block_len, H);
        }
        else
        {
            for (size_t i = 0; i < block_len; i++)
            {
                output[offset + i] = input[offset + i] ^ block[i];
            }
            gcm_ghash_update(S, input + offset, block_len, H); // GHASH original ciphertext
        }
    }

    // Step 5: GHASH length block: [aad_bits||ciphertext_bits] in bits, big-endian 64-bit each
    memset(tmp, 0, sizeof(tmp));
    uint64_t aad_bits = (uint64_t)aad_len * 8;
    uint64_t ct_bits = (uint64_t)input_len * 8;

    for (int i = 0; i < 8; i++)
    {
        tmp[7 - i] = (aad_bits >> (i * 8)) & 0xFF;
        tmp[15 - i] = (ct_bits >> (i * 8)) & 0xFF;
    }

    aes_xor_128(S, tmp, S);
    gcm_gf_mult(S, H, S);

    // Step 6: Compute tag = S ^ AES_K(J0)
    LL_ENC_AES128_Encrypt(key, J0, block);
    aes_xor_128(S, block, tag);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
