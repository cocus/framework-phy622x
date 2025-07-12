

#include <phy62xx.h>

#include <driver/clock/clock.h>

#include <phy_error.h>

#include <ble/controller/ll.h>
#include <ble/controller/ll_enc.h>

#include <stddef.h>
#include <stdbool.h>


void LL_ENC_AES128_Encrypt1( uint8_t* key,
                             uint8_t* plaintext,
                             uint8_t* ciphertext )
{
    //only turn on while working
    hal_clk_gate_enable(MOD_AES);
    LL_ENC_AES128_Encrypt0(key,plaintext,ciphertext);
    hal_clk_gate_disable(MOD_AES);
}

#define LL_ENC_BASE         0x40040000               // LL HW AES engine Base address  

#define LL_ENC_ENCRYPT_DONE_MASK        0x0001
#define LL_ENC_DECRYPT_FAIL_MASK        0x0002
#define LL_ENC_DECRYPT_SUCC_MASK        0x0004
#define LL_ENC_SINGLE_MODE_DONE_MASK    0x0008

void  LL_ENC_Encrypt1( llConnState_t* connPtr, uint8_t pktHdr, uint8_t pktLen, uint8_t* pBuf )
{
    hal_clk_gate_enable(MOD_AES);
//    LL_ENC_Encrypt0(connPtr,  pktHdr,  pktLen, pBuf );
    {
        uint8_t* pByte = NULL;
        uint16_t index;
        int i, len;
        uint32_t temp;
        // disable AES
        AP_AES->LAYER_ENABLE = 0x0;
        // Load Key
        // Note: Normally this would only need to be done once when the SK is derived
        //       from the LTK and SKD. However, when in sleep, the AES block loses
        //       this key. Also, when multiple connections are supported, the key
        //       will be different.
        LL_ENC_LoadKey( connPtr->encInfo.SK );

//      if ( llState == LL_STATE_CONN_MASTER )
        if( connPtr->llTbd1 == LL_LINK_CONNECT_COMPLETE_MASTER )
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.txPktCount,
                                  LL_ENC_TX_DIRECTION_MASTER,
                                  connPtr->encInfo.nonce );
        }
        else // assumed llState == LL_STATE_CONN_SLAVE
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.txPktCount,
                                  LL_ENC_TX_DIRECTION_SLAVE,
                                  connPtr->encInfo.nonce );
        }

        // confiig nounce
        pByte = connPtr->encInfo.nonce;
        AP_AES->NONCE[3] = pByte[0] ;
        pByte ++;
        AP_AES->NONCE[2] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        AP_AES->NONCE[1] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        AP_AES->NONCE[0] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        // config plen & aad
        AP_AES->PLEN_AND_AAD = (pktLen << 8) | pktHdr;
        // write packet to FIFO
        len = pktLen;
        index = 0;

        while (len >= 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 3] << 24 | pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
            len -= 4;
        }

        // to check the byte order
        if(len == 3)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
        }
        else if(len == 2)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 1] << 8 | pBuf[index] ;
            index += 4;
        }
        else if(len == 1)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index] ;
            index += 4;
        }

        // AES FIFO legth is 256 bytes, set other bytes 0
        for (i = index; i < 0x100; i += 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + i) = 0x0;
        }

        // set AES ctrl reg
        AP_AES->LAYER_CONTROL = 0xf00;
        // set interrupt enable
        AP_AES->INTERRUPT_MASK = 0xf;
        // enable AES
        AP_AES->LAYER_ENABLE = 0x1;

        // insert delay
        //    delay = 200;
        //    while (delay --);

        // query AES interrupt status register
        while (AP_AES->INTERRUPT_STATUS == 0) ;

        // disable AES, if not disable AES, there is no output in FIFO
        AP_AES->LAYER_ENABLE = 0x0;
        // read back the encrypt result
        index = 0;
        len = pktLen + 4;     // include 4 bytes MIC

        while (len > 0)
        {
            temp = *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index);
            pBuf[index ++] = temp & 0xff;
            pBuf[index ++] = (temp >> 8) & 0xff;
            pBuf[index ++] = (temp >> 16) & 0xff;
            pBuf[index ++] = (temp >> 24) & 0xff;
            len -= 4;
        }

        // up the count for the next TX'ed data packet
        // Note: This is supposed to be 39 bit counter, but for now, we don't
        //       envision receiving 550 billion packets during a connection!
        connPtr->encInfo.txPktCount++;
//      return;
    }
    hal_clk_gate_disable(MOD_AES);
}

uint8_t LL_ENC_Decrypt1( llConnState_t* connPtr, uint8_t pktHdr, uint8_t pktLen, uint8_t* pBuf )
{
    hal_clk_gate_enable(MOD_AES);
//    uint8_t ret = LL_ENC_Decrypt0( connPtr,  pktHdr,  pktLen, pBuf );
    {
        uint8_t* pByte = NULL;
        uint16_t index;
        int i, len;
        uint32_t temp;
        // disable AES
        AP_AES->LAYER_ENABLE = 0x0;
        // Load Key
        // Note: Normally this would only need to be done once when the SK is derived
        //       from the LTK and SKD. However, when in sleep, the AES block loses
        //       this key. Also, when multiple connections are supported, the key
        //       will be different.
        LL_ENC_LoadKey( connPtr->encInfo.SK );

//    if ( llState == LL_STATE_CONN_MASTER )
        if( connPtr->llTbd1 == LL_LINK_CONNECT_COMPLETE_MASTER )
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.rxPktCount,
                                  LL_ENC_RX_DIRECTION_MASTER,
                                  connPtr->encInfo.nonce );
        }
        else // assumed llState == LL_STATE_CONN_SLAVE
        {
            // generate the nonce based on packet count, IV, and direction
            LL_ENC_GenerateNonce( connPtr->encInfo.rxPktCount,
                                  LL_ENC_RX_DIRECTION_SLAVE,
                                  connPtr->encInfo.nonce );
        }

        // confiig nounce
        pByte = connPtr->encInfo.nonce;
        AP_AES->NONCE[3] = pByte[0]; // << 24 ;
        pByte ++;
        AP_AES->NONCE[2] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        AP_AES->NONCE[1] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        pByte += 4;
        AP_AES->NONCE[0] = pByte[0] << 24 | pByte[1] << 16 | pByte[2] << 8 | pByte[3];
        // config plen & aad
        AP_AES->PLEN_AND_AAD = (pktLen << 8) | pktHdr;
        // write packet to FIFO
        len = pktLen + 4;       // decrypt, add 4 for MIC field length
        index = 0;

        while (len >= 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 3] << 24 | pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
            len -= 4;
        }

        // fill others bytes < 1 word
        if(len == 3)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 2] << 16 |  pBuf[index + 1] << 8 | pBuf[index];
            index += 4;
        }
        else if(len == 2)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index + 1] << 8 | pBuf[index] ;
            index += 4;
        }
        else if(len == 1)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index)
                = pBuf[index] ;
            index += 4;
        }

        // AES FIFO legth is 256 bytes, set other bytes 0
        for (i = index; i < 0x100; i += 4)
        {
            *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + i) = 0x0;
        }

        // set AES ctrl reg
        AP_AES->LAYER_CONTROL = 0xf08;
        // set interrupt enable
        AP_AES->INTERRUPT_MASK = 0xf;
        // enable AES
        AP_AES->LAYER_ENABLE = 0x1;

        // insert delay
//    delay = 200;
//    while (delay --);

        // query AES interrupt status register and wait decrypt finish
        while (AP_AES->INTERRUPT_STATUS == 0) ;

        // read interrupt status reg
        temp = AP_AES->INTERRUPT_STATUS;

        if ((temp & LL_ENC_DECRYPT_FAIL_MASK)
                || ((temp & LL_ENC_DECRYPT_SUCC_MASK) == 0))
        {
            hal_clk_gate_disable(MOD_AES);
            return false;
        }

        // disable AES
        AP_AES->LAYER_ENABLE = 0x0;
        // read the decrypt result
        index = 0;
        len = pktLen;

        while (len > 0)
        {
            temp = *(volatile uint32_t*)(LL_ENC_BASE + 0x0100 + index);
            pBuf[index ++] = temp & 0xff;
            pBuf[index ++] = (temp >> 8) & 0xff;
            pBuf[index ++] = (temp >> 16) & 0xff;
            pBuf[index ++] = (temp >> 24) & 0xff;
            len -= 4;
        }

        // up the count for the next RX'ed data packet
        // Note: This is supposed to be 39 bit counter, but for now, we don't
        //       envision receiving 550 billion packets during a connection!
        connPtr->encInfo.rxPktCount++;
        hal_clk_gate_disable(MOD_AES);
        return( true );
    }
//    AP_PCR->SW_CLK    &= ~BIT(MOD_AES);
//    return ret;
}
