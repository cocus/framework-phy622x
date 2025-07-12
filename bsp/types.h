
#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>

#define BIT(n) (1ul << (n))

#define write_reg(addr, data) (*(volatile unsigned int *)(addr) = (unsigned int)(data))
// subWriteReg: write value to register zone: bit[high:low]
#define subWriteReg(addr, high, low, value) write_reg(addr, (read_reg(addr) &                                                 \
                                                             (~((((unsigned int)1 << ((high) - (low) + 1)) - 1) << (low)))) | \
                                                                ((unsigned int)(value) << (low)))


#define read_reg(addr) (*(volatile unsigned int *)(addr))

// bit operations
#define BM_SET(addr, bit) (*(addr) |= (bit))   // bit set
#define BM_CLR(addr, bit) (*(addr) &= ~(bit))  // bit clear
#define BM_IS_SET(addr, bit) (*(addr) & (bit)) // judge bit is set

#ifndef BV
#define BV(n) (1 << (n))
#endif

#ifndef BF
#define BF(x, b, s) (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n, m) (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n, m) (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n) (((n) < 0) ? -(n) : (n))
#endif

/* takes a byte out of a uint32_t : var - uint32_t,  ByteNum - byte to take out (0 - 3) */
#define BREAK_UINT32(var, ByteNum) \
    (uint8_t)((uint32_t)(((var) >> ((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
    ((uint32_t)((uint32_t)((Byte0) & 0x00FF) + ((uint32_t)((Byte1) & 0x00FF) << 8) + ((uint32_t)((Byte2) & 0x00FF) << 16) + ((uint32_t)((Byte3) & 0x00FF) << 24)))

#define BUILD_UINT16(loByte, hiByte) \
    ((uint16_t)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define BUILD_UINT8(hiByte, loByte) \
    ((uint8_t)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

// Write the 32bit value of 'val' in little endian format to the buffer pointed
// to by pBuf, and increment pBuf by 4
#define UINT32_TO_BUF_LITTLE_ENDIAN(pBuf, val) \
    do                                         \
    {                                          \
        *(pBuf)++ = (((val) >> 0) & 0xFF);     \
        *(pBuf)++ = (((val) >> 8) & 0xFF);     \
        *(pBuf)++ = (((val) >> 16) & 0xFF);    \
        *(pBuf)++ = (((val) >> 24) & 0xFF);    \
    } while (0)

// Return the 32bit little-endian formatted value pointed to by pBuf, and increment pBuf by 4
#define BUF_TO_UINT32_LITTLE_ENDIAN(pBuf) (((pBuf) += 4), BUILD_UINT32((pBuf)[-4], (pBuf)[-3], (pBuf)[-2], (pBuf)[-1]))

/*  ------------------------------------------------------------------------------------------------
                                          Standard Defines
    ------------------------------------------------------------------------------------------------
*/

#ifndef UNUSED
#define UNUSED(x) (void)(x) // To avoid unused variable warnings
#endif

#define __ATTR_SECTION_SRAM__ __attribute__((section("_section_sram_code_")))
#define __ATTR_SECTION_XIP__ __attribute__((section("_section_xip_code_")))

#endif // _TYPES_H_
