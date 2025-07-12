/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_DMA_H
#define _HAL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */



/** Burst size in Source and Destination definitions */
#define DMA_BSIZE_1     ((0UL)) /**< Burst size = 1 */
#define DMA_BSIZE_4     ((1UL)) /**< Burst size = 4 */
#define DMA_BSIZE_8     ((2UL)) /**< Burst size = 8 */
#define DMA_BSIZE_16    ((3UL)) /**< Burst size = 16 */
#define DMA_BSIZE_32    ((4UL)) /**< Burst size = 32 */
#define DMA_BSIZE_64    ((5UL)) /**< Burst size = 64 */
#define DMA_BSIZE_128   ((6UL)) /**< Burst size = 128 */
#define DMA_BSIZE_256   ((7UL)) /**< Burst size = 256 */

/** Width in Source transfer width and Destination transfer width definitions */
#define DMA_WIDTH_BYTE      ((0UL)) /**< Width = 1 byte */
#define DMA_WIDTH_HALFWORD  ((1UL)) /**< Width = 2 bytes */
#define DMA_WIDTH_WORD      ((2UL)) /**< Width = 4 bytes */
#define DMA_WIDTH_2WORD     ((3UL)) /**< Width = 8 bytes */
#define DMA_WIDTH_4WORD     ((4UL)) /**< Width = 16 bytes */
#define DMA_WIDTH_8WORD     ((5UL)) /**< Width = 32 bytes */




/** DMAC Address Increment definitions */
#define DMA_INC_INC         ((0UL)) /**< Increment */
#define DMA_INC_DEC         ((1UL)) /**< Decrement */
#define DMA_INC_NCHG        ((2UL)) /**< No change */



/**
    @brief DMAC Channel configuration structure type definition
*/
typedef struct
{
    uint32_t    transf_size;    /**< Length/Size of transfer */

    uint8_t     sinc;
    uint8_t     src_tr_width;
    uint8_t     src_msize;
    uint32_t    src_addr;

    uint8_t     dinc;
    uint8_t     dst_tr_width;
    uint8_t     dst_msize;
    uint32_t    dst_addr;

    uint8_t     enable_int;
} DMA_CH_CFG_t;

typedef enum
{
    DMA_CH_0 = 0,
    DMA_CH_1,
    DMA_CH_2,
    DMA_CH_3,
    DMA_CH_NUM,
} DMA_CH_t;

/**
    @brief DMAC callback definition
*/
typedef void (*DMA_Hdl_t)(DMA_CH_t);

typedef struct
{
    DMA_CH_t    dma_channel;
    DMA_Hdl_t   evt_handler;
} HAL_DMA_t;


int hal_dma_init_channel(HAL_DMA_t cfg);
int hal_dma_config_channel(DMA_CH_t ch, DMA_CH_CFG_t* cfg);
int hal_dma_start_channel(DMA_CH_t ch);
int hal_dma_stop_channel(DMA_CH_t ch);
int hal_dma_wait_channel_complete(DMA_CH_t ch);
int hal_dma_status_control(DMA_CH_t ch);
int hal_dma_init(void);
int hal_dma_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* _HAL_DMA_H */

