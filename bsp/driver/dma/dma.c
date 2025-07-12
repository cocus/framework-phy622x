/**
  ******************************************************************************
  * @file    dma.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   DMA BSP module driver.
  *          This file provides firmware functions to manage the DMA features:
  *           + 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dma.h"

#include <phy62xx.h>

#include <driver/pwrmgr/pwrmgr.h> /* for hal_pwrmgr_register */

#include <driver/clock/clock.h> /* for hal_clock_gate */

#include <jump_function.h> /* for JUMP_FUNCTION */

#include <phy_error.h>

#include <stddef.h> /* for NULL */

#include <types.h> /* for BIT, subWriteReg */

#include <string.h> /* for memcpy */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup CLOCK
  * @brief Clock BSP module driver
  * @{
  */


/** DMAC Connection number definitions */
typedef enum
{
    DMA_CONN_MEM=0,//               ((0))           /*memory*/
    DMA_CONN_SPI0_Tx=1,//           ((0UL))         /** SSP0 Tx */
    DMA_CONN_SPI0_Rx,//             ((1UL))         /** SSP0 Rx */
    DMA_CONN_SPI1_Tx,//             ((2UL))         /** SSP1 Tx */
    DMA_CONN_SPI1_Rx,//             ((3UL))         /** SSP1 Rx */

    DMA_CONN_I2C0_Tx=9,//           ((8UL))         /** IIC0 Tx */
    DMA_CONN_I2C0_Rx,//             ((9UL))         /** IIC0 Rx */
    DMA_CONN_I2C1_Tx,//             ((10UL))        /** IIC1 Tx */
    DMA_CONN_I2C1_Rx,//             ((11UL))        /** IIC1 Rx */

    DMA_CONN_UART0_Tx,//            ((10UL))        /** UART0 Tx */
    DMA_CONN_UART0_Rx,//            ((11UL))        /** UART0 Rx */
    DMA_CONN_UART1_Tx,//            ((12UL))        /** UART1 Tx */
    DMA_CONN_UART1_Rx,//            ((13UL))        /** UART1 Rx */
} DMA_CONN_e;




/**
    @}
*/


/* Private Macros ------------------------------------------------------------- */
/** @defgroup GPDMA_Private_Macros GPDMA Private Macros
    @{
*/

/* --------------------- BIT DEFINITIONS -------------------------------------- */
/*********************************************************************//**
    Macro defines for DMA Interrupt Status register
 **********************************************************************/
#define DMA_DMACIntStat_Ch(n)           (((1UL<<n)&0xFF))
#define DMA_DMACIntStat_BITMASK         ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Terminal Count Request Status register
 **********************************************************************/
#define DMA_DMACIntTCStat_Ch(n)         (((1UL<<n)&0xFF))
#define DMA_DMACIntTCStat_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Terminal Count Request Clear register
 **********************************************************************/
#define DMA_DMACIntTCClear_Ch(n)        (((1UL<<n)&0xFF))
#define DMA_DMACIntTCClear_BITMASK  ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Error Status register
 **********************************************************************/
#define DMA_DMACIntErrStat_Ch(n)        (((1UL<<n)&0xFF))
#define DMA_DMACIntErrStat_BITMASK  ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Tfr Clear register
 **********************************************************************/
#define DMA_DMACIntTfrClr_Ch(n)     (((1UL<<n)&0xFF))
#define DMA_DMACIntTfrClr_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Block Clear register
 **********************************************************************/
#define DMA_DMACIntBlockClr_Ch(n)       (((1UL<<n)&0xFF))
#define DMA_DMACIntBlockClr_BITMASK     ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt SrcTran Clear register
 **********************************************************************/
#define DMA_DMACIntSrcTranClr_Ch(n)     (((1UL<<n)&0xFF))
#define DMA_DMACIntSrcTranClr_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt DstTran Clear register
 **********************************************************************/
#define DMA_DMACIntDstTranClr_Ch(n)     (((1UL<<n)&0xFF))
#define DMA_DMACIntDstTranClr_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Interrupt Error Clear register
 **********************************************************************/
#define DMA_DMACIntErrClr_Ch(n)     (((1UL<<n)&0xFF))
#define DMA_DMACIntErrClr_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Raw Interrupt Terminal Count Status register
 **********************************************************************/
#define DMA_DMACRawIntTCStat_Ch(n)  (((1UL<<n)&0xFF))
#define DMA_DMACRawIntTCStat_BITMASK    ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Raw Error Interrupt Status register
 **********************************************************************/
#define DMA_DMACRawIntErrStat_Ch(n) (((1UL<<n)&0xFF))
#define DMA_DMACRawIntErrStat_BITMASK   ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Enabled Channel register
 **********************************************************************/
#define DMA_DMACEnbldChns_Ch(n)     (((1UL<<n)&0xFF))
#define DMA_DMACEnbldChns_BITMASK       ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Software Burst Request register
 **********************************************************************/
#define DMA_DMACSoftBReq_Src(n)     (((1UL<<n)&0xFFFF))
#define DMA_DMACSoftBReq_BITMASK        ((0xFFFF))

/*********************************************************************//**
    Macro defines for DMA Software Single Request register
 **********************************************************************/
#define DMA_DMACSoftSReq_Src(n)         (((1UL<<n)&0xFFFF))
#define DMA_DMACSoftSReq_BITMASK        ((0xFFFF))

/*********************************************************************//**
    Macro defines for DMA Software Last Burst Request register
 **********************************************************************/
#define DMA_DMACSoftLBReq_Src(n)        (((1UL<<n)&0xFFFF))
#define DMA_DMACSoftLBReq_BITMASK       ((0xFFFF))

/*********************************************************************//**
    Macro defines for DMA Software Last Single Request register
 **********************************************************************/
#define DMA_DMACSoftLSReq_Src(n)        (((1UL<<n)&0xFFFF))
#define DMA_DMACSoftLSReq_BITMASK       ((0xFFFF))

/*********************************************************************//**
    Macro defines for DMA Configuration register
 **********************************************************************/
#define DMA_DMAC_E                      ((0x01))     /**< DMA Controller enable*/
#define DMA_DMAC_D                      ((0x00))     /**< DMA Controller disable*/
#define DMA_DMAC_INT_E                  ((0x01))     /**< DMA Controller Interrupt enable*/


/*********************************************************************//**
    Macro defines for DMA Synchronization register
 **********************************************************************/
#define DMA_DMACSync_Src(n)         (((1UL<<n)&0xFFFF))
#define DMA_DMACSync_BITMASK            ((0xFFFF))

/*********************************************************************//**
    Macro defines for DMA Request Select register
 **********************************************************************/
#define DMA_DMAReqSel_Input(n)      (((1UL<<(n-8))&0xFF))
#define DMA_DMAReqSel_BITMASK           ((0xFF))

/*********************************************************************//**
    Macro defines for DMA Channel Linked List Item registers
 **********************************************************************/
/** DMAC Channel Linked List Item registers bit mask*/
#define DMA_DMACCxLLI_BITMASK       ((0xFFFFFFFC))

/*********************************************************************//**
    Macro defines for DMA channel control registers
 **********************************************************************/
/** Transfer size*/
#define DMA_DMACCxControl_TransferSize(n) (n&0x7FF)
/** Source burst size*/
#define DMA_DMACCxControl_SMSize(n)     (((n&0x07)<<14))
/** Destination burst size*/
#define DMA_DMACCxControl_DMSize(n)     (((n&0x07)<<11))
/** Source transfer width*/
#define DMA_DMACCxControl_SWidth(n)     (((n&0x07)<<4))
/** Destination transfer width*/
#define DMA_DMACCxControl_DWidth(n)     (((n&0x07)<<1))
/** Source increment*/
#define DMA_DMACCxControl_SInc(n)       (((n&0x03)<<9))
/** Destination increment*/
#define DMA_DMACCxControl_DInc(n)       (((n&0x03)<<7))

/*********************************************************************//**
    Macro defines for DMA Channel Configuration registers
 **********************************************************************/
/** DMAC channel write bit enable*/
#define DMA_DMACCxConfig_E(n)                       ((0x100UL<<n))
/** DMAC channel int mask bit enable*/
#define DMA_DMACCxIntMask_E(n)                      ((0x100UL<<n))

/** Source peripheral*/
#define DMA_DMACCxConfig_SrcPeripheral(n)       (((n&0x1F)))
/** Destination peripheral*/
#define DMA_DMACCxConfig_DestPeripheral(n)      (((n&0x1F)<<4))
/** This value indicates the type of transfer*/
#define DMA_DMACCxConfig_TransferType(n)        (((n&0x7)<<20))
/** Interrupt error mask*/
#define DMA_DMACCxConfig_IE                     ((1UL<<14))
/** Terminal count interrupt mask*/
#define DMA_DMACCxConfig_ITC                    ((1UL<<15))
/** Lock*/
#define DMA_DMACCxConfig_L                      ((1UL<<16))
/** Active*/
#define DMA_DMACCxConfig_A                      ((1UL<<17))
/** Halt*/
#define DMA_DMACCxConfig_H                      ((1UL<<18))
/** DMAC Channel Configuration registers bit mask */
#define DMA_DMACCxConfig_BITMASK                ((0x7FFFF))

/**
    @}
*/
#define DMA_TRANSFERTYPE_M2M            ((0UL))
/** DMAC Transfer type definitions: Memory to peripheral - DMA control */
#define DMA_TRANSFERTYPE_M2P            ((1UL))
/** DMAC Transfer type definitions: Peripheral to memory - DMA control */
#define DMA_TRANSFERTYPE_P2M            ((2UL))
/** DMAC Source peripheral to destination peripheral - DMA control */
#define DMA_TRANSFERTYPE_P2P            ((3UL))

/** If dstnation address is flash, should set this bit- DMA control */
#define DMA_DST_XIMT_IS_FLASH           ((0UL))
/** DMAC Source peripheral to destination peripheral - DMA control */
#define DMA_DST_XIMT_NOT_FLASH          ((1UL))

#define DMA_GET_MAX_TRANSPORT_SIZE(ch)      ((ch == DMA_CH_0) ? 0x7ff : 0x1f)




typedef struct
{
    uint8_t     init_ch;
    uint8_t     interrupt;
    uint8_t     xmit_busy;
    uint8_t     xmit_flash;
    DMA_Hdl_t   evt_handler;
} DMA_CH_Ctx_t;


typedef struct
{
    uint8_t init_flg;
    DMA_CH_Ctx_t dma_ch_ctx[DMA_CH_NUM];
} dma_ctx_t;


dma_ctx_t s_dma_ctx =
{
    .init_flg = 0,
};

static DMA_CONN_e get_src_conn(uint32_t addr)
{
    if(addr == (uint32_t)&(AP_SPI0->DataReg))
        return DMA_CONN_SPI0_Rx;

    if(addr == (uint32_t)&(AP_SPI1->DataReg))
        return DMA_CONN_SPI1_Rx;

    if(addr == (uint32_t)&(AP_I2C0->IC_DATA_CMD))
        return DMA_CONN_I2C0_Rx;

    if(addr == (uint32_t)&(AP_I2C1->IC_DATA_CMD))
        return DMA_CONN_I2C1_Rx;

    if(addr == (uint32_t)&(AP_UART0->RBR))
        return DMA_CONN_UART0_Rx;

    if(addr == (uint32_t)&(AP_UART1->RBR))
        return DMA_CONN_UART1_Rx;

    return DMA_CONN_MEM;
}

static DMA_CONN_e get_dst_conn(uint32_t addr)
{
    if(addr == (uint32_t)&(AP_SPI0->DataReg))
        return DMA_CONN_SPI0_Tx;

    if(addr == (uint32_t)&(AP_SPI1->DataReg))
        return DMA_CONN_SPI1_Tx;

    if(addr == (uint32_t)&(AP_I2C0->IC_DATA_CMD))
        return DMA_CONN_I2C0_Tx;

    if(addr == (uint32_t)&(AP_I2C1->IC_DATA_CMD))
        return DMA_CONN_I2C1_Tx;

    if(addr == (uint32_t)&(AP_UART0->THR))
        return DMA_CONN_UART0_Tx;

    if(addr == (uint32_t)&(AP_UART1->THR))
        return DMA_CONN_UART1_Tx;

    return DMA_CONN_MEM;
}

static void __attribute__((used)) hal_DMA_IRQHandler(void)
{
    DMA_CH_t ch;

    for(ch = DMA_CH_0; ch < DMA_CH_NUM; ch++)
    {
        if(AP_DMA->INT.StatusTfr & BIT(ch))
        {
            hal_dma_stop_channel(ch);

            if(s_dma_ctx.dma_ch_ctx[ch].evt_handler != NULL)
            {
                s_dma_ctx.dma_ch_ctx[ch].evt_handler(ch);
            }
        }
    }
}

static void dma_wakeup_handler(void)
{
    hal_clk_gate_enable(MOD_DMA);
    NVIC_SetPriority((IRQn_Type)DMAC_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)DMAC_IRQn);
    JUMP_FUNCTION(DMAC_IRQ_HANDLER)      =   (uint32_t)&hal_DMA_IRQHandler;
    AP_DMA->MISC.DmaCfgReg = DMA_DMAC_E;
}


int hal_dma_init_channel(HAL_DMA_t cfg)
{
    DMA_CH_Ctx_t* pctx;
    DMA_CH_t ch;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    ch = cfg.dma_channel;

    if(ch >= DMA_CH_NUM)
        return PPlus_ERR_INVALID_PARAM;

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx ->init_ch)
        return PPlus_ERR_INVALID_STATE;

    pctx->evt_handler = cfg.evt_handler;
    pctx->init_ch = 1;
    return PPlus_SUCCESS;
}


int hal_dma_config_channel(DMA_CH_t ch, DMA_CH_CFG_t* cfg)
{
    DMA_CH_Ctx_t* pctx;
    DMA_CONN_e src_conn,dst_conn;
    uint32_t cctrl = 0;
    uint32_t transf_type = DMA_TRANSFERTYPE_M2M;
    uint32_t transf_per = 0;
    uint32_t spif_protect = AP_SPIF->wr_protection;
    uint32_t cache_bypass = AP_PCR->CACHE_BYPASS;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(!pctx->init_ch)
        return PPlus_ERR_INVALID_STATE;

    if ((AP_DMA->MISC.ChEnReg & (DMA_DMACEnbldChns_Ch(ch))) || \
            (pctx->xmit_busy))
    {
        // This channel is enabled, return ERROR, need to release this channel first
        return PPlus_ERR_BUSY;
    }

    // Reset the Interrupt status
    AP_DMA->INT.ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
    // UnMask interrupt
    AP_DMA->INT.MaskTfr = DMA_DMACCxIntMask_E(ch);
    src_conn = get_src_conn(cfg->src_addr);
    dst_conn = get_dst_conn(cfg->dst_addr);

    /* Assign Linker List Item value */
    if(src_conn && dst_conn)
    {
        transf_type = DMA_TRANSFERTYPE_P2P;
        transf_per = DMA_DMACCxConfig_SrcPeripheral((src_conn-1))| \
                     DMA_DMACCxConfig_DestPeripheral((dst_conn-1));
    }
    else if(src_conn)
    {
        transf_type = DMA_TRANSFERTYPE_P2M;
        transf_per = DMA_DMACCxConfig_SrcPeripheral((src_conn-1));
    }
    else if(dst_conn)
    {
        transf_type = DMA_TRANSFERTYPE_M2P;
        transf_per = DMA_DMACCxConfig_DestPeripheral((dst_conn-1));
    }

    if((cfg->dst_addr > 0x11000000) && (cfg->dst_addr <= 0x11080000)) // 512 k
    {
        pctx->xmit_flash = DMA_DST_XIMT_IS_FLASH;

        if(spif_protect)
        {
            AP_SPIF->wr_protection = 0;
        }

        if(cache_bypass == 0)
        {
            AP_PCR->CACHE_BYPASS = 1;
        }
    }
    else
    {
        pctx->xmit_flash = DMA_DST_XIMT_NOT_FLASH;
    }

    AP_DMA->CH[ch].SAR = cfg->src_addr;
    AP_DMA->CH[ch].DAR = cfg->dst_addr;
    AP_DMA->CH[ch].LLP = 0;

    if(DMA_GET_MAX_TRANSPORT_SIZE(ch) < cfg->transf_size)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    AP_DMA->CH[ch].CTL_H = DMA_DMACCxControl_TransferSize(cfg->transf_size);
    subWriteReg(&(AP_DMA->CH[ch].CFG_H),15,7,transf_per);
    AP_DMA->CH[ch].CFG = 0;
    cctrl = DMA_DMACCxConfig_TransferType(transf_type)| \
            DMA_DMACCxControl_SMSize(cfg->src_msize)| \
            DMA_DMACCxControl_DMSize(cfg->dst_msize)| \
            DMA_DMACCxControl_SWidth(cfg->src_tr_width)| \
            DMA_DMACCxControl_DWidth(cfg->dst_tr_width)| \
            DMA_DMACCxControl_SInc(cfg->sinc)| \
            DMA_DMACCxControl_DInc(cfg->dinc)| \
            DMA_DMAC_INT_E;
    AP_DMA->CH[ch].CTL = cctrl;

    if(cfg->enable_int)
    {
        AP_DMA->INT.MaskTfr = DMA_DMACCxConfig_E(ch) | BIT(ch);
        pctx->interrupt = 1;
    }
    else
    {
        AP_DMA->INT.ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
        AP_DMA->INT.MaskTfr = DMA_DMACCxIntMask_E(ch);
        pctx->interrupt = 0;
    }

    return PPlus_SUCCESS;
}

int hal_dma_start_channel(DMA_CH_t ch)
{
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    pctx = &s_dma_ctx.dma_ch_ctx[ch];
    AP_DMA->MISC.ChEnReg = DMA_DMACCxConfig_E(ch) | BIT(ch);
    pctx->xmit_busy = 1;
    hal_pwrmgr_lock(MOD_DMA);
    return PPlus_SUCCESS;
}

int hal_dma_stop_channel(DMA_CH_t ch)
{
    uint32_t spif_protect = AP_SPIF->wr_protection;
    uint32_t cache_bypass = AP_PCR->CACHE_BYPASS;
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx->xmit_flash == DMA_DST_XIMT_IS_FLASH)
    {
        if(spif_protect)
        {
            AP_SPIF->wr_protection = 2;
        }

        if(cache_bypass == 0)
        {
            AP_PCR->CACHE_BYPASS = 0;
            AP_CACHE->CTRL0 = 0x01;
        }
    }

    // Reset the Interrupt status
    AP_DMA->INT.ClearTfr = DMA_DMACIntTfrClr_Ch(ch);
    // UnMask interrupt
//    AP_DMA->INT.MaskTfr = DMA_DMACCxIntMask_E(ch);
    AP_DMA->MISC.ChEnReg = DMA_DMACCxConfig_E(ch);
    pctx->xmit_busy = 0;
    hal_pwrmgr_unlock(MOD_DMA);
    return PPlus_SUCCESS;
}

int hal_dma_status_control(DMA_CH_t ch)
{
    DMA_CH_Ctx_t* pctx;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    if(ch >= DMA_CH_NUM)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    pctx = &s_dma_ctx.dma_ch_ctx[ch];

    if(pctx->interrupt == 0)
        hal_dma_wait_channel_complete(ch);

    return PPlus_SUCCESS;
}

int hal_dma_wait_channel_complete(DMA_CH_t ch)
{
    uint32_t Temp = 0;

    if(!s_dma_ctx.init_flg)
        return PPlus_ERR_NOT_REGISTED;

    while(1)
    {
        Temp ++;

        if(AP_DMA->INT.RawTfr)
        {
            break;
        }
    }

    hal_dma_stop_channel(ch);
    // LOG("wait count is %d\n",Temp);
    return PPlus_SUCCESS;
}

int hal_dma_init(void)
{
    uint8_t ret;
    hal_clk_gate_enable(MOD_DMA);
    hal_clk_reset(MOD_DMA);
    NVIC_SetPriority((IRQn_Type)DMAC_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)DMAC_IRQn);
    JUMP_FUNCTION(DMAC_IRQ_HANDLER)      =   (uint32_t)&hal_DMA_IRQHandler;
    ret = hal_pwrmgr_register(MOD_DMA,NULL, dma_wakeup_handler);

    if(ret == PPlus_SUCCESS)
    {
        s_dma_ctx.init_flg = 1;
        memset(&(s_dma_ctx.dma_ch_ctx[0]), 0, sizeof(DMA_CH_Ctx_t)*DMA_CH_NUM);
        //dmac controller enable
        AP_DMA->MISC.DmaCfgReg = DMA_DMAC_E;
    }

    return ret;
}

int hal_dma_deinit(void)
{
    //dmac controller disable
    AP_DMA->MISC.DmaCfgReg = DMA_DMAC_D;
    s_dma_ctx.init_flg = 0;
    memset(&(s_dma_ctx.dma_ch_ctx[0]), 0, sizeof(DMA_CH_Ctx_t)*DMA_CH_NUM);
    hal_pwrmgr_unregister(MOD_DMA);
    hal_clk_gate_disable(MOD_DMA);
    return PPlus_SUCCESS;
}





