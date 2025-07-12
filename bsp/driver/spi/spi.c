/**
  ******************************************************************************
  * @file    spi.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   SPI BSP module driver.
  *          This file provides firmware functions to manage the SPI modules
  *          on the MCU, the pins related to the SPIs, and provides a convenient
  *          API for sending/receiving data with them.
  *           + Initialize a SPI module
  *           + Send data through a SPI module, busy-waiting or with interrupts
  *           + Send/Receive data, busy-waiting or with interrupts
  *           + Send/Receive data to the internal? EEPROM
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <phy62xx.h>

#include "spi.h"

#include <driver/pwrmgr/pwrmgr.h>
#include <driver/clock/clock.h> /* for getMcuPrecisionCount */
#ifdef DMAC_USE
#include <driver/dma/dma.h>
#endif

#include <osal/osal_critical.h>

#include <log/log.h>

#include <jump_function.h>

#include <phy_error.h>

#include <string.h>

#include <types.h> /* for BIT, UNUSED, subWriteReg */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup SPI
  * @brief SPI BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @addtogroup SPI_Private_Typedef SPI Private Typedefs
  * @{
  */

/**
 * @brief  PHY62xx SPI transmit/receive mode.
 */
typedef enum
{
    SPI_TRXD = 0,               /*!< Transmit & Receive */
    SPI_TXD = 1,                /*!< Transmit Only */
    SPI_RXD = 2,                /*!< Receive Only */
    SPI_EEPROM = 3,             /*!< EEPROM access */
} SPI_TMOD_e;

/**
 * @brief  PHY62xx SPI working mode.
 */
typedef enum
{
    SPI_MASTER = 0U,            /*!< SPI is in Master mode */
    SPI_SLAVE = 1U,             /*!< SPI is in Slave mode */
} spi_ctx_mode_t;

/**
 * @brief  PHY62xx SPI context structure status.
 */
typedef enum
{
    SPI_CTX_UNINITIALIZED = 0U, /*!< Context is not initialized */
    SPI_CTX_INITIALIZED = 1U,   /*!< Context is initialized */
} spi_ctx_init_t;

/**
 * @brief  PHY62xx SPI transaction state structure.
 */
typedef struct
{
    spi_xmit_state_t state;     /*!< Status of the transaction */
    uint16_t xmit_len;          /*!< Size (in words) of the transmission/receive buffer */
    uint8_t *tx_buf;            /*!< Pointer to the transmit buffer */
    uint8_t *rx_buf;            /*!< Pointer to the receive buffer */
    uint16_t tx_offset;         /*!< Current offset (position) on the transmit buffer */
    uint16_t rx_offset;         /*!< Current offset (position) on the receive buffer */
    uint16_t pattern;           /*!< User-defined pattern to send repeatdly when tx_buf is NULL */
} spi_xmit_t;

/**
 * @brief  PHY62xx SPI context structure.
 */
typedef struct
{
    spi_ctx_init_t state;       /*!< Status of the context (for a given SPIx) */
    spi_Cfg_t cfg;              /*!< Copy of the user-provided configuration of the SPIx */
    spi_ctx_mode_t mode;        /*!< Mode of operation, master or slave */
    spi_xmit_t transmit;        /*!< Transaction context */
} spi_Ctx_t;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/** @addtogroup SPI_Private_Constants SPI Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @addtogroup SPI_Private_Constants SPI Private macros
  * @{
  */

#define SPI_HDL_VALIDATE(hdl)                       \
{                                                   \
    if ((hdl != SPI0) && (hdl != SPI1))             \
    {                                               \
        return PPlus_ERR_INVALID_PARAM;             \
    }                                               \
    if (m_spiCtx[hdl].state != SPI_CTX_INITIALIZED) \
    {                                               \
        return PPlus_ERR_NOT_REGISTED;              \
    }                                               \
}

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup SPI_Private_Variables SPI Private variables
  * @{
  */
static spi_Ctx_t m_spiCtx[2] =
{
    { SPI_CTX_UNINITIALIZED, },
    { SPI_CTX_UNINITIALIZED, },
};

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup SPI_Private_Functions SPI Private functions
  * @{
  */

#ifdef USE_SLAVE
static void spis_int_handle(uint8_t id, spi_Ctx_t *pctx, AP_SSI_TypeDef *Ssix)
{
    volatile uint8_t spi_irs_status;
    spi_xmit_t *trans_ptr;
    spi_evt_t evt;
    uint16_t i, cnt;
    trans_ptr = &(pctx->transmit);
    spi_irs_status = Ssix->ISR;

    if (spi_irs_status & TRANSMIT_FIFO_EMPTY)
    {
        cnt = 8 - Ssix->TXFLR;

        for (i = 0; i < cnt; i++)
        {
            if (trans_ptr->tx_offset == trans_ptr->xmit_len)
            {
                Ssix->IMR = 0x10;
                break;
            }

            if (trans_ptr->tx_buf)
            {
                Ssix->DataReg = trans_ptr->tx_buf[trans_ptr->tx_offset++];
            }
            else
            {
                trans_ptr->tx_offset++;
                Ssix->DataReg = 0;
            }
        }
    }

    if (spi_irs_status & RECEIVE_FIFO_FULL)
    {
        volatile uint32_t garbage;
        cnt = Ssix->RXFLR;

        if (trans_ptr->rx_buf)
        {
            for (i = 0; i < cnt; i++)
            {
                if (trans_ptr->xmit_len > trans_ptr->rx_offset)
                    trans_ptr->rx_buf[trans_ptr->rx_offset++] = Ssix->DataReg;
                else
                    garbage = Ssix->DataReg;
                UNUSED(garbage);
            }
        }
        else
        {
            uint8_t rxbuf[16];

            if (trans_ptr->state == SPI_XMIT_BUSY)
                trans_ptr->rx_offset += cnt;

            for (i = 0; i < cnt; i++)
            {
                *(rxbuf + i) = Ssix->DataReg;
            }

            evt.id = id;
            evt.evt = SPI_RX_DATA_S;
            evt.data = rxbuf;
            evt.len = cnt;
            pctx->cfg.evt_handler(&evt);
        }

        if (trans_ptr->state == SPI_XMIT_BUSY && trans_ptr->rx_offset >= trans_ptr->xmit_len)
        {
            memset(trans_ptr, 0, sizeof(spi_xmit_t));
            evt.id = id;
            evt.evt = SPI_RX_COMPLETED;
            evt.data = NULL;
            evt.len = cnt;
            pctx->cfg.evt_handler(&evt);
            evt.evt = SPI_TX_COMPLETED;
            pctx->cfg.evt_handler(&evt);
        }
    }
}
#endif


static void spi_write_fifo(AP_SSI_TypeDef *Ssix, uint8_t len, uint8_t *tx_rx_ptr)
{
    uint8_t i = 0;
    SPI_INDEX_e spi_index = (Ssix == AP_SPI0) ? SPI0 : SPI1;

    /* TODO!!!: is it really needed to disable interrupts??? */
    __disable_irq();

    while (i < len)
    {
        if (m_spiCtx[spi_index].cfg.spi_dfsmod <= SPI_1BYTE)
        {
            Ssix->DataReg = *(tx_rx_ptr + i);
        }
        else
        {
            Ssix->DataReg = *((uint16_t *)tx_rx_ptr + i);
        }

        i++;
    }

    __enable_irq();
}

static void spi_go_busy(SPI_INDEX_e spi)
{
    m_spiCtx[spi].transmit.state = SPI_XMIT_BUSY;

    if (m_spiCtx[spi].cfg.force_cs == SPI_FORCE_CS_ENABLED /*&& m_spiCtx[spi].mode == SPI_MASTER*/)
    {
        /* TODO!!!: get rid of this on this function */
        hal_gpio_fmux(m_spiCtx[spi].cfg.ssn_pin, Bit_DISABLE);
        /* TODO!!!: this one is ok, but pin needs to be set as output */
        hal_gpio_write(m_spiCtx[spi].cfg.ssn_pin, 0);
    }
}

static void spi_go_idle(SPI_INDEX_e spi)
{
    if (m_spiCtx[spi].cfg.force_cs == SPI_FORCE_CS_ENABLED && m_spiCtx[spi].mode == SPI_MASTER)
    {
        /* TODO!!!: this one is ok, but pin needs to be set as output */
        hal_gpio_write(m_spiCtx[spi].cfg.ssn_pin, 1);

        /* TODO!!!: shouldn't it also write the pin as 1? */
        hal_gpio_fmux(m_spiCtx[spi].cfg.ssn_pin, Bit_ENABLE);
    }

    m_spiCtx[spi].transmit.state = SPI_XMIT_IDLE;
}

static void spi_int_enable(SPI_INDEX_e spi, uint32_t mask)
{
    NVIC_EnableIRQ((IRQn_Type)(SPI0_IRQn + spi));

    AP_SSI_TypeDef *Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;
    Ssix->IMR = mask & 0x11;
}

static void spi_int_disable(SPI_INDEX_e spi)
{
    NVIC_DisableIRQ((IRQn_Type)(SPI0_IRQn + spi));

    AP_SSI_TypeDef *Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;
    Ssix->IMR = 0x00;
}

static void spi_int_handle(SPI_INDEX_e spi)
{
    spi_evt_t evt;
    uint8_t i, cnt;

    AP_SSI_TypeDef *Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;

    /* copy the ISR register value */
    uint32_t spi_irs_status = Ssix->ISR;

    if (spi_irs_status & SPIx_ISR_RX_FULL)
    {
        LOG("irq for spi0, receive fifo empty");

        cnt = Ssix->RXFTLR;

        for (i = 0; i < cnt; i++)
        {
            m_spiCtx[spi].transmit.rx_buf[m_spiCtx[spi].transmit.rx_offset++] = Ssix->DataReg;
        }

        if (m_spiCtx[spi].transmit.rx_offset == m_spiCtx[spi].transmit.xmit_len)
        {
            /* TODO!!!: ??? */
            if (m_spiCtx[spi].cfg.force_cs == SPI_FORCE_CS_ENABLED)
                hal_gpio_fmux(m_spiCtx[spi].cfg.ssn_pin, Bit_ENABLE);

            m_spiCtx[spi].transmit.state = SPI_XMIT_IDLE; /* can't call go idle because of the above fmux thing */
            m_spiCtx[spi].transmit.rx_buf = NULL;
            m_spiCtx[spi].transmit.rx_offset = 0;

            /* Notify the callback handler, if any */
            if (m_spiCtx[spi].cfg.evt_handler)
            {
                evt.id = spi;
                evt.evt = SPI_RX_COMPLETED;
                m_spiCtx[spi].cfg.evt_handler(&evt);
            }

            hal_pwrmgr_unlock((MODULE_e)(MOD_SPI0 + spi));
        }
    }

    if (spi_irs_status & SPIx_ISR_TX_EMPTY)
    {
        cnt = 8 - Ssix->TXFLR;

        if (m_spiCtx[spi].transmit.tx_offset == m_spiCtx[spi].transmit.xmit_len)
        {
            /* dirty, but oh well */
            goto finish_tx;
        }

        for (i = 0; i < cnt; i++)
        {
            if (m_spiCtx[spi].transmit.tx_buf)
            {
                /* Load the next buffer value into the FIFO, either a byte or 2 bytes */
                if (m_spiCtx[spi].cfg.spi_dfsmod <= SPI_1BYTE)
                {
                    /* just 1 byte */
                    Ssix->DataReg = m_spiCtx[spi].transmit.tx_buf[m_spiCtx[spi].transmit.tx_offset++];
                }
                else
                {
                    /* 2 bytes */
                    Ssix->DataReg = ((uint16_t*)m_spiCtx[spi].transmit.tx_buf)[m_spiCtx[spi].transmit.tx_offset++];
                }
            }
            else
            {
                if (m_spiCtx[spi].cfg.spi_dfsmod <= SPI_1BYTE)
                {
                    /* Load the pattern as 1 byte into the FIFO */
                    Ssix->DataReg = (uint8_t)m_spiCtx[spi].transmit.pattern;
                }
                else
                {
                    /* Load the pattern as 2 bytes into the FIFO */
                    Ssix->DataReg = m_spiCtx[spi].transmit.pattern;
                }

                m_spiCtx[spi].transmit.tx_offset++;
            }

            if (m_spiCtx[spi].transmit.tx_offset == m_spiCtx[spi].transmit.xmit_len)
            {
finish_tx:
                Ssix->IMR = 0x10;

                /* Notify the callback handler, if any */
                if (m_spiCtx[spi].cfg.evt_handler)
                {
                    evt.id = spi;
                    evt.evt = SPI_TX_COMPLETED;
                    m_spiCtx[spi].cfg.evt_handler(&evt);
                }

                spi_go_idle(spi);

                hal_pwrmgr_unlock((MODULE_e)(MOD_SPI0 + spi));
                break;
            }
        }
    }
}

/**
  * @brief  IRQ Handler for SPI0.
  * @param  None.
  * @retval None.
  */
static void hal_SPI0_IRQHandler(void)
{
    if (m_spiCtx[0].state == SPI_CTX_UNINITIALIZED)
    {
        /* TODO!!!: mask irq? */
        return;
    }

#ifdef USE_SLAVE
    if (m_spiCtx[0].mode == SPI_SLAVE)
        spis_int_handle(0, &m_spiCtx[0], AP_SPI0);
    else
#endif
        spi_int_handle(SPI0);
}

/**
  * @brief  IRQ Handler for SPI1.
  * @param  None.
  * @retval None.
  */
static void hal_SPI1_IRQHandler(void)
{
    if (m_spiCtx[1].state == SPI_CTX_UNINITIALIZED)
    {
        /* TODO!!!: mask irq? */
        return;
    }

#ifdef USE_SLAVE
    if (m_spiCtx[1].mode == SPI_SLAVE)
        spis_int_handle(1, &m_spiCtx[1], AP_SPI0);
    else
#endif
        spi_int_handle(SPI1);
}

/**
  * @brief  Sets up the pins for a given SPIx.
  * @param  spi: The SPI module to initialize, either SPI0 or SPI1.
  * @param  sck_pin: Clock (SCK) pin, GPIO_DUMMY if not used (why wouldn't it be not used???).
  * @param  ssn_pin: Chip Select (CS) pin, GPIO_DUMMY if not used.
  * @param  tx_pin: Master output, slave input (MOSI) pin in Master mode or Master input,
  *                 slave output (MISO) in slave mode, GPIO_DUMMY if not used.
  * @param  rx_pin: Master input, slave output (MISO) pin in Master mode or Master output,
  *                 slave input (MOSI) in Slave mode, GPIO_DUMMY if not used.
  * @retval None.
  */
static void spi_pin_init(SPI_INDEX_e spi, gpio_pin_e sck_pin, gpio_pin_e ssn_pin, gpio_pin_e tx_pin, gpio_pin_e rx_pin)
{
    if (spi == SPI0)
    {
        hal_gpio_fmux_set(sck_pin, FMUX_SPI_0_SCK);
        hal_gpio_fmux_set(ssn_pin, FMUX_SPI_0_SSN);
        hal_gpio_fmux_set(tx_pin, FMUX_SPI_0_TX);
        hal_gpio_fmux_set(rx_pin, FMUX_SPI_0_RX);
    }
    else if (spi == SPI1)
    {
        hal_gpio_fmux_set(sck_pin, FMUX_SPI_1_SCK);
        hal_gpio_fmux_set(ssn_pin, FMUX_SPI_1_SSN);
        hal_gpio_fmux_set(tx_pin, FMUX_SPI_1_TX);
        hal_gpio_fmux_set(rx_pin, FMUX_SPI_1_RX);
    }
}

/**
  * @brief  Disables the pins specified for a given SPIx.
  * @param  sck_pin: Clock (SCK) pin, GPIO_DUMMY if not used (why wouldn't it be not used???).
  * @param  ssn_pin: Chip Select (CS) pin, GPIO_DUMMY if not used.
  * @param  tx_pin: Master output, slave input (MOSI) pin in Master mode or Master input,
  *                 slave output (MISO) in slave mode, GPIO_DUMMY if not used.
  * @param  rx_pin: Master input, slave output (MISO) pin in Master mode or Master output,
  *                 slave input (MOSI) in Slave mode, GPIO_DUMMY if not used.
  * @retval None.
  */
static void hal_spi_pin_deinit(gpio_pin_e sck_pin, gpio_pin_e ssn_pin, gpio_pin_e tx_pin, gpio_pin_e rx_pin)
{
    hal_gpio_fmux(sck_pin, Bit_DISABLE);
    hal_gpio_fmux(ssn_pin, Bit_DISABLE);
    hal_gpio_fmux(tx_pin, Bit_DISABLE);
    hal_gpio_fmux(rx_pin, Bit_DISABLE);
}

#ifdef USE_SLAVE
/**************************************************************************************
    @fn          hal_spi_slave_init

    @brief       This function process for spi slave initial

    input parameters

    @param       uint32_t baud: baudrate select
                SPI_SCMOD_e scmod: Serial Clock Polarity and Phase select;  SPI_MODE0,        //SCPOL=0,SCPH=0(default)
                                                                            SPI_MODE1,        //SCPOL=0,SCPH=1
                                                                            SPI_MODE2,        //SCPOL=1,SCPH=0
                                                                            SPI_MODE3,        //SCPOL=1,SCPH=1
                SPI_TMOD_e tmod: Transfer Mode                              SPI_TRXD,        //Transmit & Receive(default)
                                                                            SPI_TXD,         //Transmit Only
                                                                            SPI_RXD,         //Receive Only
                                                                            SPI_EEPROM,      //EEPROM Read

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/
/*static*/ void hal_spi_slave_init(hal_spi_t *spi_ptr, uint32_t baud, SPI_SCMOD_e scmod, SPI_TMOD_e tmod)
{
    uint8_t shift = 0;
    AP_SSI_TypeDef *Ssix = NULL;
    uint16_t baud_temp;
    int pclk = clk_get_pclk();

    if (spi_ptr->spi_index == SPI1)
    {
        shift = 1;
    }

    Ssix = (spi_ptr->spi_index == SPI0) ? AP_SPI0 : AP_SPI1;
    Ssix->SSIEN = 0; // DISABLE_SPI;
    AP_COM->PERI_MASTER_SELECT &= ~(BIT(shift));
    Ssix->CR0 = ((Ssix->CR0) & 0xfffffc3f) | (scmod << 6) | (tmod << 8) | 0x400;
    baud_temp = (pclk + (baud >> 1)) / baud;

    if (baud_temp < 2)
    {
        baud_temp = 2;
    }
    else if (baud_temp > 65534)
    {
        baud_temp = 65534;
    }

    Ssix->BAUDR = baud_temp; // set clock(round)
    Ssix->TXFTLR = 4;        // set fifo threshold to triggle interrupt
    Ssix->RXFTLR = 1;        // threshold is 1
    Ssix->IMR = 0x11;        // enable tx and rx
    //  Ssix->SER=1;      //enable slave device
    Ssix->SSIEN = 1; // ENABLE_SPI;
}

static void hal_spi_set_slave(SPI_INDEX_e spi)
{
    /* Set the appropriate peripheral as slave on the COM */
    if (spi == SPI0)
    {
        COM_SPI_MS &= ~(COM_SPI_MS_SPI0A_MASTER | COM_SPI_MS_SPI0B_MASTER);

        SPI0_SSIEN = SPIx_SSIEN_DISABLED;

        /* Set the slave flag on the SPIx */
        SPI0_CR0 |= SPIx_CR0_SLAVE;

        /* Disable master mode */
        SPI0_SER = 0;

        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        COM_SPI_MS &= ~(COM_SPI_MS_SPI1A_MASTER | COM_SPI_MS_SPI1B_MASTER);

        SPI1_SSIEN = SPIx_SSIEN_DISABLED;

        /* Set the slave flag on the SPIx */
        SPI0_CR0 |= SPIx_CR0_SLAVE;

        /* Disable master mode */
        SPI0_SER = 0;

        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}
#endif

#ifdef DMAC_USE
static void config_dma_channel4spitx(SPI_INDEX_e spi, uint8_t *tx_buf, uint16_t tx_len)
{
    DMA_CH_CFG_t cfgc;
    AP_SSI_TypeDef *Ssix = NULL;
    Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;
    Ssix->DMACR &= 0x01;
    spi_Ctx_t *pctx;
    pctx = &m_spiCtx[spi];
    cfgc.sinc = DMA_INC_INC;

    if (pctx->cfg.spi_dfsmod <= SPI_1BYTE)
    {
        hal_spi_dfs_set(spi, SPI_1BYTE);
        cfgc.transf_size = tx_len;
        cfgc.src_tr_width = DMA_WIDTH_BYTE;
        cfgc.dst_tr_width = DMA_WIDTH_BYTE;
        cfgc.src_addr = (uint32_t)tx_buf;
    }
    else
    {
        uint16_t *size16_tx_buf = (uint16_t *)tx_buf;
        hal_spi_dfs_set(spi, SPI_2BYTE);
        cfgc.transf_size = tx_len / 2;
        cfgc.src_tr_width = DMA_WIDTH_HALFWORD;
        cfgc.dst_tr_width = DMA_WIDTH_HALFWORD;
        cfgc.src_addr = (uint32_t)size16_tx_buf;
    }

    cfgc.src_msize = DMA_BSIZE_1;
    cfgc.dinc = DMA_INC_NCHG;
    cfgc.dst_msize = DMA_BSIZE_1;
    cfgc.dst_addr = (uint32_t)&(Ssix->DataReg);
    cfgc.enable_int = 0;
    hal_dma_config_channel(DMA_CH_0, &cfgc);
    hal_dma_start_channel(DMA_CH_0);
    Ssix->DMACR |= 0x02;
    Ssix->DMATDLR = 0;
}

static void config_dma_channel4spirx(SPI_INDEX_e spi, uint8_t *rx_buf, uint16_t rx_len)
{
    DMA_CH_CFG_t cfgc;
    AP_SSI_TypeDef *Ssix = NULL;
    Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;
    Ssix->DMACR &= 0x02;
    cfgc.transf_size = rx_len;
    cfgc.sinc = DMA_INC_NCHG;
    cfgc.src_tr_width = DMA_WIDTH_BYTE;
    cfgc.src_msize = DMA_BSIZE_1;
    cfgc.src_addr = (uint32_t)&(Ssix->DataReg);
    cfgc.dinc = DMA_INC_INC;
    cfgc.dst_tr_width = DMA_WIDTH_BYTE;
    cfgc.dst_msize = DMA_BSIZE_1;
    cfgc.dst_addr = (uint32_t)rx_buf;
    cfgc.enable_int = 0;
    hal_dma_config_channel(DMA_CH_0, &cfgc);
    hal_dma_start_channel(DMA_CH_0);
    Ssix->DMACR |= 0x01;
    Ssix->DMARDLR = 0;
}

int hal_spi_dma_set(SPI_INDEX_e spi, uint8_t ten, uint8_t ren)
{
    SPI_HDL_VALIDATE(spi);

    m_spiCtx[spi].cfg.dma_rx_enable = ren;
    m_spiCtx[spi].cfg.dma_tx_enable = ten;

    return PPlus_SUCCESS;
}
#endif

static int hal_spi_xmit_polling(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_len,
    uint16_t rx_len,
    uint32_t timeout)
{
    uint32_t rx_size = rx_len, tx_size = tx_len;
    uint32_t tmp_len, i, tmp_tx_buf_len = 0;
    AP_SSI_TypeDef *Ssix = NULL;

    Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;

    /* grab the timer on entry (for timeout) */
    int to = getMcuPrecisionCount();

#ifdef DMAC_USE
    if (rx_len && m_spiCtx[spi].cfg.dma_rx_enable)
    {
        config_dma_channel4spirx(spi, rx_buf, rx_len);
    }
    else if (tx_len && m_spiCtx[spi].cfg.dma_tx_enable)
    {
        //LOG("TX DMA! 0x%04x, len %d", tx_buf, tx_len);
        config_dma_channel4spitx(spi, tx_buf, tx_len);
    }
#endif

    while (1)
    {
        if (Ssix->SR & SPIx_SR_TX_NOT_FULL && tx_size
#ifdef DMAC_USE
            && !(m_spiCtx[spi].cfg.dma_tx_enable)
#endif
        )
        // #ifdef DMAC_USE
        //         if(Ssix->SR & SPIx_SR_TX_NOT_FULL && tx_size && !(pctx->cfg.dma_tx_enable))
        // #else
        //         if(Ssix->SR & SPIx_SR_TX_NOT_FULL && tx_size)
        // #endif
        {
            tmp_len = 8 - Ssix->TXFLR;

            if (tmp_len > tx_size)
                tmp_len = tx_size;

            if (tx_buf)
            {
                spi_write_fifo(Ssix, tmp_len, tx_buf);
                tmp_tx_buf_len += tmp_len;
            }
            else
            {
                for (i = 0; i < tmp_len; i++)
                {
                    if (m_spiCtx[spi].cfg.spi_dfsmod <= SPI_1BYTE)
                    {
                        /* Load the pattern as 1 byte into the FIFO */
                        Ssix->DataReg = (uint8_t)m_spiCtx[spi].transmit.pattern;
                    }
                    else
                    {
                        /* Load the pattern as 2 bytes into the FIFO */
                        Ssix->DataReg = m_spiCtx[spi].transmit.pattern;
                    }
                }
            }

            tx_size -= tmp_len;
        }

#ifdef DMAC_USE
        if (((rx_len == 0) && ((tx_size == 0) || (tx_size && (m_spiCtx[spi].cfg.dma_tx_enable)))) ||
            (rx_len && (tx_size == 0) && (m_spiCtx[spi].cfg.dma_rx_enable)))
            break;
        else if (rx_len && !(m_spiCtx[spi].cfg.dma_rx_enable))
#else
        if ((rx_len == 0) && ((tx_size == 0)))
            break;
        else if (rx_len)
#endif
        {
            if (Ssix->RXFLR)
            {
                tmp_len = Ssix->RXFLR;

                for (i = 0; i < tmp_len; i++)
                {
                    *rx_buf++ = Ssix->DataReg;
                }

                rx_size -= tmp_len;
            }

            if (rx_size == 0)
                break;
        }

        if (hal_ms_intv(to) > timeout)
        {
            LOG("hal_spi_xmit_polling timeout!");
            return PPlus_ERR_TIMEOUT;
        }
    }

#ifdef DMAC_USE
    if ((m_spiCtx[spi].cfg.dma_rx_enable) || (m_spiCtx[spi].cfg.dma_tx_enable))
        hal_dma_status_control(DMA_CH_0);
#endif

    while (Ssix->SR & SPIx_SR_BUSY)
    {
        if (hal_ms_intv(to) > timeout)
        {
            LOG("hal_spi_xmit_polling timeout!");
            return PPlus_ERR_TIMEOUT;
        }
    }

    return PPlus_SUCCESS;
}

static void spi0_sleep_handler(void)
{
    if (m_spiCtx[0].state == SPI_CTX_INITIALIZED)
    {
        hal_spi_bus_deinit(SPI0);
    }
}

static void spi1_sleep_handler(void)
{
    if (m_spiCtx[1].state == SPI_CTX_INITIALIZED)
    {
        hal_spi_bus_deinit(SPI1);
    }
}

static void spi0_wakeup_handler(void)
{
    NVIC_SetPriority((IRQn_Type)SPI0_IRQn, IRQ_PRIO_HAL);
}

static void spi1_wakeup_handler(void)
{
    NVIC_SetPriority((IRQn_Type)SPI1_IRQn, IRQ_PRIO_HAL);
}

static void hal_spi_tmod_set(SPI_INDEX_e spi, SPI_TMOD_e mod)
{
    /* not using a pointer to AP_SPIx due to some gcc warning on subWriteReg */
    if (spi == SPI0)
    {
        SPI0_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI0->CR0), 9, 8, mod);
        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        SPI1_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI1->CR0), 9, 8, mod);
        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}

static void hal_spi_scmod_set(SPI_INDEX_e spi, SPI_SCMOD_e scmod)
{
    /* not using a pointer to AP_SPIx due to some gcc warning on subWriteReg */
    if (spi == SPI0)
    {
        SPI0_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI0->CR0), 7, 6, scmod);
        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        SPI1_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI1->CR0), 7, 6, scmod);
        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}

static void hal_spi_ndf_set(SPI_INDEX_e spi, uint16_t len)
{
    if (len == 0)
    {
        return;
    }

    if (spi == SPI0)
    {
        SPI0_SSIEN = SPIx_SSIEN_DISABLED;
        SPI0_CR1 = len - 1;
        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        SPI1_SSIEN = SPIx_SSIEN_DISABLED;
        SPI1_CR1 = len - 1;
        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}

static void hal_spi_set_speed(SPI_INDEX_e spi, uint32_t speed)
{
    uint16_t baud_temp = (clk_get_pclk() + (speed >> 1)) / speed;

    if (baud_temp < 2)
    {
        baud_temp = 2;
    }
    else if (baud_temp > 65534)
    {
        baud_temp = 65534;
    }

    if (spi == SPI0)
    {
        SPI0_SSIEN = SPIx_SSIEN_DISABLED;
        SPI0_BAUDR = baud_temp; // set clock(round)
        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        SPI1_SSIEN = SPIx_SSIEN_DISABLED;
        SPI1_BAUDR = baud_temp; // set clock(round)
        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}

static void hal_spi_set_master(SPI_INDEX_e spi)
{
    /* Set the appropriate peripheral as master on the COM */
    if (spi == SPI0)
    {
        COM_SPI_MS |= (COM_SPI_MS_SPI0A_MASTER | COM_SPI_MS_SPI0B_MASTER);

        SPI0_SSIEN = SPIx_SSIEN_DISABLED;

        /* Disable the slave flag on the SPIx */
        SPI0_CR0 &= ~SPIx_CR0_SLAVE_Msk;

        /* Enable master mode */
        SPI0_SER = 1;

        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        COM_SPI_MS |= (COM_SPI_MS_SPI1A_MASTER | COM_SPI_MS_SPI1B_MASTER);

        SPI1_SSIEN = SPIx_SSIEN_DISABLED;

        /* Disable the slave flag on the SPIx */
        SPI0_CR0 &= ~SPIx_CR0_SLAVE_Msk;

        /* Enable master mode */
        SPI0_SER = 1;

        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }
}

static void hal_spi_set_fifo_thresholds(SPI_INDEX_e spi, uint8_t tx, uint8_t rx)
{
    AP_SSI_TypeDef *Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;

    Ssix->SSIEN = 0; // DISABLE_SPI;

    Ssix->TXFTLR = tx & 0x7;        // set fifo threshold to trigger interrupts
    Ssix->RXFTLR = rx & 0x7;

    Ssix->SSIEN = 1; // ENABLE_SPI;
}

static int spi_tx_rx_common(
    SPI_INDEX_e spi,
    SPI_TMOD_e mode,
    uint16_t pattern,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_len,
    uint16_t rx_len,
    uint32_t timeout)
{
    int ret;

    SPI_HDL_VALIDATE(spi);

    if (m_spiCtx[spi].transmit.state != SPI_XMIT_IDLE)
    {
        return PPlus_ERR_BUSY;
    }

    hal_spi_tmod_set(spi, mode);

    if(mode == SPI_RXD || mode == SPI_EEPROM)  //spi receive only or eeprom read,should set read data len(ndf)
    {
        hal_spi_ndf_set(spi, rx_len);
    }

    spi_go_busy(spi);

    m_spiCtx[spi].transmit.pattern = pattern;

    ret = hal_spi_xmit_polling(spi, tx_buf, rx_buf, tx_len, rx_len, timeout);

    spi_go_idle(spi);

    return ret;
}

static int spi_tx_rx_common_it(
    SPI_INDEX_e spi,
    SPI_TMOD_e mode,
    uint16_t pattern,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_len,
    uint16_t rx_len)
{
    AP_SSI_TypeDef *Ssix = NULL;

    SPI_HDL_VALIDATE(spi);

    if (m_spiCtx[spi].transmit.state != SPI_XMIT_IDLE)
    {
        return PPlus_ERR_BUSY;
    }

    Ssix = (spi == SPI0) ? AP_SPI0 : AP_SPI1;

    if ((Ssix->SR & SPIx_SR_TX_NOT_FULL) == 0)
    {
        LOG("IS BUSY!!!");
        return PPlus_ERR_BUSY;
    }

    hal_spi_tmod_set(spi, mode);

    if(mode == SPI_RXD || mode == SPI_EEPROM)  //spi receive only or eeprom read,should set read data len(ndf)
    {
        hal_spi_ndf_set(spi, rx_len);
    }

    spi_go_busy(spi);

    spi_int_disable(spi);

    /* Set up the transmit and receive stuff */
    m_spiCtx[spi].transmit.rx_buf = rx_buf;
    m_spiCtx[spi].transmit.tx_buf = tx_buf;
    m_spiCtx[spi].transmit.xmit_len = tx_len;
    m_spiCtx[spi].transmit.pattern = pattern;

    uint8_t dummy[8] = { 0 };

    /* Kickstart the first transaction with the specified values */
    uint16_t _tx_len = (tx_len >= 8) ? 8 : tx_len;

    /* If there's a transmit buffer set, use its first 8 bytes instead of all zeros for the kickstart */
    if (tx_buf)
    {
        memcpy(dummy, tx_buf, _tx_len);
    }
    else
    {
        /* if not, set up the pattern on the dummy buffer for the kickstart */
        if (m_spiCtx[spi].cfg.spi_dfsmod > SPI_1BYTE)
        {
            ((uint16_t*)dummy)[0] = pattern;
            ((uint16_t*)dummy)[1] = pattern;
            ((uint16_t*)dummy)[2] = pattern;
            ((uint16_t*)dummy)[3] = pattern;
        }
        else
        {
            memset(dummy, pattern, sizeof(dummy));
        }
    }

    hal_pwrmgr_lock((MODULE_e)(MOD_SPI0 + spi));

    spi_write_fifo(Ssix, _tx_len, dummy);

    /* "_tx_len" words have been already transmitted by "spi_write_fifo" above */
    m_spiCtx[spi].transmit.tx_offset = _tx_len;

    spi_int_enable(spi, 0x11);

    return PPlus_SUCCESS;
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup SPI_Exported_Functions SPI related exported functions
  * @{
  */

int hal_spi_bus_init(SPI_INDEX_e spi, spi_Cfg_t cfg)
{
    if ((spi != SPI0) && (spi != SPI1))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if (m_spiCtx[spi].state == SPI_CTX_INITIALIZED)
    {
        return PPlus_ERR_BUSY;
    }

    hal_clk_gate_enable((MODULE_e)(MOD_SPI0 + spi));
    spi_int_disable(spi);
    spi_pin_init(spi, cfg.sclk_pin, cfg.ssn_pin, cfg.MOSI, cfg.MISO);
    hal_spi_set_master(spi);
    hal_spi_scmod_set(spi, cfg.spi_scmod);
    hal_spi_set_speed(spi, cfg.frequency);
    hal_spi_tmod_set(spi, SPI_TRXD); /* default */
    hal_spi_set_fifo_thresholds(spi, 4, 1); /* default */
    hal_spi_dfs_set(spi, cfg.spi_dfsmod);

    m_spiCtx[spi].cfg = cfg;
    m_spiCtx[spi].transmit.state = SPI_XMIT_IDLE;
    m_spiCtx[spi].mode = SPI_MASTER;
    m_spiCtx[spi].state = SPI_CTX_INITIALIZED;

    return PPlus_SUCCESS;
}

int hal_spi_bus_deinit(SPI_INDEX_e spi)
{
    SPI_HDL_VALIDATE(spi);

    hal_clk_gate_disable((MODULE_e)(MOD_SPI0 + spi));
    hal_spi_pin_deinit(m_spiCtx[spi].cfg.sclk_pin, m_spiCtx[spi].cfg.ssn_pin, m_spiCtx[spi].cfg.MOSI, m_spiCtx[spi].cfg.MISO);
    spi_int_disable(spi);

    m_spiCtx[spi].state = SPI_CTX_UNINITIALIZED;

    return PPlus_SUCCESS;
}

int hal_spi_transmit(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint16_t tx_len,
    uint32_t timeout)
{
    return spi_tx_rx_common(spi, SPI_TXD, 0x00, tx_buf, NULL, tx_len, 0, timeout);
}

int hal_spi_transmit_same(
    SPI_INDEX_e spi,
    uint16_t tx_pattern,
    uint16_t tx_words,
    uint32_t timeout)
{
    return spi_tx_rx_common(spi, SPI_TXD, tx_pattern, NULL, NULL, tx_words, 0, timeout);
}

int hal_spi_transmit_receive(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_len,
    uint16_t rx_len,
    uint32_t timeout)
{
    return spi_tx_rx_common(spi, SPI_TRXD, 0x00, tx_buf, rx_buf, tx_len, rx_len, timeout);
}

int hal_spi_transmit_receive_eeprom(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_len,
    uint16_t rx_len,
    uint32_t timeout)
{
    return spi_tx_rx_common(spi, SPI_EEPROM, 0x00, tx_buf, rx_buf, tx_len, rx_len, timeout);
}

int hal_spi_transmit_it(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint16_t tx_words)
{
    return spi_tx_rx_common_it(spi, SPI_TXD, 0x00, tx_buf, NULL, tx_words, 0);
}

int hal_spi_transmit_same_it(
    SPI_INDEX_e spi,
    uint16_t tx_pattern,
    uint16_t tx_words)
{
    return spi_tx_rx_common_it(spi, SPI_TXD, tx_pattern, NULL, NULL, tx_words, 0);
}

int hal_spi_transmit_receive_it(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_rx_words)
{
    return spi_tx_rx_common_it(spi, SPI_TRXD, 0x00, tx_buf, rx_buf, tx_rx_words, tx_rx_words);
}

int hal_spi_transmit_receive_eeprom_it(
    SPI_INDEX_e spi,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint16_t tx_words,
    uint16_t rx_words)
{
    return spi_tx_rx_common_it(spi, SPI_EEPROM, 0x00, tx_buf, rx_buf, tx_words, rx_words);
}

int hal_spi_init(void)
{
    int ret = 0;

    memset(m_spiCtx, 0, sizeof(m_spiCtx));

    ret = hal_pwrmgr_register(MOD_SPI0, spi0_sleep_handler, spi0_wakeup_handler);
    if (ret != PPlus_SUCCESS)
    {
        return ret;
    }

    ret = hal_pwrmgr_register(MOD_SPI1, spi1_sleep_handler, spi1_wakeup_handler);
    if (ret != PPlus_SUCCESS)
    {
        return ret;
    }

    /* disable interrupts */
    spi_int_disable(SPI0);
    spi_int_disable(SPI1);

    /* set priority of SPIx interrupts */
    NVIC_SetPriority((IRQn_Type)SPI0_IRQn, IRQ_PRIO_HAL);
    NVIC_SetPriority((IRQn_Type)SPI1_IRQn, IRQ_PRIO_HAL);

    /* setup the IRQ handler for SPIx */
    JUMP_FUNCTION(SPI0_IRQ_HANDLER) = (uint32_t)&hal_SPI0_IRQHandler;
    JUMP_FUNCTION(SPI1_IRQ_HANDLER) = (uint32_t)&hal_SPI1_IRQHandler;

    return PPlus_SUCCESS;
}

int hal_spi_dfs_set(SPI_INDEX_e spi, SPI_DFS_e mod)
{
    SPI_HDL_VALIDATE(spi);

    /* not using a pointer to AP_SPIx due to some gcc warning on subWriteReg */
    if (spi == SPI0)
    {
        SPI0_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI0->CR0), 3, 0, mod);
        SPI0_SSIEN = SPIx_SSIEN_ENABLED;
    }
    else if (spi == SPI1)
    {
        SPI1_SSIEN = SPIx_SSIEN_DISABLED;
        subWriteReg(&(AP_SPI1->CR0), 3, 0, mod);
        SPI1_SSIEN = SPIx_SSIEN_ENABLED;
    }

    m_spiCtx[spi].cfg.spi_dfsmod = mod;

    return PPlus_SUCCESS;
}

int hal_spi_set_force_cs(SPI_INDEX_e spi, spi_cfg_force_cs_t en)
{
    SPI_HDL_VALIDATE(spi);

    m_spiCtx[spi].cfg.force_cs = en;

    return PPlus_SUCCESS;
}

spi_xmit_state_t hal_spi_get_transmit_bus_state(SPI_INDEX_e spi)
{
    if ((spi != SPI0) && (spi != SPI1))
    {
        return SPI_XMIT_INVALID;
    }

    return m_spiCtx[spi].transmit.state;
}

#ifdef USE_SLAVE
int hal_spis_clear_rx(hal_spi_t *spi_ptr)
{
    AP_SSI_TypeDef *Ssix = NULL;
    volatile uint8_t rx;
    SPI_HDL_VALIDATE(spi_ptr);
    Ssix = (spi_ptr->spi_index == SPI0) ? AP_SPI0 : AP_SPI1;

    while (Ssix->RXFLR)
    {
        rx = Ssix->DataReg;
    }

    return (int)rx;
}

uint32_t hal_spis_rx_len(hal_spi_t *spi_ptr)
{
    AP_SSI_TypeDef *Ssix = NULL;
    Ssix = (spi_ptr->spi_index == SPI0) ? AP_SPI0 : AP_SPI1;
    return Ssix->RXFLR;
}

int hal_spis_read_rxn(hal_spi_t *spi_ptr, uint8_t *pbuf, uint16_t len)
{
    AP_SSI_TypeDef *Ssix = NULL;
    Ssix = (spi_ptr->spi_index == SPI0) ? AP_SPI0 : AP_SPI1;

    while (len)
    {
        *pbuf = Ssix->DataReg;
        pbuf++;
        len--;
    }

    return PPlus_SUCCESS;
}

int hal_spis_bus_init(hal_spi_t *spi_ptr, spi_Cfg_t cfg)
{
    spi_Ctx_t *pctx = NULL;

    if ((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
        return PPlus_ERR_INVALID_PARAM;

    pctx = &m_spiCtx[spi_ptr->spi_index];

    if (pctx->spi_info != NULL)
        return PPlus_ERR_BUSY;

    hal_clk_gate_enable((MODULE_e)(MOD_SPI0 + spi_ptr->spi_index));
    spi_pin_init(spi_ptr, cfg.sclk_pin, cfg.ssn_pin, cfg.MOSI, cfg.MISO);
    hal_spi_slave_init(spi_ptr, cfg.baudrate, cfg.spi_scmod, cfg.spi_tmod);
    pctx->cfg = cfg;
    memset(&(pctx->transmit), 0, sizeof(spi_xmit_t));
    pctx->spi_info = spi_ptr;
    pctx->mode = SPI_SLAVE;

    if (cfg.int_mode)
        spi_int_enable(spi_ptr, 0x10);
    else
        spi_int_disable(spi_ptr);

    return PPlus_SUCCESS;
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
