/**
  ******************************************************************************
  * @file    flash.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   Flash BSP module driver.
  *          This file provides firmware functions to manage the embedded SPI
  *          Flash on the MCU:
  *           + Initializing the XIP Cache
  *           + Read the Flash ID and capacity
  *           + Read/Write/Erase blocks/sectors of the flash
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <phy62xx.h>

#include "flash.h"

#include <driver/pwrmgr/pwrmgr.h> /* for hal_pwrmgr_register */

#include <phy_error.h> /* for PPlus_* return types */

#include <types.h> /* for __ATTR_SECTION_SRAM__ */

#include <stddef.h> /* for NULL */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup FLASH
  * @brief Flash BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @addtogroup FLASH_Private_Typedef FLASH Private Typedefs
  * @{
  */

/**
 * @brief  PHY62xx
 */
typedef struct
{
    sysclk_t spif_ref_clk;
    XFRD_FCMD_READ_t rd_instr;
} xflash_Ctx_t;

/**
 * @brief  PHY62xx Enum for state of the SPIF context structure, initialized or not.
 */
typedef enum
{
    FLASH_CTX_NOT_INITALIZED = 0U,
    FLASH_CTX_INITIALIZED = 1U
} FLASH_CTX_INIT_t;

/**
 * @brief  PHY62xx
 */
typedef struct
{
    FLASH_CTX_INIT_t init_flag;
    uint32_t IdentificationID;
    uint32_t Capacity;
} FLASH_CHIP_INFO;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants FLASH Private Constants
  * @{
  */
#define SPIF_TIMEOUT (0x7ffffff) // 1000000

#define SFLG_WIP 1
#define SFLG_WEL 2
#define SFLG_WELWIP 3

// define flash ucds
// #define FLASH_UCDS_ADDR_BASE    0x11005000

#define CHIP_ID_LENGTH 64

// #define FLASH_PROTECT_FEATURE
#define CHIP_MADDR_LEN 6
#define CHIP_MADDR_FLASH_ADDRESS (FLASH_BASE_ADDR + CHIP_ID_LENGTH * 4)

#ifndef FLASH_PROTECT_FEATURE
#define FLASH_PROTECT_FEATURE 0
#endif

#define SPIF_WAIT_IDLE_CYC (32)

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @addtogroup FLASH_Private_Constants FLASH Private macros
  * @{
  */
#define SPIF_STATUS_WAIT_IDLE(n)                               \
    do                                                         \
    {                                                          \
        while ((SPIF_FCMD & SPIF_FCMD_BUSY) == SPIF_FCMD_BUSY) \
            ;                                                  \
        volatile int delay_cycle = n;                          \
        while (delay_cycle--)                                  \
        {                                                      \
        };                                                     \
        while ((AP_SPIF->config & 0x80000000) == 0)            \
            ;                                                  \
    } while (0);

#define HAL_CACHE_ENTER_BYPASS_SECTION() \
    do                                   \
    {                                    \
        __disable_irq();                 \
        AP_CACHE->CTRL0 = 0x02;          \
        AP_PCR->CACHE_RST = 0x02;        \
        AP_PCR->CACHE_BYPASS = 1;        \
        __enable_irq();                  \
    } while (0);

#define HAL_CACHE_EXIT_BYPASS_SECTION() \
    do                                  \
    {                                   \
        __disable_irq();                \
        AP_CACHE->CTRL0 = 0x00;         \
        AP_PCR->CACHE_RST = 0x03;       \
        AP_PCR->CACHE_BYPASS = 0;       \
        __enable_irq();                 \
    } while (0);
/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Variables FLASH Private variables
  * @{
  */
/**
 * @brief  PHY62xx
 */
static xflash_Ctx_t s_xflashCtx = {
    .spif_ref_clk = SYS_CLK_DLL_64M,
    .rd_instr = XFRD_FCMD_READ_DUAL
};

/**
 * @brief  PHY62xx
 */
static FLASH_CHIP_INFO phy_flash = {
    .init_flag = FLASH_CTX_NOT_INITALIZED,
    .IdentificationID = 0x00,
    .Capacity = 0x80000,
};
/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup FLASH_Private_Functions FLASH Private functions
  * @{
  */
__ATTR_SECTION_SRAM__ static inline uint32_t spif_lock()
{
    __disable_irq();
    uint32_t vic_iser = NVIC->ISER[0];
    // mask all irq
    NVIC->ICER[0] = 0xFFFFFFFF;
    // enable ll irq and tim1 irq
    NVIC->ISER[0] = 0x100010;
    __enable_irq();
    return vic_iser;
}

__ATTR_SECTION_SRAM__ static inline void spif_unlock(uint32_t vic_iser)
{
    __disable_irq();
    NVIC->ISER[0] = vic_iser;
    __enable_irq();
}

static void hal_cache_tag_flush(void)
{
    __disable_irq();

    uint32_t cb = AP_PCR->CACHE_BYPASS;
    volatile int dly = 8;

    if (cb == 0)
    {
        AP_PCR->CACHE_BYPASS = 1;
    }

    AP_CACHE->CTRL0 = 0x02;

    while (dly--)
    {
        ;
    };

    AP_CACHE->CTRL0 = 0x03;

    dly = 8;

    while (dly--)
    {
        ;
    };

    AP_CACHE->CTRL0 = 0x00;

    if (cb == 0)
    {
        AP_PCR->CACHE_BYPASS = 0;
    }

    __enable_irq();
}

/* ROM code doesn't contain any "SPIF_STATUS_WAIT_IDLE" */
static uint8_t _spif_read_status_reg_no_rom(void)
{
    uint8_t status;

    spif_cmd(SPIF_CMD_RDST, 0, 2, 0, 0, 0);
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    spif_rddata(&status, 1);

    return status;
}

/*
 * ROM code doesn't contain any calls to WaitRTCCount() and uses the
 * _spif_read_status_reg which doesn't wait for the SPIF to be idle.
 */
static int _spif_wait_nobusy_no_rom(uint8_t flg, uint32_t tout_ns)
{
    uint8_t status;
    volatile int tout = (int)(tout_ns);

    for (; tout; tout--)
    {
        status = _spif_read_status_reg_no_rom();

        if ((status & flg) == 0)
            return PPlus_SUCCESS;

        // insert polling interval
        // 5*32us
        WaitRTCCount(5);
    }

    return PPlus_ERR_BUSY;
}

static void hal_cache_init(void)
{
    volatile int dly = 100;
    /* Enable Clock Gate of the HCLK and PCLK for the cache. */
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);

    /* cache rst ahp */
    AP_PCR->CACHE_RST = 0x02;

    while (dly--)
    {
    };

    AP_PCR->CACHE_RST = 0x03;

    hal_cache_tag_flush();

    /* cache enable */
    AP_PCR->CACHE_BYPASS = 0;
}

/**
  * @brief  Reads the information of the embedded SPI Flash. Initializes data for SPIF APIs.
  * @param  None.
  * @note   Reads the flash identification and capacity
  * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY otherwise.
  */
static int hal_flash_init(void)
{
    uint32_t cs;
    uint8_t data[4];
    int ret = PPlus_SUCCESS;

    if (phy_flash.init_flag == FLASH_CTX_INITIALIZED)
    {
        return ret;
    }

    /* disable all IRQs EXCEPT TIM1 and LL */
    cs = spif_lock();

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        /* restore IRQs */
        spif_unlock(cs);
        return ret;
    }

    spif_cmd(SPIF_CMD_RDID, 0, 3, 0, 0, 0);

    spif_rddata(data, 3);

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        /* restore IRQs */
        spif_unlock(cs);
        return ret;
    }

    /* restore IRQs */
    spif_unlock(cs);

    phy_flash.IdentificationID = (data[2] << 16) | (data[1] << 8) | data[0];

    if ((data[2] >= 0x11) && (data[2] <= 0x16)) // most use:256K~2M.reserved:128K,4M
    {
        phy_flash.Capacity = (1ul << data[2]);
        *(volatile int *)0x1fff0898 = phy_flash.Capacity; /* this is black magic */
    }
    else
    {
        phy_flash.Capacity = 512 * 1024;
        *(volatile int *)0x1fff0898 = phy_flash.Capacity; /* this is black magic */
    }

    phy_flash.init_flag = FLASH_CTX_INITIALIZED;

    return ret;
}

static void hw_spif_cache_config(void)
{
    spif_config(s_xflashCtx.spif_ref_clk,
                1,
                (uint32_t)s_xflashCtx.rd_instr,
                0,
                (s_xflashCtx.rd_instr == XFRD_FCMD_READ_QUAD) ? 1 : 0);

#ifdef XFLASH_HIGH_SPEED
    volatile uint32_t tmp = AP_SPIF->config;
    tmp = (tmp & (~(0xf << 19))) | (0 << 19);
    AP_SPIF->config = tmp;
    subWriteReg(&AP_SPIF->rddata_capture, 4, 1, 2);
#endif

    AP_SPIF->wr_completion_ctrl = 0xff010005; // set longest polling interval
    AP_SPIF->low_wr_protection = 0;
    AP_SPIF->up_wr_protection = 0x10;
    AP_SPIF->wr_protection = 0x2;

    /* Disable the SPIF IRQ forever (TODO: maybe some polling code can be updated to use IRQs) */
    NVIC_DisableIRQ(SPIF_IRQn);
    NVIC_SetPriority((IRQn_Type)SPIF_IRQn, IRQ_PRIO_HAL);

    hal_cache_init();
    hal_flash_init();
}

/**
  * @brief  Writes the status of the SPI Flash?.
  * @param  status: No idea. 0x7c write-locks the flash, 0x00 unlocks writes to the flash.
  * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
  */
static int hal_flash_wr_status(uint8_t status)
{
    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    int ret = PPlus_SUCCESS;
    uint32_t cs = spif_lock();

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

    AP_SPIF->fcmd = SPIF_FCMD_OP_WREN | SPIF_FCMD_EXEC;

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

    AP_SPIF->fcmd_wrdata[0] = status;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

    AP_SPIF->fcmd = SPIF_FCMD_OP_WRST | SPIF_FCMD_WREN | SPIF_FCMD_EXEC;
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);

out:
    /* restore IRQs */
    spif_unlock(cs);
    return ret;
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup FLASH_Exported_Functions FLASH related exported functions
  * @{
  */
void hal_spif_cache_init(sysclk_t spif_ref_clk, XFRD_FCMD_READ_t rd_instr)
{
    s_xflashCtx.spif_ref_clk = spif_ref_clk;
    s_xflashCtx.rd_instr = rd_instr;

    hw_spif_cache_config();
    hal_pwrmgr_register(MOD_SPIF, NULL, hw_spif_cache_config);
}

int hal_flash_lock(void)
{
    return hal_flash_wr_status(0x7c);
}

int hal_flash_unlock(void)
{
    return hal_flash_wr_status(0);
}

uint8_t hal_flash_get_lock_state(void)
{
    uint32_t cs = spif_lock();
    uint8_t status;

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    status = _spif_read_status_reg_no_rom();
    status = (status & 0x7c) >> 2;
    spif_unlock(cs);

    return status;
}

int hal_flash_write(uint32_t addr, uint8_t *data, uint32_t size)
{
    int ret = PPlus_SUCCESS;
    int retval;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_unlock();
#endif

    uint32_t cs = spif_lock();

    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

    retval = spif_write(addr, data, size);

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

out:
    /* restore IRQs */
    HAL_CACHE_EXIT_BYPASS_SECTION();
    spif_unlock(cs);

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_lock();
#endif

    return retval;
}

int hal_flash_write_by_dma(uint32_t addr, uint8_t *data, uint32_t size)
{
    int ret = PPlus_SUCCESS;
    int retval;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_unlock();
#endif

    uint32_t cs = spif_lock();

    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

    retval = spif_write_dma(addr, data, size);

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

out:
    /* restore IRQs */
    HAL_CACHE_EXIT_BYPASS_SECTION();
    spif_unlock(cs);

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_lock();
#endif

    return retval;
}


int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
    volatile uint8_t *u8_spif_addr = (volatile uint8_t *)((addr & 0x7ffff) | FLASH_BASE_ADDR);
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    uint32_t remap = 0;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    uint32_t cs = spif_lock();

    if (phy_flash.Capacity > 0x80000)
    {
        remap = addr & 0xf80000;

        if (remap)
        {
            AP_SPIF->remap = remap;
            AP_SPIF->config |= 0x10000;
        }
    }

    // read flash addr direct access
    // bypass cache
    if (cb == 0)
    {
        HAL_CACHE_ENTER_BYPASS_SECTION();
    }

    for (uint32_t i = 0; i < size; i++)
        data[i] = u8_spif_addr[i];

    // bypass cache
    if (cb == 0)
    {
        HAL_CACHE_EXIT_BYPASS_SECTION();
    }

    if (phy_flash.Capacity > 0x80000)
    {
        if (remap)
        {
            AP_SPIF->remap = 0;
            AP_SPIF->config &= ~0x10000ul;
        }
    }

    spif_unlock(cs);
    return PPlus_SUCCESS;
}

int hal_flash_erase_sector(unsigned int addr)
{
    int ret = PPlus_SUCCESS;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_unlock();
#endif

    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;

    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

#if 0
	retval = spif_erase_sector(addr);
#else

    AP_SPIF->fcmd = SPIF_FCMD_OP_WREN | SPIF_FCMD_EXEC;

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

    AP_SPIF->fcmd_addr = addr;

    spif_cmd(SPIF_CMD_SERASE, 3, 0, 0, 0, 0);
#endif

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WELWIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        goto out;
    }

out:
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if (cb == 0)
    {
        hal_cache_tag_flush();
    }

    /* restore IRQs */
    spif_unlock(cs);

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_lock();
#endif

    return ret;
}

int hal_flash_erase_block64(unsigned int addr)
{
    int ret = PPlus_SUCCESS;
    int retval;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_unlock();
#endif

    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;

    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

    retval = spif_erase_block64(addr);

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WELWIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

out:
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if (cb == 0)
    {
        hal_cache_tag_flush();
    }

    /* restore IRQs */
    spif_unlock(cs);

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_lock();
#endif

    return retval;
}

int hal_flash_erase_all(void)
{
    int ret = PPlus_SUCCESS;
    int retval;

    if (phy_flash.init_flag == FLASH_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_unlock();
#endif

    uint32_t cs = spif_lock();
    uint32_t cb = AP_PCR->CACHE_BYPASS;

    HAL_CACHE_ENTER_BYPASS_SECTION();
    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

    retval = spif_erase_all();

    SPIF_STATUS_WAIT_IDLE(SPIF_WAIT_IDLE_CYC);
    ret = _spif_wait_nobusy_no_rom(SFLG_WELWIP, SPIF_TIMEOUT);
    if (ret != PPlus_SUCCESS)
    {
        retval = ret;
        goto out;
    }

out:
    HAL_CACHE_EXIT_BYPASS_SECTION();

    if (cb == 0)
    {
        hal_cache_tag_flush();
    }

    /* restore IRQs */
    spif_unlock(cs);

#if (FLASH_PROTECT_FEATURE == 1)
    hal_flash_lock();
#endif

    return retval;
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
