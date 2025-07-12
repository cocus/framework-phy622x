/**
  ******************************************************************************
  * @file    clock.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   Clock BSP module driver.
  *          This file provides firmware functions to manage the clock related
  *          functionalities of the MCU:
  *           + Enabling/Disabling clock to peripherals (modules)
  *           + Resets peripherals (modules)
  *           + Configure the RTC clock
  *           + Provide delay functions with microsecond accuaracy
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "clock.h"

#include <driver/gpio/gpio.h>

#include <types.h> /* for BIT, subWriteReg */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup CLOCK
  * @brief Clock BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @addtogroup CLOCK_Private_Constants CLOCK Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup CLOCK_Exported_Functions Clock related exported functions
  * @{
  */
uint32_t sysclk_get_clk(void)
{
    switch (g_system_clk)
    {
    case SYS_CLK_RC_32M:
    case SYS_CLK_DBL_32M:
        return 32000000;
    case SYS_CLK_XTAL_16M:
        return 16000000;
    case SYS_CLK_DLL_48M:
        return 48000000;
    case SYS_CLK_DLL_64M:
        return 64000000;
    case SYS_CLK_DLL_96M:
        return 96000000;
    default:
        return 16000000; /* safe default */
    }
}

void hal_clk_gate_enable(MODULE_e module)
{
    if (module < MOD_CP_CPU)
    {
        PCR_SWCLK |= BIT(module);
    }
    else if (module < MOD_PCLK_CACHE)
    {
        PCR_SWCLK1 |= BIT(module-MOD_CP_CPU);
    }
    else if (module < MOD_USR0)
    {
        PCR_CACHE_CLOCK_GATE |= BIT(module-MOD_PCLK_CACHE);
    }
}

void hal_clk_gate_disable(MODULE_e module)
{
    // TODO!!!: Use atomic bit unset
    if (module < MOD_CP_CPU)
    {
        PCR_SWCLK &= ~(BIT(module));
    }
    else if (module < MOD_PCLK_CACHE)
    {
        PCR_SWCLK1 &= ~(BIT(module-MOD_CP_CPU));
    }
    else if (module < MOD_USR0)
    {
        PCR_CACHE_CLOCK_GATE &= ~(BIT(module-MOD_PCLK_CACHE));
    }
}

int hal_clk_gate_get(MODULE_e module)
{
    // TODO!!!: Use atomic bit unset
    if (module < MOD_CP_CPU)
    {
        return (PCR_SWCLK & BIT(module));
    }
    else if (module < MOD_PCLK_CACHE)
    {
        return (PCR_SWCLK1 & BIT(module-MOD_CP_CPU));
    }
    // else if(module < MOD_USR0)
    else
    {
        return (PCR_CACHE_CLOCK_GATE & BIT(module-MOD_PCLK_CACHE));
    }
}

void hal_clk_reset(MODULE_e module)
{
    if(module < MOD_CP_CPU)
    {
        if((module >= MOD_TIMER5) &&(module <= MOD_TIMER6))
        {
            PCR_SW_RESET0 &= ~BIT(5);
            PCR_SW_RESET0 |= BIT(5);
        }
        else
        {
            PCR_SW_RESET0 &= ~BIT(module);
            PCR_SW_RESET0 |= BIT(module);
        }
    }
    else if(module < MOD_PCLK_CACHE)
    {
        if((module >= MOD_TIMER1) &&(module <= MOD_TIMER4))
        {
            PCR_SW_RESET2 &= ~BIT(4);
            PCR_SW_RESET2 |= BIT(4);
        }
        else
        {
            PCR_SW_RESET2 &= ~BIT(module-MOD_CP_CPU);
            PCR_SW_RESET2 |= BIT(module-MOD_CP_CPU);
        }
    }
    else if(module < MOD_USR0)
    {
        PCR_CACHE_RST &= ~BIT(1-(module-MOD_HCLK_CACHE));
        PCR_CACHE_RST |= BIT(1-(module-MOD_HCLK_CACHE));
    }
}

void hal_rtc_clock_config(CLK32K_e clk32Mode)
{
    if (clk32Mode == CLK_32K_RCOSC)
    {
        subWriteReg(&(AON_PMCTL0),31,27,0x05);
        subWriteReg(&(AON_PMCTL2_0),16,7,0x3fb);
        subWriteReg(&(AON_PMCTL2_0),6,6,0x01);
        //pGlobal_config[LL_SWITCH]|=RC32_TRACKINK_ALLOW|LL_RC32K_SEL;
    }
    else if (clk32Mode == CLK_32K_XTAL)
    {
        // P16 P17 for 32K XTAL input
        hal_gpio_pull_set(P16,FLOATING);
        hal_gpio_pull_set(P17,FLOATING);
        subWriteReg(&(AON_PMCTL2_0),9,8,0x03);   //software control 32k_clk
        subWriteReg(&(AON_PMCTL2_0),6,6,0x00);   //disable software control
        subWriteReg(&(AON_PMCTL0),31,27,0x16);
        //pGlobal_config[LL_SWITCH]&=0xffffffee;
    }

    //    //ZQ 20200812 for rc32k wakeup
    //    subWriteReg(&(AON_PMCTL0),28,28,0x1);//turn on 32kxtal
    //    subWriteReg(&(AP_AON->PMCTL1),18,17,0x0);// reduce 32kxtl bias current
}

uint32_t hal_ms_intv(uint32_t tick)
{
    uint32_t diff = 0;
    uint32_t osal = getMcuPrecisionCount();

    /* each tick is 625us */
    if (osal < tick)
    {
        diff = 0xffffffff - tick;
        diff = osal + diff;
    }
    else
    {
        diff = osal - tick;
    }

    return diff * 625 / 1000;
}

void WaitMs(uint32_t msecond)
{
    WaitRTCCount((msecond << 15) / 1000); // step 32us
}

void hal_system_soft_reset(void)
{
    // HAL_ENTER_CRITICAL_SECTION();
    __disable_irq();
    /**
        config reset casue as RSTC_WARM_NDWC
        reset path walkaround dwc
    */
    PCRM_SLEEPR0 = 4;

    PCRM_CLEAR_XTAL_TRACKING_AND_CALIB;

    PCR_SW_RESET1 = 0;

    while (1)
        ;
}

void hal_rc32k_clk_tracking_init(void)
{
    if(g_counter_traking_avg == 0) {
    	counter_tracking = g_counter_traking_avg = 7812;
    	AP_PCRM->SLEEP_R[1]=0;
    }
}

#if 0
#define CHIP_RFEQ_OFF_FLASH_ADDRESS 0x11001e08 // was 0x11004008
#define CHIP_XTAK_CAP_FLASH_ADDRESS 0x11001e0c // was 0x1100400c

__ATTR_SECTION_XIP__  void hal_rfPhyFreqOff_Set(void)
{
    int32_t freqPpm=0;
    freqPpm= *(volatile int32_t*) CHIP_RFEQ_OFF_FLASH_ADDRESS; // was 0x11004008

    if((freqPpm!=-1) && (freqPpm>=-50) && (freqPpm<=50))
    {
        g_rfPhyFreqOffSet=(int8_t)freqPpm;
    }
    else
    {
        g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    }
}

__ATTR_SECTION_XIP__ void hal_xtal16m_cap_Set(void)
{
    uint32_t cap = 0;
    cap = *(volatile int32_t *)CHIP_XTAK_CAP_FLASH_ADDRESS; // was 0x1100400c

    if ((cap != 0xffffffff) && (cap <= 0x1f))
    {
        XTAL16M_CAP_SETTING(cap);
    }
    else
    {
        XTAL16M_CAP_SETTING(0x09);
    }
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
