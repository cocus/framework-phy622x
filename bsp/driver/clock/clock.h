/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_CLOCK_H
#define _HAL_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <phy62xx.h>

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup CLOCK
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup CLOCK_Exported_Types CLOCK Exported Types
  * @{
  */

/**
  * @brief  PHY62xx internal module enumeration
  */
typedef enum
{
  /*!< First group of modules: Affects the SW_CLK register */
  MOD_NONE = 0,                         /*!< ? */
  MOD_CK802_CPU = PCR_SWCTL_CK802_Pos,  /*!< ? */
  MOD_DMA = PCR_SWCTL_DMA_Pos,          /*!< DMA Module */
  MOD_AES = PCR_SWCTL_AES_Pos,          /*!< AES Module */
  MOD_IOMUX = PCR_SWCTL_IOMUX_Pos,      /*!< IOMUX Module */
  MOD_UART0 = PCR_SWCTL_UART0_Pos,      /*!< UART0 Module */
  MOD_I2C0 = PCR_SWCTL_I2C0_Pos,        /*!< I2C0 Module */
  MOD_I2C1 = PCR_SWCTL_I2C1_Pos,        /*!< I2C1 Module */
  MOD_SPI0 = PCR_SWCTL_SPI0_Pos,        /*!< SPI0 Module */
  MOD_SPI1 = PCR_SWCTL_SPI1_Pos,        /*!< SPI1 Module */
  MOD_GPIO = PCR_SWCTL_GPIO_Pos,        /*!< GPIO Module */
  MOD_QDEC = PCR_SWCTL_QDEC_Pos,        /*!< QDEC Module */
  MOD_ADCC = PCR_SWCTL_ADCC_Pos,        /*!< ADCC Module */
  MOD_PWM = PCR_SWCTL_PWM_Pos,          /*!< PWM Module */
  MOD_SPIF = PCR_SWCTL_SPIF_Pos,        /*!< SPIF Module */
  MOD_VOC = PCR_SWCTL_VOC_Pos,          /*!< VOC Module */
  MOD_TIMER5 = PCR_SWCTL_TIM5_Pos,      /*!< TIMER5 Module */
  MOD_TIMER6 = PCR_SWCTL_TIM6_Pos,      /*!< TIMER6 Module */
  MOD_UART1 = PCR_SWCTL_UART1_Pos,      /*!< UART1 Module */

  /*!< Second group of modules: Affects the SW_CLK1 register */
  MOD_CP_CPU = 0 + 32,                  /*!< CP_CPU Module */
  MOD_BB = MOD_CP_CPU + 3,              /*!< BB Module */
  MOD_TIMER = MOD_CP_CPU + 4,           /*!< TIMER Module */
  MOD_WDT = MOD_CP_CPU + 5,             /*!< WDT Module */
  MOD_COM = MOD_CP_CPU + 6,             /*!< COM Module */
  MOD_KSCAN = MOD_CP_CPU + 7,           /*!< KSCAN Module */
  MOD_BBREG = MOD_CP_CPU + 9,           /*!< BBREG Module */
  BBLL_RST = MOD_CP_CPU + 10,           /*!< BB LL "module", can reset, but can't set clock gate */
  BBTX_RST = MOD_CP_CPU + 11,           /*!< BB TX "module", can reset, but can't set clock gate */
  BBRX_RST = MOD_CP_CPU + 12,           /*!< BB RX "module", can reset, but can't set clock gate */
  BBMIX_RST = MOD_CP_CPU + 13,          /*!< BB MIX "module", can reset, but can't set clock gate */
  MOD_TIMER1 = MOD_CP_CPU + 21,         /*!< TIMER1 Module */
  MOD_TIMER2 = MOD_CP_CPU + 22,         /*!< TIMER2 Module */
  MOD_TIMER3 = MOD_CP_CPU + 23,         /*!< TIMER3 Module */
  MOD_TIMER4 = MOD_CP_CPU + 24,         /*!< TIMER4 Module */

  /*!< Third group of modules: Affects the CACHE_CLOCK_GATE register */
  MOD_PCLK_CACHE = 0 + 64,
  MOD_HCLK_CACHE = MOD_PCLK_CACHE + 1,

  /*!< Fourth group of modules: Unknown */
  MOD_USR0 = 0 + 96,
  MOD_USR1 = MOD_USR0 + 1,
  MOD_USR2 = MOD_USR0 + 2,
  MOD_USR3 = MOD_USR0 + 3,
  MOD_USR4 = MOD_USR0 + 4,
  MOD_USR5 = MOD_USR0 + 5,
  MOD_USR6 = MOD_USR0 + 6,
  MOD_USR7 = MOD_USR0 + 7,
  MOD_USR8 = MOD_USR0 + 8,
  MOD_SYSTEM = 0xFF,
} MODULE_e;

/**
  * @brief  PHY62xx 32K clock selection
  */
typedef enum
{
  CLK_32K_XTAL = 0,       /*!< External 32.768kHz XT */
  CLK_32K_RCOSC = 1,      /*!< Internal 32.768kHz RC oscillator */
} CLK32K_e;

/**
  * @brief  PHY62xx system clock selection
  */
typedef enum _SYSCLK_SEL
{
  SYS_CLK_RC_32M = 0,     /*!< Internal RC ? oscillator, 32MHz */
  SYS_CLK_DBL_32M = 1,    /*!< External 16MHz XT, PLLx2, 32MHz */
  SYS_CLK_XTAL_16M = 2,   /*!< External 16MHz XT, No PLL, 16MHz */
  SYS_CLK_DLL_48M = 3,    /*!< External 16MHz XT, PLLx3, 48MHz */
  SYS_CLK_DLL_64M = 4,    /*!< External 16MHz XT, PLLx4, 64MHz */
  SYS_CLK_DLL_96M = 5,    /*!< External 16MHz XT, PLLx5, 96MHz */
  SYS_CLK_8M = 6,         /*!< ? 8MHz */
  SYS_CLK_4M = 7,         /*!< ? 4MHz */
} sysclk_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup CLOCK_Exported_Constants CLOCK Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup CLOCK_Exported_Macros CLOCK Exported Macros
  * @{
  */
#define TIME_DELTA(x, y) ((x >= y) ? x - y : TIME_BASE - y + x)

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CLOCK_Exported_Functions
  * @{
  */
    /**
      * @brief  Enables the clock to a specified module inside the MCU.
      * @param  module: One of the MODULE_e for which the clock should be enabled.
      * @note   This effectively enables the specified module, which will
      *         start consuming power.
      * @retval None.
      */
    void hal_clk_gate_enable(MODULE_e module);

    /**
      * @brief  Disables the clock to a specified module inside the MCU.
      * @param  module: One of the MODULE_e for which the clock should be disabled.
      * @note   This effectively disables the specified module. Access to non-clocked
      *         modules might cause HardFaults (TBD!).
      * @retval None.
      */
    void hal_clk_gate_disable(MODULE_e module);

    /**
      * @brief  Returns the clock gating of a specified module inside the MCU.
      * @param  module: One of the MODULE_e to check for the clock gating.
      * @retval 1 if the module is enabled, 0 otherwise.
      */
    int hal_clk_gate_get(MODULE_e module);

    /**
      * @brief  Triggers a reset of a specified module inside the MCU.
      * @param  module: One of the MODULE_e to be reset.
      * @retval None.
      */
    void hal_clk_reset(MODULE_e module);

    /**
      * @brief  Gets the current System Clock frequency in Hz, derived from the ROM variable g_system_clk.
      * @param  None.
      * @retval System clock frequency in Hz.
      */
    uint32_t sysclk_get_clk(void);

    /**
      * @brief  Configures the RTC clock frequency to the specified one.
      * @param  clk32Mode: CLK_32K_XTAL or CLK_32K_RCOSC.
      *          * CLK_32K_XTAL for an external 32kHz XT,
      *          * CLK_32K_RCOSC for the internal RC one
      * @retval None.
      */
    void hal_rtc_clock_config(CLK32K_e clk32Mode);

    /**
      * @brief  Calculates the time that passed since the provided tick time (in 625uS steps).
      * @param  tick: Start tick time.
      * @retval Time difference in milliseconds.
      */
    uint32_t hal_ms_intv(uint32_t tick);

    /**
      * @brief  Enables the RTC and busy waits for a given number of milliseconds.
      * @param  msecond: Number of milliseconds to wait.
      * @retval None.
      */
    void WaitMs(uint32_t msecond);

    /**
      * @brief  Resets the MCU causing a warm reset.
      * @param  None.
      * @retval None.
      */
    void hal_system_soft_reset(void);

    // TODO!!!: Check if FLASH actually contains valid stuff void hal_rfPhyFreqOff_Set(void);
    // TODO!!!: Check if FLASH actually contains valid stuff void hal_xtal16m_cap_Set(void);
    void hal_rc32k_clk_tracking_init(void);

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup CLOCK_Exported_ROM_Functions CLOCK Exported ROM Functions
  * @{
  */
    /**
      * @brief  Change the System Clock to the specified one.
      * @param  h_system_clk_sel: specifies the clock to use.
      *          This parameter can be any of the SYS_CLK_x values.
      * @retval Always 0.
      */
    ATTR_ROM_FN uint32_t clk_init(sysclk_t h_system_clk_sel);

    /**
      * @brief  Return the PCLK clock frequency.
      * @param  None.
      * @retval PCLK frequency in Hz.
      */
    ATTR_ROM_FN uint32_t clk_get_pclk(void);

    /**
      * @brief  Enables the RTC and busy waits for a number of RTC cycles.
      * @param  rtcDelyCnt: Number of RTC cycles to wait. Each RTC tick is 32uS.
      * @retval None.
      */
    ATTR_ROM_FN void WaitRTCCount(uint32_t rtcDelyCnt);

    /**
      * @brief  Busy waits for a given number of microseconds using the TIM3 as a reference.
      * @param  wtTime: Number of microseconds to wait.
      * @retval None.
      */
    ATTR_ROM_FN void WaitUs(uint32_t wtTime);

    /**
      * @brief  Reads the current RTC cycles register.
      * @param  None.
      * @retval Number of RTC cycles from the RTCCNT register. Each RTC tick is 32uS.
      */
    ATTR_ROM_FN uint32_t rtc_get_counter(void);

    /**
      * @brief  Reads the current OSAL tick count.
      * @param  None.
      * @retval Number of OSAL tick count. Each tick is 625uS.
      */
    ATTR_ROM_FN uint32_t getMcuPrecisionCount(void);

    ATTR_ROM_FN int clk_spif_ref_clk(sysclk_t spif_ref_sel); /* TODO!!!: unknown */

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup CLOCK_Exported_ROM_Variables CLOCK Exported ROM Variables
  * @{
  */
    ATTR_ROM_VAR sysclk_t g_system_clk;
    ATTR_ROM_VAR uint32_t counter_tracking;
    ATTR_ROM_VAR uint32_t g_counter_traking_avg;
    ATTR_ROM_VAR uint32_t g_counter_traking_cnt;

    ATTR_ROM_VAR uint32_t g_hclk;
    ATTR_ROM_VAR uint32_t osal_sys_tick;

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

#endif /* _HAL_CLOCK_H */
