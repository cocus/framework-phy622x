/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_PWM_H
#define _HAL_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include <stddef.h> // for size_t

#include <driver/gpio/gpio.h> /* for gpio_pin_e */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup PWM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup PWM_Exported_Types PWM Exported Types
  * @{
  */

/**
  * @brief  PHY62xx PWM channels.
  */
typedef enum
{
  PWM_CH0 = 0,              /*!< PWM Channel 0 */
  PWM_CH1 = 1,              /*!< PWM Channel 1 */
  PWM_CH2 = 2,              /*!< PWM Channel 2 */
  PWM_CH3 = 3,              /*!< PWM Channel 3 */
  PWM_CH4 = 4,              /*!< PWM Channel 4 */
  PWM_CH5 = 5               /*!< PWM Channel 5 */
} PWMN_e;

/**
  * @brief  PHY62xx PWM clock prescalers. Per datasheet, master clock for PWM is 16MHz.
  */
typedef enum
{
  PWM_CLK_NO_DIV = 0,       /*!< No clock division, so PWM runs at 16MHz */
  PWM_CLK_DIV_2 = 1,        /*!< Clock/2, so PWM runs at 8MHz */
  PWM_CLK_DIV_4 = 2,        /*!< Clock/4, so PWM runs at 4MHz */
  PWM_CLK_DIV_8 = 3,        /*!< Clock/8, so PWM runs at 2MHz */
  PWM_CLK_DIV_16 = 4,       /*!< Clock/16, so PWM runs at 1MHz */
  PWM_CLK_DIV_32 = 5,       /*!< Clock/32, so PWM runs at 500kHz */
  PWM_CLK_DIV_64 = 6,       /*!< Clock/64, so PWM runs at 250kHz */
  PWM_CLK_DIV_128 = 7       /*!< Clock/128, so PWM runs at 125kHz */
} PWM_CLK_DIV_e;

/**
  * @brief  PHY62xx PWM counter modes.
  */
typedef enum
{
  PWM_CNT_UP = 0,           /*!< PWM counter goes up until it resets */
  PWM_CNT_UP_AND_DOWN = 1   /*!< PWM counter goes up and then ramps down instead of reset */
} PWM_CNT_MODE_e;

/**
  * @brief  PHY62xx PWM output polarities.
  */
typedef enum
{
  PWM_POLARITY_RISING = 0,  /*!< PWM will be active only when counter is above the threshold */
  PWM_POLARITY_FALLING = 1  /*!< PWM will be active only when counter is below the threshold */
} PWM_POLARITY_e;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup PWM_Exported_Constants PWM Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup PWM_Exported_Macros PWM Exported Macros
  * @{
  */

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PWM_Exported_Functions
  * @{
  */
    /**
      * @brief  Initializes a PWM channel with the provided parameters.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @param  pwmDiv: PWM channel prescaler/divider, one of PWM_CLK_DIV_e.
      * @param  pwmMode: PWM channel counter mode, one of PWM_CNT_MODE_e.
      * @param  pwmPolarity: PWM channel polarity, one of PWM_POLARITY_e.
      * @param  pwmPin: Pin to link to this PWM channel, one of gpio_pin_e.
      * @note   Will also enaable the clock of the PWM module, register the PWM in
      *         pwrmgr, disable the selected channel, set the prescaler/divider,
      *         counter mode and polarity and set the "instant load" flag.
      *         The pin will be muxed to this PWM channel. Remember to enable the
      *         channel!
      * @retval None.
      */
    void hal_pwm_init(PWMN_e pwmN, PWM_CLK_DIV_e pwmDiv, PWM_CNT_MODE_e pwmMode,
                      PWM_POLARITY_e pwmPolarity, gpio_pin_e pwmPin);

    /**
      * @brief  De-initializes a previously-initialized PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @note   Will set the "instant load" flag, the divisor, mode, polarity,
      *         compare value and count to zero. If there's no other PWM channel
      *         initialized, it will also disable the pwm by calling "hal_pwm_disable",
      *         and disabling the clock of the PWM module.
      * @retval None.
      */
    void hal_pwm_destroy(PWMN_e pwmN);

    /**
      * @brief  Enables the counter of a previously-initialized PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @retval None.
      */
    void hal_pwm_channel_enable(PWMN_e pwmN);

    /**
      * @brief  Disables the counter for a previously-initialized PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @note   PWM pin status will not be affected by this.
      * @retval None.
      */
    void hal_pwm_channel_disable(PWMN_e pwmN);

    /**
      * @brief  Sets the threshold value and the maximum count of a PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @param  cmpVal: Threshold value to set for this PWM channel, can't be above "cntTopVal".
      * @param  cntTopVal: Maximum value for the count of this PWM channel.
      * @retval None.
      */
    void hal_pwm_set_count_top_val(PWMN_e pwmN, uint16_t cmpVal, uint16_t cntTopVal);

    /**
      * @brief  Sets the threshold value of a PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @param  cmpVal: Threshold value to set for this PWM channel, can't be above the
      *         previously configured "cntTopVal".
      * @retval None.
      */
    void hal_pwm_set_count_val(PWMN_e pwmN, uint16_t cmpVal);

    /**
      * @brief  Sets the maximum count of a PWM channel.
      * @param  pwmN: PWM channel, one of PWMN_e.
      * @param  cntTopVal: Maximum value for the count of this PWM channel.
      * @retval None.
      */
    void hal_pwm_set_val_top(PWMN_e pwmN, uint16_t cntTopVal);

    /**
      * @brief  Enables the PWM module.
      * @note   PWM module needs to be enabled in order for PWM signals to
      *         be generated.
      * @retval None.
      */
    void hal_pwm_enable(void);

    /**
      * @brief  Disables the PWM module.
      * @retval None.
      */
    void hal_pwm_disable(void);

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup PWM_Exported_ROM_Functions PWM Exported ROM Functions
  * @{
  */

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup PWM_Exported_ROM_Variables PWM Exported ROM Variables
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

#endif /* _HAL_PWM_H */
