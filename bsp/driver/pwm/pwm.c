/**
  ******************************************************************************
  * @file    pwm.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   PWM BSP module driver.
  *          This file provides firmware functions to manage the PWM module
  *          on the MCU, the pins related to PWM channels, and provides a
  *          convenient API for setting their values.
  *           + Initializing a PWM channel, setting its clock dividers, counter
  *             mode, polarity, etc.
  *           + Enable/Disable a specific channel or the entire PWM module.
  *           + Set the threshold and maximum values for a PWM channel.
  *           + De-initialize a PWM channel.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <phy62xx.h>

#include "pwm.h"

#include <driver/pwrmgr/pwrmgr.h>

#include <log/log.h>

#include <phy_error.h>

#include <string.h>

#include <types.h> /* for BIT, UNUSED, subWriteReg */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup PWM
  * @brief PWM BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @addtogroup PWM_Private_Typedef PWM Private Typedefs
  * @{
  */

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/** @addtogroup PWM_Private_Constants PWM Private Constants
  * @{
  */

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @addtogroup PWM_Private_Constants PWM Private macros
  * @{
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @addtogroup PWM_Private_Variables GPIO Private variables
  * @{
  */
static uint8_t is_clocked = 0;

static gpio_pin_e pwm_gpio_map[] =
{
    GPIO_DUMMY,
    GPIO_DUMMY,
    GPIO_DUMMY,
    GPIO_DUMMY,
    GPIO_DUMMY,
    GPIO_DUMMY,
};

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup PWM_Private_Functions PWM Private functions
  * @{
  */

inline static void pwm_instant_load(PWMN_e pwmN, uint8_t value)
{
    if (value)
    {
        AP_PWM->PWM[pwmN].ctrl0 |= PWM_CTRL0_INST_LOAD;
    }
    else
    {
        AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_INST_LOAD;
    }
}

inline static void pwm_load(PWMN_e pwmN, uint8_t value)
{
    if (value)
    {
        AP_PWM->PWM[pwmN].ctrl0 |= PWM_CTRL0_LOAD;
    }
    else
    {
        AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_LOAD;
    }
}

inline static void pwm_set_div(PWMN_e pwmN, PWM_CLK_DIV_e value)
{
    AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_DIV_Msk;
    AP_PWM->PWM[pwmN].ctrl0 |= ((uint32_t)value) << PWM_CTRL0_DIV_Pos;
}

inline static void pwm_set_mode(PWMN_e pwmN, PWM_CNT_MODE_e value)
{
    if (value == PWM_CNT_UP_AND_DOWN)
    {
        AP_PWM->PWM[pwmN].ctrl0 |= PWM_CTRL0_MODE;
    }
    else
    {
        AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_MODE;
    }
}

inline static void pwm_set_pol(PWMN_e pwmN, PWM_POLARITY_e value)
{
    if (value == PWM_POLARITY_FALLING)
    {
        AP_PWM->PWM[pwmN].ctrl0 |= PWM_CTRL0_POL;
    }
    else
    {
        AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_POL;
    }
}

inline static void pwm_ch_en(PWMN_e pwmN, uint8_t value)
{
    if (value)
    {
        AP_PWM->PWM[pwmN].ctrl0 |= PWM_CTRL0_EN;
    }
    else
    {
        AP_PWM->PWM[pwmN].ctrl0 &= ~PWM_CTRL0_EN;
    }
}

inline static void pwm_set_cmp(PWMN_e pwmN, uint16_t cmp)
{
    AP_PWM->PWM[pwmN].ctrl1 &= ~PWM_CTRL1_CMP_Msk;
    AP_PWM->PWM[pwmN].ctrl1 |= ((uint32_t)cmp) << PWM_CTRL1_CMP_Pos;
}

inline static void pwm_set_cnt(PWMN_e pwmN, uint16_t cnt)
{
    AP_PWM->PWM[pwmN].ctrl1 &= ~PWM_CTRL1_CNT_Msk;
    AP_PWM->PWM[pwmN].ctrl1 |= ((uint32_t)cnt) << PWM_CTRL1_CNT_Pos;
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup PWM_Exported_Functions PWM related exported functions
  * @{
  */

void hal_pwm_init(
    PWMN_e pwmN,
    PWM_CLK_DIV_e pwmDiv,
    PWM_CNT_MODE_e pwmMode,
    PWM_POLARITY_e pwmPolarity,
    gpio_pin_e pwmPin)
{
    if (is_clocked == 0)
    {
        hal_clk_gate_enable(MOD_PWM);
        hal_pwrmgr_register(MOD_PWM, NULL, NULL);
        is_clocked = 1;
    }

    pwm_ch_en(pwmN, 0);
    pwm_set_div(pwmN, pwmDiv);
    pwm_set_mode(pwmN, pwmMode);
    pwm_set_pol(pwmN, pwmPolarity);
    pwm_instant_load(pwmN, 1);

    hal_gpio_fmux_set(pwmPin, (gpio_fmux_e)(FMUX_PWM0 + pwmN));
    pwm_gpio_map[pwmN] = pwmPin;
    pwm_ch_en(pwmN, 1);
}

void hal_pwm_destroy(PWMN_e pwmN)
{
    if (is_clocked == 0)
    {
        return;
    }

    pwm_ch_en(pwmN, 0);

    if(pwm_gpio_map[pwmN] != GPIO_DUMMY)
    {
        hal_gpio_fmux(pwm_gpio_map[pwmN], Bit_DISABLE);
        pwm_gpio_map[pwmN] = GPIO_DUMMY;
    }

    pwm_load(pwmN, 0);
    pwm_instant_load(pwmN, 0);
    pwm_set_div(pwmN, 0);
    pwm_set_mode(pwmN, 0);
    pwm_set_pol(pwmN, 0);
    pwm_set_cmp(pwmN, 0);
    pwm_set_cnt(pwmN, 0);

    /* Look for any other PWM in use */
    for (size_t i = 0; i < sizeof(pwm_gpio_map)/sizeof(pwm_gpio_map[0]); i++)
    {
        if (pwm_gpio_map[i] != GPIO_DUMMY)
        {
            /* This one is in use, so don't de-init the PWM */
            return;
        }
    }

    hal_pwm_disable();
    hal_clk_gate_disable(MOD_PWM);
    is_clocked = 0;
}

void hal_pwm_channel_enable(PWMN_e pwmN)
{
    if (is_clocked == 0)
    {
        return;
    }

    pwm_ch_en(pwmN, 1);
}

void hal_pwm_channel_disable(PWMN_e pwmN)
{
    if (is_clocked == 0)
    {
        return;
    }

    pwm_ch_en(pwmN, 0);
}

void hal_pwm_set_count_top_val(PWMN_e pwmN, uint16_t cmpVal, uint16_t cntTopVal)
{
    if (is_clocked == 0)
    {
        return;
    }

    if(cmpVal > cntTopVal)
    {
        return;
    }

    pwm_load(pwmN, 0);
    pwm_set_cmp(pwmN, cmpVal);
    pwm_set_cnt(pwmN, cntTopVal);
    pwm_load(pwmN, 1);
}

void hal_pwm_set_count_val(PWMN_e pwmN, uint16_t cmpVal)
{
    if (is_clocked == 0)
    {
        return;
    }

    pwm_load(pwmN, 0);
    pwm_set_cmp(pwmN, cmpVal);
    pwm_load(pwmN, 1);
}

void hal_pwm_set_val_top(PWMN_e pwmN, uint16_t cntTopVal)
{
    if (is_clocked == 0)
    {
        return;
    }

    pwm_load(pwmN, 0);
    pwm_set_cnt(pwmN, cntTopVal);
    pwm_load(pwmN, 1);
}

void hal_pwm_enable(void)
{
    if (is_clocked == 0)
    {
        return;
    }

    if((PWM_PWMEN & (PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL)) != (PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL))
    {
        hal_pwrmgr_lock(MOD_PWM);
        PWM_PWMEN |= (PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL);
    }
}

void hal_pwm_disable(void)
{
    if (is_clocked == 0)
    {
        return;
    }

    if((PWM_PWMEN & (PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL)) == (PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL))
    {
        PWM_PWMEN &= ~(PWM_PWMEN_EN_ALL | PWM_PWMEN_LOAD_ALL);
        hal_pwrmgr_unlock(MOD_PWM);
    }
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
