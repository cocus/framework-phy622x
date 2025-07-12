/**
  ******************************************************************************
  * @file    adc.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   ADC BSP module driver.
  *          This file provides firmware functions to manage the ADCs.
  *           + 
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <phy62xx.h>

#include "adc.h"

#include <driver/clock/clock.h> /* for g_system_clk */

#include <driver/pwrmgr/pwrmgr.h> /* for hal_pwrmgr_register */

#include <driver/gpio/gpio.h> /* for gpio_pin_e */

#include <log/log.h> /* for LOG */

#include <jump_function.h> /* for JUMP_FUNCTION */

#include <types.h> /* for subWriteReg and BIT */

#include <stddef.h> /* for NULL */

#include <string.h> /* for memcpy */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @defgroup ADC
  * @brief ADC BSP module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/** @addtogroup ADC_Private_Typedef ADC Private Typedefs
  * @{
  */

/**
 * @brief  Status of the ADC context structure.
 */
typedef enum
{
    ADC_CTX_NOT_INITALIZED = 0U,        /*!< Context is not initialized yet */
    ADC_CTX_INITIALIZED = 1U            /*!< Context is initialized */
} ADC_CTX_INIT_t;

/**
 * @brief  ADC context structure, holding the status of the ADC module.
 */
typedef struct _adc_Contex_t
{
    ADC_CTX_INIT_t  state;              /*!< Status (initialized or not) */

    uint16_t        irq_bits;           /*!< IRQ bits to set on ADCC->intr* fields */

    uint16_t        adc_cal_postive;    /*!< Positive calibration value for the ADC module */
    uint16_t        adc_cal_negtive;    /*!< Negative calibration value for the ADC module */

    int             adc_cal_sum;        /*!< Pre-computed value of adc_cal_postive + adc_cal_negtive */
    int             adc_cal_diff;       /*!< Pre-computed value of adc_cal_postive - adc_cal_negtive */

    adc_Cfg_t       channels[NUM_ADCS]; /*!< */

    adc_Hdl_t       evt_handler;        /*!< User-defined callback */
} adc_Ctx_t;

/**
 * @brief  ADC description structure, containing registers, bits, channel mappings, etc.
 */
typedef struct
{
    adc_CH_t        hal_adc;                /*!< Old style HAL channel number (TODO!!!: why is this needed?) */
    adc_channels_t  ch;                     /*!< ADC channel number */
    gpio_pin_e      single_ended_pin;       /*!< GPIO pin related to the ADC channel */
    uint8_t         pmctl2_1_bit;           /*!< PMCTL2_1 bit related to the ADC channel */
    uint8_t         aio_differential_pair;  /*!< technically the other PMCTL2_1 bit for the other pin of a differential input pair */
    uint8_t         buffer_index;           /*!< IRQ bit number and ADC sample buffer index related to the ADC channel */
    __IO uint32_t * adc_ctl;                /*!< Pointer to PCRM_ADCCTLx related to the ADC channel */
    uint8_t         adc_ctl_is_n;           /*!< Set to 1 if the channel is the negative side of a differential pair */
} adc_description_t;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
/** @addtogroup ADC_Private_Constants ADC Private Constants
  * @{
  */
#define SPIF_RSVD_AREA_1                (0x1000)
#define pSPIF_RSVD1_ADC_CALIBRATE       ((volatile uint32_t*)(FLASH_BASE_ADDR + SPIF_RSVD_AREA_1))
#define SPIF_RSVD1_ADC_CALIBRATE        (FLASH_BASE_ADDR + SPIF_RSVD_AREA_1)

#define ADC_CALIBRATION_INVALID         0xFFF
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @addtogroup ADC_Private_Variables ADC Private variables
  * @{
  */

static adc_Ctx_t mAdc_Ctx = {
    .state = ADC_CTX_NOT_INITALIZED,
    .irq_bits = 0x0000,
    .adc_cal_postive = ADC_CALIBRATION_INVALID,
    .adc_cal_negtive = ADC_CALIBRATION_INVALID,
    .channels = { { 0 } },
    .evt_handler = NULL
};

static const adc_description_t adcs[] = {
    /* aio<0> = gpio<11>, input b neg */
    [0] = { ADC_CH1N_P11, CH0, P11, 0, 1, 3, &PCRM_ADCCTL1, 1 },
    /* aio<1> = gpio<23>, input b pos, microphone bias ref voltage */
    [1] = { ADC_CH1P_P23, CH1, P23, 1, 0, 2, &PCRM_ADCCTL1, 0 },

    /* aio<2> = gpio<24>, input c neg */
    [2] = { ADC_CH2N_P24, CH2, P24, 2, 3, 5, &PCRM_ADCCTL2, 1 },
    /* aio<3> = gpio<14>, input c pos */
    [3] = { ADC_CH2P_P14, CH3, P14, 3, 2, 4, &PCRM_ADCCTL2, 0 },

    /* aio<4> = gpio<15>, input d neg */
    [4] = { ADC_CH3N_P15, CH4, P15, 4, 7, 7, &PCRM_ADCCTL3, 1 },
    /* aio<9> = gpio<20>, input d pos, PGA neg */
    [9] = { ADC_CH3P_P20, CH9, P20, 7, 4, 6, &PCRM_ADCCTL3, 0 },

    /* aio<5> = gpio<16>, 32k xtal input */
    [5] = { 0, CH5, P16, 0, 0, 0, NULL, 1 },
    /* aio<6> = gpio<17>, 32k xtal output */
    [6] = { 0, CH6, P17, 0, 0, 0, NULL, 0 },

    /* aio<7> = gpio<18>, input a pos, PGA pos */
    [7] = { ADC_CH0DIFF, CH7, P18, 0, 0, 1 /* TBD */, &PCRM_ADCCTL0, 1 },
    /* aio<8> = gpio<25>, input a neg */
    [8] = { ADC_CH0DIFF, CH8, P25, 0, 0, 0 /* TBD */, &PCRM_ADCCTL0, 0 },
};

#if 0
static const struct {
    gpio_pin_e      pin_pos;
    gpio_pin_e      pin_neg;
    uint8_t         differential_anactl;
    uint8_t         mask;
} adcs_diff[] = {
    { P18, P25, 1, 0x01 },
    { P23, P11, 2, 0x04 },
    { P14, P24, 3, 0x10 },
    { P20, P15, 4, 0x40 }
};
#endif

#if (SDK_VER_CHIP == __DEF_CHIP_QFN32__)
const unsigned int adc_Lambda[] =
    {
        4519602, // P11
        4308639, // P23
        4263287, // P24
        4482718, // P14
        4180401, // P15
        4000000, // Unknown
        4000000, // Unknown
        4000000, // Unknown P18
        4000000, // Unknown P25
        4072069, // P20
};

#elif (SDK_VER_CHIP == __DEF_CHIP_TSOP16__)
const unsigned int adc_Lambda[] =
    {
        4488156, // P11
        4308639, // P23,
        4263287, // P24,
        4467981, // P14
        4142931, // P15
        0,
        0,
        0, // P18
        0, // P25
        4054721, // P20
};
#endif

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @addtogroup ADC_Private_Functions ADC Private functions
  * @{
  */
static void __attribute__((used)) hal_ADC_IRQHandler(void)
{
    uint32_t status = 0;
    static uint16_t adc_samples_buffer[64];

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        /* clear all IRQs */
        AP_ADCC->intr_clear = 0x3FF;
        AP_ADCC->intr_mask = 0;
        return;
    }

    /* Read the IRQ status */
    status = AP_ADCC->intr_status & 0x3ff;

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        /* Check if there's a bit set on the interrupt status for this particular ADC channel */
        if ((BIT(adcs[i].buffer_index) & status) == 0)
        {
            /* Nope, on to the next one */
            continue;
        }

        if (mAdc_Ctx.channels[i].enabled == 0)
        {
            /* An IRQ happened for a channel that wasn't enabled! */
            AP_ADCC->intr_mask &= ~BIT(adcs[i].buffer_index);
            AP_ADCC->intr_clear = BIT(adcs[i].buffer_index);
            continue;
        }

        /* do this ASAP as the ADC is still running! */
        memcpy(adc_samples_buffer, (uint16_t*)&AP_ADCC->adc_data[adcs[i].buffer_index], sizeof(adc_samples_buffer));

        /* If this channel is not set up as a continuously sampled channel, mask it so it won't cause further IRQs */
        if (mAdc_Ctx.channels[i].continuously_sampled_mode == 0)
        {
            AP_ADCC->intr_mask &= ~BIT(adcs[i].buffer_index);
        }

        /* Call the provided event handler, if any */
        if (mAdc_Ctx.evt_handler)
        {
            mAdc_Ctx.evt_handler(adcs[i].ch, adc_samples_buffer);
        }

        /* Clear the Interrupt flag of this channel */
        AP_ADCC->intr_clear = BIT(adcs[i].buffer_index);
    }
    /* If no other ADCs are set up to cause an interrupt, disable the ADC */
    if ((AP_ADCC->intr_mask & 0x3ff) == 0)
    {
        hal_adc_stop();
    }
}

static void hal_adc_load_calibration_value_from_spif(void)
{
    uint32_t adc_cal = read_reg(SPIF_RSVD1_ADC_CALIBRATE);
    mAdc_Ctx.adc_cal_negtive = (uint16_t)(adc_cal & 0x0fff);
    mAdc_Ctx.adc_cal_postive = (uint16_t)((adc_cal >> 16) & 0x0fff);

    if ((mAdc_Ctx.adc_cal_negtive < 0x733) || (mAdc_Ctx.adc_cal_negtive > 0x8cc) ||
        (mAdc_Ctx.adc_cal_postive < 0x733) || (mAdc_Ctx.adc_cal_postive > 0x8cc))
    {
        mAdc_Ctx.adc_cal_negtive = ADC_CALIBRATION_INVALID;
        mAdc_Ctx.adc_cal_postive = ADC_CALIBRATION_INVALID;
    }
    else
    {
        mAdc_Ctx.adc_cal_sum = mAdc_Ctx.adc_cal_postive + mAdc_Ctx.adc_cal_negtive;
        mAdc_Ctx.adc_cal_diff = mAdc_Ctx.adc_cal_postive - mAdc_Ctx.adc_cal_negtive;
    }
}

static void set_channel_res(uint8_t aio, const uint8_t is_attenuated)
{
    uint8_t atten = is_attenuated ? 0 : 1;
    uint8_t pass = is_attenuated ? 1 : 0;

    /* PMCTL2_1 [15:8] => Attenuation bits for AIO_9, 8, 7, 4, 3, 2, 1, 0 (1 = attenuate 1/4, 0 = no atten)*/
    subWriteReg(&(AON_PMCTL2_1), (aio + 8), (aio + 8), atten);
    /* PMCTL2_1 [7:0] => Pass control bits for AIO_9, 8, 7, 4, 3, 2, 1, 0 (1 = connect, 0 = disconnect) */
    subWriteReg(&(AON_PMCTL2_1), aio, aio, pass);
}

static void clear_adcc_cfg(void)
{
    mAdc_Ctx.state = ADC_CTX_NOT_INITALIZED;
    mAdc_Ctx.irq_bits = 0x0000;
    mAdc_Ctx.adc_cal_postive = ADC_CALIBRATION_INVALID;
    mAdc_Ctx.adc_cal_negtive = ADC_CALIBRATION_INVALID;
    memset(&mAdc_Ctx.channels, 0, sizeof(mAdc_Ctx.channels));
    mAdc_Ctx.evt_handler = NULL;
}

static void adc_wakeup_hdl(void)
{
    NVIC_SetPriority((IRQn_Type)ADCC_IRQn, IRQ_PRIO_HAL);
}

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @defgroup ADC_Exported_Functions ADC related exported functions
  * @{
  */
int hal_adc_init(void)
{
    if (mAdc_Ctx.state == ADC_CTX_INITIALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    hal_adc_stop();

    /* Setup the ADC IRQ handler NOW */
    JUMP_FUNCTION(ADCC_IRQ_HANDLER) = (uint32_t)&hal_ADC_IRQHandler;

    hal_pwrmgr_register(MOD_ADCC, NULL, adc_wakeup_hdl);

    clear_adcc_cfg();

    /* Set default priority */
    NVIC_SetPriority((IRQn_Type)ADCC_IRQn, IRQ_PRIO_HAL);

    hal_adc_load_calibration_value_from_spif();

    /* Set all ADC channels to be switched off (attn = 0, pass = 0 for each bit) */
    AON_PMCTL2_1 = 0x00;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    /* CLK_1P28M_ENABLE */
    PCRM_CLKSEL |= PCRM_CLKSEL_1P28M;

    // ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    PCRM_CLKHF_CTL0 |= PCRM_CLKHF_CTL0_XTALOUT;

    // ENABLE_DLL;                  //enable DLL
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_DLL;

    // ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADCDBL_Msk; // check

    // ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_ADC;

    // set adc mode,1:mannual,0:auto mode
    PCRM_ADCCTL4 |= PCRM_ADCCTL4_MODE_MANUAL;
    PCRM_ADCCTL4 |= BIT(0); /* TBD */

    /* Channel enable bit set to 0 on all channels */
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_PCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_PCHEN;

    /* Disable Mic Bias */
    PCRM_ANACTL &= ~PCRM_ANACTL_MICBIAS;

    /* Re-enable the clock of the ADCC module */
    hal_clk_gate_enable(MOD_ADCC);

    mAdc_Ctx.state = ADC_CTX_INITIALIZED;

    return PPlus_SUCCESS;
}

int hal_adc_deinit(void)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    AON_PMCTL2_1 = 0x00;

    hal_adc_stop();

    /* CLK_1P28M Disabled */
    PCRM_CLKSEL &= ~PCRM_CLKSEL_1P28M;

    /* enable xtal 16M output, generate the 32M DLL clock */
    PCRM_CLKHF_CTL0 &= ~PCRM_CLKHF_CTL0_XTALOUT;

    /* Disable 32M DLL */
    PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_DLL;

    if (g_system_clk != SYS_CLK_DBL_32M)
    {
        PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADC;
    }

    hal_clk_reset(MOD_ADCC);
    hal_clk_gate_disable(MOD_ADCC);

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if (mAdc_Ctx.channels[i].enabled == 0)
        {
            continue;
        }

        hal_gpio_cfg_analog_io(adcs[i].single_ended_pin, Bit_DISABLE);
    }

    clear_adcc_cfg();

    hal_pwrmgr_unlock(MOD_ADCC);

    return PPlus_SUCCESS;
}

int hal_adc_configure_channel(adc_channels_t channel, const adc_Cfg_t cfg)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((channel & BIT(i)) == 0)
        {
            continue;
        }

        set_channel_res(adcs[i].pmctl2_1_bit, cfg.attenuated);

        if (cfg.enabled)
        {
            hal_gpio_ds_control(adcs[i].single_ended_pin, Bit_ENABLE);
            hal_gpio_cfg_analog_io(adcs[i].single_ended_pin, Bit_ENABLE);
        }
        else
        {
            hal_gpio_ds_control(adcs[i].single_ended_pin, Bit_DISABLE);
            hal_gpio_cfg_analog_io(adcs[i].single_ended_pin, Bit_DISABLE);
        }

        /* Sample time is only 4 bits, so mask it (and make it uint32_t so it can be left shifted for the "N" register bits) */
        uint32_t sample_time = cfg.sample_time & 0xF;

        if (adcs[i].adc_ctl_is_n)
        {
            if (cfg.enabled)
            {
                *adcs[i].adc_ctl |= PCRM_ADCCTL_NCHEN;  /* Channel Enabled */
            }
            else
            {
                *adcs[i].adc_ctl &= ~PCRM_ADCCTL_NCHEN; /* Channel Disabled */

            }
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_NDIFF;     /* Channel is single ended (NOT differential) */
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_NONE;      /* Channel set to continuous mode (NOT one shot, because this flag doesn't work!) */
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_NSAMP_Msk; /* Channel sampling time 0? whatever TODO!!!: investigate! */
            *adcs[i].adc_ctl |= sample_time << PCRM_ADCCTL_NSAMP_Pos;
        }
        else
        {
            if (cfg.enabled)
            {
                *adcs[i].adc_ctl |= PCRM_ADCCTL_PCHEN;  /* Channel Enabled */
            }
            else
            {
                *adcs[i].adc_ctl &= ~PCRM_ADCCTL_PCHEN; /* Channel Disabled */

            }
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_PDIFF;     /* Channel is single ended (NOT differential) */
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_PONE;      /* Channel set to continuous mode (NOT one shot, because this flag doesn't work!) */
            *adcs[i].adc_ctl &= ~PCRM_ADCCTL_PSAMP_Msk; /* Channel sampling time 0? whatever TODO!!!: investigate! */
            *adcs[i].adc_ctl |= sample_time << PCRM_ADCCTL_PSAMP_Pos;
        }

        mAdc_Ctx.channels[i] = cfg;

        if (cfg.enabled)
        {

            mAdc_Ctx.irq_bits |= BIT(adcs[i].buffer_index);
        }
        else
        {
            mAdc_Ctx.irq_bits &= ~BIT(adcs[i].buffer_index);
        }

        return PPlus_SUCCESS;
    }

    return PPlus_ERR_INVALID_PARAM;
}

void hal_adc_pga_set_gain(adc_gain_first_t first, adc_gain_second_t second)
{
    if (first == ADC_GAIN_15)
    {
        PCRM_ANACTL |= PCRM_ANACTL_PGA_1ST;
    }
    else
    {
        PCRM_ANACTL &= ~PCRM_ANACTL_PGA_1ST;
    }

    PCRM_ANACTL &= PCRM_ANACTL_PGA_2ND_Msk;

    uint32_t val = (uint32_t)second;
    PCRM_ANACTL |= val << PCRM_ANACTL_PGA_2ND_Pos;
}

int hal_adc_clock_config(adc_CLOCK_SEL_t clk)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    subWriteReg(&(PCRM_ADCCTL4), 2, PCRM_ADCCTL4_SEL_Pos, clk);

    return PPlus_SUCCESS;
}

int hal_adc_start(uint8_t single_ended, uint8_t auto_mode, adc_Hdl_t evt_handler)
{
    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_INVALID_STATE;
    }

    hal_pwrmgr_lock(MOD_ADCC);

    if (single_ended)
    {
        /* single ended mode */
        PCRM_ANACTL |= PCRM_ANACTL_DIFF1;
        PCRM_ANACTL |= PCRM_ANACTL_DIFF2;
    }

    if (auto_mode)
    {
        /* Enable auto mode (i.e. continuous) */
        PCRM_ADCCTL4 &= ~PCRM_ADCCTL4_MODE_MANUAL;
    }

    mAdc_Ctx.evt_handler = evt_handler;

    /* Just in case, clear pending IRQs */
    AP_ADCC->intr_clear = 0x1FF;

    /* ADC Enabled */
    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    /* Analog LDO on */
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO;

    AP_ADCC->intr_mask = mAdc_Ctx.irq_bits; // ENABLE_ADC_INT;

    /* Enable IRQs from the ADC */
    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;

    return PPlus_SUCCESS;
}

int hal_adc_stop(void)
{
    /* Disable IRQs from the ADC */
    NVIC_DisableIRQ((IRQn_Type)ADCC_IRQn);

    AP_ADCC->intr_clear = 0x1FF;

    /* ADC Disabled */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Analog LDO off */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;

    return PPlus_SUCCESS;
}

float hal_adc_value_cal(adc_channels_t channel, const uint16_t *buf, uint32_t size)
{
    uint32_t i;
    int adc_sum = 0;
    volatile float result = 0.0;
    uint8_t is_neg = 0;
    uint8_t attenuated = 0;
    uint8_t diff_mode = 0;
    uint16_t adc_cal_postive = mAdc_Ctx.adc_cal_postive;
    uint16_t adc_cal_negtive = mAdc_Ctx.adc_cal_negtive;
    float lambda = 0;

    /* average */
    for (i = 0; i < size; i++)
    {
        adc_sum += (buf[i] & 0xfff);
    }
    result = ((float)adc_sum) / size;

    for (size_t i = 0; i < sizeof(adcs)/sizeof(adcs[0]); i++)
    {
        if ((channel & BIT(i)) == 0)
        {
            continue;
        }

        is_neg = adcs[i].adc_ctl_is_n;
        lambda = (float)adc_Lambda[i];
        attenuated = mAdc_Ctx.channels[i].attenuated;
        diff_mode = 0;
    }

    if ((adc_cal_postive != 0xfff) && (adc_cal_negtive != 0xfff))
    {
        float delta = (mAdc_Ctx.adc_cal_diff) / 2.0;

        if (is_neg)
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / mAdc_Ctx.adc_cal_sum)
                                 : ((result + delta) / mAdc_Ctx.adc_cal_sum);
        }
        else
        {
            result = (diff_mode) ? ((result - 2048 - delta) * 2 / mAdc_Ctx.adc_cal_sum)
                                 : ((result - delta) / mAdc_Ctx.adc_cal_sum);
        }
    }
    else
    {
        result = (diff_mode) ? (float)(result / 2048 - 1) : (float)(result / 4096);
    }

    if (attenuated == 1)
    {
        result *= 800.0;
    }
    else
    {
        result = (float)result * lambda * 0.8 / 1000;
    }
    return result;
}

#if 0

static void set_sampling_resolution(adc_CH_t channel, uint8_t is_attenuated_resolution, uint8_t is_differential_mode)
{
    for (size_t i = 0; i < (sizeof(adcs)/sizeof(adcs[0])); i++)
    {
        if (adcs[i].hal_adc == channel)
        {
            set_channel_res(adcs[i].pmctl2_1_bit, is_attenuated_resolution);
            set_channel_res(adcs[i].aio_differential_pair, is_attenuated_resolution);
            return;
        }
    }
}

static void set_sampling_resolution_auto(uint8_t channel, uint8_t is_attenuated_resolution, uint8_t is_differential_mode)
{
    uint8_t i_channel;
    adc_CH_t a_channel;
    AON_PMCTL2_1 = 0x00;

    for (i_channel = MIN_ADC_CH; i_channel <= MAX_ADC_CH; i_channel++)
    {
        if (channel & BIT(i_channel))
        {
            a_channel = (adc_CH_t)i_channel;
            set_sampling_resolution(a_channel,
                                    (is_attenuated_resolution & BIT(i_channel)),
                                    (is_differential_mode & BIT(i_channel)));
        }
    }
}

int hal_adc_start1(void)
{
    uint8_t irq_bits2 = /*(((mAdc_Ctx.chs_en_shadow & 0x80) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x40) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x20) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x10) << 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x08) >> 1) |
                            ((mAdc_Ctx.chs_en_shadow & 0x04) << 1));*/ 0;

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    // LOG("irq_bits2:0x%x",irq_bits2);
    hal_pwrmgr_lock(MOD_ADCC);

    for (int i = MIN_ADC_CH; i <= MAX_ADC_CH; i++)
    {
        if (irq_bits2 & (BIT(i)))
        {
            switch (i)
            {
            case ADC_CH1N_P11:
                PCRM_ADCCTL1 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH1P_P23:
                PCRM_ADCCTL1 |= PCRM_ADCCTL_PCHEN;
                break;

            case ADC_CH2N_P24:
                PCRM_ADCCTL2 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH2P_P14:
                PCRM_ADCCTL2 |= PCRM_ADCCTL_PCHEN;
                break;

            case ADC_CH3N_P15:
                PCRM_ADCCTL3 |= PCRM_ADCCTL_NCHEN;
                break;

            case ADC_CH3P_P20:
                PCRM_ADCCTL3 |= PCRM_ADCCTL_PCHEN;
                break;
            }
        }
    }

    PCRM_ANACTL |= PCRM_ANACTL_ADCEN; // ENABLE_ADC;
    PCRM_ANACTL |= PCRM_ANACTL_ADLDO; // new

    NVIC_EnableIRQ((IRQn_Type)ADCC_IRQn);      // ADC_IRQ_ENABLE;
    AP_ADCC->intr_mask = mAdc_Ctx.irq_bits; // ENABLE_ADC_INT;

    // disableSleep();
    return PPlus_SUCCESS;
}

int hal_adc_config_channel(adc_Cfg_t cfg, adc_Hdl_t evt_handler)
{
    uint8_t i;
    uint8_t chn_sel = 0;
    gpio_pin_e pin, pin_neg;

    if (mAdc_Ctx.state == ADC_CTX_NOT_INITALIZED)
    {
        return PPlus_ERR_NOT_REGISTED;
    }

    if (evt_handler == NULL)
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if ((cfg.channel & BIT(0)) || (cfg.channel & BIT(1)))
    {
        return PPlus_ERR_NOT_SUPPORTED;
    }

    if ((!cfg.channel & BIT(1)) && (cfg.is_differential_mode && (cfg.channel & BIT(1))))
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if (cfg.is_differential_mode != 0)
    {
        if ((cfg.is_differential_mode != 0x80) && (cfg.is_differential_mode != 0x20) && (cfg.is_differential_mode != 0x08))
        {
            return PPlus_ERR_INVALID_PARAM;
        }
    }

    clear_adcc_cfg();

    /* Set all ADC channels to be switched off (attn = 0, pass = 0 for each bit) */
    AON_PMCTL2_1 = 0x00;

    /* Analog LDO off */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADLDO;

    /* ADC Disabled */
    PCRM_ANACTL &= ~PCRM_ANACTL_ADCEN;

    /* Disable the clock of the ADCC module */
    hal_clk_gate_disable(MOD_ADCC);
    hal_clk_reset(MOD_ADCC);

    mAdc_Ctx.continue_mode = cfg.is_continue_mode;
    mAdc_Ctx.irq_bits = cfg.channel & 0x03;

    for (i = 2; i < 8; i++)
    {
        if (cfg.channel & BIT(i))
        {
            if (i % 2)
            {
                /* 3=>2, 5=>4, 7=>6*/
                mAdc_Ctx.irq_bits |= BIT(i - 1);
            }
            else
            {
                /* 2=>3, 4=>5, 6=>7 */
                mAdc_Ctx.irq_bits |= BIT(i + 1);
            }
        }
    }
    ////////mAdc_Ctx.chs_en_shadow = mAdc_Ctx.irq_bits;
    // LOG("cfg.channel:0x%x",cfg.channel);
    /// LOG("mAdc_Ctx.irq_bits:0x%x", mAdc_Ctx.irq_bits);

    /* Re-enable the clock of the ADCC module */
    hal_clk_gate_enable(MOD_ADCC);

    /* CLK_1P28M_ENABLE; this might be used only for the DMIC? */
    PCRM_CLKSEL |= PCRM_CLKSEL_1P28M;

    // ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
    PCRM_CLKHF_CTL0 |= PCRM_CLKHF_CTL0_XTALOUT;

    // ENABLE_DLL;                  //enable DLL
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_DLL;
    // ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
    PCRM_CLKHF_CTL1 &= ~PCRM_CLKHF_CTL1_ADCDBL_Msk; // check
    // subWriteReg(0x4000F044,21,20,3);
    // ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
    PCRM_CLKHF_CTL1 |= PCRM_CLKHF_CTL1_ADC;

    // subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
    PCRM_ADCCTL4 |= PCRM_ADCCTL4_MODE_MANUAL;
    PCRM_ADCCTL4 |= BIT(0);

    set_sampling_resolution_auto(cfg.channel, cfg.is_attenuated_resolution, cfg.is_differential_mode);

    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL0 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL1 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL2 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_NCHEN;
    PCRM_ADCCTL3 &= ~PCRM_ADCCTL_PCHEN;

    PCRM_ANACTL &= ~PCRM_ANACTL_MICBIAS; // disable micbias

    mAdc_Ctx.evt_handler = evt_handler;

    if (cfg.is_differential_mode == 0)
    {
        PCRM_ANACTL |= PCRM_ANACTL_DIFF1;
        PCRM_ANACTL |= PCRM_ANACTL_DIFF2;

        PCRM_ADCCTL4 &= ~PCRM_ADCCTL4_MODE_Msk; // enable auto mode
        for (i = MIN_ADC_CH; i <= MAX_ADC_CH; i++)
        {
            if (cfg.channel & BIT(i))
            {
                gpio_pin_e pin = s_pinmap[i];
                hal_gpio_ds_control(pin, Bit_ENABLE);
                hal_gpio_cfg_analog_io(pin, Bit_ENABLE);
            }
        }
    }
    else
    {
        switch (cfg.is_differential_mode)
        {
        case 0x80:
            pin = P20;
            pin_neg = P15;
            chn_sel = 0x04;
            break;

        case 0x20:
            pin = P14;
            pin_neg = P24;
            chn_sel = 0x03;
            break;

        case 0x08:
            pin = P23;
            pin_neg = P11;
            chn_sel = 0x02;
            break;

        case 0x02:
            pin = P18;
            pin_neg = P25;
            chn_sel = 0x01;

            AON_PMCTL2_1 = 0x0060; /* AIO_7 and AIO_8 passthrough enable */
            break;

        default:
            break;
        }

        hal_gpio_ds_control(pin, Bit_ENABLE);
        subWriteReg(&(PCRM_ANACTL), 7, 5, chn_sel);

        PCRM_ANACTL &= ~PCRM_ANACTL_DIFF1;
        PCRM_ANACTL &= ~PCRM_ANACTL_DIFF2;

    // LOG("%d %d %x",pin,pin_neg,*(volatile int*)0x40003800);
        hal_gpio_cfg_analog_io(pin, Bit_ENABLE);
        hal_gpio_cfg_analog_io(pin_neg, Bit_ENABLE);
        // LOG("%d %d %x",pin,pin_neg,*(volatile int*)0x40003800);
        mAdc_Ctx.irq_bits = (cfg.is_differential_mode >> 1);
    }

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
