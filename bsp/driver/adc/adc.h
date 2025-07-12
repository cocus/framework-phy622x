/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_ADC_H
#define _HAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup ADC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup ADC_Exported_Types ADC Exported Types
  * @{
  */

/**
  * @brief  PHY62xx old style HAL channel selection
  */
typedef enum
{
  ADC_CH0DIFF = 1,/*p18(positive),p25(negative),only works in diff*/
  ADC_CH0 = 2,ADC_CH1N_P11 = 2,MIN_ADC_CH = 2,
  ADC_CH1 = 3,ADC_CH1P_P23 = 3,ADC_CH1DIFF = 3,/*P23 and P11*/
  ADC_CH2 = 4,ADC_CH2N_P24 = 4,
  ADC_CH3 = 5,ADC_CH2P_P14 = 5,ADC_CH2DIFF = 5,/*P14 and P24*/
  ADC_CH4 = 6,ADC_CH3N_P15 = 6,
  ADC_CH9 = 7,ADC_CH3P_P20 = 7,MAX_ADC_CH = 7,ADC_CH3DIFF = 7,/*P20 and P15*/
  ADC_CH_VOICE = 8,
  ADC_CH_NUM =9,
} adc_CH_t;

/**
  * @brief  PHY62xx ADC Channel indices.
  */
typedef enum
{
  CH0 = 1,    /*!< AIO_0, P11, CH1N (Input B-) */
  CH1 = 2,    /*!< AIO_1, P23, CH1P (Input B+), Mic Bias reference voltage */
  CH2 = 4,    /*!< AIO_2, P24, CH2N (Input C-) */
  CH3 = 8,    /*!< AIO_3, P14, CH2P (Input C+) */
  CH4 = 16,   /*!< AIO_4, P15, CH3N (Input D-) */
  CH5 = 32,   /*!< AIO_5, P16, 32k XTAL Input */
  CH6 = 64,   /*!< AIO_6, P17, 32k XTAL Output */
  CH7 = 128,  /*!< AIO_7, P18, CH0P (Input A+), PGA Pos */
  CH8 = 256,  /*!< AIO_8, P25, CH0N (Input A-) */
  CH9 = 512   /*!< AIO_9, P20, CH3P (Input D+), PGA Neg */
} adc_channels_t;

typedef enum
{
  ADC_GAIN_5 = 0,   /*!< 5V/V gain */
  ADC_GAIN_15 = 1,  /*!< 15V/V gain */
} adc_gain_first_t;

typedef enum
{
  ADC_GAIN_37_4 = 0,
  ADC_GAIN_36_5 = 1,
  ADC_GAIN_35_6 = 2,
  ADC_GAIN_34_7 = 3,
  ADC_GAIN_33_8 = 4,
  ADC_GAIN_32_9 = 5,
  ADC_GAIN_31_10 = 6,
  ADC_GAIN_30_11 = 7,
} adc_gain_second_t;

/**
  * @brief  PHY62xx ADC Sample clock selection.
  */
typedef enum
{
  HAL_ADC_CLOCK_80K = 0,  /*!< 80kHz */
  HAL_ADC_CLOCK_160K = 1, /*!< 160kHz */
  HAL_ADC_CLOCK_320K = 2, /*!< 320kHz */
} adc_CLOCK_SEL_t;

/**
  * @brief  PHY62xx ADC channel configuration structure.
  */
typedef struct
{
  uint8_t  enabled;                   /*!< 1 = enables the channel, 0 = disables the channel */
  uint8_t  continuously_sampled_mode; /*!< 1 = continuously sample the channel, 0 = sample the channel only once */
  //uint8_t  differential;          /*!< */
  uint8_t  attenuated;                /*!< 1 = attenuates the input to 1/4 of the input, 0 = directly sample the input */
  uint8_t  sample_time;               /*!< How much time takes a single sample, in 1/ADC_clock units */
} adc_Cfg_t;

/**
  * @brief  PHY62xx ADC IRQ event callback.
  */
typedef void (*adc_Hdl_t)(const adc_channels_t ch, const uint16_t* data);

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup ADC_Exported_Constants ADC Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup ADC_Exported_Macros ADC Exported Macros
  * @{
  */

#define     MAX_ADC_SAMPLE_SIZE     32
#define     NUM_ADCS                10

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_Exported_Functions
  * @{
  */
    /**
      * @brief  Initializes the ADC module of the MCU.
      * @note   This effectively enables the ADC module, registers a handler for the ADC IRQ,
      *         enables te 1.28MHz clock for the ADC, and disables the MIC Bias.
      * @retval PPlus_SUCCESS if the ADC was initialized or PPlus_ERR_INVALID_STATE if it was
      *         previously initialized.
      */
    int hal_adc_init(void);

    /**
      * @brief  Disables the clock to a specified module inside the MCU.
      * @note   This effectively disables the ADC, disables the clock for the ADC.
      * @retval PPlus_SUCCESS if the ADC was previously initialized or PPlus_ERR_INVALID_STATE if not.
      */
    int hal_adc_deinit(void);

    /**
      * @brief  Configures a channel of the ADC.
      * @param  channel: One of the adc_channels_t to configure.
      * @param  cfg: Channel configuration values stored on a adc_Cfg_t struct.
      * @note   Also configures the pin as analog if the channel is enabled. See the
      *         adc_Cfg_t struct to figure out which parameters can be configured.
      * @retval PPlus_SUCCESS if the ADC was previously initialized, PPlus_ERR_INVALID_STATE if not,
      *         or PPlus_ERR_INVALID_PARAM if the wrong channel is specified.
      */
    int hal_adc_configure_channel(adc_channels_t channel, const adc_Cfg_t cfg);

    /**
      * @brief  Sets the PGA gains of the first and second stages.
      * @param  first: One of the adc_gain_first_t values (either 5v/v or 15v/v).
      * @param  second: One of the adc_gain_second_t values.
      * @retval None.
      */
    void hal_adc_pga_set_gain(adc_gain_first_t first, adc_gain_second_t second);

    /**
      * @brief  Selects the clock source for the ADC.
      * @param  clk: One of the adc_CLOCK_SEL_t, 80kHz, 160kHz or 320kHz.
      * @retval PPlus_SUCCESS if the ADC was previously initialized or PPlus_ERR_INVALID_STATE if not.
      */
    int hal_adc_clock_config(adc_CLOCK_SEL_t clk);

    /**
      * @brief  Starts the ADC with the previously configured channels.
      * @param  single_ended: Set to 1 to sample single-ended channels only.
      * @param  auto_mode: Set to 1 to use the automatic mode.
      * @param  evt_handler: Callback function to call once the ADC completes sampling a channel.
      * @retval PPlus_SUCCESS if the ADC was previously initialized or PPlus_ERR_INVALID_STATE if not.
      */
    int hal_adc_start(uint8_t single_ended, uint8_t auto_mode, adc_Hdl_t evt_handler);

    /**
      * @brief  Stops the ADC.
      * @note   Only disables the ADC IRQ, the ADCEN flag and the Analog LDO.
      * @retval PPlus_SUCCESS always.
      */
    int hal_adc_stop(void);

    /**
      * @brief  Gets the calibrated value out of an ADC samples buffer.
      * @param  channel: One of the adc_channels_t to configure.
      * @param  buf: Samples buffer.
      * @param  size: Number of samples.
      * @retval A float value of the averaged analog value of the previously sampled buffer.
      */
    float hal_adc_value_cal(adc_channels_t channel, const uint16_t* buf, uint32_t size);


/*
    ADC note:
    There are ten pins which can config as analogy,there are some differences between them.
    hardware analogy index:
    gpio<11>/aio<0>
    gpio<23>/aio<1>/micphone bias reference voltage
    gpio<24>/aio<2>
    gpio<14>/aio<3>
    gpio<15>/aio<4>/micphone bias
    gpio<16>/aio<5>/32K XTAL input
    gpio<17>/aio<6>/32K XTAL output
    gpio<18>/aio<7>/pga in+
    gpio<25>/aio<8>
    gpio<20>/aio<9>/pga in-

    There are six pins which can work in adc single mode.Such as:
    ADC_CH0 = 2,ADC_CH1N_P11 = 2,
    ADC_CH1 = 3,ADC_CH1P_P23 = 3,
    ADC_CH2 = 4,ADC_CH2N_P24 = 4,
    ADC_CH3 = 5,ADC_CH2P_P14 = 5,
    ADC_CH4 = 6,ADC_CH3N_P15 = 6,
    ADC_CH9 = 7,ADC_CH3P_P20 = 7,

    There are four pair pins which can work in adc diff mode.Such as:
    ADC_CH0DIFF = 1,p18(p) and P25(n)
    ADC_CH1DIFF = 3,P23(p) and P11(n)
    ADC_CH2DIFF = 5,P14(p) and P24(n)
    ADC_CH3DIFF = 7,P20(p) and P15(n)

    There are two pins which uses with 32.768K crystal oscillator.
    gpio<16>/aio<5>/32K XTAL input
    gpio<17>/aio<6>/32K XTAL output

    There are four pins which uses as pga,voice and so on.
    gpio<23>/aio<1>/micphone bias reference voltage,this pin is selected
    gpio<15>/aio<4>/micphone bias
    gpio<18>/aio<7>/pga in+
    gpio<20>/aio<9>/pga in-
*/


/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup ADC_Exported_ROM_Functions ADC Exported ROM Functions
  * @{
  */

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup ADC_Exported_ROM_Variables ADC Exported ROM Variables
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

#endif /* _HAL_ADC_H */
