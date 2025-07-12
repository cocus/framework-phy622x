/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_SPI_H
#define _HAL_SPI_H

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

/** @addtogroup SPI
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief  PHY62xx SPI mode.
  */
typedef enum
{
  SPI_MODE0 = 0,          /*!< SPI Mode 0, SCPOL=0, SCPH=0 */
  SPI_MODE1 = 1,          /*!< SPI Mode 1, SCPOL=0, SCPH=1 */
  SPI_MODE2 = 2,          /*!< SPI Mode 2, SCPOL=1, SCPH=0 */
  SPI_MODE3 = 3,          /*!< SPI Mode 3, SCPOL=1, SCPH=1 */
} SPI_SCMOD_e;

/**
  * @brief  PHY62xx SPI bits per transfer.
  */
typedef enum
{
  SPI_4BIT = 0x03,        /*!< 4 bits (1 byte) per transfer */
  SPI_5BIT = 0x04,        /*!< 5 bits (1 byte) per transfer */
  SPI_6BIT = 0x05,        /*!< 6 bits (1 byte) per transfer */
  SPI_7BIT = 0x06,        /*!< 7 bits (1 byte) per transfer */
  SPI_8BIT = 0x07,        /*!< 8 bits (1 byte) per transfer */
  SPI_1BYTE = 0x07,       /*!< 8 bits (1 byte) per transfer */
  SPI_9BIT = 0x08,        /*!< 9 bits (2 byte) per transfer */
  SPI_10BIT = 0x09,       /*!< 10 bits (2 bytes) per transfer */
  SPI_11BIT = 0x0a,       /*!< 11 bits (2 bytes) per transfer */
  SPI_12BIT = 0x0b,       /*!< 12 bits (2 bytes) per transfer */
  SPI_13BIT = 0x0c,       /*!< 13 bits (2 bytes) per transfer */
  SPI_14BIT = 0x0d,       /*!< 14 bits (2 bytes) per transfer */
  SPI_15BIT = 0x0e,       /*!< 15 bits (2 bytes) per transfer */
  SPI_16BIT = 0x0f,       /*!< 16 bits (2 bytes) per transfer */
  SPI_2BYTE = 0x0f,       /*!< 16 bits (2 bytes) per transfer */
} SPI_DFS_e;

/**
  * @brief  PHY62xx SPI modules.
  */
typedef enum
{
  SPI0 = 0,               /*!< SPI module 0 */
  SPI1 = 1,               /*!< SPI module 1 */
} SPI_INDEX_e;

/**
  * @brief  PHY62xx SPI event types used in callbacks.
  */
typedef enum
{
  SPI_TX_COMPLETED = 1,   /*!< Transmission finished */
  SPI_RX_COMPLETED,       /*!< Reception finished */
  SPI_TX_REQ_S,           /*!< Slave mode, TX */
  SPI_RX_DATA_S,          /*!< Slave mode, RX */
} SPI_EVT_e;

/**
  * @brief  PHY62xx SPI event structure passed to the callback function.
  */
typedef struct
{
  SPI_INDEX_e id;         /*!< The SPI module that triggered this event */
  SPI_EVT_e evt;          /*!< Event type */
  uint8_t *data;          /*!< Pointer to data related to the event */
  size_t len;             /*!< Length of data related to the event */
} spi_evt_t;

typedef void (*spi_hdl_t)(spi_evt_t *pevt);

/**
  * @brief  PHY62xx Forces the usage of the Chip Select pin as a GPIO rather than a Mux function of the SPIx.
  */
typedef enum
{
  SPI_FORCE_CS_DISABLED = 0U,     /*!< Connect the CS pin to the SPIx's SSN signal */
  SPI_FORCE_CS_ENABLED = 1U,      /*!< Lets the driver set the CS pin's value manually */
} spi_cfg_force_cs_t;

/**
  * @brief  PHY62xx SPI configuration strucutre used to set up a SPI module.
  */
typedef struct
{
  gpio_pin_e sclk_pin;            /*!< Clock Pin */
  gpio_pin_e ssn_pin;             /*!< Slave Chip Select Pin */
  gpio_pin_e MOSI;                /*!< Master Output, Slave Input Pin */
  gpio_pin_e MISO;                /*!< Master Input, Slave Output Pin */
  uint32_t frequency;             /*!< Frequency for the bus, in Hz */
  SPI_SCMOD_e spi_scmod;          /*!< SPI Mode, one of SPI_SCMOD_e */
  SPI_DFS_e spi_dfsmod;           /*!< Bits per transfer, one of SPI_DFS_e */
#ifdef DMAC_USE
  uint8_t dma_tx_enable;          /*!< Enable DMA for transmit */
  uint8_t dma_rx_enable;          /*!< Enable DMA for receive */
#endif
  spi_cfg_force_cs_t force_cs;    /*!< Forces the use of the Chip Select, one of spi_cfg_force_cs_t */
  spi_hdl_t evt_handler;          /*!< User-provided callback for interrupt events */
} spi_Cfg_t;

/**
  * @brief  PHY62xx SPI module status.
  */
typedef enum
{
  SPI_XMIT_INVALID = 0U,          /*!< SPIx is not configured */
  SPI_XMIT_IDLE = 1U,             /*!< SPIx is Idle */
  SPI_XMIT_BUSY = 2U,             /*!< SPIx is Busy */
} spi_xmit_state_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions
  * @{
  */
    /**
      * @brief  Initializes the state of the SPI driver.
      * @note   Initializes the IRQ handlers for both SPI modules, sets the IRQ priority, and
      *         zeroes out the context of the SPI driver.
      * @retval PPlus_SUCCESS if succees, others on failure.
      */
    int hal_spi_init(void);

    /**
      * @brief  Initializes a SPI module with its bus pins and parameters.
      * @param  spi: The SPI module to initialize, either SPI0 or SPI1.
      *         cfg: Configuration structure with the settings for the specified module.
      * @note   Enables the clock gating for the SPIx, sets up the specified pins as a mux of the
      *         SPIx signals, configures the frequency of the module, and sets it in master mode.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         or PPlus_ERR_BUSY if already initialized.
      */
    int hal_spi_bus_init(SPI_INDEX_e spi, spi_Cfg_t cfg);

    /**
      * @brief  De-initializes a SPI module.
      * @param  spi: The SPI module to de-initialize, either SPI0 or SPI1.
      * @note   Disables the clock gating for the SPIx, return all the pins to their default state,
      *         and disables the inerrupts from this SPIx.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         or PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously.
      */
    int hal_spi_bus_deinit(SPI_INDEX_e spi);

    /**
      * @brief  Initiates a SPI transaction, transmit only, and waits for it to finish for a given time.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  tx_len: The number of "transmissions" to send. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @param  timeout: Ticks to wait to complete the transmission of the buffer. The unit is the same
      *         as the OSAL timer, which is 625uS per tick.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, PPlus_ERR_TIMEOUT if
      *         a timeout occurs while sending the data, or PPlus_ERR_BUSY if the SPIx module is busy.
      */
    int hal_spi_transmit(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint16_t tx_len,
        uint32_t timeout);

    /**
      * @brief  Initiates a SPI transaction, transmit only, sending the same pattern, and waits for it to finish for a given time.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_pattern: A pattern of bits to send repeatedly. Must be the same size specified on the bits per
      *         transfer.
      * @param  tx_words: Indicates how many transmissions should occur. Note that each transmission will send
      *         the pre-defined bits per transfer. For instance, if it's set to 8 bits or less, this parameter
      *         matches the number of bytes to send.
      * @param  timeout: Ticks to wait to complete the transmission of the buffer. The unit is the same
      *         as the OSAL timer, which is 625uS per tick.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, PPlus_ERR_TIMEOUT if
      *         a timeout occurs while sending the data, or PPlus_ERR_BUSY if the SPIx module is busy.
      */
    int hal_spi_transmit_same(
        SPI_INDEX_e spi,
        uint16_t tx_pattern,
        uint16_t tx_words,
        uint32_t timeout);

    /**
      * @brief  Initiates a SPI transaction, transmitting and receiving, and waits for it to finish for a given time.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  rx_buf: A pointer to a buffer where the received data will be stored.
      * @param  tx_len: The number of "transmissions" to send. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @param  rx_len: Same as tx_len but for the received data.
      * @param  timeout: Ticks to wait to complete the transmission of the buffer. The unit is the same
      *         as the OSAL timer, which is 625uS per tick.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. It's not clear why there's a tx_len and rx_len.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, PPlus_ERR_TIMEOUT if
      *         a timeout occurs while sending the data, or PPlus_ERR_BUSY if the SPIx module is busy.
      */
    int hal_spi_transmit_receive(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint8_t *rx_buf,
        uint16_t tx_len,
        uint16_t rx_len,
        uint32_t timeout);

    /**
      * @brief  Initiates a SPI transaction, transmitting and receiving from the internal? EEPROM, and
      *         waits for it to finish for a given time.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  rx_buf: A pointer to a buffer where the received data will be stored.
      * @param  tx_len: The number of "transmissions" to send. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @param  rx_len: Same as tx_len but for the received data.
      * @param  timeout: Ticks to wait to complete the transmission of the buffer. The unit is the same
      *         as the OSAL timer, which is 625uS per tick.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. It's not clear why there's a tx_len and rx_len. It's also unknown what
      *         do they mean with EEPROM.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, PPlus_ERR_TIMEOUT if
      *         a timeout occurs while sending the data, or PPlus_ERR_BUSY if the SPIx module is busy.
      */
    int hal_spi_transmit_receive_eeprom(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint8_t *rx_buf,
        uint16_t tx_len,
        uint16_t rx_len,
        uint32_t timeout);

    /**
      * @brief  Initiates a SPI transaction, transmit only, and calls a callback from an interrupt when done.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  tx_words: The number of "transmissions" to send. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. Returns immediately on success. An interrupt will be triggered when the
      *         transmission is over, calling a user-passed callback with an appropriate event value, if any.
      *         ATTENTION: The tx_buf pointer MUST REMAIN VALID until the transaction is finished!.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, or PPlus_ERR_BUSY if the
      *         SPIx module is busy.
      */
    int hal_spi_transmit_it(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint16_t tx_words);

    /**
      * @brief  Initiates a SPI transaction, transmit only, sending the same pattern, and calls a callback from an interrupt when done.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_pattern: A pattern of bits to send repeatedly. Must be the same size specified on the bits per
      *         transfer.
      * @param  tx_words: Indicates how many transmissions should occur. Note that each transmission will send
      *         the pre-defined bits per transfer. For instance, if it's set to 8 bits or less, this parameter
      *         matches the number of bytes to send.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. Returns immediately on success. An interrupt will be triggered when the
      *         transmission is over, calling a user-passed callback with an appropriate event value, if any.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, or PPlus_ERR_BUSY if the
      *         SPIx module is busy.
      */
    int hal_spi_transmit_same_it(
        SPI_INDEX_e spi,
        uint16_t tx_pattern,
        uint16_t tx_words);

    /**
      * @brief  Initiates a SPI transaction, transmitting and receiving, and calls a callback from an interrupt when done.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  rx_buf: A pointer to a buffer where the received data will be stored.
      * @param  tx_rx_words: The number of "transmissions" to send and receive. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. Returns immediately on success. An interrupt will be triggered when the
      *         transmission is over, calling a user-passed callback with an appropriate event value, if any.
      *         ATTENTION: The tx_buf and rx_buf pointers MUST REMAIN VALID until the transaction is finished!.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, or PPlus_ERR_BUSY if the
      *         SPIx module is busy.
      */
    int hal_spi_transmit_receive_it(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint8_t *rx_buf,
        uint16_t tx_rx_words);

    /**
      * @brief  Initiates a SPI transaction, transmitting and receiving from the internal? EEPROM, and calls
      *         a callback from an interrupt when done.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  tx_buf: A pointer to a buffer containing the data to send.
      * @param  rx_buf: A pointer to a buffer where the received data will be stored.
      * @param  tx_words: The number of "transmissions" to send. If the bits per transfer is less or equal
      *         to a byte, then tx_len is equivalent to number of bytes to send. However, if it's above
      *         a byte (i.e. more than 8 bits), then tx_len is the number of 16-bits transfers to send,
      *         which is half the size of bytes.
      * @param  rx_words: Same as tx_len but for the received data.
      * @note   DMA can be used to speed up this operation. It will use DMA Channel 0, regardless of which
      *         SPIx is selected. Returns immediately on success. An interrupt will be triggered when the
      *         transmission is over, calling a user-passed callback with an appropriate event value, if any.
      *         ATTENTION: The tx_buf and rx_buf pointers MUST REMAIN VALID until the transaction is finished!.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong,
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously, or PPlus_ERR_BUSY if the
      *         SPIx module is busy.
      */
    int hal_spi_transmit_receive_eeprom_it(
        SPI_INDEX_e spi,
        uint8_t *tx_buf,
        uint8_t *rx_buf,
        uint16_t tx_words,
        uint16_t rx_words);

    /**
      * @brief  Sets the bits per transfer of a previously configured SPIx module.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  mod: Bits per transfer, one of SPI_DFS_e.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong, or
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously.
      */
    int hal_spi_dfs_set(SPI_INDEX_e spi, SPI_DFS_e mod);

    /**
      * @brief  Sets the "force CS" of a previously configured SPIx module.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @param  en: Sets if the force CS pin is set or not, one of spi_cfg_force_cs_t.
      * @retval PPlus_SUCCESS if succees, PPlus_ERR_INVALID_PARAM if the spi parameter is wrong, or
      *         PPlus_ERR_NOT_REGISTED if the SPIx was not initialized previously.
      */
    int hal_spi_set_force_cs(SPI_INDEX_e spi, spi_cfg_force_cs_t en);

    /**
      * @brief  Gets the status of a given SPIx module.
      * @param  spi: The SPI module which needs to be previously initialized, either SPI0 or SPI1.
      * @retval One of spi_xmit_state_t, depending on the current status of the SPIx module.
      */
    spi_xmit_state_t hal_spi_get_transmit_bus_state(SPI_INDEX_e spi);

#ifdef USE_SLAVE
int hal_spis_clear_rx(hal_spi_t *spi_ptr);
uint32_t hal_spis_rx_len(hal_spi_t *spi_ptr);
int hal_spis_read_rxn(hal_spi_t *spi_ptr, uint8_t *pbuf, uint16_t len);
int hal_spis_bus_init(hal_spi_t *spi_ptr, spi_Cfg_t cfg);
#endif

#ifdef DMAC_USE
int hal_spi_dma_set(SPI_INDEX_e spi, uint8_t ten, uint8_t ren);
#endif

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup SPI_Exported_ROM_Functions SPI Exported ROM Functions
  * @{
  */

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup SPI_Exported_ROM_Variables SPI Exported ROM Variables
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

#endif /* _HAL_SPI_H */
