/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_FLASH_H
#define _HAL_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <driver/clock/clock.h> /* for sysclk_t */

#include <rom/rom_attr.h> /* for ATTR_ROM_VAR and ATTR_ROM_FN */

/** @addtogroup PHY62XX_BSP_Driver
  * @{
  */

/** @addtogroup FLASH
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup FLASH_Exported_Types FLASH Exported Types
  * @{
  */

/**
  * @brief  PHY62xx Execute-in-place (XIP) read instruction mode.
  */
typedef enum
{
  XFRD_FCMD_READ_DUAL = 0x801003B,    /*!< Use DIO */
  XFRD_FCMD_READ_QUAD = 0x801006B,    /*!< Use QIO */
} XFRD_FCMD_READ_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
  */

  /**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */

  /**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup FLASH_Exported_Functions
  * @{
  */
    /**
      * @brief  Initializes the embedded SPI Flash and its XIP cache.
      * @param  spif_ref_clk: ? might or might not need to match the system clock?
      * @param  rd_instr: Selects the XIP code read mode, dual or quad.
      * @retval None.
      */
    void hal_spif_cache_init(sysclk_t spif_ref_clk, XFRD_FCMD_READ_t rd_instr);

    /**
      * @brief  Write-locks the embedded SPI Flash.
      * @param  None.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_lock(void);

    /**
      * @brief  Write-unlocks the embedded SPI Flash.
      * @param  None.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_unlock(void);

    /**
      * @brief  Returns if the embedded SPI Flash is write-locked or not.
      * @param  None.
      * @retval 0 if unlocked, any other value otherwise.
      */
    uint8_t hal_flash_get_lock_state(void);

    /**
      * @brief  Writes data on the embedded SPI Flash.
      * @param  addr: Offset on the SPI Flash to write to.
      * @param  data: Pointer to the buffer which contains the data to write.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_write(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Writes data on the embedded SPI Flash, using DMA.
      * @param  addr: Offset on the SPI Flash to write to.
      * @param  data: Pointer to the buffer which contains the data to write.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_write_by_dma(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Reads data from the embedded SPI Flash.
      * @param  addr: Offset on the SPI Flash to read from.
      * @param  data: Pointer to the buffer to read the data from the flash.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_read(uint32_t addr, uint8_t *data, uint32_t size);

    /* TODO!!!: hal_flash_read_dma maybe? */

    /**
      * @brief  Erases a sector of the embedded SPI Flash.
      * @param  addr: Sector number. I have no idea what's the sector size.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_erase_sector(unsigned int addr);

    /**
      * @brief  Erases a block of 64kb on the embedded SPI Flash.
      * @param  addr: Block number.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_erase_block64(unsigned int addr);

    /**
      * @brief  Erases all SPI Flash?
      * @param  None.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    int hal_flash_erase_all(void);

/**
  * @}
  */

/* Exported ROM functions ----------------------------------------------------*/
/** @defgroup FLASH_Exported_ROM_Functions FLASH Exported ROM Functions
  * @{
  */
    /**
      * @brief  Writes data on the embedded SPI Flash.
      * @param  addr: Offset on the SPI Flash to write to.
      * @param  data: Pointer to the buffer which contains the data to write.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_write(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Writes data on the embedded SPI Flash, using DMA.
      * @param  addr: Offset on the SPI Flash to write to.
      * @param  data: Pointer to the buffer which contains the data to write.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_write_dma(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Reads data from the embedded SPI Flash.
      * @param  addr: Offset on the SPI Flash to read from.
      * @param  data: Pointer to the buffer to read the data from the flash.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_read(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Reads data from the embedded SPI Flash, using DMA.
      * @param  addr: Offset on the SPI Flash to read from.
      * @param  data: Pointer to the buffer to read the data from the flash.
      * @param  size: Size of the buffer.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_read_dma(uint32_t addr, uint8_t *data, uint32_t size);

    /**
      * @brief  Erases a sector of the embedded SPI Flash.
      * @param  addr: Sector number. I have no idea what's the sector size.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_erase_sector(unsigned int addr);

    /**
      * @brief  Erases a block of 64kb on the embedded SPI Flash.
      * @param  addr: Block number.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_erase_block64(unsigned int addr);

    /**
      * @brief  Erases all SPI Flash?
      * @param  None.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if busy, PPlus_ERR_INVALID_STATE if SPIF not initialized.
      */
    ATTR_ROM_FN int spif_erase_all(void);

    /**
      * @brief  Executes a command on the embedded SPI Flash.
      * @param  op: Command, one of SPIF_CMD_*.
      * @param  addrlen: ?
      * @param  rdlen: ?
      * @param  wrlen: ?
      * @param  mbit: ?
      * @param  dummy: ?
      * @retval None.
      */
    ATTR_ROM_FN void spif_cmd(uint8_t op, uint8_t addrlen, uint8_t rdlen, uint8_t wrlen, uint8_t mbit, uint8_t dummy);

    /**
      * @brief  Reads data from a command executed on the embedded SPI Flash.
      * @param  data: Pointer to write the data.
      * @param  len: Length of the data to read.
      * @retval None.
      */
    ATTR_ROM_FN void spif_rddata(uint8_t *data, uint8_t len);

    /**
      * @brief  Configures the embedded SPI Flash peripheral.
      * @param  ref_clk: Clock source.
      * @param  div: Clock divider.
      * @param  rd_instr: XIP code mode.
      * @param  QE: QIO enable if 1.
      * @retval PPlus_SUCCESS on success, PPlus_ERR_BUSY if SPIF is busy, PPlus_ERR_SPI_FLASH if something went wrong.
      */
    ATTR_ROM_FN int spif_config(sysclk_t ref_clk, uint8_t div, uint32_t rd_instr, uint8_t mode_bit, uint8_t QE);

/**
  * @}
  */

/* Exported ROM variables ----------------------------------------------------*/
/** @defgroup FLASH_Exported_ROM_Variables FLASH Exported ROM Variables
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

#endif /* _HAL_FLASH_H */
