/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HAL_I2C_H
#define _HAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

#include <stddef.h> // for size_t

#include <driver/gpio/gpio.h> /* for gpio_pin_e */

typedef enum
{
    I2C_0                 =0,    //define master mode,1:master mode,0:salve mode
    I2C_1                 =1    //define master mode,1:master mode,0:salve mode
} i2c_dev_t;

typedef enum
{
    I2C_CLOCK_100K  = 0x00,
    I2C_CLOCK_400K,
} I2C_CLOCK_e;

typedef struct _I2C_Evt_t
{
    uint16_t   type;
    uint8_t*  data;
    uint8_t   len;
} I2C_Evt_t;


int hal_i2c_init(i2c_dev_t dev, I2C_CLOCK_e i2c_clock_rate, gpio_pin_e pin_sda, gpio_pin_e pin_clk, uint8_t use_pull);
int hal_i2c_master_receive(i2c_dev_t dev, uint8_t addr, uint8_t* data, uint8_t size, uint32_t timeout);
int hal_i2c_master_read_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size, uint32_t timeout);

int hal_i2c_master_transmit(i2c_dev_t dev, uint8_t addr, const uint8_t* data, uint8_t size, uint32_t timeout);
int hal_i2c_master_write_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t size, uint32_t timeout);

int hal_i2c_deinit(i2c_dev_t dev);
int hal_i2c_wait_tx_completed(i2c_dev_t dev, uint32_t timeout);


#ifdef __cplusplus
}
#endif

#endif /* _HAL_I2C_H */
