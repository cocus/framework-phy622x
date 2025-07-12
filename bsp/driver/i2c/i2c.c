/**
  ******************************************************************************
  * @file    i2c.c
  * @author  PhyPlus, Santiago Hormazabal
  * @brief   I2C BSP module driver.
  *          This file provides firmware functions to manage the I2C modules
  *          on the MCU, the pins related to them, and provides a convenient
  *          API for sending/receiving data.
  *           + Initialize a I2C module
  *           + Send/Receive data to a given slave device
  *           + Write/Read a register on a given slave device
  *           + TODO: interrupts
  *           + TODO: slave mode
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <phy62xx.h>

#include "i2c.h"

#include <driver/clock/clock.h>

#include <osal/osal_critical.h>

#include <phy_error.h>


static gpio_pin_e i2c_pins[2][2] = {
    { GPIO_DUMMY, GPIO_DUMMY },
    { GPIO_DUMMY, GPIO_DUMMY }
};

static inline void i2c_send_byte(AP_I2C_TypeDef* pi2cdev, uint8_t data)
{
    pi2cdev->IC_DATA_CMD = data;
}

static void i2c_send_byte_and_wait(AP_I2C_TypeDef* pi2cdev, uint8_t data)
{
    pi2cdev->IC_DATA_CMD = data;
    while(!(pi2cdev->IC_RAW_INTR_STAT & I2Cx_RAW_INT_STAT_TXEMPTY));
}

static const struct {
    uint32_t pclk;
    uint32_t hcnt;
    uint32_t lcnt;
    I2C_CLOCK_e rate;
} speed_lut[] = {
    /* Not sure how these are calculated, so I've made a simple LUT */
    { 16000000,  70,  76, I2C_CLOCK_100K },
    { 32000000, 148, 154, I2C_CLOCK_100K },
    { 48000000, 230, 236, I2C_CLOCK_100K },
    { 64000000, 307, 320, I2C_CLOCK_100K },
    { 96000000, 460, 470, I2C_CLOCK_100K },
    { 16000000,  10,  17, I2C_CLOCK_400K },
    { 32000000,  30,  35, I2C_CLOCK_400K },
    { 48000000,  48,  54, I2C_CLOCK_400K },
    { 64000000,  67,  75, I2C_CLOCK_400K },
    { 96000000, 105, 115, I2C_CLOCK_400K },
};

static int hal_i2c_master_transmit_maybe_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, uint8_t use_reg, const uint8_t* data, uint8_t size, uint32_t timeout)
{
    AP_I2C_TypeDef* pi2cdev = NULL;

    if (dev == I2C_0)
    {
        pi2cdev = AP_I2C0;
    }
    else if (dev == I2C_1)
    {
        pi2cdev = AP_I2C1;
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    uint8_t cnt = 0;

    pi2cdev->IC_ENABLE = I2Cx_ENABLE_DISABLE;

    pi2cdev->IC_TAR = addr;

    while (size)
    {
        HAL_ENTER_CRITICAL_SECTION();

        pi2cdev->IC_ENABLE = I2Cx_ENABLE_ENABLE;

        if (use_reg)
        {
            i2c_send_byte(pi2cdev, reg);
            cnt = (size > 6) ? 6 : size;
        }
        else
        {
            cnt = (size > 7) ? 7 : size;
        }

        size -= cnt;

        while (cnt--)
        {
            i2c_send_byte_and_wait(pi2cdev, *data++);
            reg++;
        }

        HAL_EXIT_CRITICAL_SECTION();

        if (hal_i2c_wait_tx_completed(dev, timeout) == PPlus_ERR_TIMEOUT)
        {
            return PPlus_ERR_TIMEOUT;
        }
    }

    return PPlus_SUCCESS;
}

static int i2c_master_receive_maybe_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, uint8_t use_reg, uint8_t* data, uint8_t size, uint32_t timeout)
{
    AP_I2C_TypeDef* pi2cdev = NULL;

    if (dev == I2C_0)
    {
        pi2cdev = AP_I2C0;
    }
    else if (dev == I2C_1)
    {
        pi2cdev = AP_I2C1;
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    uint8_t cnt = 0;
    while (size)
    {
        pi2cdev->IC_ENABLE = I2Cx_ENABLE_DISABLE;

        pi2cdev->IC_TAR = addr;

        HAL_ENTER_CRITICAL_SECTION();

        pi2cdev->IC_ENABLE = I2Cx_ENABLE_ENABLE;

        if (use_reg)
        {
            i2c_send_byte(pi2cdev, reg);
        }

        cnt = (size > 7) ? 7 : size;
        size -= cnt;

        uint8_t i = cnt;
        while (i--)
        {
            pi2cdev->IC_DATA_CMD = I2Cx_DATA_CMD_CMD_READ;
        }

        HAL_EXIT_CRITICAL_SECTION();

        uint32_t to = osal_sys_tick;
        while (1)
        {
            if (pi2cdev->IC_STATUS & I2Cx_STATUS_RFNE)
            {
                *data = (pi2cdev->IC_DATA_CMD & 0xff);
                data++;
                cnt--;
                reg++;

                if (cnt == 0)
                {
                    break;
                }
            }

            if (osal_sys_tick - to > timeout)
            {
                return PPlus_ERR_TIMEOUT;
            }
        }
    }

    return PPlus_SUCCESS;
}

int hal_i2c_wait_tx_completed(i2c_dev_t dev, uint32_t timeout)
{
    AP_I2C_TypeDef* pi2cdev = NULL;

    if (dev == I2C_0)
    {
        pi2cdev = AP_I2C0;
    }
    else if (dev == I2C_1)
    {
        pi2cdev = AP_I2C1;
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    uint32_t to = osal_sys_tick;
    while(1)
    {
        if (pi2cdev->IC_RAW_INTR_STAT & I2Cx_RAW_INT_STAT_STOP_DET)// check tx empty
        {
            break;
        }
        if (osal_sys_tick - to > timeout)
        {
            return PPlus_ERR_TIMEOUT;
        }
    }

    return PPlus_SUCCESS;
}

int hal_i2c_deinit(i2c_dev_t dev)
{
    if (dev == I2C_0)
    {
        I2C0_ENABLE = I2Cx_ENABLE_DISABLE;

        if (i2c_pins[0][0] != GPIO_DUMMY)
        {
            hal_gpio_fmux(i2c_pins[0][0], Bit_DISABLE);
        }
        if (i2c_pins[0][1] != GPIO_DUMMY)
        {
            hal_gpio_fmux(i2c_pins[0][1], Bit_DISABLE);
        }

        i2c_pins[0][0] = GPIO_DUMMY;
        i2c_pins[0][1] = GPIO_DUMMY;

        hal_clk_gate_disable(MOD_I2C0);
    }
    else if (dev == I2C_1)
    {
        I2C1_ENABLE = I2Cx_ENABLE_DISABLE;

        if (i2c_pins[1][0] != GPIO_DUMMY)
        {
            hal_gpio_fmux(i2c_pins[1][0], Bit_DISABLE);
        }
        if (i2c_pins[1][1] != GPIO_DUMMY)
        {
            hal_gpio_fmux(i2c_pins[1][1], Bit_DISABLE);
        }

        i2c_pins[1][0] = GPIO_DUMMY;
        i2c_pins[1][1] = GPIO_DUMMY;

        hal_clk_gate_disable(MOD_I2C1);
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_i2c_init(i2c_dev_t dev, I2C_CLOCK_e i2c_clock_rate, gpio_pin_e pin_sda, gpio_pin_e pin_clk, uint8_t use_pull)
{
    AP_I2C_TypeDef* pi2cdev = NULL;
    uint32_t pclk = clk_get_pclk();

    if (dev == I2C_0)
    {
        pi2cdev = AP_I2C0;

        i2c_pins[0][0] = pin_clk;
        i2c_pins[0][1] = pin_sda;

        hal_gpio_fmux_set(pin_clk, FMUX_IIC0_SCL);
        hal_gpio_fmux_set(pin_sda, FMUX_IIC0_SDA);
        hal_clk_gate_enable(MOD_I2C0);
    }
    else if (dev == I2C_1)
    {
        pi2cdev = AP_I2C1;

        i2c_pins[1][0] = pin_clk;
        i2c_pins[1][1] = pin_sda;

        hal_gpio_fmux_set(pin_clk, FMUX_IIC1_SCL);
        hal_gpio_fmux_set(pin_sda, FMUX_IIC1_SDA);
        hal_clk_gate_enable(MOD_I2C1);
    }
    else
    {
        return PPlus_ERR_INVALID_PARAM;
    }

    if (use_pull)
    {
        hal_gpio_pull_set(pin_sda, GPIO_PULL_UP_S);
        hal_gpio_pull_set(pin_clk, GPIO_PULL_UP_S);
    }

    pi2cdev->IC_ENABLE = I2Cx_ENABLE_DISABLE;

    uint8_t found = 0;
    for (size_t i = 0; i < (sizeof(speed_lut)/sizeof(speed_lut[0])); i++)
    {
        if (speed_lut[i].pclk == pclk && speed_lut[i].rate == i2c_clock_rate)
        {
            pi2cdev->IC_FS_SCL_HCNT = speed_lut[i].hcnt;
            pi2cdev->IC_FS_SCL_LCNT = speed_lut[i].lcnt;
            found = 1;
            break;
        }
    }

    if (found == 0)
    {
        hal_i2c_deinit(dev);
        return PPlus_ERR_INVALID_PARAM;
    }

    pi2cdev->IC_CON = I2Cx_CON_SLAVE_DISABLE | I2Cx_CON_RESTART_EN | I2Cx_CON_MASTER;

    if (i2c_clock_rate == I2C_CLOCK_100K)
    {
        pi2cdev->IC_CON |= I2Cx_CON_SPEED_100kHz;
    }
    else if (i2c_clock_rate == I2C_CLOCK_400K)
    {
        pi2cdev->IC_CON |= I2Cx_CON_SPEED_400kHz;
    }
    else
    {
        hal_i2c_deinit(dev);
        return PPlus_ERR_INVALID_PARAM;
    }

    pi2cdev->IC_TAR = 0x00;
    pi2cdev->IC_INTR_MASK = 0;
    pi2cdev->IC_RX_TL = 0x0;
    pi2cdev->IC_TX_TL = 0x1;
    pi2cdev->IC_ENABLE = I2Cx_ENABLE_ENABLE;

    return PPlus_SUCCESS;
}

int hal_i2c_master_receive(i2c_dev_t dev, uint8_t addr, uint8_t* data, uint8_t size, uint32_t timeout)
{
    return i2c_master_receive_maybe_reg(dev, addr, 0, 0, data, size, timeout);
}

int hal_i2c_master_read_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, uint8_t* data, uint8_t size, uint32_t timeout)
{
    return i2c_master_receive_maybe_reg(dev, addr, reg, 1, data, size, timeout);
}

int hal_i2c_master_transmit(i2c_dev_t dev, uint8_t addr, const uint8_t* data, uint8_t size, uint32_t timeout)
{
    return hal_i2c_master_transmit_maybe_reg(dev, addr, 0, 0, data, size, timeout);
}

int hal_i2c_master_write_reg(i2c_dev_t dev, uint8_t addr, uint8_t reg, const uint8_t* data, uint8_t size, uint32_t timeout)
{
    return hal_i2c_master_transmit_maybe_reg(dev, addr, reg, 1, data, size, timeout);
}
