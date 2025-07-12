
#include <types.h> /* for UNUSED */
#include <driver/pwm/pwm.h>

#include <log/log.h>

#include "FreeRTOS.h"
#include "task.h"

#include "osal_nuker.h"

// LED
#define GPIO_LED GPIO_P00


void genericTask(void *argument)
{
    UNUSED(argument);
    LOG("Hi from genericTask");

    gpio_pin_e pin = GPIO_LED;
    hal_gpio_pin_init(pin, GPIO_INPUT);
    hal_gpio_pull_set(pin, WEAK_PULL_UP);

    PWMN_e pwmN = PWM_CH1;
    hal_pwm_init(pwmN, PWM_CLK_DIV_128, PWM_CNT_UP, PWM_POLARITY_RISING, pin);
    hal_pwm_set_count_top_val(pwmN, 0, 256);
    hal_pwm_enable();

    uint16_t val = 0;

    while(1)
    {
        if (val++ == 256)
        {
            val = 0;
        }
        hal_pwm_set_count_val(pwmN, val);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void)
{
    /* init stuff as if OSAL was in charge */
    osal_nuker_init(SYS_CLK_DLL_96M, CLK_32K_RCOSC);

    LOG("g_hclk %d", g_hclk);

    xTaskCreate(genericTask, "genericTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

    LOG("starting FreeRTOS scheduler");

    vTaskStartScheduler();

    return 0;
}
