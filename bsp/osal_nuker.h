#ifndef OSAL_NUKER_H
#define OSAL_NUKER_H

#include <driver/clock/clock.h>

void osal_nuker_init(sysclk_t clk, CLK32K_e rtcclk);

void osal_nuker_interrupt_init(void);

void osal_nuker_ble_init(void);

#endif // OSAL_NUKER_H
