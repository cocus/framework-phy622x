/*************
 timer.c
 SDK_LICENSE
***************/
#ifndef __TIMER_H__
#define __TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <phy62xx.h>

#define FREE_TIMER_NUMBER 2

typedef enum
{
    AP_TIMER_ID_5 = 5,
    AP_TIMER_ID_6 = 6,

} User_Timer_e;

enum
{
    HAL_EVT_TIMER_5 = AP_TIMER_ID_5,
    HAL_EVT_TIMER_6 = AP_TIMER_ID_6,
    HAL_EVT_WAKEUP = 0x10,
    HAL_EVT_SLEEP
};

typedef void(*ap_tm_hdl_t)(uint8_t evt);

int hal_timer_init(ap_tm_hdl_t callback);

int hal_timer_deinit(void);

int hal_timer_set(User_Timer_e timeId, uint32_t us);

int hal_timer_mask_int(User_Timer_e timeId, uint8_t en);

int hal_timer_stop(User_Timer_e timeId);

void __attribute__((used)) hal_TIMER5_IRQHandler(void);
void __attribute__((used)) hal_TIMER6_IRQHandler(void);

extern void set_timer(AP_TIM_TypeDef* TIMx, int time);

extern uint32_t read_current_fine_time(void);

extern uint32_t  read_LL_remainder_time(void);

extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern uint8_t isTimer1Running(void);
extern uint8_t isTimer4Running(void);
extern void clear_timer(AP_TIM_TypeDef* TIMx);

extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_TIM2_IRQ_PendingTick;

#ifndef BASE_TIME_UINTS
#define BASE_TIME_UNITS   (0x3fffff)
#endif

#ifdef __cplusplus
}
#endif

#endif
