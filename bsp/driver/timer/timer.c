/*************
 timer.h
 SDK_LICENSE
***************/
#include "rom_sym_def.h"
#include "timer.h"
#include "error.h"
#include "clock.h"
#include "pwrmgr.h"
#include "jump_function.h"

AP_TIM_TypeDef* const TimerIndex[FREE_TIMER_NUMBER]= {AP_TIM5,AP_TIM6};
static ap_tm_hdl_t s_ap_callback = NULL;

static int hal_timer_clear_int(AP_TIM_TypeDef* TIMx)
{
    return TIMx->EOI;
}

static void hal_timer_stop_counter(AP_TIM_TypeDef* TIMx)
{
    TIMx->ControlReg = 0;
    TIMx->LoadCount = 0;   //0x0
    TIMx->CurrentCount = 0;//0x4
}

static void hal_timer_set_loadtimer(AP_TIM_TypeDef* TIMx, int time)
{
    if(time>0)
    {
        TIMx->ControlReg = 0x0;
        TIMx->ControlReg = 0x2;
        TIMx->LoadCount = 4*time;// 4MHz system timer, * 4 to convert to 1MHz timer
        TIMx->ControlReg = 0x3;
    }
    else
    {
        TIMx->ControlReg = 0x0;
    }
}

void __attribute__((used)) hal_TIMER5_IRQHandler(void)
{
    if(AP_TIM5->status & 0x1)
    {
        hal_timer_clear_int(AP_TIM5);

        if(s_ap_callback)
            s_ap_callback(HAL_EVT_TIMER_5);
    }
}

void __attribute__((used)) hal_TIMER6_IRQHandler(void)
{
    if(AP_TIM6->status & 0x1)
    {
        hal_timer_clear_int(AP_TIM6);

        if(s_ap_callback)
            s_ap_callback(HAL_EVT_TIMER_6);
    }
}

static void hal_timer_wakeup_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_WAKEUP);
}

void hal_timer_sleep_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_SLEEP);
}

int hal_timer_mask_int(User_Timer_e timeId, uint8_t en)
{
    volatile AP_TIM_TypeDef* TIMx;
    TIMx = TimerIndex[timeId-AP_TIMER_ID_5];

    if(en)
        TIMx->ControlReg |= (1 << 2);
    else
        TIMx->ControlReg &= ~(1 << 2);

    return PPlus_SUCCESS;
}

int hal_timer_set(User_Timer_e timeId, uint32_t us)
{
    uint32_t time = us;

    switch(timeId)
    {
    case AP_TIMER_ID_5:
        JUMP_FUNCTION(TIM5_IRQ_HANDLER)                  =   (uint32_t)&hal_TIMER5_IRQHandler;
        NVIC_EnableIRQ((IRQn_Type)TIM5_IRQn);
        NVIC_SetPriority((IRQn_Type)TIM5_IRQn, IRQ_PRIO_HAL);
        hal_timer_set_loadtimer(AP_TIM5, time);
        hal_clk_gate_enable(MOD_TIMER5);
        break;

    case AP_TIMER_ID_6:
        JUMP_FUNCTION(TIM6_IRQ_HANDLER)                  =   (uint32_t)&hal_TIMER6_IRQHandler;
        NVIC_EnableIRQ((IRQn_Type)TIM6_IRQn);
        NVIC_SetPriority((IRQn_Type)TIM6_IRQn, IRQ_PRIO_HAL);
        hal_timer_set_loadtimer(AP_TIM6, time);
        hal_clk_gate_enable(MOD_TIMER6);
        break;

    default:
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_timer_stop(User_Timer_e timeId)
{
    switch(timeId)
    {
    case AP_TIMER_ID_5:
        JUMP_FUNCTION(TIM5_IRQ_HANDLER) = 0;
        hal_timer_stop_counter(AP_TIM5);
        NVIC_DisableIRQ((IRQn_Type)TIM5_IRQn);
        hal_clk_gate_disable(MOD_TIMER5);
        break;

    case AP_TIMER_ID_6:
        JUMP_FUNCTION(TIM6_IRQ_HANDLER) = 0;
        hal_timer_stop_counter(AP_TIM6);
        NVIC_DisableIRQ((IRQn_Type)TIM6_IRQn);
        hal_clk_gate_disable(MOD_TIMER6);
        break;

    default:
        return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_timer_init(ap_tm_hdl_t callback)
{
    s_ap_callback = callback;
    hal_timer_stop(AP_TIMER_ID_5);
    hal_timer_stop(AP_TIMER_ID_6);
    return hal_pwrmgr_register(MOD_TIMER, hal_timer_sleep_handler, hal_timer_wakeup_handler);
}

int hal_timer_deinit(void)
{
    s_ap_callback = NULL;
    return hal_pwrmgr_unregister(MOD_TIMER);
}
