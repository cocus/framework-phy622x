#ifndef OSAL_CRITICAL_H
#define OSAL_CRITICAL_H

extern void drv_irq_init(void);
extern int drv_enable_irq(void);
extern int drv_disable_irq(void);

#define HAL_CRITICAL_SECTION_INIT() drv_irq_init()
#define HAL_ENTER_CRITICAL_SECTION() drv_disable_irq()
#define HAL_EXIT_CRITICAL_SECTION() drv_enable_irq()

extern int m_in_critical_region;

#endif // OSAL_CRITICAL_H
