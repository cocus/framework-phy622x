/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup phy62xx
  * @{
  */

#ifndef __PHY62XX_H
#define __PHY62XX_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/** @addtogroup Device_Included
  * @{
  */

#include "phy6222.h"
/**
  * @}
  */

/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

/* Use of CMSIS compiler intrinsics for register exclusive access */
/* Atomic 32-bit register access macro to set one or several bits */
#define ATOMIC_SET_BIT(REG, BIT)                             \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear one or several bits */
#define ATOMIC_CLEAR_BIT(REG, BIT)                           \
  do {                                                       \
    uint32_t val;                                            \
    do {                                                     \
      val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 32-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFY_REG(REG, CLEARMSK, SETMASK)                          \
  do {                                                                     \
    uint32_t val;                                                          \
    do {                                                                   \
      val = (__LDREXW((__IO uint32_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U);               \
  } while(0)

/* Atomic 16-bit register access macro to set one or several bits */
#define ATOMIC_SETH_BIT(REG, BIT)                            \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) | (BIT);       \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear one or several bits */
#define ATOMIC_CLEARH_BIT(REG, BIT)                          \
  do {                                                       \
    uint16_t val;                                            \
    do {                                                     \
      val = __LDREXH((__IO uint16_t *)&(REG)) & ~(BIT);      \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U); \
  } while(0)

/* Atomic 16-bit register access macro to clear and set one or several bits */
#define ATOMIC_MODIFYH_REG(REG, CLEARMSK, SETMASK)                         \
  do {                                                                     \
    uint16_t val;                                                          \
    do {                                                                   \
      val = (__LDREXH((__IO uint16_t *)&(REG)) & ~(CLEARMSK)) | (SETMASK); \
    } while ((__STREXH(val,(__IO uint16_t *)&(REG))) != 0U);               \
  } while(0)


/**
  \brief   Enable Interrupts
  \details Enables a device specific set of interrupts in the NVIC interrupt controller.
  \param [in]      IRQs  Device specific interrupt bits.
  \note    IRQs must not be negative.
 */
__STATIC_INLINE void NVIC_EnableIRQs(uint32_t IRQs)
{
  NVIC->ISER[0U] = IRQs;
}

/**
  \brief   Disable Interrupts
  \details Disables a device specific set of interrupts in the NVIC interrupt controller.
  \param [in]      IRQs  Device specific interrupt bits.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE void NVIC_DisableIRQs(uint32_t IRQs)
{
  NVIC->ICER[0U] = IRQs;
  __DSB();
  __ISB();
}

/**
  \brief   Get Interrupt Enable status
  \details Returns a device specific interrupt enable bitmap status from the NVIC interrupt controller.
  \return             Bitmap of enabled interrupts.
  \note    IRQn must not be negative.
 */
__STATIC_INLINE uint32_t NVIC_GetEnableIRQs(void)
{
  return NVIC->ISER[0U];
}

/**
  * @}
  */

#if defined (USE_BSP_DRIVER)
 #include "phy62xx_bsp.h"
#endif /* USE_BSP_DRIVER */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PHY62XX_H */
/**
  * @}
  */

/**
  * @}
  */
