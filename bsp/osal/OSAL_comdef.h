/**************************************************************************************************
    Filename:       comdef.h
    Revised:
    Revision:

    Description:    Type definitions and macros.

 SDK_LICENSE

**************************************************************************************************/

#ifndef COMDEF_H
#define COMDEF_H

#ifdef __cplusplus
extern "C"
{
#endif



/*********************************************************************
    INCLUDES
*/

/* HAL */

#include <types.h>


/*********************************************************************
    Lint Keywords
*/

/*********************************************************************
    CONSTANTS
*/

/*** Generic Status Return Values ***/
#define SUCCESS                   0x00
#define FAILURE                   0x01
#define INVALIDPARAMETER          0x02
#define INVALID_TASK              0x03
#define MSG_BUFFER_NOT_AVAIL      0x04
#define INVALID_MSG_POINTER       0x05
#define INVALID_EVENT_ID          0x06
#define INVALID_INTERRUPT_ID      0x07
#define NO_TIMER_AVAIL            0x08
#define NV_ITEM_UNINIT            0x09
#define NV_OPER_FAILED            0x0A
#define INVALID_MEM_SIZE          0x0B
#define NV_BAD_ITEM_LEN           0x0C

/*********************************************************************
    TYPEDEFS
*/

// Generic Status return
typedef uint8_t Status_t;

// Data types
typedef int32_t   int24;
typedef uint32_t  uint24;

/*********************************************************************
    Global System Events
*/

#define SYS_EVENT_MSG               0x8000  // A message is waiting event

/*********************************************************************
    Global Generic System Messages
*/

#define KEY_CHANGE                0xC0    // Key Events

// OSAL System Message IDs/Events Reserved for applications (user applications)
// 0xE0 � 0xFC

/*********************************************************************
    MACROS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* COMDEF_H */
