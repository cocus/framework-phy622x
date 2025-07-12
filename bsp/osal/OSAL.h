/******************************************************************************
    Filename:     OSAL.h
    Revised:
    Revision:

     SDK_LICENSE

******************************************************************************/

#ifndef OSAL_H
#define OSAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include <types.h>

// #include "comdef.h"
// #include "OSAL_Memory.h"
// #include "OSAL_Timers.h"

/*********************************************************************
    MACROS
*/
#if (UINT_MAX == 65535) /* 8-bit and 16-bit devices */
#define osal_offsetof(type, member) ((uint16_t)&(((type *)0)->member))
#else /* 32-bit devices */
#define osal_offsetof(type, member) ((uint32_t)&(((type *)0)->member))
#endif

#define OSAL_MSG_NEXT(msg_ptr) ((osal_msg_hdr_t *)(msg_ptr) - 1)->next

#define OSAL_MSG_Q_INIT(q_ptr) *(q_ptr) = NULL

#define OSAL_MSG_Q_EMPTY(q_ptr) (*(q_ptr) == NULL)

#define OSAL_MSG_Q_HEAD(q_ptr) (*(q_ptr))

#define OSAL_MSG_LEN(msg_ptr) ((osal_msg_hdr_t *)(msg_ptr) - 1)->len

#define OSAL_MSG_ID(msg_ptr) ((osal_msg_hdr_t *)(msg_ptr) - 1)->dest_id

/*********************************************************************
    CONSTANTS
*/

/*** Interrupts ***/
#define INTS_ALL 0xFF

    /*********************************************************************
        TYPEDEFS
    */
    typedef struct
    {
        void *next;
        uint16_t len;
        uint8_t dest_id;
    } osal_msg_hdr_t;

    typedef struct
    {
        uint8_t event;
        uint8_t status;
    } osal_event_hdr_t;

    typedef void *osal_msg_q_t;

    /*********************************************************************
        GLOBAL VARIABLES
    */

    /* Initialized to NULL by osal_init_system() */
    extern volatile osal_msg_q_t osal_qHead;

    /*********************************************************************
        FUNCTIONS
    */

    /*** Message Management ***/

    /*
        Task Message Allocation
    */
    extern uint8_t *osal_msg_allocate(uint16_t len);

    /*
        Task Message Deallocation
    */
    extern uint8_t osal_msg_deallocate(uint8_t *msg_ptr);

    /*
        Send a Task Message
    */
    extern uint8_t osal_msg_send(uint8_t destination_task, uint8_t *msg_ptr);

    /*
        Push a Task Message to head of queue
    */
    extern uint8_t osal_msg_push_front(uint8_t destination_task, uint8_t *msg_ptr);

    /*
        Receive a Task Message
    */
    extern uint8_t *osal_msg_receive(uint8_t task_id);

    /*
        Find in place a matching Task Message / Event.
    */
    extern osal_event_hdr_t *osal_msg_find(uint8_t task_id, uint8_t event);

    /*
        Enqueue a Task Message
    */
    extern void osal_msg_enqueue(osal_msg_q_t *q_ptr, void *msg_ptr);

    /*
        Enqueue a Task Message Up to Max
    */
    extern uint8_t osal_msg_enqueue_max(osal_msg_q_t *q_ptr, void *msg_ptr, uint8_t max);

    /*
        Dequeue a Task Message
    */
    extern void *osal_msg_dequeue(osal_msg_q_t *q_ptr);

    /*
        Push a Task Message to head of queue
    */
    extern void osal_msg_push(osal_msg_q_t *q_ptr, void *msg_ptr);

    /*
        Extract and remove a Task Message from queue
    */
    extern void osal_msg_extract(osal_msg_q_t *q_ptr, void *msg_ptr, void *prev_ptr);

    /*** Task Synchronization  ***/

    /*
        Set a Task Event
    */
    extern uint8_t osal_set_event(uint8_t task_id, uint16_t event_flag);

    /*
        Clear a Task Event
    */
    extern uint8_t osal_clear_event(uint8_t task_id, uint16_t event_flag);

    /*** Interrupt Management  ***/

    /*
        Register Interrupt Service Routine (ISR)
    */
    extern uint8_t osal_isr_register(uint8_t interrupt_id, void (*isr_ptr)(uint8_t *));

    /*
        Enable Interrupt
    */
    extern uint8_t osal_int_enable(uint8_t interrupt_id);

    /*
        Disable Interrupt
    */
    extern uint8_t osal_int_disable(uint8_t interrupt_id);

    /*** Task Management  ***/

    /*
        Initialize the Task System
    */
    extern uint8_t osal_init_system(void);

/*
    System Processing Loop
*/
#if defined(ZBIT)
    extern __declspec(dllexport) void osal_start_system(void);
#else
extern void osal_start_system(void);
#endif

    /*
        One Pass Throu the OSAL Processing Loop
    */
    extern void osal_run_system(void);

    /*
        Get the active task ID
    */
    extern uint8_t osal_self(void);

    /*** Helper Functions ***/

    /*
        String Length
    */
    extern int osal_strlen(char *pString);

    /*
        Memory copy
    */
    extern void *osal_memcpy(void *, const void *, unsigned int);

    /*
        Memory Duplicate - allocates and copies
    */
    extern void *osal_memdup(const void *src, unsigned int len);

    /*
        Reverse Memory copy
    */
    extern void *osal_revmemcpy(void *, const void *, unsigned int);

    /*
        Memory compare
    */
    extern uint8_t osal_memcmp(const void *src1, const void *src2, unsigned int len);

    /*
        Memory set
    */
    extern void *osal_memset(void *dest, uint8_t value, int len);

    /*
        Build a uint16_t out of 2 bytes (0 then 1).
    */
    extern uint16_t osal_build_uint16_t(uint8_t *swapped);

    /*
        Build a uint32_t out of sequential bytes.
    */
    extern uint32_t osal_build_uint32_t(uint8_t *swapped, uint8_t len);

/*
    Convert long to ascii string
*/
#if !defined(ZBIT) && !defined(ZBIT2) && !defined(UBIT)
    extern uint8_t *_ltoa(uint32_t l, uint8_t *buf, uint8_t radix);
#endif

    /*
        Random number generator
    */
    extern uint16_t osal_rand(void);

    /*
        Buffer an uint32_t value - LSB first.
    */
    extern uint8_t *osal_buffer_uint32_t(uint8_t *buf, uint32_t val);

    /*
        Buffer an uint24 value - LSB first
    */
    // extern uint8_t* osal_buffer_uint24( uint8_t* buf, uint24 val );

    /*
        Is all of the array elements set to a value?
    */
    extern uint8_t osal_isbufset(uint8_t *buf, uint8_t val, uint8_t len);

    /*********************************************************************
    *********************************************************************/

uint8_t osal_set_event0( uint8_t task_id, uint16_t event_flag );
uint8_t osal_msg_send0( uint8_t destination_task, uint8_t* msg_ptr );


void app_sleep_process(void);

void app_wakeup_process(void);

extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_osalTickTrim_mod;

#ifdef __cplusplus
}
#endif

#endif /* OSAL_H */
