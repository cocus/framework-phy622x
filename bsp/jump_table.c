/**************************************************************************************************
    Filename:       jump_table.c
    Revised:
    Revision:

    Description:    Jump table that holds function pointers and veriables used in ROM code.

 SDK_LICENSE

**************************************************************************************************/

/*******************************************************************************
    INCLUDES
*/
#include <jump_function.h>
// #include "global_config.h"

/*******************************************************************************
    CONSTANTS
*/
// jump table, this table save the function entry which will be called by ROM code
// item 1 - 4 for OSAL task entry
// item 224 - 255 for ISR(Interrupt Service Routine) entry
// others are reserved by ROM code
const uint32_t *jump_table_base[256] __attribute__((section("jump_table_mem_area"))) =
    {
        (const uint32_t *)0,          // 0. write Log
        (const uint32_t *)0,          // 1. init entry of app
        (const uint32_t *)0,          // 2. task list
        (const uint32_t *)0,          // 3. task count
        (const uint32_t *)0,          // 4. task events
        0, 0, 0, 0, 0,                // 5 - 9, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10 - 19, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20 - 29, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0,       // <30 - - 37>
        0, 0,
        0, 0, 0, 0, 0, 0,             // 40 - 45
        0, 0, 0, 0,                   // 46 - 49
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50 - 59, reserved for rom patch
        0,                            // < 60 -
        0,
        0,
        0,
        0, 0, 0, 0, 0, 0,             //  -69>, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 70 -79, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 80 - 89, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 90 - 99, reserved for rom patch
        (const uint32_t *)0,          // <100 -
        (const uint32_t *)0,
        [RF_INIT] = (const uint32_t *)0,
        0,
        0,
        0,
        0, 0, 0, 0,                   // - 109, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 110 -119, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 120 -129, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 130 -139, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 140 -149, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 150 -159, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 160 -169, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 170 -179, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 180 -189, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 190 -199, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 200 - 209, reserved for rom patch
        0, 0, 0, 0, 0, 0, 0, 0, 0,    // 210 - 218, reserved for rom patch
        [NMI_HANDLER] = 0,            // 219
        [HARDFAULT_HANDLER] = 0,
        [SVC_HANDLER] = 0,
        [PENDSV_HANDLER] = 0,
        [SYSTICK_HANDLER] = 0,
        0,
        0,
        0,
        0,                                       // 227
        0,                                       // 228
        0,                                       // 229
        0, 0, 0, 0, 0,                           // 230 - 234
        [V11_IRQ_HANDLER] = (const uint32_t *)0, // 235 uart irq handler
        0, 0, 0, 0, 0,                           // 236 - 240
        0, 0, 0, 0, 0, 0, 0, 0, 0,               // 241 - 249, for ISR entry
        0, 0, 0, 0, 0, 0                         // 250 - 255, for ISR entry
};

/*******************************************************************************
    Prototypes
*/

/*******************************************************************************
    LOCAL VARIABLES
*/

/*********************************************************************
    EXTERNAL VARIABLES
*/
uint32_t global_config[256] __attribute__((section("global_config_area")));
