/*************
 ll_sleep.h
 SDK_LICENSE
***************/

#ifndef LL_SLEEP__H_
#define LL_SLEEP__H_

#include <stdint.h>

/*******************************************************************************
    MACROS
*/

// convert 625us units to 32kHz units without round: the ratio of 32 kHz ticks
// to 625 usec ticks is 32768/1600 = 20.48 or 512/25
#define LL_SLEEP_625US_TO_32KHZ( us )       ((((uint32_t) (us)) * 512) / 25)

// convert 31.25ns units to 32kHz units without round: the ratio of 31.25ns usec
// ticks to 32 kHz ticks is 32M/32768 = 976.5625 or 15625/16, but using 976 is
// close enough given the accuracy
#define LL_SLEEP_31_25NS_TO_32KHZ( ns )     (((uint32_t) (ns)) / 976)


// 32KHz timer:
//  crystal: 32768Hz
//  RC     : 32768Hz Should be same as Xtal
//  timer1 - 4 : 4MHz
#define TIMER_TO_32K_CRYSTAL          122           //  122.0703
#define TIMER_TO_32K_RC               122           //  125

#define   STD_RC32_8_CYCLE_16MHZ_CYCLE     3906             // standard 16Mhz cycles for 8 RC32KHz tick
#define   STD_CRY32_8_CYCLE_16MHZ_CYCLE    3906             // standard 16Mhz cycles for 8 crystal 32KHz tick
#define   ERR_THD_RC32_CYCLE                200             // error threshold for N+x rcosc tracking cycle


#define   CRY32_8_CYCLE_16MHZ_CYCLE_MAX    (3906 + 196)     // tracking value range std +/- 5%
#define   CRY32_8_CYCLE_16MHZ_CYCLE_MIN    (3906 - 196)

#define   STD_RC32_16_CYCLE_16MHZ_CYCLE     (7812)          // standard 16Mhz cycles for 16 RC32KHz tick
#define   STD_CRY32_16_CYCLE_16MHZ_CYCLE    (7812)          // standard 16Mhz cycles for 16 crystal 32KHz tick


#define   CRY32_16_CYCLE_16MHZ_CYCLE_MAX    (7812 + 391)     // tracking value range std +/- 5%
#define   CRY32_16_CYCLE_16MHZ_CYCLE_MIN    (7812 - 391)

#define   SLEEP_MAGIC                 0x032141B6


/*******************************************************************************
    TYPEDEFS
*/
typedef enum
{
    MCU_SLEEP_MODE,
    SYSTEM_SLEEP_MODE,
    SYSTEM_OFF_MODE
}  Sleep_Mode;



/*******************************************************************************
    Functions
*/

// is sleep allow
uint8_t isSleepAllow(void);

void enableSleep(void);

void disableSleep(void);

void setSleepMode(Sleep_Mode mode);

Sleep_Mode getSleepMode(void);

void enterSleepProcess(uint32_t time);

void wakeupProcess(void);

void set_sleep_flag(int flag);

unsigned int get_sleep_flag(void);

void config_RTC(uint32_t time);

void enter_sleep_off_mode(Sleep_Mode mode);

#endif     // LL_SLEEP__H_




