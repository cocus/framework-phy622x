#ifndef BOOT_MISC_H
#define BOOT_MISC_H

#include <stdint.h>

void boot_init0(void);

void wakeup_init0(void);

extern uint32_t rtc_count_delay_on_wakeup;

extern uint32_t **global_config_alias;


#endif // BOOT_MISC_H
