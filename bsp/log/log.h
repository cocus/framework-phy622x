/*******************************************************************************
    @file     log.h
    @brief    Contains all functions support for uart driver
    @version  0.0
    @date     31. Jan. 2018
    @author   eagle.han

 SDK_LICENSE

*******************************************************************************/

#ifndef __LOG_H__
#define __LOG_H__


#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    extern volatile uint32_t s_rom_debug_level;

    typedef void (*std_putc)(char *data, uint16_t size);

    //extern void log_vsprintf(std_putc putc, const char *fmt, va_list args);
    extern void log_printf(const char *format, ...);
    extern void log_set_putc(std_putc putc);
    extern void log_clr_putc(std_putc putc);
    extern int log_debug_level(uint8_t level);
    extern uint32_t log_get_debug_level(void);

    void dbg_printf(const char *format, ...);
    void log_timed_printf(const char *format, ...);
    void dbg_printf_init(void);
    uint32_t dbg_time(void);
    void my_dump_byte(uint8_t *pData, int dlen);
#ifndef DEBUG_INFO
#define DEBUG_INFO 3 /* max by default */
#endif

#if (DEBUG_INFO == 1)
#define AT_LOG(...)
#define LOG_DEBUG(...)
#define LOG(fmt, ...) dbg_printf("[%ld] %s: " fmt "\n", dbg_time(), __FUNCTION__, ##__VA_ARGS__)
#define LOG_INIT() dbg_printf_init()
#define LOG_DUMP_BYTE(a, b) my_dump_byte(a, b)
#elif (DEBUG_INFO == 2)
#define AT_LOG(...) dbg_printf(__VA_ARGS__)
#define LOG_DEBUG(...)
#define LOG(fmt, ...) dbg_printf("[%ld] %s: " fmt "\n", dbg_time(), __FUNCTION__, ##__VA_ARGS__)
#define LOG_INIT() dbg_printf_init()
#define LOG_DUMP_BYTE(a, b) my_dump_byte(a, b)
#elif (DEBUG_INFO == 3)
#define LOG(fmt, ...) dbg_printf("[%ld] %s: " fmt "\n", dbg_time(), __FUNCTION__, ##__VA_ARGS__)
#define AT_LOG(...) dbg_printf(__VA_ARGS__)
#define LOG_DEBUG(...) dbg_printf(__VA_ARGS__)
#define LOG_INIT() dbg_printf_init()
#define LOG_DUMP_BYTE(a, b) my_dump_byte(a, b)
#else
#define AT_LOG(...)
#define LOG_DEBUG(...)
#define LOG(...)
#define LOG_INIT() //{clk_gate_enable(MOD_UART);clk_reset(MOD_UART);clk_gate_disable(MOD_UART);}
#define LOG_DUMP_BYTE(a, b)
#endif

#ifdef __cplusplus
}
#endif

#endif //__LOG_H__
