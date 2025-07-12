
#include "osal_nuker.h"

#include <jump_function.h>
#include <global_config.h>
#include <phy62xx.h>
#include <log/log.h>
#include <driver/flash/flash.h>
#include <boot/boot_misc.h>
#include <osal/osal_critical.h>
#include <osal/OSAL.h>
#include <osal/OSAL_Tasks.h>
#include <osal/OSAL_Clock.h>
#include <osal/OSAL_Timers.h>
#include <osal/OSAL_Memory.h>

#include <ble/hci/hci_tl.h>
#include <ble/include/l2cap.h>
#include <ble/include/sm.h>
#include <ble/include/gap.h>
#include <ble/include/gatt.h>
#include <ble/controller/ll.h>
#include <ble/host/gattservapp.h>

#include <ble/controller/ll.h>
#include <ble/controller/rf_phy_driver.h>

#ifdef ENABLE_FREERTOS
#include "FreeRTOS.h" /* for portX functions */
#include "task.h"     /* for taskX functions */
#else
#define portDISABLE_INTERRUPTS()                __asm volatile ( " cpsid i " ::: "memory" )
#define portENABLE_INTERRUPTS()                 __asm volatile ( " cpsie i " ::: "memory" )
#endif

#include <phy_error.h>

#ifdef ENABLE_BTSTACK
#define BLE_MAX_ALLOW_CONNECTION 1
#define BLE_MAX_ALLOW_PKT_PER_EVENT_TX 3
#define BLE_MAX_ALLOW_PKT_PER_EVENT_RX 5

#define BLE_PKT_VERSION BLE_PKT_VERSION_5_1
#define BLE_PKT_BUF_SIZE (((BLE_PKT_VERSION == BLE_PKT_VERSION_5_1) ? 1 : 0) * BLE_PKT51_LEN + ((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN + (sizeof(struct ll_pkt_desc) - 2))
#define BLE_MAX_ALLOW_PER_CONNECTION ((BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE * 2) + (BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE) + BLE_PKT_BUF_SIZE)
#define BLE_CONN_BUF_SIZE (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)

_Alignas(4) uint8_t g_pConnectionBuffer[BLE_CONN_BUF_SIZE];
llConnState_t pConnContext[BLE_MAX_ALLOW_CONNECTION];
#define LARGE_HEAP_SIZE (4 * 1024)
_Alignas(4) uint8_t g_largeHeap[LARGE_HEAP_SIZE];
#endif

volatile uint8_t g_clk32K_config = CLK_32K_RCOSC;

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void SysTick_Handler(void);

static void init_config(void)
{
    int i;

    for (i = 0; i < 256; i++)
        global_config[i] = 0;

    // save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    global_config[INITIAL_STACK_PTR] = (uint32_t)(&g_irqstack_top);
    // LL switch setting
    global_config[LL_SWITCH] = /*LL_DEBUG_ALLOW |*/ SLAVE_LATENCY_ALLOW | LL_WHITELIST_ALLOW | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW;

    if (g_clk32K_config == CLK_32K_XTAL)
        global_config[LL_SWITCH] &= 0xffffffee;
    else
        global_config[LL_SWITCH] |= LL_RC32K_SEL | RC32_TRACKINK_ALLOW;

    // sleep delay
    global_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10; // 10ms, temporary set
    // system clock setting
    global_config[CLOCK_SETTING] = g_system_clk; // CLOCK_32MHZ;
    //------------------------------------------------------------------------
    // wakeup time cose
    // t1. HW_Wakeup->MCU relase 62.5us
    // t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
    // t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
    // t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
    // WAKEUP_ADVANCE should be larger than t1+t2+t3+t4
    //------------------------------------------------------------------------
    // wakeup advance time, in us
    global_config[WAKEUP_ADVANCE] = 1850; // 1850;//650;//600;//310;

    global_config[WAKEUP_DELAY] = 16; // 16;

    // sleep time, in us
    global_config[MAX_SLEEP_TIME] = 30000000;
    global_config[MIN_SLEEP_TIME] = 1600;
    global_config[ALLOW_TO_SLEEP_TICK_RC32K] = 55; // 30.5 per tick
    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    // LL engine settle time
    global_config[LL_HW_BB_DELAY] = 54; // 54-8;
    global_config[LL_HW_AFE_DELAY] = 8;
    global_config[LL_HW_PLL_DELAY] = 40; // sdk3.1.3 = 30//было 40;//45;//52;
    // Tx2Rx and Rx2Tx interval
    // Tx2Rx could be advanced a little
    // Rx2Tx should be ensure T_IFS within150us+-2us
    global_config[LL_HW_Rx_TO_TX_INTV] = 62 - RF_PHY_EXT_PREAMBLE_US;
    global_config[LL_HW_Tx_TO_RX_INTV] = 50; // 65
    //------------------------------------------------2MPHY
    // LL engine settle time
    global_config[LL_HW_BB_DELAY_2MPHY] = 59;
    global_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    global_config[LL_HW_PLL_DELAY_2MPHY] = 40; // 45;//52;
    // Tx2Rx and Rx2Tx interval
    // Tx2Rx could be advanced a little
    // Rx2Tx should be ensure T_IFS within150us+-2us
    global_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 73 - RF_PHY_EXT_PREAMBLE_US; // 20200822 ZQ
    global_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;                          // 72
    //------------------------------------------------CODEPHY 500K
    // LL engine settle time CODEPHY 500K
    global_config[LL_HW_BB_DELAY_500KPHY] = 50; // 54-8;
    global_config[LL_HW_AFE_DELAY_500KPHY] = 8;
    global_config[LL_HW_PLL_DELAY_500KPHY] = 40; // 45;//52;
    // Tx2Rx and Rx2Tx interval
    // Tx2Rx could be advanced a little
    // Rx2Tx should be ensure T_IFS within150us+-2us
    global_config[LL_HW_Rx_TO_TX_INTV_500KPHY] = 2;
    global_config[LL_HW_Tx_TO_RX_INTV_500KPHY] = 66; // 72
    //------------------------------------------------CODEPHY 125K
    // LL engine settle time CODEPHY 125K
    global_config[LL_HW_BB_DELAY_125KPHY] = 30; // 54-8;
    global_config[LL_HW_AFE_DELAY_125KPHY] = 8;
    global_config[LL_HW_PLL_DELAY_125KPHY] = 40; // 45;//52;
    // Tx2Rx and Rx2Tx interval
    // Tx2Rx could be advanced a little
    // Rx2Tx should be ensure T_IFS within150us+-2us
    global_config[LL_HW_Rx_TO_TX_INTV_125KPHY] = 5;  // sdk3.1.3 = 32
    global_config[LL_HW_Tx_TO_RX_INTV_125KPHY] = 66; // 72
    // LL engine settle time, for advertisement
    global_config[LL_HW_BB_DELAY_ADV] = 90;
    global_config[LL_HW_AFE_DELAY_ADV] = 8;
    global_config[LL_HW_PLL_DELAY_ADV] = 60;
    // adv channel interval
    global_config[ADV_CHANNEL_INTERVAL] = 1400;    // sdk3.1.3 = 1600 //было:1400;//6250;
    global_config[NON_ADV_CHANNEL_INTERVAL] = 666; // 6250;

    // conn_req -> slave connection event calibration time, will advance the receive window
    global_config[CONN_REQ_TO_SLAVE_DELAY] = 500; // было:300;//192;//500;//192;
    // calibration time for 2 connection event, will advance the next conn event receive window
    // SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
    global_config[SLAVE_CONN_DELAY] = 1500;            // было:300;//0;//1500;//0;//3000;//0;          ---> update 11-20
    global_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 500; // 160 NG//500 OK
    // RTLP timeout
    global_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
    global_config[LL_HW_RTLP_TO_GAP] = 1000;
    global_config[LL_HW_RTLP_1ST_TIMEOUT] = 2000 + global_config[SLAVE_CONN_DELAY] * 2; // 500;
    // direct adv interval configuration
    global_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    global_config[LDC_DIRECT_ADV_INTERVAL] = 6250;
    // A1 ROM metal change for HDC direct adv,
    global_config[DIR_ADV_DELAY] = 115; // in us, consider both direct adv broadcast time & SW delay, ... etc.
    // A1 ROM metal change
    global_config[LL_TX_PKTS_PER_CONN_EVT] = 6;    // 8;
    global_config[LL_RX_PKTS_PER_CONN_EVT] = 6;    // 8;
    global_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8; // 0:        disable adaptive
    // other:    adaptive max limitation
    //    global_config[LL_TX_PWR_TO_REG_BIAS]   = 0x15;   // assume when g_rfPhyTxPower = 0x1f, tx power = 10dBm
    // smart window configuration
    global_config[LL_SMART_WINDOW_COEF_ALPHA] = 2;
    global_config[LL_SMART_WINDOW_TARGET] = 600;
    global_config[LL_SMART_WINDOW_INCREMENT] = 9;
    global_config[LL_SMART_WINDOW_LIMIT] = 20000;
    global_config[LL_SMART_WINDOW_ACTIVE_THD] = 8;
    global_config[LL_SMART_WINDOW_ACTIVE_RANGE] = 0; // 300
    global_config[LL_SMART_WINDOW_FIRST_WINDOW] = 5000;
    extern uint32_t g_smartWindowSize;
    g_smartWindowSize = global_config[LL_HW_RTLP_1ST_TIMEOUT];

    //====== A2 metal change add, for scanner & initiator
    if (g_system_clk == SYS_CLK_XTAL_16M)
    {
        // scan req -> scan rsp timing
        global_config[SCAN_RSP_DELAY] = 13 + RF_PHY_EXT_PREAMBLE_US;           // 21;
        global_config[LL_ADV_TO_SCAN_REQ_DELAY] = 18 + RF_PHY_EXT_PREAMBLE_US; // 26;      //  2019/3/19 A2: 20 --> 18
        global_config[LL_ADV_TO_CONN_REQ_DELAY] = 25 + RF_PHY_EXT_PREAMBLE_US; // 33;      //  2019/3/19 A2: 27 --> 25
    }
    else if (g_system_clk == SYS_CLK_DBL_32M)
    {
        global_config[SCAN_RSP_DELAY] = 8 + RF_PHY_EXT_PREAMBLE_US;            // 16;
        global_config[LL_ADV_TO_SCAN_REQ_DELAY] = 12 + RF_PHY_EXT_PREAMBLE_US; //  2019/3/26 add
        global_config[LL_ADV_TO_CONN_REQ_DELAY] = 16 + RF_PHY_EXT_PREAMBLE_US;
    }
    else if (g_system_clk == SYS_CLK_DLL_48M)
    {
        // scan req -> scan rsp timing
        global_config[SCAN_RSP_DELAY] = 6 + RF_PHY_EXT_PREAMBLE_US;           // 20201207 set           //14;        // 12    //  2019/3/19 A2: 12 --> 9
        global_config[LL_ADV_TO_SCAN_REQ_DELAY] = 8 + RF_PHY_EXT_PREAMBLE_US; // 12;       //  2019/3/19 A2: 12 --> 10
        global_config[LL_ADV_TO_CONN_REQ_DELAY] = 11 + RF_PHY_EXT_PREAMBLE_US;
    }
    else if (g_system_clk == SYS_CLK_DLL_64M)
    {
        global_config[SCAN_RSP_DELAY] = 4 + RF_PHY_EXT_PREAMBLE_US;           // 2020.12.07 set         //12;
        global_config[LL_ADV_TO_SCAN_REQ_DELAY] = 6 + RF_PHY_EXT_PREAMBLE_US; //  2019/3/26 add
        global_config[LL_ADV_TO_CONN_REQ_DELAY] = 8 + RF_PHY_EXT_PREAMBLE_US;
    }

    // TRLP timeout
    global_config[LL_HW_TRLP_LOOP_TIMEOUT] = 50000; // enough for 8Tx + 8Rx : (41 * 8 + 150) * 16 - 150 = 7498us
    global_config[LL_HW_TRLP_TO_GAP] = 1000;
    global_config[LL_MOVE_TO_MASTER_DELAY] = 100;
    global_config[LL_CONN_REQ_WIN_SIZE] = 5;
    global_config[LL_CONN_REQ_WIN_OFFSET] = 2;
    global_config[LL_MASTER_PROCESS_TARGET] = 200;            // reserve time for preparing master conn event, delay should be insert if needn't so long time
    global_config[LL_MASTER_TIRQ_DELAY] = 0;                  // timer IRQ -> timer ISR delay
    global_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;            // 0.125us
    global_config[MAC_ADDRESS_LOC] = (uint32_t)ownPublicAddr; // 0x11001F00;
    // for simultaneous conn & adv/scan
    global_config[LL_NOCONN_ADV_EST_TIME] = 1400 * 3;
    global_config[LL_NOCONN_ADV_MARGIN] = 600;
    global_config[LL_SEC_SCAN_MARGIN] = 2500; // 1400;  to avoid mesh proxy llTrigErr 0x15
    global_config[LL_MIN_SCAN_TIME] = 2000;
    //  BBB new
    global_config[TIMER_ISR_ENTRY_TIME] = 30; // 15;
    global_config[LL_MULTICONN_MASTER_PREEMP] = 0;
    global_config[LL_MULTICONN_SLAVE_PREEMP] = 0;
    global_config[LL_EXT_ADV_TASK_DURATION] = 20000;
    global_config[LL_PRD_ADV_TASK_DURATION] = 20000;
    global_config[LL_CONN_TASK_DURATION] = 5000;
    global_config[LL_EXT_ADV_INTER_PRI_CHN_INT] = 5000;
    global_config[LL_EXT_ADV_INTER_SEC_CHN_INT] = 5000;
    global_config[LL_EXT_ADV_PRI_2_SEC_CHN_INT] = 1500;
    global_config[LL_EXT_ADV_RSC_PERIOD] = 1000000;
    global_config[LL_EXT_ADV_RSC_SLOT_DURATION] = 10000;
    global_config[LL_PRD_ADV_RSC_PERIOD] = 1000000;
    global_config[LL_PRD_ADV_RSC_SLOT_DURATION] = 10000;
    global_config[LL_EXT_ADV_PROCESS_TARGET] = 500;
    global_config[LL_PRD_ADV_PROCESS_TARGET] = 500;
}

static void Custom_SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState == 1)
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
#endif
        SysTick_Handler();
#if (INCLUDE_xTaskGetSchedulerState == 1)
    }
#endif
}

__attribute__((__used__)) static void _hard_fault(void *arg)
{
    uint32_t *stk = (uint32_t *)arg;
    dbg_printf("\n[Hard fault handler]\n");
    dbg_printf("R0   = 0x%08x\n", stk[9]);
    dbg_printf("R1   = 0x%08x\n", stk[10]);
    dbg_printf("R2   = 0x%08x\n", stk[11]);
    dbg_printf("R3   = 0x%08x\n", stk[12]);
    dbg_printf("R4   = 0x%08x\n", stk[1]);
    dbg_printf("R5   = 0x%08x\n", stk[2]);
    dbg_printf("R6   = 0x%08x\n", stk[3]);
    dbg_printf("R7   = 0x%08x\n", stk[4]);
    dbg_printf("R8   = 0x%08x\n", stk[5]);
    dbg_printf("R9   = 0x%08x\n", stk[6]);
    dbg_printf("R10  = 0x%08x\n", stk[7]);
    dbg_printf("R11  = 0x%08x\n", stk[8]);
    dbg_printf("R12  = 0x%08x\n", stk[13]);
    dbg_printf("SP   = 0x%08x\n", stk[0]);
    dbg_printf("LR   = 0x%08x\n", stk[14]);
    dbg_printf("PC   = 0x%08x\n", stk[15]);
    dbg_printf("PSR  = 0x%08x\n", stk[16]);
    dbg_printf("ICSR = 0x%08x\n", *(volatile uint32_t *)0xE000ED04);

    while (1)
        ;
}

/*
  This contraption is used so GCC doesn't complain about _hard_fault using the
  indices of the argument "arg" beyond 1.
 */
__attribute__((naked)) static void gcc_shut_warn(void *arg)
{
    UNUSED(arg);
    __asm volatile(
        "B _hard_fault\n");
}

__attribute__((naked)) static void Custom_HardFault_Handler(void)
{
    uint32_t arg = 0;
    gcc_shut_warn(&arg);
}

void Custom_enter_sleep_process(uint32_t time)
{
    LOG("ENTER_SLEEP_PROCESS called, but not implemented! Time: %u", time);
}

void Custom_enter_sleep_off_process(uint32_t mode)
{
    LOG("ENTER_SLEEP_OFF_MODE called, but not implemented! Mode: %u", mode);
}

void Custom_wakeup_process(void)
{
    /* This is rough, but oh well... */
    hal_system_soft_reset();
}

static int drv_disable_irq1(void)
{
    NVIC_DisableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(TIM4_IRQn) | BIT(BB_IRQn));
    /* TODO: maybe add a mutex here */
    m_in_critical_region++;
    return m_in_critical_region;
}

static int drv_enable_irq1(void)
{
    /* TODO: maybe add a mutex here */
    m_in_critical_region--;

    if (m_in_critical_region == 0)
    {
        NVIC_EnableIRQs(BIT(TIM1_IRQn) | BIT(TIM2_IRQn) | BIT(TIM4_IRQn) | BIT(BB_IRQn));
    }

    return m_in_critical_region;
}

#ifdef ENABLE_BTSTACK
//__ATTR_SECTION_SRAM__
static const pTaskEventHandlerFn tasksArr[] =
    {
        LL_ProcessEvent,
        HCI_ProcessEvent,
        // L2CAP_ProcessEvent,
        // SM_ProcessEvent,
        // GAP_ProcessEvent,
        // GATT_ProcessEvent,
        // GAPRole_ProcessEvent,
        // GATTServApp_ProcessEvent,
};

//__ATTR_SECTION_SRAM__
static const uint8_t tasksCnt = 2; //sizeof(tasksArr) / sizeof(tasksArr[0]);

//__ATTR_SECTION_SRAM__
static uint16_t tasksEvents[] = { 0 };

typedef struct
{
    uint8_t task_id;             // Task ID
    uint16_t events;             // Events for the task
    pTaskEventHandlerFn handler; // Event handler function
    const char *name;            // Task name (optional, can be NULL)
} fake_task_t;

fake_task_t osal_fake_tasks_stuff[] =
    {
        {255, 0, LL_ProcessEvent, "LL"},  // LL task ID is 0
        {255, 0, HCI_ProcessEvent, "HCI"} // HCI task ID is 1
};

__ATTR_SECTION_SRAM__
fake_task_t *osal_get_fake_task_by_id(uint8_t task_id)
{
    for (size_t i = 0; i < sizeof(osal_fake_tasks_stuff) / sizeof(osal_fake_tasks_stuff[0]); i++)
    {
        if (osal_fake_tasks_stuff[i].task_id == task_id)
        {
            return &osal_fake_tasks_stuff[i];
        }
    }
    return NULL;
}

__ATTR_SECTION_SRAM__
uint8_t Custom_osal_msg_send(uint8_t destination_task, uint8_t *msg_ptr)
{
    if (destination_task == hciTaskID)
    {
        /* Pass the HCI message to the BTstack */
        extern uint8_t pplus_ble_recv_msg(uint8_t destination_task, uint8_t *msg_ptr);
        pplus_ble_recv_msg(destination_task, msg_ptr);
        goto dealloc;
    }

    fake_task_t *t = osal_get_fake_task_by_id(destination_task);
    if (t == NULL)
    {
        LOG(" <<<<< send msg: destination_task %02X, msg_ptr %08X",
            destination_task, (uint32_t)msg_ptr);
        osal_msg_deallocate(msg_ptr);
        return PPlus_ERR_NO_MEM;
    }

    LOG(" <<<<< msg send: destination_task %02X (%s), msg_ptr %08X, len %02X, id %08X",
        destination_task, t->name ? t->name : "null", (uint32_t)msg_ptr, OSAL_MSG_LEN(msg_ptr), OSAL_MSG_ID(msg_ptr));

dealloc:
    // TODO!!!: hook here for HCI/BLE events
    osal_msg_deallocate(msg_ptr);
    return PPlus_SUCCESS;
}

__ATTR_SECTION_SRAM__
uint8_t Custom_osal_set_event(uint8_t task_id, uint16_t event_flag)
{
    if (task_id == hciTaskID)
    {
        if (event_flag & 0x8000)
        {
            uint8_t* msg = osal_msg_receive(hciTaskID);
            LOG("msg pointer %08x", msg);
            if (msg)
                Custom_osal_msg_send(hciTaskID, msg);
            return osal_set_event0(task_id, event_flag);
        }
    }
    /*fake_task_t *t = osal_get_fake_task_by_id(task_id);
    if (t == NULL)
    {
        LOG(" <<<<< set event: task_id %02X, event_flag %04X",
            task_id, event_flag);
        return PPlus_ERR_NO_MEM;
    }

    LOG(" <<<<< set event: task_id %02X (%s), event_flag %04X",
        task_id, t->name ? t->name : "null", event_flag);

    // TODO!!!: guard this with a mutex?
    t->events |= event_flag;*/

    return osal_set_event0(task_id, event_flag);

    return PPlus_ERR_NO_MEM;
}

extern volatile osalTimerRec_t *timerHead;

void osal_dump_timers(void)
{
    extern int OSAL_timeSeconds;
    if (timerHead == NULL)
    {
        LOG("(%dsec) OSAL timers dump: timerHead is NULL", OSAL_timeSeconds);
        return;
    }

    volatile osalTimerRec_t *result = timerHead;
    LOG("(%dsec) OSAL timers dump, timerHead = %08X", (uint32_t)timerHead, OSAL_timeSeconds);
    while (result != NULL)
    {
        LOG("Timer: %08X, next: %08X, task_id: %02X, event_flag: %04X, timeout: %u",
            (uint32_t)result, (uint32_t)result->next,
            result->task_id, result->event_flag, result->timeout);
        result = result->next;
    }
}

void osal_fake_timer(void *arg)
{
    (void)arg; // unused

    // osal_dump_timers();
    // osal_start_timerEx(0x69, 0x69, 1000); // start a timer for 1 second

    uint32_t bb_irq = 0;
    uint32_t count = 0;
    while (1)
    {
        osalTimeUpdate();
        if ((NVIC_GetEnableIRQs() & BIT(BB_IRQn)) != bb_irq)
        {
            LOG("BB_IRQ change detected: was %d, now %d", bb_irq, NVIC_GetEnableIRQs() & BIT(BB_IRQn));
            bb_irq = NVIC_GetEnableIRQs() & BIT(BB_IRQn);
        }

        extern volatile uint32_t ll_irq_numbers;
        if (count != ll_irq_numbers)
        {
            LOG("ll_irq_numbers change to %d", ll_irq_numbers);
            count = ll_irq_numbers;
        }
        // osal_dump_timers(); // TODO!!!: sleep the least amount of time for the set up timers, or wait for events

        for (int i = 0; i < sizeof(osal_fake_tasks_stuff) / sizeof(osal_fake_tasks_stuff[0]); i++)
        {
            uint16_t new_events = 0;
            if (osal_fake_tasks_stuff[i].handler)
            {
                uint16_t events = osal_fake_tasks_stuff[i].events;
                osal_fake_tasks_stuff[i].events = 0;
                // LOG("osal_fake_tasks_stuff[%d] handler %p, task_id %02X, events %04X",
                //     i, osal_fake_tasks_stuff[i].handler, *osal_fake_tasks_stuff[i].task_id, events);

                OSAL_current_task_id = osal_fake_tasks_stuff[i].task_id; // Set the current task ID
                new_events = osal_fake_tasks_stuff[i].handler(osal_fake_tasks_stuff[i].task_id, events);
                OSAL_current_task_id = -1; // Set the current task ID
            }
            osal_fake_tasks_stuff[i].events = new_events;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
        // osal_simulate_task_event(0);
        // osal_simulate_task_event(1);
        // extern int OSAL_timeSeconds;
        // LOG("tick OSAL_timeSeconds = %d", OSAL_timeSeconds);
    }
}
#endif

typedef enum
{
    EFUSE_BLOCK_0 = 0,
    EFUSE_BLOCK_1 = 1,
    EFUSE_BLOCK_2 = 2,
    EFUSE_BLOCK_3 = 3,
} EFUSE_block_t;

extern int efuse_read(EFUSE_block_t block,uint32_t* buf);

void efuse_init(void)
{
    write_reg(0x4000f054,0x0);
    write_reg(0x4000f140,0x0);
    write_reg(0x4000f144,0x0);
}

int _efuse_chip_version_check(void)
{
    uint32_t buf[2];
    // uint8_t key[16];
    for (int i = 0; i < 4; i++)
    {
        int v = efuse_read(i, buf);
        LOG("efuse read block %d = %d: %08X %08X", i, v, buf[0], buf[1]);
    }
    return PPlus_SUCCESS;
}

void _rom_sec_boot_init(void)
{
    efuse_init();

    if (_efuse_chip_version_check() == PPlus_SUCCESS)
    {
        typedef void (*my_function)(void);
        my_function pFunc = (my_function)(0xa2e1);
        // ble_main();
        pFunc();
        return;
    }
    else
    {
        while (1)
            ;
    }
}

static void hal_rfphy_init(void)
{
    // Watchdog_Init(NULL);
    //============config the txPower
    g_rfPhyTxPower = RF_PHY_TX_POWER_0DBM;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet = RF_PHY_FREQ_FOFF_00KHZ; //	hal_rfPhyFreqOff_Set();
    //============config xtal 16M cap
    XTAL16M_CAP_SETTING(0x09); 					//	hal_xtal16m_cap_Set();
    XTAL16M_CURRENT_SETTING(0x01);
    hal_rc32k_clk_tracking_init();
}

void osal_nuker_interrupt_init(void)
{
    /* Disable all interrupts */
    portDISABLE_INTERRUPTS();
    // NVIC->ICER[0] = 0xFFFFFFFF;

    JUMP_FUNCTION(HARDFAULT_HANDLER) = (uint32_t)&Custom_HardFault_Handler;
    LOG("New HardFault handler at %08x", JUMP_FUNCTION(HARDFAULT_HANDLER));

    JUMP_FUNCTION(ENTER_SLEEP_PROCESS) = (uint32_t)&Custom_enter_sleep_process;
    LOG("New ENTER_SLEEP_PROCESS at %08x", JUMP_FUNCTION(ENTER_SLEEP_PROCESS));

    JUMP_FUNCTION(ENTER_SLEEP_OFF_MODE) = (uint32_t)&Custom_enter_sleep_off_process;
    LOG("New ENTER_SLEEP_OFF_MODE at %08x", JUMP_FUNCTION(ENTER_SLEEP_OFF_MODE));

    JUMP_FUNCTION(WAKEUP_PROCESS) = (uint32_t)&Custom_wakeup_process;
    LOG("New WAKEUP_PROCESS at %08x", JUMP_FUNCTION(WAKEUP_PROCESS));

#ifdef ENABLE_FREERTOS
    /* FreeRTOS requires the SVC, PendSV and SysTick handlers routed to them */
    JUMP_FUNCTION(SVC_HANDLER) = (uint32_t)&vPortSVCHandler;
    LOG("New SVC handler at %08x", JUMP_FUNCTION(SVC_HANDLER));

    JUMP_FUNCTION(PENDSV_HANDLER) = (uint32_t)&xPortPendSVHandler;
    LOG("New PendSV handler at %08x", JUMP_FUNCTION(PENDSV_HANDLER));

    JUMP_FUNCTION(SYSTICK_HANDLER) = (uint32_t)&Custom_SysTick_Handler;
    LOG("New SysTick handler at %08x", JUMP_FUNCTION(SYSTICK_HANDLER));
#endif
    /* Patch out the disable/enable IRQ functions that OSAL used to use */
    JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE) = (uint32_t)&drv_disable_irq1;
    LOG("New HAL_DRV_IRQ_DISABLE at %08x", JUMP_FUNCTION(HAL_DRV_IRQ_DISABLE));

    JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE) = (uint32_t)&drv_enable_irq1;
    LOG("New HAL_DRV_IRQ_ENABLE at %08x", JUMP_FUNCTION(HAL_DRV_IRQ_ENABLE));

#ifdef ENABLE_BTSTACK
    //JUMP_FUNCTION(OSAL_MSG_SEND) = (uint32_t)&Custom_osal_msg_send;
    //LOG("New OSAL_MSG_SEND at now %08X", (uint32_t)&Custom_osal_msg_send);

    JUMP_FUNCTION(OSAL_SET_EVENT) = (uint32_t)&Custom_osal_set_event;
    LOG("New OSAL_SET_EVENT at %08X", (uint32_t)&Custom_osal_set_event);
#endif

    hal_clk_gate_enable(MOD_TIMER); /* systick */

    portENABLE_INTERRUPTS();
}

__ATTR_SECTION_SRAM__
void osal_nuker_init(sysclk_t clk, CLK32K_e rtcclk)
{
    // portDISABLE_INTERRUPTS();

    g_system_clk = clk;
    /* first of all, set the "m_in_critical_region" to zero as what drv_irq_init() would */
    m_in_critical_region = 0;

    clk_init(g_system_clk);
    g_clk32K_config = rtcclk;
    hal_rtc_clock_config((CLK32K_e)g_clk32K_config);

    spif_config(SYS_CLK_DLL_64M, 1, 0x801003b, 0, 0);
    hal_spif_cache_init(SYS_CLK_DLL_64M, XFRD_FCMD_READ_DUAL);

    LOG_INIT();
    LOG("Build time: %s %s", __DATE__, __TIME__);

    init_config();
    LOG("Config initialized");

#ifdef ENABLE_BTSTACK
    extern void init_patch(void);
    init_patch();
    LOG("Patch initialized");

    // extern void ll_patch_slave(void);
    // ll_patch_slave();
    // extern void ll_patch_master(void);
    // ll_patch_master();
    extern void ll_patch_multi(void);
    ll_patch_multi();
    LOG("LL patch slave initialized");
#endif

    hal_rfphy_init();
    LOG("RF PHY initialized");

    /*
     1. boot_init: AON stuff
     2. wakeup_init -> wakeup_init1(): clk_init, set_timer with TIM2 and 3, enable IRQ BB TIM1 TIM2 TIM4)
     3. rf_init -> nothing
     4. rf_calibrate -> rf_calibrate1()
     */
    _rom_sec_boot_init();
    LOG("ROM security boot initialized");

#ifdef ENABLE_BTSTACK
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
    LOG("OSAL memory heap set to %p, size %u", g_largeHeap, LARGE_HEAP_SIZE);

    LL_InitConnectContext(pConnContext,
                          g_pConnectionBuffer,
                          BLE_MAX_ALLOW_CONNECTION,
                          BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                          BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                          BLE_PKT_VERSION);

    /* so the osal_allocate and deallocate functions work */
    osal_init_system(); /* doesn't really do much */
    LOG("OSAL system initialized");
#endif

    /* init interrupt stuff related to what OSAL used */
    osal_nuker_interrupt_init();
    LOG("OSAL nuker interrupt initialized");

#ifdef ENABLE_BTSTACK
    ownPublicAddr[0] = 0xc0;
    ownPublicAddr[1] = 0xc0;
    ownPublicAddr[2] = 0xca;
    ownPublicAddr[3] = 0xfe;
    ownPublicAddr[4] = 0xca;
    ownPublicAddr[5] = 0xca;

    LOG("ownPublicAddr: %02X:%02X:%02X:%02X:%02X:%02X",
        ownPublicAddr[0], ownPublicAddr[1], ownPublicAddr[2],
        ownPublicAddr[3], ownPublicAddr[4], ownPublicAddr[5]);

    JUMP_FUNCTION(TASKS_ARRAY) = (uint32_t)tasksArr;
    JUMP_FUNCTION(TASK_COUNT) = (uint32_t)&tasksCnt;
    JUMP_FUNCTION(TASK_EVENTS) = (uint32_t)&tasksEvents;

    osal_fake_tasks_stuff[0].task_id = 0;
    LL_Init(osal_fake_tasks_stuff[0].task_id); // 0xc1 is the task ID for LL

    //osal_fake_tasks_stuff[1].task_id = 1;
    //HCI_Init(osal_fake_tasks_stuff[1].task_id);
    HCI_Init(1);

    xTaskCreate(
        osal_fake_timer,      /* Task function */
        "OSAL Fake Timer",    /* Task name */
        256,                  /* Stack size */
        NULL,                 /* Task parameters */
        tskIDLE_PRIORITY + 1, /* Task priority */
        NULL);                /* Task handle */

#endif
}
