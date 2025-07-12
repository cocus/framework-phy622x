/**
 ****************************************************************************************

    @file jump_fucntion.h

    @brief This file contains the definitions of the macros and functions that are
    architecture dependent.  The implementation of those is implemented in the
    appropriate architecture directory.


    $Rev:  $

 SDK_LICENSE

 ****************************************************************************************
*/

#ifndef _JUMP_FUNC_H_
#define _JUMP_FUNC_H_

#include <stdint.h>

// =====================  MACROS =======================
#if 0
#define JUMP_BASE_ADDR 0x1fff0000
#define JUMP_FUNCTION(x) (*(uint32 *)(JUMP_BASE_ADDR + (x << 2)))
#else
extern const uint32_t *jump_table_base[];
#define JUMP_FUNCTION(x) (*(uint32_t *)(jump_table_base + x))
#endif

extern unsigned int g_irqstack_top;

// ROM function entries
enum jump_function_entries
{
    // 0 - 10 for common
    OSAL_INIT_TASKS = 1,
    TASKS_ARRAY = 2,
    TASK_COUNT = 3,
    TASK_EVENTS = 4,
    OSAL_MEM_INIT = 5,

    LL_INIT = 11,
    LL_PROCESS_EVENT = 12,
    LL_RESET = 13,
    LL_TXDATA = 14,
    LL_DISCONNECT = 15,
    LL_SET_ADV_PARAM = 16,
    LL_SET_ADV_DATA = 17,
    LL_SET_ADV_CONTROL = 18,
    LL_SET_DEFAULT_CONN_PARAM = 19,

    LL_EXT_SET_TX_POWER = 20,

    LL_CLEAR_WHITE_LIST = 21,
    LL_ADD_WHITE_LIST_DEV = 22,
    LL_REMOVE_WHITE_LIST_DEV = 23,
    LL_READ_WHITE_LIST_SIZE = 24,
    LL_NUM_EMPTY_WL_ENTRIES = 25,

    LL_SLAVE_EVT_ENDOK = 26,
    LL_SETUP_NEXT_SLAVE_EVT = 27,
    LL_CHK_LSTO_DURING_SL = 28,
    LL_PROCESS_SLAVE_CTRL_PROC = 29,

    LL_PROCESS_SLAVE_CTRL_PKT = 30,
    LL_SLAVE_EVT_ABORT = 31,
    LL_PROCESS_RX_DATA = 32,
    LL_PROCESS_TX_DATA = 33,
    LL_CONN_TERMINATE = 34,
    LL_WRITE_TX_DATA = 35,

    LL_EVT_SCHEDULE = 36,
    LL_MOVE_TO_SLAVE_FUNCTION = 37,
    LL_SLAVE_CONN_EVENT = 38,

    LL_SETUP_ADV = 39,

    LL_SETUP_UNDIRECT_ADV = 40,
    LL_SETUP_NOCONN_ADV = 41,

    LL_SETUP_SCAN_ADV = 42,
    LL_SETUP_DIRECT_ADV = 43,

    LL_CALC_TIMER_DRIFT = 44,
    LL_GENERATE_TX_BUFFER = 45,
    LL_READ_RX_FIFO = 46,
    LL_READ_TX_FIFO_RTLP = 47,
    LL_READ_TX_FIFO_PKT = 48,

    LL_HW_PROCESS_RTO = 49,

    LL_HW_SET_TIMING = 50,
    LL_RELEASE_CONN_ID = 51,

    LL_READ_TX_PWR_LVL = 52,          //  A1 ROM metal change add
    LL_READ_ADV_TX_PWR_LVL = 53,      //  A1 ROM metal change add
    LL_READ_RSSI = 54,                //  A1 ROM metal change add
    LL_READ_REMOTE_USE_FEATURES = 55, //  A1 ROM metal change add
    LL_ENCRYPT = 56,                  //  A1 ROM metal change add

    LL_DIRECT_TEST_END = 57,     //  A1 ROM metal change add
    LL_DIRECT_TEST_TX_TEST = 58, //  A1 ROM metal change add
    LL_DIRECT_TEST_RX_TEST = 59, //  A1 ROM metal change add

    OSAL_POWER_CONSERVE = 60,
    ENTER_SLEEP_PROCESS = 61,
    WAKEUP_PROCESS = 62,
    CONFIG_RTC = 63,
    ENTER_SLEEP_OFF_MODE = 64, //  A1 ROM metal change add

    HAL_PROCESS_POLL = 65,  //  A1 ROM metal change add
    LL_HW_GO = 66,          //  A1 ROM metal change add
    LL_HW_TRIGGER = 67,     //  A1 ROM metal change add
    LL_SET_TX_PWR_LVL = 68, //  A1 ROM metal change add

    // LL AES
    LL_AES128_ENCRYPT = 70,  //  A1 ROM metal change add
    LL_GEN_TRUE_RANDOM = 71, //  A1 ROM metal change add
    LL_GEN_DEVICE_SKD = 72,  //  A1 ROM metal change add
    LL_GEN_DEVICE_IV = 73,   //  A1 ROM metal change add
    LL_GENERATE_NOUNCE = 74, //  A1 ROM metal change add
    LL_ENC_ENCRYPT = 75,     //  A1 ROM metal change add
    LL_ENC_DECRYPT = 76,     //  A1 ROM metal change add

    // host entries
    SMP_INIT = 80,
    SMP_PROCESS_EVENT = 81,

    // l2cap entries
    L2CAP_PARSE_PACKET = 82,
    L2CAP_ENCAP_PACKET = 83,
    L2CAP_PKT_TO_SEGBUFF = 84,
    L2CAP_SEGBUFF_TO_LINKLAYER = 85,
    L2CAP_PROCESS_FREGMENT_TX_DATA = 86,

    // gap linkmgr entries
    GAP_LINK_MGR_PROCESS_CONNECT_EVT = 87,
    GAP_LINK_MGR_PROCESS_DISCONNECT_EVT = 88,

    // hci tl
    HCI_INIT = 90,          //  A1 ROM metal change add
    HCI_PROCESS_EVENT = 91, //  A1 ROM metal change add

    // app entries
    APP_SLEEP_PROCESS = 100,
    APP_WAKEUP_PROCESS = 101,
    RF_INIT = 102,
    WAKEUP_INIT = 103,
    BOOT_INIT = 104,
    DEBUG_PRINT = 105,
    RF_CALIBRATTE = 106, //  A1 ROM metal change add
    RF_PHY_CHANGE = 107, //  A1 ROM metal change add

    // LL master, A2 ROM metal change add
    LL_MASTER_EVT_ENDOK = 110,
    LL_SETUP_NEXT_MASTER_EVT = 111,
    LL_PROCESS_MASTER_CTRL_PROC = 112,
    LL_PROCESS_MASTER_CTRL_PKT = 113,
    LL_MOVE_TO_MASTER_FUNCTION = 114,
    LL_MASTER_CONN_EVENT = 115,

    LL_SET_SCAN_CTRL = 116,
    LL_SET_SCAN_PARAM = 117,

    LL_CREATE_CONN = 118,
    LL_CREATE_CONN_CANCEL = 119,

    LL_START_ENCRYPT = 120,

    LL_SETUP_SCAN = 121,

    LL_SETUP_SEC_NOCONN_ADV = 122,
    LL_SETUP_SEC_SCAN = 123,
    LL_SEC_ADV_ALLOW = 124,
    LL_CALC_MAX_SCAN_TIME = 125,

    // A2 multi-connection
    LL_SETUP_SEC_ADV_ENTRY = 126,
    LL_SETUP_SEC_CONN_ADV = 127,
    LL_SETUP_SEC_SCANNABLE_ADV = 128,

    // DLE
    LL_SET_DATA_LENGTH = 130,
    LL_PDU_LENGTH_UPDATE = 131,
    LL_TRX_NUM_ADJUST = 132,
    // PHY UPDATE
    LL_SET_PHY_MODE = 133,
    LL_PHY_MODE_UPDATE = 134,
    LL_SET_NEXT_PHY_MODE = 135,

    LL_ADP_ADJ_NEXT_TIME = 136,
    LL_ADP_SMART_WINDOW = 137,
    LL_SET_NEXT_DATA_CHN = 138,
    LL_PLUS_DISABLE_LATENCY = 139,
    LL_PLUS_ENABLE_LATENCY = 140,

    LL_SETUP_EXT_ADV_EVENT = 141,
    LL_SETUP_PRD_ADV_EVENT = 142,
    LL_SETUP_ADV_EXT_IND_PDU = 143,
    LL_SETUP_AUX_ADV_IND_PDU = 144,
    LL_SETUP_AUX_SYNC_IND_PDU = 145,
    LL_SETUP_AUX_CHAIN_IND_PDU = 146,
    LL_SETUP_AUX_CONN_REQ_PDU = 147,
    LL_SETUP_AUX_CONN_RSP_PDU = 148,

    LL_SCHEDULER = 149,
    LL_ADD_TASK = 150,
    LL_DEL_TASK = 151,

    LL_ADV_SCHEDULER = 152,
    LL_ADV_ADD_TASK = 153,
    LL_ADV_DEL_TASK = 154,

    LL_ADV_SCHEDULER_PRD = 155,
    LL_ADV_ADD_TASK_PRD = 156,
    LL_ADV_DEL_TASK_PRD = 157,

    LL_GET_NEXT_AUX_CHN = 158,
    LL_SETUP_AUX_SCAN_RSP_PDU = 159,

    LL_PROCESSBASICIRQ_SRX = 160,
    LL_PROCESSBASICIRQ_SECADVTRX = 161,
    LL_PROCESSBASICIRQ_SCANTRX = 162,
    LL_PROCESSBASICIRQ_SECSCANSRX = 163,
    LL_PROCESSBASICIRQ_SECINITSRX = 164,

    // 2020-02-13 Add for CTE
    LL_CONNLESS_CTE_TX_PARAM = 203,
    LL_CONNLESS_CTE_TX_ENABLE = 204,
    LL_CONNLESS_IQ_SAMPLE_ENABLE = 205,
    LL_CONN_CTE_RECV_PARAM = 206,
    LL_CONN_CTE_REQ_EN = 207,
    LL_CONN_CTE_TX_PARAM = 208,
    LL_CONN_CTE_RSP_EN = 209,

    // OSAL
    OSAL_SET_EVENT = 210,
    OSAL_MSG_SEND = 211,
    HAL_DRV_IRQ_INIT = 212,
    HAL_DRV_IRQ_ENABLE = 213,
    HAL_DRV_IRQ_DISABLE = 214,

    HAL_WATCHDOG_INIT = 215,

    // interrupt request handler
    NMI_HANDLER = 219,
    HARDFAULT_HANDLER = 220,
    SVC_HANDLER = 221,
    PENDSV_HANDLER = 222,
    SYSTICK_HANDLER = 223,

    V0_IRQ_HANDLER = 224,
    V1_IRQ_HANDLER = 225,
    V2_IRQ_HANDLER = 226,
    V3_IRQ_HANDLER = 227,

    V4_IRQ_HANDLER = 228,
    BB_IRQ_HANDLER = V4_IRQ_HANDLER,

    V5_IRQ_HANDLER = 229,
    KSCAN_IRQ_HANDLER = V5_IRQ_HANDLER,

    V6_IRQ_HANDLER = 230,
    RTC_IRQ_HANDLER = V6_IRQ_HANDLER,

    V7_IRQ_HANDLER = 231,
    CP_COM_IRQ_HANDLER = V7_IRQ_HANDLER,

    V8_IRQ_HANDLER = 232,
    AP_COM_IRQ_HANDLER = V8_IRQ_HANDLER,

    V9_IRQ_HANDLER = 233,
    V10_IRQ_HANDLER = 234,
    WDT_IRQ_HANDLER = V10_IRQ_HANDLER,

    V11_IRQ_HANDLER = 235,
    UART0_IRQ_HANDLER = V11_IRQ_HANDLER,

    V12_IRQ_HANDLER = 236,
    I2C0_IRQ_HANDLER = V12_IRQ_HANDLER,

    V13_IRQ_HANDLER = 237,
    I2C1_IRQ_HANDLER = V13_IRQ_HANDLER,

    V14_IRQ_HANDLER = 238,
    SPI0_IRQ_HANDLER = V14_IRQ_HANDLER,

    V15_IRQ_HANDLER = 239,
    SPI1_IRQ_HANDLER = V15_IRQ_HANDLER,

    V16_IRQ_HANDLER = 240,
    GPIO_IRQ_HANDLER = V16_IRQ_HANDLER,

    V17_IRQ_HANDLER = 241,
    UART1_IRQ_HANDLER = V17_IRQ_HANDLER,

    V18_IRQ_HANDLER = 242,
    SPIF_IRQ_HANDLER = V18_IRQ_HANDLER,

    V19_IRQ_HANDLER = 243,
    DMAC_IRQ_HANDLER = V19_IRQ_HANDLER,

    V20_IRQ_HANDLER = 244,
    TIM1_IRQ_HANDLER = V20_IRQ_HANDLER,

    V21_IRQ_HANDLER = 245,
    TIM2_IRQ_HANDLER = V21_IRQ_HANDLER,

    V22_IRQ_HANDLER = 246,
    TIM3_IRQ_HANDLER = V22_IRQ_HANDLER,

    V23_IRQ_HANDLER = 247,
    TIM4_IRQ_HANDLER = V23_IRQ_HANDLER,

    V24_IRQ_HANDLER = 248,
    TIM5_IRQ_HANDLER = V24_IRQ_HANDLER,

    V25_IRQ_HANDLER = 249,
    TIM6_IRQ_HANDLER = V25_IRQ_HANDLER,

    V26_IRQ_HANDLER = 250,
    V27_IRQ_HANDLER = 251,

    V28_IRQ_HANDLER = 252,
    AES_IRQ_HANDLER = V28_IRQ_HANDLER,

    V29_IRQ_HANDLER = 253,
    ADCC_IRQ_HANDLER = V29_IRQ_HANDLER,

    V30_IRQ_HANDLER = 254,
    QDEC_IRQ_HANDLER = V30_IRQ_HANDLER,

    V31_IRQ_HANDLER = 255,
    RNG_IRQ_HANDLER = V31_IRQ_HANDLER,
};




#endif // _JUMP_FUNC_H_
