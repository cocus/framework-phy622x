/*************
 gatt_internal.h
 SDK_LICENSE
***************/

#ifndef GATT_INTERNAL_H
#define GATT_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include <ble/include/gatt.h>
#include <osal/osal_cbtimer.h>


/*********************************************************************
    MACROS
*/
#define TIMER_VALID( id )                ( ( (id) != INVALID_TIMER_ID ) && \
                                           ( (id) != TIMEOUT_TIMER_ID ) )

#define TIMER_STATUS( id )               ( (id) == TIMEOUT_TIMER_ID ? bleTimeout : \
                                           (id) == INVALID_TIMER_ID ? SUCCESS : blePending )

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/
// Srtucture for Attribute Version Information attribute
typedef struct
{
    uint8_t attVersion;        // Attribute Protocol Version
    uint8_t gattVersion;       // Generic Attribute Profile Version
    uint16_t manufacturerName; // Manufacturer Name
} gattVersionInfo_t;

// Function prototype to parse an attribute protocol request message
typedef bStatus_t (*gattParseReq_t)( uint8_t sig, uint8_t cmd, uint8_t* pParams, uint16_t len, attMsg_t* pMsg );

// Function prototype to parse an attribute protocol response message
typedef bStatus_t (*gattParseRsp_t)( uint8_t* pParams, uint16_t len, attMsg_t* pMsg );

// Function prototype to process an attribute protocol message
typedef bStatus_t (*gattProcessMsg_t)( uint16_t connHandle,  attPacket_t* pPkt );

// Function prototype to process an attribute protocol request message
typedef bStatus_t (*gattProcessReq_t)( uint16_t connHandle,  attMsg_t* pMsg );

/*********************************************************************
    VARIABLES
*/
extern uint8_t gattTaskID;

/*********************************************************************
    FUNCTIONS
*/
extern void gattRegisterServer( gattProcessMsg_t pfnProcessMsg );

extern void gattRegisterClient( gattProcessMsg_t pfnProcessMsg );

extern bStatus_t gattNotifyEvent( uint8_t taskId, uint16_t connHandle, uint8_t status,
                                  uint8_t method, gattMsg_t* pMsg );

extern void gattStartTimer( pfnCbTimer_t pfnCbTimer, uint8_t* pData,
                            uint16_t timeout, uint8_t* pTimerId );

extern void gattStopTimer( uint8_t* pTimerId );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATT_INTERNAL_H */
