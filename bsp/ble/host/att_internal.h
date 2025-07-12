/*************
 att_internal.h
 SDK_LICENSE
***************/

#ifndef ATT_INTERNAL_H
#define ATT_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include <ble/include/l2cap.h>
#include <ble/include/att.h>

/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

// Function prototype to build an attribute protocol message
typedef uint16_t (*attBuildMsg_t)( uint8_t* pBuf, uint8_t* pMsg );

/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

extern uint16_t attBuildExecuteWriteRsp( uint8_t* pBuf, uint8_t* pMsg );

extern uint16_t attBuildHandleValueCfm( uint8_t* pBuf, uint8_t* pMsg );

extern bStatus_t attSendMsg( uint16_t connHandle, attBuildMsg_t pfnBuildMsg,
                             uint8_t opcode, uint8_t* pMsg );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ATT_INTERNAL_H */
