/*************
 l2cap_internal.h
 SDK_LICENSE
***************/

#ifndef L2CAP_INTERNAL_H
#define L2CAP_INTERNAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
//#include <ble/include/hci.h>
#include <ble/include/l2cap.h>

/*********************************************************************
    MACROS
*/

// Macro to see if a given channel is a fixed channel
#define FIX_CHANNEL( CID )        ( (CID) == L2CAP_CID_GENERIC ||\
                                    (CID) == L2CAP_CID_SIG ||\
                                    (CID) == L2CAP_CID_ATT ||\
                                    (CID) == L2CAP_CID_SMP )

// Marco to convert a channel ID to an index into L2CAP Channel table
#define CID_TO_INDEX( CID )       ( (CID) - BASE_DYNAMIC_CID )

// Marco to convert a fixed channel ID to an index into L2CAP Fixed Channel table
#define FCID_TO_INDEX( CID )      ( (CID) - L2CAP_CID_ATT )

// Macro to return the record maintained a given fix channel
#define FIX_CHANNEL_REC( CID )    ( l2capFixedChannels[FCID_TO_INDEX( CID )] )

/*********************************************************************
    CONSTANTS
*/
// Signaling command header: Code (1 byte) + Identifier (1 byte) + Length (2 bytes)
#define SIGNAL_HDR_SIZE                            4

// Maximum size of data field of Signaling commands
#define SIGNAL_DATA_SIZE                           ( L2CAP_SIG_MTU_SIZE - SIGNAL_HDR_SIZE )

/*********************************************************************
    L2CAP Channel States: states used for l2capChannel 'state' field
*/
// Closed - no channel associated with this CID
#define L2CAP_CLOSED                               0x00

// Waiting for Echo Response
#define L2CAP_W4_ECHO_RSP                          0x01

// Waiting for Info Response
#define L2CAP_W4_INFO_RSP                          0x02

// Waiting for Connection Parameter Update Response
#define L2CAP_W4_PARAM_UPDATE_RSP                  0x03

/*********************************************************************
    TYPEDEFS
*/

// L2CAP Channel structure. Allocated one per application connection
// between two devices. CID assignment is relative to a particular device
// and a device can assign CIDs independently from other devices (except
// for the reserved CIDs). The CIDs are dynamically allocated in the range
// from 0x0040 to 0xFFFF.
typedef struct
{
    // Channel info
    uint8_t  state; // Channel connection state
    uint16_t CID;   // Local channel id
    uint8_t  id;    // Local identifier - matches responses with requests

    // Link info
    uint16_t connHandle; // link connection handle

    // Application info
    uint8_t taskId; // task that channel belongs to

    // Timer id
    uint8_t timerId;
} l2capChannel_t;

// L2CAP Fixed Channel structure. Allocated one for each fixed channel.
typedef struct
{
    uint16_t CID;   // channel id
    uint8_t taskId; // task registered with channel
} l2capFixedChannel_t;

// Signaling packet header format
typedef struct
{
    uint8_t opcode; // type of command
    uint8_t id;     // identifier - matches responses with requests
    uint16_t len;   // length of data field (doesn't cover Code, Identifier and Length fields)
} l2capSignalHdr_t;

/**
    @brief   Callback function prototype for building a Signaling command.

    @param   pBuf - pointer to buffer to hold command data
    @param   pData - pointer to command data

    @return  length of the command data
*/
typedef uint16_t (*pfnL2CAPBuildCmd_t)( uint8_t* pBuf, uint8_t* pData );

/*********************************************************************
    GLOBAL VARIABLES
*/
extern uint8_t l2capTaskID;
extern l2capChannel_t l2capChannels[];
extern l2capFixedChannel_t l2capFixedChannels[];

/*********************************************************************
    FUNCTIONS - API
*/

/*
    Send L2CAP Command.
*/
extern bStatus_t l2capSendCmd( uint16_t connHandle, uint8_t opcode, uint8_t id,
                               uint8_t* pCmd, pfnL2CAPBuildCmd_t pfnBuildCmd );
/*
    Send L2CAP Request.
*/
extern bStatus_t l2capSendReq( uint16_t connHandle, uint8_t opcode, uint8_t* pReq,
                               pfnL2CAPBuildCmd_t pfnBuildCmd, uint8_t state, uint8_t taskId );
/*
    Build Echo Request.
*/
extern uint16_t l2capBuildEchoReq( uint8_t* pBuf, uint8_t* pCmd );

/*
    Build Info Request.
*/
extern uint16_t l2capBuildInfoReq( uint8_t* pBuf, uint8_t* pCmd );

/*
    Build Parameter Update Request.
*/
extern uint16_t l2capBuildParamUpdateReq( uint8_t* pBuf, uint8_t* pData );

/*
    Encapsulate and send L2CAP packet.
*/
extern bStatus_t l2capEncapSendData( uint16_t connHandle, l2capPacket_t* pPkt );

/*
    Parse L2CAP packet.
*/
extern uint8_t l2capParsePacket( l2capPacket_t* pPkt, hciDataEvent_t* pHciMsg );

/*
    Parse L2CAP Signaling header.
*/
extern void l2capParseSignalHdr( l2capSignalHdr_t* pHdr, uint8_t* pData );

/*
    Build Echo Response.
*/
extern uint16_t l2capBuildEchoRsp( uint8_t* pBuf, uint8_t* pCmd );

/*
    Parse Command Reject.
*/
extern bStatus_t l2capParseCmdReject( l2capSignalCmd_t* pCmd, uint8_t* pData, uint16_t len );

/*
    Parse Echo Response.
*/
extern bStatus_t l2capParseEchoRsp( l2capSignalCmd_t* pCmd, uint8_t* pData, uint16_t len );

/*
    Parse Information Response.
*/
extern bStatus_t l2capParseInfoRsp( l2capSignalCmd_t* pCmd, uint8_t* pData, uint16_t len );

/*
    Parse Connection Parameter Update Response.
*/
extern bStatus_t l2capParseParamUpdateRsp( l2capSignalCmd_t* pCmd, uint8_t* pData, uint16_t len );

/*
    Find a channel using the local identifier.
*/
extern l2capChannel_t* l2capFindLocalId( uint8_t id );

/*
    Free a channel.
*/
extern void l2capFreeChannel( l2capChannel_t* pChannel );

/*
    Stop an active timer for a given channel.
*/
extern void l2capStopTimer( l2capChannel_t* pChannel );

/*
    Handle an incoming packet error.
*/
extern void l2capHandleRxError( uint16_t connHandle );

/*
    Forward a data message to upper layer application.
*/
extern bStatus_t l2capNotifyData( uint8_t taskId, uint16_t connHandle, l2capPacket_t* pPkt );

/*
    Send a Signaling command to upper layer application.
*/
extern void l2capNotifySignal( uint8_t taskId, uint16_t connHandle, uint8_t status,
                               uint8_t opcode, uint8_t id, l2capSignalCmd_t* pCmd );

extern void* L2CAP_Fragment_bm_alloc( uint16_t size );

extern uint8_t L2CAP_Fragment_SendDataPkt( uint16_t connHandle, uint8_t fragFlg,uint16_t pktLen, uint8_t* pBuf );


/*********************************************************************
    @fn      l2capInfoRsp

    @brief   Send Info Response.

            Use like: l2capInfoRsp( uint16_t connHandle, uint8_t id, l2capInfoRsp_t *pInfoRsp );

    @param   connHandle - connection to use
    @param   id - identifier received in request
    @param   pInfoRsp - pointer to Info Response to be sent

    @return  SUCCESS: Request was sent successfully.
            INVALIDPARAMETER: Data can not fit into one packet.
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
            bleNotConnected: Connection is down.
            bleMemAllocError: Memory allocation error occurred.
*/
#define l2capInfoRsp( connHandle, id, pInfoRsp )  l2capSendCmd( (connHandle), L2CAP_INFO_RSP, (id),\
                                                                (uint8_t *)(pInfoRsp), L2CAP_BuildInfoRsp )

/*********************************************************************
    @fn      l2capEchoRsp

    @brief   Send Ehco Response.

            Use like: l2capEchoRsp( uint16_t connHandle, uint8_t id, l2capEchoRsp_t *pEchoRsp );

    @param   connHandle - connection to use
    @param   id - identifier received in request
    @param   pEchoRsp - pinter to Echo Response to be sent

    @return  SUCCESS: Request was sent successfully.
            INVALIDPARAMETER: Data can not fit into one packet.
            MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
            bleNotConnected: Connection is down.
            bleMemAllocError: Memory allocation error occurred.
*/
#define l2capEchoRsp( connHandle, id, pEchoRsp )  l2capSendCmd( (connHandle), L2CAP_ECHO_RSP, (id),\
                                                                (uint8_t *)(pEchoRsp), l2capBuildEchoRsp )


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* L2CAP_INTERNAL_H */
