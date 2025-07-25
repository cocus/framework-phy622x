
/**
  @headerfile:       gattservapp.h
  $Date: 
  $Revision: 

  @mainpage BLE GATT Server Application API

  Description:    This file contains the GATT Server Application (GATTServApp)
                  definitions and prototypes.<BR><BR>
 
  SDK_LICENSE

**************************************************************************************************/

#ifndef GATTSERVAPP_H
#define GATTSERVAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ble/include/gatt.h>
/*********************************************************************
 * CONSTANTS
 */

/** @defgroup GATT_SERV_MSG_EVENT_DEFINES GATT Server Message IDs
 * @{
 */

#define GATT_CLIENT_CHAR_CFG_UPDATED_EVENT  0x00 //!< Sent when a Client Characteristic Configuration is updated.  This event is sent as an OSAL message defined as gattCharCfgUpdatedEvent_t.

/** @} End GATT_SERV_MSG_EVENT_DEFINES */


/** @defgroup GATT_PROP_BITMAPS_DEFINES GATT Characteristic Properties Bit Fields
 * @{
 */

#define GATT_PROP_BCAST                  0x01 //!< Permits broadcasts of the Characteristic Value
#define GATT_PROP_READ                   0x02 //!< Permits reads of the Characteristic Value
#define GATT_PROP_WRITE_NO_RSP           0x04 //!< Permits writes of the Characteristic Value without response
#define GATT_PROP_WRITE                  0x08 //!< Permits writes of the Characteristic Value with response
#define GATT_PROP_NOTIFY                 0x10 //!< Permits notifications of a Characteristic Value without acknowledgement
#define GATT_PROP_INDICATE               0x20 //!< Permits indications of a Characteristic Value with acknowledgement
#define GATT_PROP_AUTHEN                 0x40 //!< Permits signed writes to the Characteristic Value
#define GATT_PROP_EXTENDED               0x80 //!< Additional characteristic properties are defined in the Characteristic Extended Properties Descriptor

/** @} End GATT_PROP_BITMAPS_DEFINES */

/** @defgroup GATT_EXT_PROP_BITMAPS_DEFINES GATT Characteristic Extended Properties Bit Fields
 * @{
 */

#define GATT_EXT_PROP_RELIABLE_WRITE     0x0001 //!< Permits reliable writes of the Characteristic Value
#define GATT_EXT_PROP_WRITABLE_AUX       0x0002 //!< Permits writes to the characteristic descriptor

/** @} End GATT_EXT_PROP_BITMAPS_DEFINES */

/** @defgroup GATT_CLIENT_CFG_BITMAPS_DEFINES GATT Client Characteristic Configuration Bit Fields
 * @{
 */

#define GATT_CLIENT_CFG_NOTIFY           0x0001 //!< The Characteristic Value shall be notified
#define GATT_CLIENT_CFG_INDICATE         0x0002 //!< The Characteristic Value shall be indicated

/** @} End GATT_CLIENT_CFG_BITMAPS_DEFINES */

/** @defgroup GATT_SERV_CFG_BITMAPS_DEFINES GATT Server Characteristic Configuration Bit Fields
 * @{
 */

#define GATT_SERV_CFG_BCAST              0x0001 //!< The Characteristic Value shall be broadcast when the server is in the broadcast procedure if advertising data resources are available

/** @} End GATT_SERV_CFG_BITMAPS_DEFINES */

#define GATT_CFG_NO_OPERATION            0x0000 // No operation

/** @defgroup GATT_FORMAT_TYPES_DEFINES GATT Characteristic Format Types
 * @{
 */

#define GATT_FORMAT_BOOL                 0x01 //!< Unsigned 1 bit; 0 = false, 1 = true
#define GATT_FORMAT_2BIT                 0x02 //!< Unsigned 2 bit integer
#define GATT_FORMAT_NIBBLE               0x03 //!< Unsigned 4 bit integer
#define GATT_FORMAT_UINT8                0x04 //!< Unsigned 8 bit integer
#define GATT_FORMAT_UINT12               0x05 //!< Unsigned 12 bit integer
#define GATT_FORMAT_UINT16               0x06 //!< Unsigned 16 bit integer
#define GATT_FORMAT_UINT24               0x07 //!< Unsigned 24 bit integer
#define GATT_FORMAT_UINT32               0x08 //!< Unsigned 32 bit integer
#define GATT_FORMAT_UINT48               0x09 //!< Unsigned 48 bit integer
#define GATT_FORMAT_UINT64               0x0a //!< Unsigned 64 bit integer
#define GATT_FORMAT_UINT128              0x0b //!< Unsigned 128 bit integer
#define GATT_FORMAT_SINT8                0x0c //!< Signed 8 bit integer
#define GATT_FORMAT_SINT12               0x0d //!< Signed 12 bit integer
#define GATT_FORMAT_SINT16               0x0e //!< Signed 16 bit integer
#define GATT_FORMAT_SINT24               0x0f //!< Signed 24 bit integer
#define GATT_FORMAT_SINT32               0x10 //!< Signed 32 bit integer
#define GATT_FORMAT_SINT48               0x11 //!< Signed 48 bit integer
#define GATT_FORMAT_SINT64               0x12 //!< Signed 64 bit integer
#define GATT_FORMAT_SINT128              0x13 //!< Signed 128 bit integer
#define GATT_FORMAT_FLOAT32              0x14 //!< IEEE-754 32 bit floating point
#define GATT_FORMAT_FLOAT64              0x15 //!< IEEE-754 64 bit floating point
#define GATT_FORMAT_SFLOAT               0x16 //!< IEEE-11073 16 bit SFLOAT
#define GATT_FORMAT_FLOAT                0x17 //!< IEEE-11073 32 bit FLOAT
#define GATT_FORMAT_DUINT16              0x18 //!< IEEE-20601 format
#define GATT_FORMAT_UTF8S                0x19 //!< UTF-8 string
#define GATT_FORMAT_UTF16S               0x1a //!< UTF-16 string
#define GATT_FORMAT_STRUCT               0x1b //!< Opaque structure

/** @} End GATT_FORMAT_TYPES_DEFINES */

/** @defgroup GATT_NS_TYPES_DEFINES GATT Namespace Types
 * @{
 */

#define GATT_NS_NONE                     0x00 //!< No namespace
#define GATT_NS_BT_SIG                   0x01 //!< Bluetooth SIG namespace

/** @} End GATT_NS_TYPES_DEFINES */

/** @defgroup GATT_NS_BT_DESC_DEFINES GATT Bluetooth Namespace Descriptions
 * @{
 */

#define GATT_NS_BT_DESC_UNKNOWN          0x0000 //!< The description is unknown

/** @} End GATT_NS_BT_DESC_DEFINES */

// All profile services bit fields
#define GATT_ALL_SERVICES                0xFFFFFFFF

// GATT Services bit fields
#define GATT_SERVICE                     0x00000001

#if defined ( TESTMODES )
  // GATT Test Modes
  #define GATT_TESTMODE_OFF              0 // Test mode off
  #define GATT_TESTMODE_NO_RSP           1 // Ignore incoming request
  #define GATT_TESTMODE_PREPARE_WRITE    2 // Forward Prepare Write Request right away
  #define GATT_TESTMODE_MAX_MTU_SIZE     3 // Use Max ATT MTU size with Exchange MTU Rsp
  #define GATT_TESTMODE_CORRUPT_PW_DATA  4 // Corrupt incoming Prepare Write Request data
#endif

// GATT Server Parameters
#define GATT_PARAM_NUM_PREPARE_WRITES    0 // RW  uint8_t

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * MACROS
 */

// The number of attribute records in a given attribute table
#define GATT_NUM_ATTRS( attrs )          ( sizeof( attrs ) / sizeof( gattAttribute_t ) )

// The handle of a service is the handle of the first attribute
#define GATT_SERVICE_HANDLE( attrs )     ( (attrs)[0].handle )

// The handle of the first included service (i = 1) is the value of the second attribute
#define GATT_INCLUDED_HANDLE( attrs, i ) ( *((uint16_t *)((attrs)[(i)].pValue)) )

/*********************************************************************
 * TYPEDEFS
 */

/**
 * @defgroup GATT_SERV_APP_CB_API GATT Server App Callback API Functions
 *
 * @{
 */

/**
 * @brief   Callback function prototype to read an attribute value.
 *
 * @param   connHandle - connection request was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be read (to be returned)
 * @param   pLen - length of data (to be returned)
 * @param   offset - offset of the first octet to be read
 * @param   maxLen - maximum length of data to be read
 *
 * @return  SUCCESS: Read was successfully.<BR>
 *          Error, otherwise: ref ATT_ERR_CODE_DEFINES.<BR>
 */
typedef bStatus_t (*pfnGATTReadAttrCB_t)( uint16_t connHandle, gattAttribute_t* pAttr,
                                          uint8_t* pValue, uint16_t* pLen, uint16_t offset,
                                          uint8_t maxLen );
/**
 * @brief   Callback function prototype to write an attribute value.
 *
 * @param   connHandle - connection request was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   pLen - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  SUCCESS: Write was successfully.<BR>
 *          Error, otherwise: ref ATT_ERR_CODE_DEFINES.<BR>
 */
typedef bStatus_t (*pfnGATTWriteAttrCB_t)( uint16_t connHandle, gattAttribute_t* pAttr,
                                           uint8_t* pValue, uint16_t len, uint16_t offset );
/**
 * @brief   Callback function prototype to authorize a Read or Write operation
 *          on a given attribute.
 *
 * @param   connHandle - connection request was received on
 * @param   pAttr - pointer to attribute
 * @param   opcode - request opcode (ATT_READ_REQ or ATT_WRITE_REQ)
 *
 * @return  SUCCESS: Operation authorized.<BR>
 *          ATT_ERR_INSUFFICIENT_AUTHOR: Authorization required.<BR>
 */
typedef bStatus_t (*pfnGATTAuthorizeAttrCB_t)( uint16_t connHandle, gattAttribute_t *pAttr,
                                               uint8_t opcode );
/**
 * @}
 */

/**
 * GATT Structure for Characteristic Presentation Format Value.
 */
typedef struct
{
  uint8_t format;    //!< Format of the value of this characteristic
  int8_t exponent;   //!< A sign integer which represents the exponent of an integer
  uint16_t unit;     //!< Unit of this attribute as defined in the data dictionary
  uint8_t nameSpace; //!< Name space of the description
  uint16_t desc;     //!< Description of this attribute as defined in a higher layer profile
} gattCharFormat_t;

/**
 * GATT Structure for Client Characteristic Configuration.
 */
typedef struct
{
  uint16_t connHandle; //!< Client connection handle
  uint8_t  value;      //!< Characteristic configuration value for this client
} gattCharCfg_t;

/**
 * GATT Structure for service callback functions - must be setup by the application
 * and used when GATTServApp_RegisterService() is called.
 */
typedef struct
{
  pfnGATTReadAttrCB_t pfnReadAttrCB;           //!< Read callback function pointer
  pfnGATTWriteAttrCB_t pfnWriteAttrCB;         //!< Write callback function pointer
  pfnGATTAuthorizeAttrCB_t pfnAuthorizeAttrCB; //!< Authorization callback function pointer
} gattServiceCBs_t;

/**
 * GATT Server App event header format.
 */
typedef struct
{
  osal_event_hdr_t  hdr;           //!< GATT_SERV_MSG_EVENT and status
  uint16_t connHandle;               //!< Connection message was received on
  uint8_t method;                    //!< GATT type of command. Ref: @ref GATT_SERV_MSG_EVENT_DEFINES
} gattEventHdr_t;

/**
 * GATT_CLIENT_CHAR_CFG_UPDATED_EVENT message format.  This message is sent to
 * the app when a Client Characteristic Configuration is updated.
 */
typedef struct
{
  osal_event_hdr_t hdr; //!< GATT_SERV_MSG_EVENT and status
  uint16_t connHandle;    //!< Connection message was received on
  uint8_t method;         //!< GATT_CLIENT_CHAR_CFG_UPDATED_EVENT
  uint16_t attrHandle;    //!< attribute handle
  uint16_t value;         //!< attribute new value
} gattClientCharCfgUpdatedEvent_t;



typedef void (*gattServMsgCB_t)( gattMsgEvent_t*pMsg);

/*********************************************************************
 * VARIABLES
 */

/*********************************************************************
 * API FUNCTIONS
 */

/**
 * @defgroup GATT_SERV_APP_API GATT Server App API Functions
 *
 * @{
 */

/**
 * @brief   Register your task ID to receive event messages
 *          from the GATT Server Application.
 *
 * @param   taskID - Default task ID to send events.
 *
 * @return  none
 */
extern void GATTServApp_RegisterForMsg( uint8_t taskID );

/**
 * @brief   Register a service's attribute list and callback functions with
 *          the GATT Server Application.
 *
 * @param   pAttrs - Array of attribute records to be registered
 * @param   numAttrs - Number of attributes in array
 * @param   pServiceCBs - Service callback function pointers
 *
 * @return  SUCCESS: Service registered successfully.<BR>
 *          INVALIDPARAMETER: Invalid service field.<BR>
 *          FAILURE: Not enough attribute handles available.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
extern bStatus_t GATTServApp_RegisterService( gattAttribute_t *pAttrs, uint16_t numAttrs,
                                              const gattServiceCBs_t *pServiceCBs );
/**
 * @brief   Deregister a service's attribute list and callback functions from
 *          the GATT Server Application.
 *
 *          NOTE: It's the caller's responsibility to free the service attribute
 *          list returned from this API.
 *
 * @param   handle - handle of service to be deregistered
 * @param   p2pAttrs - pointer to array of attribute records (to be returned)
 *
 * @return  SUCCESS: Service deregistered successfully.
 *          FAILURE: Service not found.
 */
bStatus_t GATTServApp_DeregisterService( uint16_t handle, gattAttribute_t **p2pAttrs );

/**
 * @brief       Find the attribute record within a service attribute
 *              table for a given attribute value pointer.
 *
 * @param       pAttrTbl - pointer to attribute table
 * @param       numAttrs - number of attributes in attribute table
 * @param       pValue - pointer to attribute value
 *
 * @return      Pointer to attribute record. NULL, if not found.
 */
extern gattAttribute_t *GATTServApp_FindAttr( gattAttribute_t *pAttrTbl,
                                              uint16_t numAttrs, uint8_t *pValue );
/**
 * @brief   Add function for the GATT Service.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service added successfully.<BR>
 *          INVALIDPARAMETER: Invalid service field.<BR>
 *          FAILURE: Not enough attribute handles available.<BR>
 *          bleMemAllocError: Memory allocation error occurred.<BR>
 */
extern bStatus_t GATTServApp_AddService( uint32_t services );

/**
 * @brief   Delete function for the GATT Service.
 *
 * @param   services - services to delete. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service deleted successfully.<BR>
 *          FAILURE: Service not found.<BR>
 */
extern bStatus_t GATTServApp_DelService( uint32_t services );

/**
 * @brief   Set a GATT Server parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   pValue - pointer to data to write. This is dependent on the
 *                   parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be cast
 *                   to uint16_t pointer).
 *
 * @return  SUCCESS: Parameter set successful
 *          FAILURE: Parameter in use
 *          INVALIDPARAMETER: Invalid parameter
 *          bleInvalidRange: Invalid value
 *          bleMemAllocError: Memory allocation failed
 */
extern bStatus_t GATTServApp_SetParameter( uint8_t param, uint8_t len, void *pValue );

/**
 * @brief   Get a GATT Server parameter.
 *
 * @param   param - Profile parameter ID
 * @param   pValue - pointer to data to put. This is dependent on the
 *                   parameter ID and WILL be cast to the appropriate
 *                   data type (example: data type of uint16_t will be
 *                   cast to uint16_t pointer).
 *
 * @return  SUCCESS: Parameter get successful
 *          INVALIDPARAMETER: Invalid parameter
 */
extern bStatus_t GATTServApp_GetParameter( uint8_t param, void *pValue );

/**
 * @brief   Update the Client Characteristic Configuration for a given
 *          Client.
 *
 *          Note: This API should only be called from the Bond Manager.
 *
 * @param   connHandle - connection handle.
 * @param   attrHandle - attribute handle.
 * @param   value - characteristic configuration value.
 *
 * @return  SUCCESS: Parameter get successful
 *          INVALIDPARAMETER: Invalid parameter
 */
extern bStatus_t GATTServApp_UpdateCharCfg( uint16_t connHandle, uint16_t attrHandle, uint16_t value );

/**
 * @brief   Initialize the client characteristic configuration table.
 *
 *          Note: Each client has its own instantiation of the Client
 *                Characteristic Configuration. Reads/Writes of the Client
 *                Characteristic Configuration only only affect the
 *                configuration of that client.
 *
 * @param   connHandle - connection handle (0xFFFF for all connections).
 * @param   charCfgTbl - client characteristic configuration table.
 *
 * @return  none
 */
extern void GATTServApp_InitCharCfg( uint16_t connHandle, gattCharCfg_t *charCfgTbl );

/**
 * @brief   Read the client characteristic configuration for a given
 *          client.
 *
 *          Note: Each client has its own instantiation of the Client
 *                Characteristic Configuration. Reads of the Client
 *                Characteristic Configuration only shows the configuration
 *                for that client.
 *
 * @param   connHandle - connection handle.
 * @param   charCfgTbl - client characteristic configuration table.
 *
 * @return  attribute value
 */
extern uint16_t GATTServApp_ReadCharCfg( uint16_t connHandle, gattCharCfg_t *charCfgTbl );

/**
 * @brief   Write the client characteristic configuration for a given
 *          client.
 *
 *          Note: Each client has its own instantiation of the Client
 *                Characteristic Configuration. Writes of the Client
 *                Characteristic Configuration only only affect the
 *                configuration of that client.
 *
 * @param   connHandle - connection handle.
 * @param   charCfgTbl - client characteristic configuration table.
 * @param   value - attribute new value.
 *
 * @return  Success or Failure
 */
extern uint8_t GATTServApp_WriteCharCfg( uint16_t connHandle, gattCharCfg_t *charCfgTbl, uint16_t value );

/**
 * @brief   Process the client characteristic configuration
 *          write request for a given client.
 *
 * @param   connHandle - connection message was received on.
 * @param   pAttr - pointer to attribute.
 * @param   pValue - pointer to data to be written.
 * @param   len - length of data.
 * @param   offset - offset of the first octet to be written.
 * @param   validCfg - valid configuration.
 *
 * @return  Success or Failure
 */
extern bStatus_t GATTServApp_ProcessCCCWriteReq( uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint8_t len, uint16_t offset,
                                          uint16_t validCfg );

/**
 * @brief   Process Client Charateristic Configuration change.
 *
 * @param   charCfgTbl - characteristic configuration table.
 * @param   pValue - pointer to attribute value.
 * @param   authenticated - whether an authenticated link is required.
 * @param   attrTbl - attribute table.
 * @param   numAttrs - number of attributes in attribute table.
 * @param   taskId - task to be notified of confirmation.
 *
 * @return  Success or Failure
 */
extern bStatus_t GATTServApp_ProcessCharCfg( gattCharCfg_t *charCfgTbl, uint8_t *pValue,
                                        uint8_t authenticated, gattAttribute_t *attrTbl,
                                        uint16_t numAttrs, uint8_t taskId );

/**
 * @brief   Build and send the GATT_CLIENT_CHAR_CFG_UPDATED_EVENT to
 *          the application.
 *
 * @param   connHandle - connection handle
 * @param   attrHandle - attribute handle
 * @param   value - attribute new value
 *
 * @return  none
 */
extern void GATTServApp_SendCCCUpdatedEvent( uint16_t connHandle, uint16_t attrHandle, uint16_t value );

/**
 * @brief   Send out a Service Changed Indication.
 *
 * @param   connHandle - connection to use
 * @param   taskId - task to be notified of confirmation
 *
 * @return  SUCCESS: Indication was sent successfully.<BR>
 *          FAILURE: Service Changed attribute not found.<BR>
 *          INVALIDPARAMETER: Invalid connection handle or request field.<BR>
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.<BR>
 *          bleNotConnected: Connection is down.<BR>
 *          blePending: A confirmation is pending with this client.<BR>
 */
extern bStatus_t GATTServApp_SendServiceChangedInd( uint16_t connHandle, uint8_t taskId );

/**
 * @brief       Read an attribute. If the format of the attribute value
 *              is unknown to GATT Server, use the callback function
 *              provided by the Service.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       service - handle of owner service
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
extern uint8_t GATTServApp_ReadAttr( uint16_t connHandle, gattAttribute_t* pAttr,
                                   uint16_t service, uint8_t* pValue, uint16_t* pLen,
                                   uint16_t offset, uint8_t maxLen );

/**
 * @brief   Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   handle - attribute handle
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
extern uint8_t GATTServApp_WriteAttr( uint16_t connHandle, uint16_t handle,
                                    uint8_t *pValue, uint16_t len, uint16_t offset );

/**
 * @}
 */

/**
 * @brief   Set a GATT Server Application Parameter value. Use this
 *          function to change the default GATT parameter values.
 *
 * @param   value - new param value
 *
 * @return  void
 */
extern void GATTServApp_SetParamValue( uint16_t value );

/**
 * @brief   Get a GATT Server Application Parameter value.
 *
 * @param   none
 *
 * @return  GATT Parameter value
 */
extern uint16_t GATTServApp_GetParamValue( void );

/*-------------------------------------------------------------------
 * TASK API - These functions must only be called by OSAL.
 */

/**
 * @internal
 *
 * @brief   Initialize the GATT Server Test Application.
 *
 * @param   taskId - Task identifier for the desired task
 *
 * @return  void
 *
 */
extern void GATTServApp_Init( uint8_t taskId );

/**
 * @internal
 *
 * @brief   GATT Server Application Task event processor. This function
 *          is called to process all events for the task. Events include
 *          timers, messages and any other user defined events.
 *
 * @param   task_id - The OSAL assigned task ID.
 * @param   events - events to process. This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
extern uint16_t GATTServApp_ProcessEvent( uint8_t taskId, uint16_t events );

bStatus_t gattServApp_RegisterCB(gattServMsgCB_t cb);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATTSERVAPP_H */
