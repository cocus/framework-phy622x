/**************************************************************************************************
    Filename:       gatt_uuid.c
    Revised:
    Revision:

    Description:    This file contains Generic Attribute Profile (GATT)
                  UUID types.

	SDK_LICENSE

**************************************************************************************************/


/*********************************************************************
    INCLUDES
*/
#include "gatt_uuid.h"
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

/*********************************************************************
    GLOBAL VARIABLES
*/

/**
    GATT Services
*/
// Generic Access Profile Service UUID 0x1800
const uint8_t gapServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GAP_SERVICE_UUID ), HI_UINT16( GAP_SERVICE_UUID )
};

// Generic Attribute Profile Service UUID 0x1801
const uint8_t gattServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_SERVICE_UUID ), HI_UINT16( GATT_SERVICE_UUID )
};

/**
    GATT Declarations
*/
// Primary Service UUID 0x2801
const uint8_t primaryServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_PRIMARY_SERVICE_UUID ), HI_UINT16( GATT_PRIMARY_SERVICE_UUID )
};

// Secondary Service UUID 0x2801
const uint8_t secondaryServiceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_SECONDARY_SERVICE_UUID ), HI_UINT16( GATT_SECONDARY_SERVICE_UUID )
};

// Include UUID 0x2802
const uint8_t includeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_INCLUDE_UUID ), HI_UINT16( GATT_INCLUDE_UUID )
};

// Characteristic UUID 0x2803
const uint8_t characterUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CHARACTER_UUID ), HI_UINT16( GATT_CHARACTER_UUID )
};

/**
    GATT Descriptors
*/
// Characteristic Extended Properties UUID 0x2900
const uint8_t charExtPropsUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CHAR_EXT_PROPS_UUID ), HI_UINT16( GATT_CHAR_EXT_PROPS_UUID )
};

// Characteristic User Description UUID 0x2901
const uint8_t charUserDescUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CHAR_USER_DESC_UUID ), HI_UINT16( GATT_CHAR_USER_DESC_UUID )
};

// Client Characteristic Configuration UUID 0x2902
const uint8_t clientCharCfgUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CLIENT_CHAR_CFG_UUID ), HI_UINT16( GATT_CLIENT_CHAR_CFG_UUID )
};

// Server Characteristic Configuration UUID 0x2903
const uint8_t servCharCfgUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_SERV_CHAR_CFG_UUID ), HI_UINT16( GATT_SERV_CHAR_CFG_UUID )
};

// Characteristic Presentation Format UUID 0x2904
const uint8_t charFormatUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CHAR_FORMAT_UUID ), HI_UINT16( GATT_CHAR_FORMAT_UUID )
};

// Characteristic Aggregate Format UUID 0x2905
const uint8_t charAggFormatUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_CHAR_AGG_FORMAT_UUID ), HI_UINT16( GATT_CHAR_AGG_FORMAT_UUID )
};

/**
    GATT Characteristics
*/
// Device Name UUID
const uint8_t deviceNameUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( DEVICE_NAME_UUID ), HI_UINT16( DEVICE_NAME_UUID )
};

// Appearance UUID
const uint8_t appearanceUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( APPEARANCE_UUID ), HI_UINT16( APPEARANCE_UUID )
};

// Peripheral Privacy Flag UUID
const uint8_t periPrivacyFlagUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( PERI_PRIVACY_FLAG_UUID ), HI_UINT16( PERI_PRIVACY_FLAG_UUID )
};

// Reconnection Address UUID
const uint8_t reconnectAddrUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( RECONNECT_ADDR_UUID ), HI_UINT16( RECONNECT_ADDR_UUID )
};

// Peripheral Preferred Connection Parameters UUID
const uint8_t periConnParamUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( PERI_CONN_PARAM_UUID ), HI_UINT16( PERI_CONN_PARAM_UUID )
};

// Service Changed UUID
const uint8_t serviceChangedUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( SERVICE_CHANGED_UUID ), HI_UINT16( SERVICE_CHANGED_UUID )
};

// Valid Range UUID
const uint8_t validRangeUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_VALID_RANGE_UUID ), HI_UINT16( GATT_VALID_RANGE_UUID )
};

// External Report Reference Descriptor
const uint8_t extReportRefUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_EXT_REPORT_REF_UUID ), HI_UINT16( GATT_EXT_REPORT_REF_UUID )
};

// Report Reference characteristic descriptor
const uint8_t reportRefUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16( GATT_REPORT_REF_UUID ), HI_UINT16( GATT_REPORT_REF_UUID )
};

/*********************************************************************
    GLOBAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/

/*********************************************************************
    LOCAL FUNCTIONS
*/

/*********************************************************************
    API FUNCTIONS
*/

/*********************************************************************
    @fn      GATT_FindUUIDRec

    @brief   Find the UUID record for a given UUID.

    @param   pUUID - UUID to look for.
    @param   len - length of UUID.

    @return  Pointer to UUID record. NULL, otherwise.
*/
const uint8_t* GATT_FindUUIDRec( const uint8_t* pUUID, uint8_t len )
{
    const uint8_t* pRec = NULL;

    if ( len == ATT_BT_UUID_SIZE )
    {
        // 16-bit UUID
        uint16_t uuid = BUILD_UINT16( pUUID[0], pUUID[1] );

        switch ( uuid )
        {
        /*** GATT Services ***/
        case GAP_SERVICE_UUID:
            pRec = gapServiceUUID;
            break;

        case GATT_SERVICE_UUID:
            pRec = gattServiceUUID;
            break;

        /*** GATT Declarations ***/

        case GATT_PRIMARY_SERVICE_UUID:
            pRec = primaryServiceUUID;
            break;

        case GATT_SECONDARY_SERVICE_UUID:
            pRec = secondaryServiceUUID;
            break;

        case GATT_INCLUDE_UUID:
            pRec = includeUUID;
            break;

        case GATT_CHARACTER_UUID:
            pRec = characterUUID;
            break;

        /*** GATT Descriptors ***/

        case GATT_CHAR_EXT_PROPS_UUID:
            pRec = charExtPropsUUID;
            break;

        case GATT_CHAR_USER_DESC_UUID:
            pRec = charUserDescUUID;
            break;

        case GATT_CLIENT_CHAR_CFG_UUID:
            pRec = clientCharCfgUUID;
            break;

        case GATT_SERV_CHAR_CFG_UUID:
            pRec = servCharCfgUUID;
            break;

        case GATT_CHAR_FORMAT_UUID:
            pRec = charFormatUUID;
            break;

        case GATT_CHAR_AGG_FORMAT_UUID:
            pRec = charAggFormatUUID;
            break;

        case GATT_VALID_RANGE_UUID:
            pRec = validRangeUUID;
            break;

        case GATT_EXT_REPORT_REF_UUID:
            pRec = extReportRefUUID;
            break;

        case GATT_REPORT_REF_UUID:
            pRec = reportRefUUID;
            break;

        /*** GATT Characteristics ***/

        case DEVICE_NAME_UUID:
            pRec = deviceNameUUID;
            break;

        case APPEARANCE_UUID:
            pRec = appearanceUUID;
            break;

        case RECONNECT_ADDR_UUID:
            pRec = reconnectAddrUUID;
            break;

        case PERI_PRIVACY_FLAG_UUID:
            pRec = periPrivacyFlagUUID;
            break;

        case PERI_CONN_PARAM_UUID:
            pRec = periConnParamUUID;
            break;

        case SERVICE_CHANGED_UUID:
            pRec = serviceChangedUUID;
            break;

        /*** GATT Units ***/

        default:
            break;
        }
    }
    else if ( len == ATT_UUID_SIZE )
    {
        // 128-bit UUID
    }

    return ( pRec );
}

/****************************************************************************
****************************************************************************/
