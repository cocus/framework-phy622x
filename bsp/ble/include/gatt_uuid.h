/**************************************************************************************************
    Filename:       gatt_uuid.h
    Revised:
    Revision:

    Description:    This file contains Generic Attribute Profile (GATT)
                  UUID types.

	SDK_LICENSE

**************************************************************************************************/

#ifndef GATT_UUID_H
#define GATT_UUID_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
    INCLUDES
*/
#include <stdint.h>

/*********************************************************************
    CONSTANTS
*/

/*
    WARNING: The 16-bit UUIDs are assigned by the Bluetooth SIG and published
            in the Bluetooth Assigned Numbers page. Do not change these values.
            Changing them will cause Bluetooth interoperability issues.
*/

/**
    GATT Services
*/
#define GAP_SERVICE_UUID                           0x1800 // Generic Access Profile
#define GATT_SERVICE_UUID                          0x1801 // Generic Attribute Profile

/**
    GATT Declarations
*/
#define GATT_PRIMARY_SERVICE_UUID                  0x2800 // Primary Service
#define GATT_SECONDARY_SERVICE_UUID                0x2801 // Secondary Service
#define GATT_INCLUDE_UUID                          0x2802 // Include
#define GATT_CHARACTER_UUID                        0x2803 // Characteristic

/**
    GATT Descriptors
*/
#define GATT_CHAR_EXT_PROPS_UUID                   0x2900 // Characteristic Extended Properties
#define GATT_CHAR_USER_DESC_UUID                   0x2901 // Characteristic User Description
#define GATT_CLIENT_CHAR_CFG_UUID                  0x2902 // Client Characteristic Configuration
#define GATT_SERV_CHAR_CFG_UUID                    0x2903 // Server Characteristic Configuration
#define GATT_CHAR_FORMAT_UUID                      0x2904 // Characteristic Presentation Format
#define GATT_CHAR_AGG_FORMAT_UUID                  0x2905 // Characteristic Aggregate Format
#define GATT_VALID_RANGE_UUID                      0x2906 // Valid Range
#define GATT_EXT_REPORT_REF_UUID                   0x2907 // External Report Reference Descriptor
#define GATT_REPORT_REF_UUID                       0x2908 // Report Reference Descriptor

/**
    GATT Characteristics
*/
#define DEVICE_NAME_UUID                           0x2A00 // Device Name
#define APPEARANCE_UUID                            0x2A01 // Appearance
#define PERI_PRIVACY_FLAG_UUID                     0x2A02 // Peripheral Privacy Flag
#define RECONNECT_ADDR_UUID                        0x2A03 // Reconnection Address
#define PERI_CONN_PARAM_UUID                       0x2A04 // Peripheral Preferred Connection Parameters
#define SERVICE_CHANGED_UUID                       0x2A05 // Service Changed

/*********************************************************************
    MACROS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    VARIABLES
*/

/**
    GATT Services
*/
extern CONST uint8_t gapServiceUUID[];
extern CONST uint8_t gattServiceUUID[];

/**
    GATT Attribute Types
*/
extern CONST uint8_t primaryServiceUUID[];
extern CONST uint8_t secondaryServiceUUID[];
extern CONST uint8_t includeUUID[];
extern CONST uint8_t characterUUID[];

/**
    GATT Characteristic Descriptors
*/
extern CONST uint8_t charExtPropsUUID[];
extern CONST uint8_t charUserDescUUID[];
extern CONST uint8_t clientCharCfgUUID[];
extern CONST uint8_t servCharCfgUUID[];
extern CONST uint8_t charFormatUUID[];
extern CONST uint8_t charAggFormatUUID[];
extern CONST uint8_t validRangeUUID[];
extern CONST uint8_t extReportRefUUID[];
extern CONST uint8_t reportRefUUID[];

/**
    GATT Characteristic Types
*/
extern CONST uint8_t deviceNameUUID[];
extern CONST uint8_t appearanceUUID[];
extern CONST uint8_t periPrivacyFlagUUID[];
extern CONST uint8_t reconnectAddrUUID[];
extern CONST uint8_t periConnParamUUID[];
extern CONST uint8_t serviceChangedUUID[];
extern CONST uint8_t manuNameUUID[];
extern CONST uint8_t serialNumUUID[];
extern CONST uint8_t manuAddrUUID[];

/*********************************************************************
    FUNCTIONS
*/
extern const uint8_t* GATT_FindUUIDRec( const uint8_t* pUUID, uint8_t len );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATT_UUID_H */
