/**************************************************************************************************
  Filename:       linkdb.h
  Revised:        
  Revision:       

  Description:    This file contains the linkDB interface.

  SDK_LICENSE
**************************************************************************************************/

#ifndef LINKDB_H
#define LINKDB_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include <ble/include/bcomdef.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Special case connection handles
#define INVALID_CONNHANDLE              0xFFFF  // Invalid connection handle, used for no connection handle
#define LOOPBACK_CONNHANDLE             0xFFFE  // Loopback connection handle, used to loopback a message
  
// Link state flags
#define LINK_NOT_CONNECTED              0x00    // Link isn't connected
#define LINK_CONNECTED                  0x01    // Link is connected
#define LINK_AUTHENTICATED              0x02    // Link is authenticated
#define LINK_BOUND                      0x04    // Link is bonded
#define LINK_ENCRYPTED                  0x10    // Link is encrypted

// Link Database Status callback changeTypes
#define LINKDB_STATUS_UPDATE_NEW        0       // New connection created
#define LINKDB_STATUS_UPDATE_REMOVED    1       // Connection was removed
#define LINKDB_STATUS_UPDATE_STATEFLAGS 2       // Connection state flag changed
  
// Link Authentication Errors
#define LINKDB_ERR_INSUFFICIENT_AUTHEN      0x05  // Link isn't even encrypted
#define LINBDB_ERR_INSUFFICIENT_KEYSIZE     0x0c  // Link is encrypted but the key size is too small
#define LINKDB_ERR_INSUFFICIENT_ENCRYPTION  0x0f  // Link is encrypted but it's not authenticated

/*********************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint8_t srk[KEYLEN];  // Signature Resolving Key
  uint32_t signCounter; // Sign Counter 
} linkSec_t;

typedef struct
{
  uint8_t ltk[KEYLEN];             // Long Term Key
  uint16_t div;                    // Diversifier
  uint8_t rand[B_RANDOM_NUM_SIZE]; // random number
  uint8_t keySize;                 // LTK Key Size
} encParams_t;

typedef struct
{
  uint8_t taskID;            // Application that controls the link
    uint16_t connectionHandle; // Controller connection handle
    uint8_t stateFlags;        // LINK_CONNECTED, LINK_AUTHENTICATED...
    uint8_t role;               // 2020-04-22 add (case for multi-role SMP )
    uint8_t addrType;          // Address type of connected device
    uint8_t addr[B_ADDR_LEN];  // Other Device's address
  uint16_t connInterval;     // The connection's interval (n * 1.23 ms)
  linkSec_t sec;           // Connection Security related items
  encParams_t *pEncParams; // pointer to LTK, ediv, rand. if needed.
} linkDBItem_t;

// function pointer used to register for a status callback
typedef void (*pfnLinkDBCB_t)( uint16_t connectionHandle, uint8_t changeType );

// function pointer used to perform specialized link database searches
typedef void (*pfnPerformFuncCB_t)( linkDBItem_t *pLinkItem );

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
  /*
   * linkDB_Init - Initialize the Link Database.
   */
  extern void linkDB_Init( void );

  /*
   * linkDB_Register - Register with this function to receive a callback when
   *              status changes on a connection.
   */
  extern uint8_t linkDB_Register( pfnLinkDBCB_t pFunc );

  /*
   * linkDB_Add - Adds a record to the link database.
   */
extern uint8_t linkDB_Add( uint8_t taskID, uint16_t connectionHandle, uint8_t  stateFlags, uint8_t role,
                         uint8_t addrType, uint8_t* pAddr, uint16_t connInterval );

  /*
   * linkDB_Remove - Removes a record from the link database.
   */
  extern uint8_t linkDB_Remove( uint16_t connectionHandle );

  /*
   * linkDB_Update - This function is used to update the stateFlags of 
   *              a link record.
   */
  extern uint8_t linkDB_Update( uint16_t connectionHandle, uint8_t newState );

  /*
   * linkDB_NumActive - returns the number of active connections.
   */
  extern uint8_t linkDB_NumActive( void );

  /*
   * linkDB_Find - Find link database item (link information)
   * 
   *    returns a pointer to the link item, NULL if not found
   */
  extern linkDBItem_t *linkDB_Find( uint16_t connectionHandle );

  /*
   * linkDB_FindFirst - Find the first link that matches the taskID.
   * 
   *    returns a pointer to the link item, NULL if not found
   */
  extern linkDBItem_t *linkDB_FindFirst( uint8_t taskID );

  /*
   * linkDB_State - Check to see if a physical link is in a specific state.
   * 
   *    returns TRUE is the link is in state. FALSE, otherwise.
   */
  extern uint8_t linkDB_State( uint16_t connectionHandle, uint8_t state );

  /*
   * linkDB_Authen - Check to see if the physical link is encrypted and authenticated.
   *    returns SUCCESS if the link is authenticated or 
   *            bleNotConnected - connection handle is invalid, 
   *            LINKDB_ERR_INSUFFICIENT_AUTHEN - link is not encrypted,
   *            LINBDB_ERR_INSUFFICIENT_KEYSIZE - key size encrypted is not large enough,
   *            LINKDB_ERR_INSUFFICIENT_ENCRYPTION - link is encrypted, but not authenticated
   */
  extern uint8_t linkDB_Authen( uint16_t connectionHandle, uint8_t keySize, uint8_t mitmRequired );
  
  /*
   * linkDB_PerformFunc - Perform a function of each connection in the link database.
   */
  extern void linkDB_PerformFunc( pfnPerformFuncCB_t cb );
  
  /*
   * linkDB_Up - Check to see if a physical link is up (connected).
   *    Use like:  uint8_t linkDB_Up( uint16_t connectionHandle );
   *            connectionHandle - controller link connection handle.
   *            returns TRUE if the link is up. FALSE, otherwise.
   */
  #define linkDB_Up( connectionHandle )  linkDB_State( (connectionHandle), LINK_CONNECTED )

  /*
   * linkDB_Encrypted - Check to see if the physical link is encrypted.
   *    Use like:  linkDB_Encrypted( uint16_t connectionHandle );
   *            connectionHandle - controller link connection handle.
   *            returns TRUE if the link is encrypted. FALSE, otherwise.
   */
  #define linkDB_Encrypted( connectionHandle )  linkDB_State( (connectionHandle), LINK_ENCRYPTED )
  
  /*
   * linkDB_Authenticated - Check to see if the physical link is authenticated.
   *    Use like:  linkDB_Authenticated( uint16_t connectionHandle );
   *            connectionHandle - controller link connection handle.
   *            returns TRUE if the link is authenticated. FALSE, otherwise.
   */
  #define linkDB_Authenticated( connectionHandle )  linkDB_State( (connectionHandle), LINK_AUTHENTICATED )

  /*
   * linkDB_Bonded - Check to see if the physical link is bonded.
   *    Use like:  linkDB_Bonded( uint16_t connectionHandle );
   *            connectionHandle - controller link connection handle.
   *            returns TRUE if the link is bonded. FALSE, otherwise.
   */
  #define linkDB_Bonded( connectionHandle )  linkDB_State( (connectionHandle), LINK_BOUND )
  
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* LINKDB_H */
