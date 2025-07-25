/**************************************************************************************************
    Filename:       osal_bufmgr.h
    Revised:
    Revision:

    Description:    This file contains the buffer management definitions.

 SDK_LICENSE


**************************************************************************************************/

#ifndef OSAL_BUFMGR_H
#define OSAL_BUFMGR_H

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


/*********************************************************************
    VARIABLES
*/


/*********************************************************************
    MACROS
*/


/*********************************************************************
    TYPEDEFS
*/


/*********************************************************************
    VARIABLES
*/

/*********************************************************************
    FUNCTIONS
*/

/*
    Allocate a block of memory.
*/
extern void* osal_bm_alloc( uint16_t size );

/*
    Add or remove header space for the payload pointer.
*/
extern void* osal_bm_adjust_header( void* payload_ptr, int16_t size );

/*
    Add or remove tail space for the payload pointer.
*/
extern void* osal_bm_adjust_tail( void* payload_ptr, int16_t size );

/*
    Free a block of memory.
*/
extern void osal_bm_free( void* payload_ptr );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OSAL_BUFMGR_H */
