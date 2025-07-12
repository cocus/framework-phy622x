#include "OSAL.h"
#include <string.h>

/*** Helper Functions ***/

/*
    String Length
*/
size_t strlen(const char* string)
{
    // Use the OSAL string length function
    return (size_t)osal_strlen((char *)string);
}
/*
    Memory copy
*/
void *memcpy(void * dest, const void * srtc, size_t sz)
{
    // Use the OSAL memory copy function
    return osal_memcpy(dest, srtc, (unsigned int)sz);
}

/*
    Memory compare
*/
int memcmp(const void *src1, const void *src2, size_t sz)
{
    // Use the OSAL memory compare function
    return (int)osal_memcmp(src1, src2, (unsigned int)sz);
}

/*
    Memory set
*/
void *memset(void *dest, int value, size_t len)
{
    // Use the OSAL memory set function
    return osal_memset(dest, (uint8_t)value, (int)len);
}
