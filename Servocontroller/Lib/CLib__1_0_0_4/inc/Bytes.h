/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BYTES_H
#define __BYTES_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "CLib.h"

uint32_t Bytes_ToUint32(uint8_t *bytes);		
void Bytes_Swap_EachTwoBytes(char *bytes, uint32_t length);
void Bytes_Print_ToHex(uint8_t *pBytes, uint32_t len);
	
#ifdef __cplusplus
}
#endif

#endif 
