/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UINT32_H
#define __UINT32_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

void Uint32_ToBytes(uint32_t value, uint8_t *srcBuf);
uint32_t Uint32_Swap( uint32_t val );
	
	
	
#ifdef __cplusplus
}
#endif

#endif 
