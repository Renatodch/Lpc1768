/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

uint16_t Uint16_Checksum(uint8_t *pBytes, uint32_t len);	
uint8_t Ascii_To_Digit(uint8_t ascii);
	
	
#ifdef __cplusplus
}
#endif

#endif 
