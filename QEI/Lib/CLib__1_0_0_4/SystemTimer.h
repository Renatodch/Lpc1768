/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_TIMER_H
#define __SYSTEM_TIMER_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "stm32l1xx.h"
#include "Lib.h"	

typedef struct
{
   uint32_t	start;
   uint32_t	timeuot;	
}SystemTimer;

uint32_t 		SystemTimer3__Get(SystemTimer* p);
char 				SystemTimer3__Is_Timeout(SystemTimer* p);
void 				SystemTimer3__Timer(SystemTimer* p);
void 				SystemTimer3__Init(SystemTimer* p);

#ifdef __cplusplus
}
#endif

#endif 
