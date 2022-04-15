/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

typedef struct
{
  	uint32_t 		startTime;  
  	uint32_t 		interval;  
		uint8_t		  enabled;
}Timer;

uint32_t Timer_GetRemainingTimeInSeconds(Timer *p);
uint32_t 		Timer_GetRemainingTime(Timer *p);
uint32_t  	Timer_GetTime(Timer *p);
void  			Timer_SetInterval(Timer *p, uint32_t interval);
void 				Timer_Start(Timer *p);
void 				Timer_Stop(Timer *p);
uint8_t  		Timer_Elapsed(Timer *p);
void 				Timer_Init(Timer *p, uint32_t interval);

#ifdef __cplusplus
}
#endif

#endif 
