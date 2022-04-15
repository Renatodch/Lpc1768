/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_TIME_H
#define __SYSTEM_TIME_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "CLib.h"

typedef struct
{
 uint32_t 	time;
}SystemTime;

uint32_t 		SystemTime_GetTime(void);
void 				SystemTime_Timer(void);
void 				SystemTime_To_String(char* str);
void 				SystemTime_Init(void);

#ifdef __cplusplus
}
#endif

#endif 
