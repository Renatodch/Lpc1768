/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_H
#define __TASK_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "stm32l1xx.h"
#include "Lib.h"

typedef void (*TaskFunction)();	

typedef struct
{
		SystemTimer 	systemTimer;
		TaskFunction 	taskFunction;
		char 					active;
}Task;

void Task__Call(Task* p);

#ifdef __cplusplus
}
#endif

#endif 
