/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FIFO_H
#define __FIFO_H

#ifdef __cplusplus
 extern "C" 
{
#endif


#include "Timer.h"

#define FIFO_SIZE 	512

typedef struct Fifo
{
		char 		buf_w[FIFO_SIZE];
		char    buf_r[FIFO_SIZE];
		int 		head;
		int 		tail;
		int 		size;
		int 		cnt_FreeWrite;
		int 		cnt_FreeRead;
		int 		tmr_Inactive;
		Timer		inactive_Timer;
	
} Fifo;

int 	Fifo_Read(Fifo * f, void * buf, int nbytes);
int 	Fifo_Write				(Fifo * f, const void * buf, int nbytes);
char 	Fifo_OnInactive		(Fifo * f);
void 	Fifo_Timer				(Fifo * f);
void 	Fifo_StartTimerInactive(Fifo * f);
void 	Fifo_Init					(Fifo * f);


#ifdef __cplusplus
}
#endif

#endif 
