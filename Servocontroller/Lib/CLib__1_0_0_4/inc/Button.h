/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BUTTON_H
#define __BUTTON_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"
#include "CLib.h"	

	
typedef void (*FuncEdgeHandler)();
	
typedef enum
{ 
	BUTTON_EVT_HIGH,
	BUTTON_EVT_DEBOUNCE_TIMEOUT
}BUTTON_EVT;

typedef enum
{ 
		Button_Ste_Low,
		Button_Ste_High
}Button_Ste;	
	
	
typedef struct
{
		Timer						debounceTimer;
		uint8_t 				State;
		FuncEdgeHandler FuncEdgeHandler;
}Button;

void Button_Init(Button *p, FuncEdgeHandler FuncEdgeHandler);
void Button_Timer(Button *p);
void 	Button_Fsm(Button *p, BUTTON_EVT e);
void Button_Events(Button *p);

#ifdef __cplusplus
}
#endif

#endif 
