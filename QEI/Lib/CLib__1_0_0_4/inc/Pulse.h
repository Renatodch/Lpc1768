/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PULSE_H
#define __PULSE_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "CLib.h"	
	

typedef enum
{ 
		Pulse_Event_Clock
	
}PULSE_EVT;

typedef enum
{ 
		Pulse_State_Pulse, 
		Pulse_State_Break,
		Pulse_State_Count,
	
}PULSE_STATE;

typedef struct
{
		uint32_t 	state;	
		Timer			timer;	
		uint32_t 	setpoint; 
		uint32_t 	count;    
		uint8_t 	nivel;
		
}Pulse;


void 			Pulse_Set_Setpoint(Pulse* p, uint32_t setpoint);
uint8_t 	Pulse_Get_Nivel(Pulse* p);
void 			Pulse_Fsm(Pulse* p, PULSE_EVT event);
void 			Pulse_Init(Pulse* p);

#ifdef __cplusplus
}
#endif

#endif 

