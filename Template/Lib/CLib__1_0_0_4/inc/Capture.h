/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAPTURE_H
#define __CAPTURE_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

typedef void (*RisingEdgeHandler)(void* o);
typedef void (*FallingEdgeHandler)(void* o);
	
	
typedef struct
{
				uint8_t  idx;	
				uint32_t lowTime;  
				uint32_t highTime;
				uint8_t  nivel;
				RisingEdgeHandler risingEdgeHandler;
				FallingEdgeHandler fallingEdgeHandler;
				uint8_t  initNivel;
				uint8_t  firstRead;
				uint32_t pulses;
				uint8_t  pulseFlag;	
}Capture;


uint32_t  	Capture_GetTotalTime(Capture *p);
void 				Capture_Handlers(Capture *p, uint8_t  nivel);
void 				Capture_Clock(Capture *p);
void 				Capture_Init(Capture *p, uint8_t  initNivel, RisingEdgeHandler risingEdgeHandler, FallingEdgeHandler fallingEdgeHandler);

#ifdef __cplusplus
}
#endif

#endif 
