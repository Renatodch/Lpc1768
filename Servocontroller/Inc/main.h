#ifndef __MAIN_H
#define __MAIN_H

#include "LPC17xx.h"
#include "LPC17xx_it.h"
#include "User.h"
#include "Peripheral.h"

#include "Dev.h"
#include "Handlers.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "cdc.h"
#include "cdcuser.h"
#include "uart.h"

#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdarg.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"

#include "CLib.h"
#include "build_time.h"

typedef enum
{
	NONE,
	LEFT,
	RIGHT,
}DIR_E;

typedef struct
{
	uint32_t DC;
	uint32_t RPM;
	uint32_t W;
	uint32_t W_max;
	uint32_t dw_i;
	uint32_t Pos_Max;
	uint32_t Pos_Min;
	uint32_t Pos_Ini;
	uint32_t Pos_Actual;
	DIR_E Dir;
}Articulacion;

typedef struct
{
	uint32_t SetPoint; 
	uint32_t Ejecutando;
	uint32_t Edges;
	Articulacion Prismatica;
	int Error;
	
}Trayectoria;


/** Velocity capture period definition (in microsecond) */
#define CAP_PERIOD		125000UL// => 0.125 s con PCLK = 100 M   

/** Delay time to Read Velocity Accumulator and display (in microsecond)*/
#define DISP_TIME			1000000UL
/** Max velocity capture times calculated */
#define MAX_CAP_TIMES		(DISP_TIME/CAP_PERIOD)

#define ENC_RES	 			200UL	/**< Encoder resolution (PPR) */


#endif 













