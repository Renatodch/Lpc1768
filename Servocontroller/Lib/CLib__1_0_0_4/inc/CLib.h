/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLIB_H
#define __CLIB_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#define	boolean uint8_t		
			
	
#include <string.h>
#include <stdio.h>	
#include "lpc17xx.h"
	
#include "SystemTime.h"		
#include "Timer.h"	
#include "Fifo.h"			
#include "str.h"	


#include "Bool.h"		

#include "serial.h"
#include "Pulse.h"	
#include "Button.h"			
#include "Capture.h"	
#include "Parameters.h"	
#include "Bytes.h"	
#include "Utils.h"		
#include "Uint32.h"		

#ifdef __cplusplus
}
#endif

#endif 
