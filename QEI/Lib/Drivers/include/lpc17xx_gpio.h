/**********************************************************************
* $Id$		lpc17xx_gpio.h				2018-04-02
*//**
* @file		lpc17xx_gpio.h
* @brief	Contains all macro definitions and function prototypes
* 			support for GPDMA firmware library on LPC17xx
* @version	3.0
* @date		2. April. 2018
* @author	Renato Dc
*
* All rights reserved.
*/

#ifndef LPC17XX_GPIO_H_
#define LPC17XX_GPIO_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define Pin(x)  (0x01UL<<x)

typedef enum{
	GPIO_MODE_PU,
	GPIO_MODE_R,
	GPIO_MODE_FLOATING,
	GPIO_MODE_PD
}
MODE_e;

typedef enum{
	GPIO_DEFAULT,
	GPIO_FUNC_1,
	GPIO_FUNC_2,
	GPIO_FUNC_3
}
FUNC_e;

typedef enum{
	GPIO_NO_OD,
	GPIO_OD
}
OD_e;



#define Lock(Port,Pin)	     			(LPC_GPIO##Port->FIOMASK |= (uint32_t)(Pin))   // 1: locked   
#define Unlock(Port,Pin)    			(LPC_GPIO##Port->FIOMASK &= ~(uint32_t)(Pin))  // 0: unlocked

#define Output(Port,Pin)		 			(LPC_GPIO##Port->FIODIR |= (uint32_t)(Pin))    // 1: output
#define Input(Port,Pin)	  				(LPC_GPIO##Port->FIODIR &= ~(uint32_t)(Pin))   // 0: input	  (default)
#define High(Port,Pin)      			(LPC_GPIO##Port->FIOSET = (uint32_t)(Pin))		  // 1: high
#define Low(Port,Pin)       		  (LPC_GPIO##Port->FIOCLR = (uint32_t)(Pin))     // 0: low			
#define Toggle(Port,Pin)    		  (LPC_GPIO##Port->FIOPIN ^= (uint32_t)(Pin))    // 0 default
#define Read(Port,Pin)	 	   			(LPC_GPIO##Port->FIOPIN & (Pin))

#define Set_edge(Port,Pin,Edge) 	(LPC_GPIOINT->IO##Port##IntEn##Edge |= (uint32_t)(Pin))
#define Clr_edge(Port,Pin,Edge) 	(LPC_GPIOINT->IO##Port##IntEn##Edge &= (uint32_t)(Pin))
#define Clr_int(Port,Pin)   			(LPC_GPIOINT->IO##Port##IntClr |= (uint32_t)(Pin))  //Ports: 0,2 Edge: F,R
#define Read_int(Port,Pin,Edge)   (LPC_GPIOINT->IO##Port##IntStat##Edge & (Pin))  


#define Turn_On(PPType)						(LPC_SC->PCONP |= ((PPType) & CLKPWR_PCONP_BITMASK))
#define Turn_Off(PPType)					(LPC_SC->PCONP &= ((~PPType) & CLKPWR_PCONP_BITMASK))

void GPIO_Init(char Port, uint32_t Pins, FUNC_e Funcnum, MODE_e PinMode, OD_e Od);

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_GPIO_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
