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
	GPIO_DIR_IN,
	GPIO_DIR_OUT
}GPIO_DIR_e;

typedef enum{
	GPIO_MASK_UNLOCK,
	GPIO_MASK_LOCK
}GPIO_MASK_e;

typedef enum{
	GPIO_LVL_DONT_MATTER,
	GPIO_LVL_LOW,
	GPIO_LVL_HIGH
}GPIO_STATE_e;

typedef enum{
	GPIO_INT_EDGE_DISABLE,
	GPIO_INT_EDGE_RISING,
	GPIO_INT_EDGE_FALLING,
	GPIO_INT_EDGE_BOTH
}GPIO_EDGE_INT_e;

typedef struct
{
	uint32_t  				Pins;
	GPIO_DIR_e 				Dir;
	GPIO_MASK_e 			Mask;
	GPIO_EDGE_INT_e 	EdgeInt;
	GPIO_STATE_e			State;
}GPIO_InitTypeDef;



#define lock(Port,Pins)	     			(LPC_GPIO##Port->FIOMASK |= (uint32_t)(Pins))   // 1: locked   
#define unlock(Port,Pins)    			(LPC_GPIO##Port->FIOMASK &= ~(uint32_t)(Pins))  // 0: unlocked
#define output(Port,Pins)		 			(LPC_GPIO##Port->FIODIR |= (uint32_t)(Pins))    // 1: output
#define input(Port,Pins)	  			(LPC_GPIO##Port->FIODIR &= ~(uint32_t)(Pins))   // 0: input	
#define high(Port,Pins)      			(LPC_GPIO##Port->FIOSET = (uint32_t)(Pins))		  // 1: high
#define low(Port,Pins)       		  (LPC_GPIO##Port->FIOCLR = (uint32_t)(Pins))     // 0: low
#define edge(Port,Pins,Edge) 		  (LPC_GPIOINT->IO##Port##IntEn##Edge = (uint32_t)(Pins))

#define clr_int(Port,Pins)   			(LPC_GPIOINT->IO##Port##IntClr = (uint32_t)(Pins))  //Ports: 0,2 Edge: F,R
#define read_int(Port,Edge)       (LPC_GPIOINT->IO##Port##IntStat##Edge)  
#define toggle(Port,Pins)    		  (LPC_GPIO##Port->FIOPIN ^= (uint32_t)(Pins))
#define read(Port,Pin)	 	   			(LPC_GPIO##Port->FIOPIN & Pin)

__STATIC_INLINE void GPIO_Init(GPIO_InitTypeDef * gpio, LPC_GPIO_TypeDef *GPIOx )
{
	if(gpio->Dir == GPIO_DIR_OUT) 	
	{
		GPIOx->FIODIR |= (uint32_t)(gpio->Pins);
		
		if(gpio->State == GPIO_LVL_HIGH) GPIOx->FIOSET |= (uint32_t)(gpio->Pins);
		else														 GPIOx->FIOCLR |= (uint32_t)(gpio->Pins);
	}
	else if(gpio->Dir == GPIO_DIR_IN) 	
	{
		GPIOx->FIODIR &= ~(uint32_t)(gpio->Pins);
		
		if(GPIOx == LPC_GPIO0)
		{
			if(gpio->EdgeInt == GPIO_INT_EDGE_RISING)				LPC_GPIOINT->IO0IntEnR |= (uint32_t)(gpio->Pins);
			else if(gpio->EdgeInt == GPIO_INT_EDGE_FALLING)	LPC_GPIOINT->IO0IntEnF |= (uint32_t)(gpio->Pins);
			else if(gpio->EdgeInt == GPIO_INT_EDGE_BOTH)	  LPC_GPIOINT->IO0IntEnF |= LPC_GPIOINT->IO0IntEnR |= (uint32_t)(gpio->Pins);
			else if(gpio->EdgeInt == GPIO_INT_EDGE_DISABLE)	LPC_GPIOINT->IO0IntEnR &= LPC_GPIOINT->IO0IntEnF &= ~(uint32_t)(gpio->Pins);
			
			LPC_GPIOINT->IO0IntClr |= (uint32_t)(gpio->Pins);
		}
		if(GPIOx == LPC_GPIO2)
		{
			if(gpio->EdgeInt == GPIO_INT_EDGE_RISING)				LPC_GPIOINT->IO2IntEnR |= (uint32_t)(gpio->Pins);
			else if(gpio->EdgeInt == GPIO_INT_EDGE_FALLING)	LPC_GPIOINT->IO2IntEnF |= (uint32_t)(gpio->Pins);	
			else if(gpio->EdgeInt == GPIO_INT_EDGE_BOTH)	  LPC_GPIOINT->IO2IntEnF |= LPC_GPIOINT->IO2IntEnR |= (uint32_t)(gpio->Pins);
			else if(gpio->EdgeInt == GPIO_INT_EDGE_DISABLE) LPC_GPIOINT->IO2IntEnR &= LPC_GPIOINT->IO2IntEnF &= ~(uint32_t)(gpio->Pins);
			
			LPC_GPIOINT->IO2IntClr |= (uint32_t)(gpio->Pins);
		}
	}
	
	if(gpio->Mask == GPIO_MASK_LOCK)  GPIOx->FIOMASK |= (uint32_t)(gpio->Pins);   
	else															GPIOx->FIOMASK &= ~(uint32_t)(gpio->Pins);
}


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_GPIO_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
