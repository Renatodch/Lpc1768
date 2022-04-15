/**********************************************************************
* $Id$		lpc17xx_pinsel.h				2010-05-21
*//**
* @file		lpc17xx_pinsel.h
* @brief	Contains all macro definitions and function prototypes
* 			support for Pin connect block firmware library on LPC17xx
* @version	2.0
* @date		21. May. 2010
* @author	Renato Dc
*
* All rights reserved.
*/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup PINSEL PINSEL (Pin Selection)
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_PINSEL_H_
#define LPC17XX_PINSEL_H_

#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/***********************************************************************
 * Macros define for I2C mode
 ***********************************************************************/
#define	PINSEL_I2C_Normal_Mode		((0))	/**< The standard drive mode */
#define	PINSEL_I2C_Fast_Mode		((1)) 	/**<  Fast Mode Plus drive mode */

/**
 * @}
 */

/* Private Macros ------------------------------------------------------------- */
/** @defgroup PINSEL_Private_Macros PINSEL Private Macros
 * @{
 */

/* Pin selection define */
/* I2C Pin Configuration register bit description */
#define PINSEL_I2CPADCFG_SDADRV0 	_BIT(0) /**< Drive mode control for the SDA0 pin, P0.27 */
#define PINSEL_I2CPADCFG_SDAI2C0	_BIT(1) /**< I2C mode control for the SDA0 pin, P0.27 */
#define PINSEL_I2CPADCFG_SCLDRV0	_BIT(2) /**< Drive mode control for the SCL0 pin, P0.28 */
#define PINSEL_I2CPADCFG_SCLI2C0	_BIT(3) /**< I2C mode control for the SCL0 pin, P0.28 */


typedef enum{
	PIN_FUNC_DEFAULT,
	PIN_FUNC_1,
	PIN_FUNC_2,
	PIN_FUNC_3
}	AlTERNATE_FUNC_e;

typedef enum{
	PIN_MODE_PU,
	PIN_MODE_R,
	PIN_MODE_FLOATING,
	PIN_MODE_PD
}
MODE_e;

typedef enum{
	PIN_NO_OD,
	PIN_OD
}OD_e;

typedef enum{
	PIN_GPIO_0,
	PIN_GPIO_1,
	PIN_GPIO_2,
	PIN_GPIO_3,
	PIN_GPIO_4
}	GPIOn_e;

typedef struct
{
	uint8_t Pinnum;		/**< Pin Number, should be PINSEL_PIN_x,
						where x should be in range from 0 to 31 */
	GPIOn_e Portnum;	/**< Port Number, should be PINSEL_PORT_x,
						where x should be in range from 0 to 4 */
	AlTERNATE_FUNC_e Funcnum;	/**< Function Number, should be PINSEL_FUNC_x,
						where x should be in range from 0 to 3 */
	MODE_e Pinmode;	/**< Pin Mode, should be:
						- PINSEL_PINMODE_PULLUP: Internal pull-up resistor
						- PINSEL_PINMODE_TRISTATE: Tri-state
						- PINSEL_PINMODE_PULLDOWN: Internal pull-down resistor */
	OD_e OpenDrain;	/**< OpenDrain mode, should be:
						- PINSEL_PINMODE_NORMAL: Pin is in the normal (not open drain) mode
						- PINSEL_PINMODE_OPENDRAIN: Pin is in the open drain mode */
} PINSEL_InitTypeDef;


/******** PinConnect Definitions ***********/
/****** PINSEL ********** MODE **************
/00:		default					Pull-Up	
/01:		function 1			Retainer Level
/10:		function 2			Floating
/11:    function 3			Pull-Down 
******************OD************************
/0:     Normal
/1:     OpenDrain
********************************************/

__STATIC_INLINE void PINCON_Init(PINSEL_InitTypeDef * pincfg)
{	
	uint8_t Port 		   = (uint8_t)pincfg->Portnum;
	uint8_t PinNum     = pincfg->Pinnum;
	uint8_t Od_Port    = (uint8_t)pincfg->Portnum;
	uint8_t Od_PinNum  = pincfg->Pinnum;
	
	uint32_t *reg_pinsel = (uint32_t*)&LPC_PINCON->PINSEL0;
	uint32_t *reg_mode 	 = (uint32_t*)&LPC_PINCON->PINMODE0;
	uint32_t *reg_od 		 = (uint32_t*)&LPC_PINCON->PINMODE_OD0;
	
	Port*= 2;
	
	if(PinNum>= 16)
	{
		Port ++;
		PinNum -= 16;
	}
	
	PinNum *= 2;
	reg_pinsel += Port;
	reg_mode   += Port;
	
	*reg_pinsel &= ~(uint32_t)(0x03 << PinNum); 
	*reg_pinsel |= (uint32_t)(pincfg->Funcnum << PinNum);
	
	*reg_mode &= ~(uint32_t)(0x03 << PinNum); 
	*reg_mode |= (uint32_t)(pincfg->Pinmode << PinNum);
	
	if(Od_PinNum >= 32)
	{
		Od_Port ++;
		Od_PinNum -= 32;
	}	

	reg_od += Od_Port;
	
	if(pincfg->OpenDrain == PIN_OD) *reg_od |= (uint32_t)(1 << Od_PinNum);
	else 		 												*reg_od &= ~(uint32_t)(1 << Od_PinNum);
}

__STATIC_INLINE void PINCON_I2C(char port, char num , OD_e od){

	/*Pending*/
}		



/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_PINSEL_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */

