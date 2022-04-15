/**********************************************************************
* $Id$		lpc17xx_exti.c				2018-04-03
*//**
* @file		lpc17xx_exti.c
* @brief	Contains all functions support for External interrupt firmware
* 			library on LPC17xx
* @version	3.0
* @date		03. April. 2018
* @author	Renato Dc
* All rights reserved.
*/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup EXTI
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_exti.h"


/* Public Functions ----------------------------------------------------------- */
/** @addtogroup EXTI_Public_Functions
 * @{
 */


/*********************************************************************//**
 * @brief 		Configuration for EXT
 * 				- Set EXTINT, EXTMODE, EXTPOLAR register
 * @param[in]	EXTICfg	Pointer to a EXTI_InitTypeDef structure
 *              that contains the configuration information for the
 *              specified external interrupt
 * @return 		None
 **********************************************************************/
void EXTI_Init(EXTI_InitTypeDef *EXTICfg)
{
	LPC_SC->EXTINT = 0xF;
	LPC_SC->EXTMODE = 0x0;
	LPC_SC->EXTPOLAR = 0x0;
	
	LPC_SC->EXTINT = 0x0;
	
	if(EXTICfg->EXTI_polarity == EXTI_POLARITY_HIGH_ACTIVE_OR_RISING_EDGE)
	{
		LPC_SC->EXTPOLAR |= (1 << EXTICfg->EXTI_Line);
	}
	else if(EXTICfg->EXTI_polarity == EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE)
	{
		LPC_SC->EXTPOLAR &= ~(1 << EXTICfg->EXTI_Line);
	}
	
	if(EXTICfg->EXTI_Mode == EXTI_MODE_EDGE_SENSITIVE)
	{
		LPC_SC->EXTMODE |= (1 << EXTICfg->EXTI_Line);
	}
	else if(EXTICfg->EXTI_Mode == EXTI_MODE_LEVEL_SENSITIVE)
	{
		LPC_SC->EXTMODE &= ~(1 << EXTICfg->EXTI_Line);
	}
	
	EXTI_ClearEXTIFlag(EXTICfg->EXTI_Line);
}

/*********************************************************************//**
* @brief 		Clear External interrupt flag
* @param[in]	EXTILine	 external interrupt line, should be:
* 				- EXTI_EINT0: external interrupt line 0
* 				- EXTI_EINT1: external interrupt line 1
* 				- EXTI_EINT2: external interrupt line 2
* 				- EXTI_EINT3: external interrupt line 3
* @return 		None
*********************************************************************/
void EXTI_ClearEXTIFlag(EXTI_LINE_ENUM EXTILine)
{
		LPC_SC->EXTINT = (1 << EXTILine);
}

/* --------------------------------- End Of File ------------------------------ */

