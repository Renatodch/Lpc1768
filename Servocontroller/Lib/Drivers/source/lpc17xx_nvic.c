/**********************************************************************
* $Id$		lpc17xx_nvic.c				2018-03-04
*//**
* @file		lpc17xx_nvic.c
* @brief	Contains all expansion functions support for
* 			NVIC firmware library on LPC17xx. The main
* 			NVIC functions are defined in core_cm3.h
* @version	2.0
* @date		04. April. 2018
* @author	Renato Dc
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*/

/* Peripheral group ----------------------------------------------------------- */
/** @addtogroup NVIC
 * @{
 */

/* Includes ------------------------------------------------------------------- */
#include "lpc17xx_nvic.h"

/* Private Macros ------------------------------------------------------------- */
/** @addtogroup NVIC_Private_Macros
 * @{
 */

/* Vector table offset bit mask */
#define NVIC_VTOR_MASK              0x3FFFFF80

/**
 * @}
 */


/* Public Functions ----------------------------------------------------------- */
/** @addtogroup NVIC_Public_Functions
 * @{
 */

void NVIC_Init(NVIC_InitTypeDef * NVIC_InitStructure)
{
	uint8_t tmppriority = 0x00, tmppre = 0x00, tmpsub = 0x1F;
	
	if(NVIC_InitStructure->Status == ENABLE)
	{
    tmppriority = (0x700 - ((SCB->AIRCR) & (uint32_t)0x700))>> 0x08;  /*0 , 1, ,2, 3, 4, 5*/
    tmppre = (0x5 - tmppriority);
    tmpsub = tmpsub >> tmppriority;

    tmppriority = NVIC_InitStructure->PrePriority << tmppre;
    tmppriority |=  (uint8_t)(NVIC_InitStructure->SubPriority& tmpsub);
        
    tmppriority = tmppriority << 0x04;
        
    NVIC->IP[NVIC_InitStructure->IRQHandler] = tmppriority;
		
    NVIC->ISER[((uint32_t)(NVIC_InitStructure->IRQHandler) >> 5)] = (1 << ((uint32_t)(NVIC_InitStructure->IRQHandler) & 0x1F)); /* enable interrupt */
	}
		
	else	
	{
		NVIC->ICER[((uint32_t)(NVIC_InitStructure->IRQHandler) >> 5)] = (1 << ((uint32_t)(NVIC_InitStructure->IRQHandler) & 0x1F)); /* disable interrupt */
	}
}



/*****************************************************************************//**
 * @brief		De-initializes the NVIC peripheral registers to their default
 * 				reset values.
 * @param		None
 * @return      None
 *
 * These following NVIC peripheral registers will be de-initialized:
 * - Disable Interrupt (32 IRQ interrupt sources that matched with LPC17xx)
 * - Clear all Pending Interrupts (32 IRQ interrupt source that matched with LPC17xx)
 * - Clear all Interrupt Priorities (32 IRQ interrupt source that matched with LPC17xx)
 *******************************************************************************/
void NVIC_DeInit(void)
{
	uint8_t tmp;

	/* Disable all interrupts */
	NVIC->ICER[0] = 0xFFFFFFFF;
	NVIC->ICER[1] = 0x00000001;
	/* Clear all pending interrupts */
	NVIC->ICPR[0] = 0xFFFFFFFF;
	NVIC->ICPR[1] = 0x00000001;

	/* Clear all interrupt priority */
	for (tmp = 0; tmp < 32; tmp++) {
		NVIC->IP[tmp] = 0x00;
	}
}

/*****************************************************************************//**
 * @brief			De-initializes the SCB peripheral registers to their default
 *                  reset values.
 * @param			none
 * @return 			none
 *
 * These following SCB NVIC peripheral registers will be de-initialized:
 * - Interrupt Control State register
 * - Interrupt Vector Table Offset register
 * - Application Interrupt/Reset Control register
 * - System Control register
 * - Configuration Control register
 * - System Handlers Priority Registers
 * - System Handler Control and State Register
 * - Configurable Fault Status Register
 * - Hard Fault Status Register
 * - Debug Fault Status Register
 *******************************************************************************/
void NVIC_SCBDeInit(void)
{
	uint8_t tmp;

	SCB->ICSR = 0x0A000000;
	SCB->VTOR = 0x00000000;
	SCB->AIRCR = 0x05FA0000;
	SCB->SCR = 0x00000000;
	SCB->CCR = 0x00000000;

	for (tmp = 0; tmp < 32; tmp++) {
		SCB->SHP[tmp] = 0x00;
	}

	SCB->SHCSR = 0x00000000;
	SCB->CFSR = 0xFFFFFFFF;
	SCB->HFSR = 0xFFFFFFFF;
	SCB->DFSR = 0xFFFFFFFF;
}


/*****************************************************************************//**
 * @brief		Set Vector Table Offset value
 * @param		offset Offset value
 * @return      None
 *******************************************************************************/
void NVIC_SetVTOR(uint32_t offset)
{
//	SCB->VTOR  = (offset & NVIC_VTOR_MASK);
	SCB->VTOR  = offset;
}

/**
 * @}
 */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
