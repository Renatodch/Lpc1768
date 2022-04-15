/**********************************************************************
* $Id$		lpc17xx_nvic.h				2010-05-21
*//**
* @file		lpc17xx_nvic.h
* @brief	Contains all macro definitions and function prototypes
* 			support for Nesting Vectored Interrupt firmware library
* 			on LPC17xx
* @version	2.0
* @date		21. May. 2010
* @author	NXP MCU SW Application Team
*
* Copyright(C) 2010, NXP Semiconductor
* All rights reserved.
*/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup NVIC NVIC (Nested Vectored Interrupt Controller)
 * @ingroup LPC1700CMSIS_FwLib_Drivers
 * @{
 */

#ifndef LPC17XX_NVIC_H_
#define LPC17XX_NVIC_H_

/* Includes ------------------------------------------------------------------- */
#include "LPC17xx.h"
#include "lpc_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	IRQn_Type IRQHandler;
	uint32_t PrePriority;
	uint32_t SubPriority;
	FunctionalState Status;
}NVIC_InitTypeDef;


/* Public Functions ----------------------------------------------------------- */
/** @defgroup NVIC_Public_Functions NVIC Public Functions
 * @{
 */
void NVIC_Init(NVIC_InitTypeDef * NVIC_InitStructure);
void NVIC_DeInit(void);
void NVIC_SCBDeInit(void);
void NVIC_SetVTOR(uint32_t offset);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* LPC17XX_NVIC_H_ */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
