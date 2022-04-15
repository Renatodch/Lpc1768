/* Includes ------------------------------------------------------------------*/
#include "CLib.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/	
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Bool_Toggle
* Description    : Toggle the bool.
* Input          : Pointer to bool.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Bool_Toggle(uint8_t* pBool)
{
		if(*pBool)
				*pBool = 0;
		else
				*pBool = 1;
}