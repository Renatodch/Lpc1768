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
* Function Name  : Uint16_Checksum
* Description    : Genera checksum and save in two bytes (Uint16)
* Input          : The bytes and length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint16_t Uint16_Checksum(uint8_t *pBytes, uint32_t len)
{
		int i;
		uint16_t sum;
		sum = 0;
		
		for(i=0; i<len; i++)
		{
			sum += pBytes[i];
		}
		
		return sum;
}

/*******************************************************************************
* Function Name  : Ascii_To_Digit
* Description    : Covert ascii to hex.
* Input          : ascii: the ascii character.
* Output         : None.
* Return         : uint8: the hex value.
*******************************************************************************/
uint8_t Ascii_To_Digit(uint8_t ascii)
{
		if( ascii >= 0x30 && ascii < 0x3A )
		{
			return ascii & 0x0F;
		}
		
		return ascii - 55;
}



