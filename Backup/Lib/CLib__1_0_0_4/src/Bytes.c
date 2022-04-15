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
* Function Name  : Bytes_To_Uint32
* Description    : Swap the bytes (Two by two) in the buffer.
* Input          : Pointer to buffer and the length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t Bytes_ToUint32(uint8_t *bytes)
{
			uint32_t uint  = bytes[3] | ( (uint32_t)bytes[2] << 8 ) | ( (uint32_t)bytes[1] << 16 ) | ( (uint32_t)bytes[0] << 24 );		
			return uint;
}


/*******************************************************************************
* Function Name  : Bytes_Swap_EachTwoBytes
* Description    : Swap the bytes (Two by two) in the buffer.
* Input          : Pointer to buffer and the length.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Bytes_Swap_EachTwoBytes(char *bytes, uint32_t length)
{
	char byte;
	int i;

	for (i = 0; i < (length/2); i++)
	{
		byte = bytes[i * 2];
		bytes[i * 2] = bytes[ (i * 2) + 1 ];
		bytes[ (i * 2) + 1 ] = byte;
	}
}

/*******************************************************************************
* Function Name  : Bytes_Print_ToHex
* Description    : Print bytes to hex.
* Input          : The bytes and length.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Bytes_Print_ToHex(uint8_t *pBytes, uint32_t len)
{
	int i;
	char buf[1024];
	char s[12];	

		strcpy(buf, "");

		for(i=0; i<len; i++)
		{
				strcpy(s, "");
				sprintf(s, "0x%02X  ", pBytes[i]);
			
				strcat(buf, s);
		}
		
		//T("%s", buf);
}




