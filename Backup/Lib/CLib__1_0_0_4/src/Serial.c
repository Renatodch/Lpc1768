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
* Function Name  : Serial_Events
* Description    : Verify if have data
* Input          : Pointer to serial.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Serial_Events(Serial *p)
{
	uint32_t length;
	char buffer[512]={0};
	
	if( Fifo_OnInactive( &p->fifo ) )
	{		
			length = Fifo_Read( &p->fifo,buffer,FIFO_SIZE); 
		
			if (length)
			{
				p->receiver_EventHandler(buffer, length);	
			}
	}	
}

/*******************************************************************************
* Function Name  : Serial_Send
* Description    : Send data.
* Input          : Pointer to serial.
* Input          : msg: the message.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Serial_Send(Serial *p, char *msg)
{
//	USART_TX(p->Port, msg);
}

/*******************************************************************************
* Function Name  : Serial_Init
* Description    : Initializa the serial.
* Input          : 'p' Pointer to serial.
* Input          : 'port' the port name.
* Input          : 'receiver_EventHandler' the funtion to call when have data.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Serial_Init(Serial *p, Receiver_EventHandler receiver_EventHandler)
{
	Fifo_Init(&p->fifo);
	p->receiver_EventHandler = receiver_EventHandler;

}

void Usb_InitVar(Serial *p, Receiver_EventHandler receiver_EventHandler)
{
p->fifo.head = 0;
p->fifo.tail = 0;
p->fifo.size = FIFO_SIZE;
p->fifo.cnt_FreeWrite = 0;
p->fifo.tmr_Inactive = 10;
p->fifo.cnt_FreeRead = 0;
Timer_Init(&p->fifo.inactive_Timer, p->fifo.tmr_Inactive);
p->receiver_EventHandler = receiver_EventHandler;
}
