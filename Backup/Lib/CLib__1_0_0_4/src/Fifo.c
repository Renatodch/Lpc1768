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
* Function Name  : Fifo_OnInactive
* Description    : Verify that the fifo had end of receiver data.
* Input          : Pointer to fifo.
* Output         : None.
* Return         : None.
*******************************************************************************/
char Fifo_OnInactive(Fifo * f)
{
	if( Timer_Elapsed(&f->inactive_Timer) )
	{
		//f->tmr_Inactive = 0;
		return 1;
	}
	return 0;
}


/*******************************************************************************
* Function Name  : Fifo_Read
* Description    : This reads nbytes bytes from the FIFO.
* Input          : Pointer to fifo, the buffer read and bytes count.
* Output         : None.
* Return         : The number of bytes read is returned.
*******************************************************************************/
int Fifo_Read(Fifo * f, void * buf, int nbytes)
{
    int i;
    char * p;
	
    p = buf;
    for(i=0; i < nbytes; i++)
		{
        if( f->tail != f->head )
				{ //see if any data is available
              *p++ = f->buf_w[f->tail];  //grab a byte from the buffer
              f->tail++;  //increment the tail
							f->cnt_FreeRead++;
              if( f->tail == f->size )
							{  		//check for wrap-around
                    f->tail = 0;
              }
        } 
				else 
				{
            return i; //number of bytes read 
        }
     }
     return nbytes;
}

/*******************************************************************************
* Function Name  : Fifo_Write
* Description    : This writes up to nbytes bytes to the FIFO. 
*										If the head runs in to the tail, not all bytes are written.
* Input          : Pointer to fifo, the buffer write and bytes count.
* Output         : None.
* Return         : The number of bytes written is returned.
*******************************************************************************/
int Fifo_Write(Fifo * f, const void * buf, int nbytes)
{
     int i;
     const char * p;
     p = buf;
     for(i=0; i < nbytes; i++)
		 {
           //first check to see if there is space in the buffer
           if( (f->head + 1 == f->tail) || ( (f->head + 1 == f->size) && (f->tail == 0) ) ){
                 return i; //no more room
           } 
					 else 
					 {
								f->buf_w[f->head] = *p++;
								f->head++;  //increment the head
								f->cnt_FreeWrite++;
						
								Timer_Start(&f->inactive_Timer);
						
								if( f->head == f->size )
								{  //check for wrap-around
                    f->head = 0;
								}
           }
     }
     return nbytes;
}

/*******************************************************************************
* Function Name  : Fifo_Init
* Description    : Init the fifo
* Input          : Pointer to fifo.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Fifo_Init(Fifo * f)
{
		f->head = 0;
		f->tail = 0;
		f->size = FIFO_SIZE;
		f->cnt_FreeWrite = 0;
		f->tmr_Inactive = 0;
		f->cnt_FreeRead = 0;
		Timer_Init(&f->inactive_Timer, 20);
}
