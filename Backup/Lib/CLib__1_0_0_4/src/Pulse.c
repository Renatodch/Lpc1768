/* Includes ------------------------------------------------------------------*/
#include "CLib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PULSE_BREAK_TIME 	2500
/* Private variables ---------------------------------------------------------*/
uint8_t		pulse_sync_Nivel	= 0;
Timer			pulse_syncronous_Timer;
/* Extern variables ----------------------------------------------------------*/	
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Pulse_Set_Setpoint
* Description    : Set the number of pulses.
* Input          : Pointer to struct and number of pulses.
* Output         : None.
* Return         : None.
*******************************************************************************/
void 		Pulse_Set_Setpoint(Pulse* p, uint32_t setpoint)
{
		 p->setpoint = setpoint;
}


/*******************************************************************************
* Function Name  : Pulse_Fsm
* Description    : Event to state machine.
* Input          : Pointer to struct and event.
* Output         : None.
* Return         : None.
*******************************************************************************/
void 		Pulse_Fsm(Pulse* p, PULSE_EVT event)
{
	switch(p->state)
	{
		case Pulse_State_Pulse  :
			
				Bool_Toggle(&p->nivel);
			
				if (p->nivel)
				{
						Timer_Init(&p->timer, 100);
						Timer_Start(&p->timer);					
				}
				else
				{
						Timer_Init(&p->timer, 300);
						Timer_Start(&p->timer);					
				}
			
				if (p->setpoint)
				{
						p->state = Pulse_State_Break;
					
						Timer_Init(&p->timer, PULSE_BREAK_TIME);
						Timer_Start(&p->timer);					
						p->nivel = 0;
				}
		break; 
					
			
		case Pulse_State_Break  :
			
				p->state = Pulse_State_Count;
		
				p->count = p->setpoint;
				Timer_Init(&p->timer, 100);
				Timer_Start(&p->timer);		
		
		break; 
				
		
		case Pulse_State_Count  :

				Bool_Toggle(&p->nivel);

				Timer_Init(&p->timer, 100);
				Timer_Start(&p->timer);		

				if (p->nivel)
				{

				}	
				else
				{
					if( p->setpoint == 0 )
					{
						p->state = Pulse_State_Pulse ;
						
						p->count = 0;
						break;
					}
					
					
					p->count--;
					
					if (!p->count)
					{
						p->state = Pulse_State_Break;
						
						Timer_Init(&p->timer, PULSE_BREAK_TIME);
						Timer_Start(&p->timer);											
						p->nivel = 0;												
					}
				}
		break; 	

		default : 
		break;
	}
}

/*******************************************************************************
* Function Name  : Pulse_Sync
* Description    : Syncronus the pulse.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void 		Pulse_Sync(void)
{
		if( Timer_Elapsed( &pulse_syncronous_Timer ) )
		{
					Bool_Toggle(&pulse_sync_Nivel);
				
					if (pulse_sync_Nivel)
					{
								Timer_Init(&pulse_syncronous_Timer, 50);
					}
					else
					{
								Timer_Init(&pulse_syncronous_Timer, 200);
					}
					
					Timer_Start(&pulse_syncronous_Timer);					
		}
}

/*******************************************************************************
* Function Name  : Pulse_Get_Nivel
* Description    : Get nivel of pulse.
* Input          : Pointer to struct and number of pulses.
* Output         : None.
* Return         : Nivel.
*******************************************************************************/
uint8_t Pulse_Get_Nivel(Pulse* p)
{
	
		Pulse_Sync();
	
		if (  Timer_Elapsed( &p->timer )  )				
						Pulse_Fsm(p, Pulse_Event_Clock);
	
	
		if (p->state == Pulse_State_Pulse)
				return pulse_sync_Nivel;
		else
				return p->nivel;
}

/*******************************************************************************
* Function Name  : Pulse_Init
* Description    : Function to initilize.
* Input          : Pointer to struct.
* Output         : None.
* Return         : None.
*******************************************************************************/
void 		Pulse_Init(Pulse* p)
{
		p->setpoint = 0;
		p->count = 0;
		p->state = Pulse_State_Pulse;
	
		Timer_Init(&p->timer, 100);
		Timer_Start(&p->timer);
	
		Timer_Init(&pulse_syncronous_Timer, 300);
		Timer_Start(&pulse_syncronous_Timer);	
}



