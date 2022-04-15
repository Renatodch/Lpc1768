#include "Button.h"

/*******************************************************************************
* Function Name  : Button_Fsm
* Description    : Handle the events of button.
* Input          : Event of button.
* Output         : None.
* Return         : None.
*******************************************************************************/
void 	Button_Fsm(Button *p, BUTTON_EVT e)
{
		switch(p->State)
		{
			
			case Button_Ste_Low  :
				
						switch (e)
						{
							case BUTTON_EVT_HIGH:
									Timer_Start(&p->debounceTimer);
									p->State = Button_Ste_High;
								break;
						}
			break; 
						
			case Button_Ste_High  :
				
						switch (e)
						{
							case BUTTON_EVT_HIGH:
									Timer_Start(&p->debounceTimer);
								break;
							
							case BUTTON_EVT_DEBOUNCE_TIMEOUT:
								  p->FuncEdgeHandler();
									p->State = Button_Ste_Low;
								break;
						}
			break; 
		}	
}	

/*******************************************************************************
* Function Name  : Button_Events
* Description    : Verify the events.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Button_Events(Button *p)
{
		if ( 	Timer_Elapsed( &p->debounceTimer ) )	
		{
				Button_Fsm(p, BUTTON_EVT_DEBOUNCE_TIMEOUT);
		}	
}


/*******************************************************************************
* Function Name  : Button_Init
* Description    : Initialize the Button
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Button_Init(Button *p, FuncEdgeHandler FuncEdgeHandler)
{
		p->FuncEdgeHandler = FuncEdgeHandler;
		//p->DebounceTimer = 0;
		Timer_Init(&p->debounceTimer, 100000);
		p->State = Button_Ste_Low;
}
