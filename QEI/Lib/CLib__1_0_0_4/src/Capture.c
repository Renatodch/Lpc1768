#include "CLib.h"

/*******************************************************************************
* Function Name  : Capture
* Description    : 
* Input          : p : pointer to timer struct
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t  	Capture_GetTotalTime(Capture *p)
{
		return (p->highTime + p->lowTime) / 1000;
}

/*******************************************************************************
* Function Name  : Capture
* Description    : 
* Input          : p : pointer to timer struct
* Output         : None.
* Return         : None.
*******************************************************************************/
void  			Capture_Handlers(Capture *p, uint8_t  nivel)
{

			if(p->nivel == 0 && nivel == 1 && p->initNivel == 0)
			//if(p->nivel == 0 && nivel == 1)
			{
					p->risingEdgeHandler(p);
					p->highTime = 0;
			}
		
			if(p->nivel == 1 && nivel == 0 && p->initNivel == 1)
			//if(p->nivel == 1 && nivel == 0)
			{
					p->fallingEdgeHandler(p);
					p->lowTime = 0;
			}
	
		p->nivel = nivel;
}

/*******************************************************************************
* Function Name  : Capture
* Description    : 
* Input          : p : pointer to timer struct
* Output         : None.
* Return         : None.
*******************************************************************************/
void  			Capture_Clock(Capture *p)
{
		if(p->nivel)
			p->highTime++;
		else
			p->lowTime++;
}

/*******************************************************************************
* Function Name  : Capture
* Description    : 
* Input          : p : pointer to timer struct
* Output         : None.
* Return         : None.
*******************************************************************************/
void  			Capture_ToString(Capture *p, char* s)
{
			Parameters_Add_Uint(s, "firstRead", p->firstRead);	
			Parameters_Add_Uint(s, "initNivel", p->initNivel);
			Parameters_Add_Uint(s, "  nivel", p->nivel);
			Parameters_Add_Uint(s, "  highTime", p->highTime / 1000);
			Parameters_Add_Uint(s, "  lowTime", p->lowTime / 1000);	
			Parameters_Add_UintToLine(s, "  pulses", p->pulses);	
			//Parameters_Add_Uint(s, "  fallingEdgeHandler", (int) p->fallingEdgeHandler);
			//Parameters_Add_UintToLine(s, "  risingEdgeHandler", (int) p->risingEdgeHandler);
}

/*******************************************************************************
* Function Name  : Capture
* Description    : 
* Input          : p : pointer to timer struct
* Output         : None.
* Return         : None.
*******************************************************************************/
void  			Capture_Init(Capture *p, uint8_t  initNivel, RisingEdgeHandler risingEdgeHandler, FallingEdgeHandler fallingEdgeHandler)
{
		p->highTime = 0; 
		p->lowTime = 0;
		p->nivel = initNivel;
		p->risingEdgeHandler = risingEdgeHandler;
		p->fallingEdgeHandler = fallingEdgeHandler;
		p->initNivel = initNivel;
		p->firstRead = 1;
		p->pulses = 0;
		p->pulseFlag=0;
}







