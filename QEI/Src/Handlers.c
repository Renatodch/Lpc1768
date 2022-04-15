#include "main.h"
#include "Handlers.h"

extern uint32_t c[10];
extern uint8_t cx[10];
extern uint8_t cy[10];
extern uint8_t cz[10];

extern uint8_t px;
extern uint8_t py;
extern uint8_t pz;

extern uint32_t ind;

//extern char Buffer_RAM2[32000] __attribute__((at(0x2007C000))); 

void Button_Handler(void)
{

		
}

void USB_Receiver_Handler(char *buf, int len)
{
	USB_Trace(buf);
	
	if(strstr(buf,"test")!=NULL)
	{
		ind = 0;
		px = cx[ind];
		py = cy[ind];
		pz = cz[ind];	

		while(!px && !py && !pz)
		{	
			ind++;
			
			px = cx[ind];
			py = cy[ind];
			pz = cz[ind];			
		}
		
		if( px || py || pz ) 
		{
			LPC_PWM1->MR0 = c[ind];
			
			if(px)
			{
				LPC_PWM1->MR2 = c[ind] >> 1;
				px--;
			}
			else LPC_PWM1->MR2 =  0;
			if(py)
			{
				LPC_PWM1->MR3 = c[ind] >> 1;
				py--;
			}
			else LPC_PWM1->MR3 =  0;		
			if(pz)
			{
				LPC_PWM1->MR4 = c[ind] >> 1;
				pz--;
			}
			else LPC_PWM1->MR4 =  0;
			
			LPC_PWM1->LER |= (1<<0 | 1<<2 | 1<<3 | 1<<4);
			LPC_PWM1->TCR |= PWM_TCR_COUNTER_RESET;
			LPC_PWM1->TCR &= (~PWM_TCR_COUNTER_RESET) & PWM_TCR_BITMASK;		
			
			LPC_PWM1->MCR |= 0x01;
				
		}
	}
}










