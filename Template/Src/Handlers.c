#include "main.h"
#include "Handlers.h"



Excelente *p ;
char msg[16]={0x2D, 0x43, 0x30, 0x30, 0x52, 0x75, 0x01, 0x0D, 0x00, 0x02, 0x21, 0x05,0x06,0x01,0x02,0x03}; 
int x = 0xF5;
int y = 0xFF;
int *a;


void Button_Handler(void)
{
	a = (int *)&y;
	p = (void *)msg;
	
	USB_Trace("%X\r\n",p->uno);
	USB_Trace("%X\r\n",p->dos);
	USB_Trace("%X\r\n",p->tres);
	USB_Trace("%X\r\n",p->cuatro);
	USB_Trace("%X\r\n",p->cinco);
	USB_Trace("%X\r\n",p->seis);
	USB_Trace("%X\r\n",p->siete);
	USB_Trace("%X\r\n",p->ocho);
	USB_Trace("%X\r\n",p->boundarie);
	
}

void USB_Receiver_Handler(char *buf, int len)
{
	float T,F;
	USB_Trace(buf);
	
	if(strstr(buf,"num=")!=NULL)	
	{
		uint32_t mr0 = atoi(buf+4);
		//T = mr0/100000000;
		F = 100000000/mr0;
		USB_Trace("Valor de mr0 = %d",mr0);
		USB_Trace("Valor de F = %f",F);
		
		PWM_MatchUpdate(LPC_PWM1, 0, mr0, PWM_MATCH_UPDATE_NOW);
		PWM_MatchUpdate(LPC_PWM1, 2, mr0 >> 1, PWM_MATCH_UPDATE_NOW);
		PWM_MatchUpdate(LPC_PWM1, 3, mr0 >> 1, PWM_MATCH_UPDATE_NOW);
		PWM_MatchUpdate(LPC_PWM1, 4, mr0 >> 1, PWM_MATCH_UPDATE_NOW);
		
		
	}
	
		
	
	
	
	
	
	
	

}
