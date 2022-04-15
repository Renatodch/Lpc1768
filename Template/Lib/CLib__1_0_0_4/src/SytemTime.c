#include "SystemTime.h"

SystemTime st;

uint32_t SystemTime_GetTime()
{
		return st.time;
}

void SystemTime_Timer()
{
		st.time++;
}

void SystemTime_To_String(char* str)
{
				int timeInSeconds = st.time / 1000;
	
				int seg =  timeInSeconds % 60;
	
				int timeInMinutes = timeInSeconds / 60;		
				int min = timeInMinutes % 60;
	
				int hora = timeInMinutes / 60;	
			
				sprintf(str, "%d:%02d:%02d", hora, min, seg);
}

void SystemTime_Init()
{
		st.time = 0;
}


