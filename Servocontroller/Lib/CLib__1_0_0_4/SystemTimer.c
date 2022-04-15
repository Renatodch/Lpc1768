#include "Lib.h"

char SystemTimer3__Is_Timeout(SystemTimer* p)
{
		uint32_t currentTime = SystemClock__Get();
	
		if( (currentTime - p->start) >= p->timeuot){
				p->start = currentTime;
				return 1;
		}
		
		return 0;
}

void SystemTimer3__Init(SystemTimer* p)
{
		
}


