#include "Lib.h"

void Task__Call(Task* p)
{
			if(p->taskFunction == NULL)
					return;
			
			if(SystemTimer3__Is_Timeout(&p->systemTimer)){
				
						p->taskFunction();
			}			
}

void Task__Init(Task* p)
{
}


