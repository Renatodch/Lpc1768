#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t myTask1Handle = NULL;
TaskHandle_t myTask2Handle = NULL;
TaskHandle_t myTask3Handle = NULL;

int fputc(int c, FILE *f) {
  return(ITM_SendChar(c));
}

void myTask1(void * p)
{
	int c = (int *) p;
	while(1)
	{
		printf("This is from Task1 c=%d\r\n",c++);
		if(c == 30)
			//vTaskDelete(myTask1Handle);
			vTaskSuspend(myTask1Handle); // Or vTaskSuspend(NULL) suspends task itself 
			//vTaskResume(myTask1Handle);
		vTaskDelay(100);
	}
}

void test(void * p)
{
	if(p == (void *)20) printf("New Knowledge\r\n");
}

int main (void)
{
	int StartValue = 20;
	unsigned int i;
	
	test((void *)StartValue);
	
	xTaskCreate(myTask1, "Task1", 200, (void *)StartValue, tskIDLE_PRIORITY, &myTask1Handle);
	
	while(--StartValue)
	{
		for(i = 0 ; i<100000000 ; i++){};
		printf("Hello\r\n");
	}
	
	vTaskStartScheduler();
	
	while(1);

}



