#include "main.h"
#include "CLib.h"

//Stack and heap comes after at end of SRAM1. IRAM2 unselected as default
char Buffer_RAM2[4000] __attribute__((at(0x2007C000))) = {0};
char fsf[2121];
char buf[2121];
int hola[21];

void Loop(void)
{
	
	
	delay(10);
}


void Variables(void)
{

}

void Config(void)
{


}


