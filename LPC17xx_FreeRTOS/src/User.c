#include "main.h"

//Stack and heap comes after at end of SRAM1. IRAM2 unselected as default
char Buffer_RAM2[4000] __attribute__((at(0x2007C000))) = {0};

void Config_Variables(void)
{

}






