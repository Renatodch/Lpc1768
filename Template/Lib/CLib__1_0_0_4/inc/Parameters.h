/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_H
#define __PARAMETERS_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

void Parameters_Add_Str(char *s1, char *name, char* val);
	
void Parameters_Add_Uint(char *s1, char *name, uint32_t i);
	
void Parameters_Add_Float(char *s1, char *name, float f);	
	
void Parameters_Add_StrToLine(char *s1, char *name, char* val);
	
void Parameters_Add_UintToLine(char *s1, char *name, uint32_t i);	
	
void Parameters_Add_FloatToLine(char *s1, char *name, float f);	
	
void Parameters_Get_StrWhitDefault(char *data, char* name, char *val, char *_default);	
	
void Parameters_Set_Lines(char* paramsIn, char* paramsOut);
	
uint8_t Parameters_Compare(char* paramsIn, char* paramsOut);	

void Prm_AddInt(char *s1, char *name, int i);

void Prm_AddStr(char *s1, char *name, char* val);

#ifdef __cplusplus
}
#endif

#endif 
