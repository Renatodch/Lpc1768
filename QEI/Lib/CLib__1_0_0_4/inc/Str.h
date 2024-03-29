/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STR_H
#define __STR_H

#ifdef __cplusplus
 extern "C" 
{
#endif

#include "lpc17xx.h"

#define STR_EMPTY 	""

	
void String_Between(char *data, char *start, char *end, char *val);	
void String_Remplace(char *dest, char *src, char *delim, char *remplaze);
void String_Trim_Total(char *data);
char String_Has(char *data);
char String_Is_Empty(char *data);
unsigned char	Str_NotEqual(char *str1, char *str2);
unsigned char String_Compare(char *str1, char *str2);
	
#ifdef __cplusplus
}
#endif

#endif 
