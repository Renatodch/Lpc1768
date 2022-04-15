/* Includes ------------------------------------------------------------------*/
#include "CLib.h"
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_Str(char *s1, char *name, char* val)
{
      strcat(s1, name);   
      strcat(s1, "=");      
      strcat(s1, val);
      strcat(s1, " ");   
}

/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_Uint(char *s1, char *name, uint32_t i)
{
      char val[32];
   
      sprintf(val, "%d ", i);
      strcat(s1, name);   
      strcat(s1, "=");         
      strcat(s1, val);      
}

/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_Float(char *s1, char *name, float f)
{
      char val[32];
   
      sprintf(val, "%4.1f ", f);
      strcat(s1, name);   
      strcat(s1, "=");         
      strcat(s1, val);      
}


/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_StrToLine(char *s1, char *name, char* val)
{
      strcat(s1, name);   
      strcat(s1, "=");      
      strcat(s1, val);
      strcat(s1, "\r\n");   
}

/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_UintToLine(char *s1, char *name, uint32_t i)
{
      char val[32];
   
      sprintf(val, "%d\r\n", i);
      strcat(s1, name);   
      strcat(s1, "=");         
      strcat(s1, val);      
}


/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Prm_AddInt(char *s1, char *name, int i)
{
		char val[32];
	
		sprintf(val, "%d\r\n", i);
		strcat(s1, name);	
		strcat(s1, "=");			
		strcat(s1, val);		
}


/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Prm_AddStr(char *s1, char *name, char* val)
{
		strcat(s1, name);	
		strcat(s1, "=");		
		strcat(s1, val);
		strcat(s1, "\r\n");	
}


/*******************************************************************************
* Function Name  : Str_Cpy
* Description    : Cat s2 to s1.
* Input          : s1.
* Input          : s2.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Add_FloatToLine(char *s1, char *name, float f)
{
      char val[32];
   
      sprintf(val, "%4.1f\r\n", f);
      strcat(s1, name);   
      strcat(s1, "=");         
      strcat(s1, val);      
}

/*******************************************************************************
* Function Name  : Parameters_Get_StrWhitDefault
* Description    : Get parametre from data.
* Input          : The data, name the parameter, where save the val and the default value if not find the parameter.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Get_StrWhitDefault(char *data, char* name, char *val, char *_default)
{
	String_Between(data, name, "\r\n", val);
	
	if(strcmp(val, STR_EMPTY) == 0)
		strcpy(val, _default);
}

/*******************************************************************************
* Function Name  : Parameters_Set
* Description    : Set parametre to data.
* Input          : params: Parameters where will be copied.
* Input          : name: the parameter name.
* Input          : paramValue: the parameter value.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Set(char* params, char* name, char *paramValue)
{
	char result[256];
	char* scan;
	
	strcpy(result, "");
	
	scan = strstr(params, name);
	
	if(scan == 0)
	{
		strcat(result, params);	
		strcat(result, name);
		strcat(result, paramValue);
		strcat(result, "\r\n");		
		strcpy(params, result);		
		return;
	}
	

	String_Between(params, "", name, result);
	
	strcat(result, name);
	strcat(result, paramValue);
	strcat(result, "\r\n");	
	
	scan = strstr(scan, "\r\n");
	strcat(result, scan + 2);		
	
	strcpy(params, result);
	//T("params: %s",params);
}

/*******************************************************************************
* Function Name  : Parameters_Set_Line
* Description    : Copy the line in params.
* Input          : params: Parameters where will be copied the line.
* Input          : line: the parameter to copy.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Set_Line(char* params, char* line)
{
	char* scan;
	char name[64];
	char value[64];
	
	scan = strstr(line, "\r\n");
	if(scan == 0)
	{		
		strcat(line, "\r\n");		
	}	

	String_Between(line, "", "=", name);
	strcat(name, "=");
	//T("name: %s", name);
	
	String_Between(line, "=", "\r\n", value);
	//T("value: %s", value);
	
	Parameters_Set(params, name, value);
}

/*******************************************************************************
* Function Name  : Parameters_Set_Lines
* Description    : Copy the paramsIn to paramsOut.
* Input          : paramsIn: Parameters that would be copied.
* Input          : paramsOut: Parameters where will be copied.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Parameters_Set_Lines(char* paramsIn, char* paramsOut)
{
	char line[64];
	char *token;
	
	token = strtok(paramsIn, "\r\n");
	
	while(  token != NULL ) 
	{   
			//strcat(token, "\r\n");
			strcpy(line, token);
			Parameters_Set_Line(paramsOut, line);
			//T("line = %s", line);
			token = strtok(NULL, "\r\n");				 
	}		
}

/*******************************************************************************
* Function Name  : Parameters_Compare
* Description    : Compare if the paramsIn has the same value that in parmasOut.
* Input          : paramsIn: Parameters that would be compared.
* Input          : paramsOut: Parameters where will be compared.
* Output         : None.
* Return         : bool: 1 - If they are equal. 0 - If they are not equal
*******************************************************************************/
uint8_t Parameters_Compare(char* paramsIn, char* paramsOut)
{
	char *token;
	uint8_t result = 1;
	
	token = strtok(paramsIn, "\r\n");
	
	while(  token != NULL ) 
	{   
			if ( strstr(paramsOut, token) == NULL )
						result = 0;

			token = strtok(NULL, "\r\n");				 
	}

	return result;
}

