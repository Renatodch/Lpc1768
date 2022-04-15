/*****************************************************************************
 * Fecha y Hora de Compilacion de Proyecto:
 * ---------------------------------------
 * **** TELCOMIP SAC **** 
 *****************************************************************************/

#include "build_time.h"
/*-----Do not modify, get_build_date_time same with build_date_time() for validity check------*/
char* get_build_date_time(void)
{
	static char build_bin_date_time_str[] = "2018/06/26 00:29:04";
	return build_bin_date_time_str;
}
/*-----Do not modify, oa_lib_build_date_time same with build_date_time() for validity check------*/
char* get_lib_build_date_time(void)
{
	static char build_lib_date_time_str[] = "";
	return build_lib_date_time_str;
}

/*-----Do not modify, oa_lib_build_date_time same with build_date_time() for validity check------*/
char* get_version(void)
{
	static char version_soft_str[] = "QEI";
	return version_soft_str;
}

/********************FIN*****************/

