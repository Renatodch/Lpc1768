#line 1 "Lib\\Drivers\\source\\lpc17xx_iap.c"


 


























 
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_iap.h"


 



























 

#line 1 ".\\Lib\\Drivers\\include\\lpc_types.h"


 





























 

 



 




 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 46 ".\\Lib\\Drivers\\include\\lpc_types.h"


 


 



 
typedef enum {FALSE = 0, TRUE = !FALSE} Bool;



 
typedef enum {RESET = 0, SET = !RESET} FlagStatus, IntStatus, SetState;




 
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;




 
typedef enum {ERROR = 0, SUCCESS = !ERROR} Status;




 
typedef enum
{
	NONE_BLOCKING = 0,		 
	BLOCKING				 
} TRANSFER_BLOCK_Type;


 
typedef void (*PFV)();

 
typedef int32_t(*PFI)();



 


 


 




 

 





 

 














 

 


 




 


 

 


#line 154 ".\\Lib\\Drivers\\include\\lpc_types.h"



 


 


 

 
typedef char CHAR;

 
typedef uint8_t UNS_8;

 
typedef int8_t INT_8;

 
typedef	uint16_t UNS_16;

 
typedef	int16_t INT_16;

 
typedef	uint32_t UNS_32;

 
typedef	int32_t INT_32;

 
typedef int64_t INT_64;

 
typedef uint64_t UNS_64;

 
typedef Bool BOOL_32;

 
typedef Bool BOOL_16;

 
typedef Bool BOOL_8;



 






 

 
#line 36 ".\\Lib\\Drivers\\include\\lpc17xx_iap.h"




 



 

 




 



 



 
typedef enum
{
    IAP_PREPARE = 50,       
    IAP_COPY_RAM2FLASH = 51,     
    IAP_ERASE = 52,              
    IAP_BLANK_CHECK = 53,        
    IAP_READ_PART_ID = 54,       
    IAP_READ_BOOT_VER = 55,      
    IAP_COMPARE = 56,            
    IAP_REINVOKE_ISP = 57,       
    IAP_READ_SERIAL_NUMBER = 58, 
}  IAP_COMMAND_CODE;



 
typedef enum
{
    CMD_SUCCESS,	             
    INVALID_COMMAND,             
    SRC_ADDR_ERROR,              
    DST_ADDR_ERROR,              
    SRC_ADDR_NOT_MAPPED,         
    DST_ADDR_NOT_MAPPED,         
    COUNT_ERROR,	               
    INVALID_SECTOR,	           
    SECTOR_NOT_BLANK,	           
    SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,	
    COMPARE_ERROR,               
    BUSY,		                   
} IAP_STATUS_CODE;



 
typedef enum {
  IAP_WRITE_256  = 256,
  IAP_WRITE_512  = 512,
  IAP_WRITE_1024 = 1024,
  IAP_WRITE_4096 = 4096,
} IAP_WRITE_SIZE;



 
typedef struct {
    uint32_t cmd;   
    uint32_t param[4];      
    uint32_t status;        
    uint32_t result[4];     
} IAP_COMMAND_Type;



 
 
 


 

 
uint32_t GetSecNum (uint32_t adr);
 
IAP_STATUS_CODE PrepareSector(uint32_t start_sec, uint32_t end_sec);
 
IAP_STATUS_CODE CopyRAM2Flash(uint8_t * dest, uint8_t* source, IAP_WRITE_SIZE size);
 
IAP_STATUS_CODE EraseSector(uint32_t start_sec, uint32_t end_sec);
 
IAP_STATUS_CODE BlankCheckSector(uint32_t start_sec, uint32_t end_sec,
                                 uint32_t *first_nblank_loc, 
								 uint32_t *first_nblank_val);
 
IAP_STATUS_CODE ReadPartID(uint32_t *partID);
 
IAP_STATUS_CODE ReadBootCodeVer(uint8_t *major, uint8_t* minor);
 
IAP_STATUS_CODE ReadDeviceSerialNum(uint32_t *uid);
 
IAP_STATUS_CODE Compare(uint8_t *addr1, uint8_t *addr2, uint32_t size);
 
void InvokeISP(void);



 



 



#line 32 "Lib\\Drivers\\source\\lpc17xx_iap.c"
#line 1 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\system_lpc17xx.h"
 





















 









#line 34 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\system_lpc17xx.h"



 

extern uint32_t SystemCoreClock;      










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);







 

#line 33 "Lib\\Drivers\\source\\lpc17xx_iap.c"


typedef void (*IAP)(uint32_t *cmd,uint32_t *result);
IAP iap_entry = (IAP) (0x1FFF1FF1UL);





 
 

 






 
 uint32_t GetSecNum (uint32_t adr)
{
    uint32_t n;

    n = adr >> 12;                               
    if (n >= 0x10) {
      n = 0x0E + (n >> 3);                       
    } 

    return (n);                                  
}

 







 
IAP_STATUS_CODE PrepareSector(uint32_t start_sec, uint32_t end_sec)
{
    IAP_COMMAND_Type command;
    command.cmd    = IAP_PREPARE;                    
    command.param[0] = start_sec;                    
    command.param[1] = end_sec;                      
    iap_entry (&command.cmd, &command.status);        
    return (IAP_STATUS_CODE)command.status;
}

 












 
IAP_STATUS_CODE CopyRAM2Flash(uint8_t * dest, uint8_t* source, IAP_WRITE_SIZE size)
{
    uint32_t sec;
    IAP_STATUS_CODE status;
    IAP_COMMAND_Type command;

	
    sec = GetSecNum((uint32_t)dest);
   	status = PrepareSector(sec, sec);
	if(status != CMD_SUCCESS)
        return status;
   
	
	command.cmd    = IAP_COPY_RAM2FLASH;             
    command.param[0] = (uint32_t)dest;                 
    command.param[1] = (uint32_t)source;               
    command.param[2] =  size;                          
    command.param[3] =  SystemCoreClock / 1000;         
    iap_entry (&command.cmd, &command.status);              
	  
    return (IAP_STATUS_CODE)command.status;             
}

 










 
IAP_STATUS_CODE EraseSector(uint32_t start_sec, uint32_t end_sec)
{
    IAP_COMMAND_Type command;
    IAP_STATUS_CODE status;

	
   	status = PrepareSector(start_sec, end_sec);
	if(status != CMD_SUCCESS)
        return status;

	
    command.cmd    = IAP_ERASE;                    
    command.param[0] = start_sec;                  
    command.param[1] = end_sec;                    
    command.param[2] =  SystemCoreClock / 1000;         
    iap_entry (&command.cmd, &command.status);      
    return (IAP_STATUS_CODE)command.status;  
}

 












 
IAP_STATUS_CODE BlankCheckSector(uint32_t start_sec, uint32_t end_sec,
                                 uint32_t *first_nblank_loc, 
								 uint32_t *first_nblank_val)
{
    IAP_COMMAND_Type command;
	
    command.cmd    = IAP_BLANK_CHECK;                
    command.param[0] = start_sec;                    
    command.param[1] = end_sec;                      
    iap_entry (&command.cmd, &command.status);        

	if(command.status == SECTOR_NOT_BLANK)
	{
	  
	  if(first_nblank_loc != ((void*) 0))
	      *first_nblank_loc =  command.result[0];
	  if(first_nblank_val != ((void*) 0))
	      *first_nblank_val =  command.result[1];
    }

    return (IAP_STATUS_CODE)command.status;
}

 






 
IAP_STATUS_CODE ReadPartID(uint32_t *partID)
{
   IAP_COMMAND_Type command;
   command.cmd = IAP_READ_PART_ID;
   iap_entry (&command.cmd, &command.status);        

   if(command.status == CMD_SUCCESS)
   {
      if(partID != ((void*) 0))
	     *partID = command.result[0];
   }

   return (IAP_STATUS_CODE)command.status;
}

 







 
IAP_STATUS_CODE ReadBootCodeVer(uint8_t *major, uint8_t* minor)
{
   IAP_COMMAND_Type command;
   command.cmd = IAP_READ_BOOT_VER;
   iap_entry (&command.cmd, &command.status);        

   if(command.status == CMD_SUCCESS)
   {
      if(major != ((void*) 0))
	     *major = (command.result[0] >> 8) & 0xFF;
      if(minor != ((void*) 0))
	     *minor = (command.result[0]) & 0xFF;
   }

   return (IAP_STATUS_CODE)command.status;
}

 






 
IAP_STATUS_CODE ReadDeviceSerialNum(uint32_t *uid)
{
   IAP_COMMAND_Type command;
   command.cmd = IAP_READ_SERIAL_NUMBER;
   iap_entry (&command.cmd, &command.status);        

   if(command.status == CMD_SUCCESS)
   {
      if(uid != ((void*) 0))
	  {
	    uint32_t i = 0;
		for(i = 0; i < 4; i++)
	       uid[i] =  command.result[i];
	  }
   }

   return (IAP_STATUS_CODE)command.status;
}

 












 
IAP_STATUS_CODE Compare(uint8_t *addr1, uint8_t *addr2, uint32_t size)
{
   IAP_COMMAND_Type command;
   command.cmd = IAP_COMPARE;
   command.param[0] = (uint32_t)addr1;
   command.param[1] = (uint32_t)addr2;
   command.param[2] = size;
   iap_entry (&command.cmd, &command.status);        

   return (IAP_STATUS_CODE)command.status;
}

 






 
void InvokeISP(void)
{
   IAP_COMMAND_Type command;
   command.cmd = IAP_REINVOKE_ISP;
   iap_entry (&command.cmd, &command.status);        
}



 
 

