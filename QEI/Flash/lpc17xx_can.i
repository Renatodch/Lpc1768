#line 1 "Lib\\Drivers\\source\\lpc17xx_can.c"


 


























 

 


 

 
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_can.h"


 



























 

 



 




 
#line 1 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"
 






















 









 



 

 
typedef enum IRQn
{
 
  Reset_IRQn                    = -15,       
  NonMaskableInt_IRQn           = -14,       
  HardFault_IRQn                = -13,       
  MemoryManagement_IRQn         = -12,       
  BusFault_IRQn                 = -11,       
  UsageFault_IRQn               = -10,       
  SVCall_IRQn                   = -5,        
  DebugMonitor_IRQn             = -4,        
  PendSV_IRQn                   = -2,        
  SysTick_IRQn                  = -1,        

 
  WDT_IRQn                      = 0,         
  TIMER0_IRQn                   = 1,         
  TIMER1_IRQn                   = 2,         
  TIMER2_IRQn                   = 3,         
  TIMER3_IRQn                   = 4,         
  UART0_IRQn                    = 5,         
  UART1_IRQn                    = 6,         
  UART2_IRQn                    = 7,         
  UART3_IRQn                    = 8,         
  PWM1_IRQn                     = 9,         
  I2C0_IRQn                     = 10,        
  I2C1_IRQn                     = 11,        
  I2C2_IRQn                     = 12,        
  SPI_IRQn                      = 13,        
  SSP0_IRQn                     = 14,        
  SSP1_IRQn                     = 15,        
  PLL0_IRQn                     = 16,        
  RTC_IRQn                      = 17,        
  EINT0_IRQn                    = 18,        
  EINT1_IRQn                    = 19,        
  EINT2_IRQn                    = 20,        
  EINT3_IRQn                    = 21,        
  ADC_IRQn                      = 22,        
  BOD_IRQn                      = 23,        
  USB_IRQn                      = 24,        
  CAN_IRQn                      = 25,        
  DMA_IRQn                      = 26,        
  I2S_IRQn                      = 27,        
  ENET_IRQn                     = 28,        
  RIT_IRQn                      = 29,        
  MCPWM_IRQn                    = 30,        
  QEI_IRQn                      = 31,        
  PLL1_IRQn                     = 32,        
  USBActivity_IRQn              = 33,        
  CANActivity_IRQn              = 34,        
} IRQn_Type;






 

 





#line 1 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"
 







 

























 
























 




 


 

 













#line 110 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"


 







#line 145 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"

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






 
#line 147 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 







 

























 






 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 










 











 









 









 









 











 











 











 







 










 










 









 





#line 684 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 148 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 







 

























 






 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 307 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 634 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 

#line 149 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"








 
#line 179 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"

 






 
#line 195 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"

 












 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 




#line 422 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"

 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1[1];

} SCnSCB_Type;

 



 










 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   







 


 
typedef struct
{
  volatile const  uint32_t TYPE;                     
  volatile uint32_t CTRL;                     
  volatile uint32_t RNR;                      
  volatile uint32_t RBAR;                     
  volatile uint32_t RASR;                     
  volatile uint32_t RBAR_A1;                  
  volatile uint32_t RASR_A1;                  
  volatile uint32_t RBAR_A2;                  
  volatile uint32_t RASR_A2;                  
  volatile uint32_t RBAR_A3;                  
  volatile uint32_t RASR_A3;                  
} MPU_Type;

 









 









 



 









 






























 







 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1253 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1262 "C:\\Keil_v5\\ARM\\CMSIS\\Include\\core_cm3.h"






 










 

 



 




 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 5)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 5)) & 0xff);    }         
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 5)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 5)));  }  
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 5) ? 5 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 5) < 7) ? 0 : PriorityGroupTmp - 7 + 5;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0))  return (1);       

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = ticks - 1;                                   
  NVIC_SetPriority (SysTick_IRQn, (1<<5) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 





#line 107 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"
#line 1 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"
 





















 









#line 34 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\system_LPC17xx.h"



 

extern uint32_t SystemCoreClock;      










 
extern void SystemInit (void);









 
extern void SystemCoreClockUpdate (void);







 

#line 108 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"


 
 
 


#pragma anon_unions


 
 
typedef struct
{
  volatile uint32_t FLASHCFG;                
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;                 
  volatile uint32_t PLL0CFG;
  volatile const  uint32_t PLL0STAT;
  volatile  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const  uint32_t PLL1STAT;
  volatile  uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t	CANSLEEPCLR;
  volatile uint32_t	CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;                  
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;                    
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;                     
  volatile uint32_t IRCTRIM;                 
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;                
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;               
 } LPC_SC_TypeDef;

 
 
typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

 
 
typedef struct
{
  union {
    volatile uint32_t FIODIR;
    struct {
      volatile uint16_t FIODIRL;
      volatile uint16_t FIODIRH;
    };
    struct {
      volatile uint8_t  FIODIR0;
      volatile uint8_t  FIODIR1;
      volatile uint8_t  FIODIR2;
      volatile uint8_t  FIODIR3;
    };
  };
  uint32_t RESERVED0[3];
  union {
    volatile uint32_t FIOMASK;
    struct {
      volatile uint16_t FIOMASKL;
      volatile uint16_t FIOMASKH;
    };
    struct {
      volatile uint8_t  FIOMASK0;
      volatile uint8_t  FIOMASK1;
      volatile uint8_t  FIOMASK2;
      volatile uint8_t  FIOMASK3;
    };
  };
  union {
    volatile uint32_t FIOPIN;
    struct {
      volatile uint16_t FIOPINL;
      volatile uint16_t FIOPINH;
    };
    struct {
      volatile uint8_t  FIOPIN0;
      volatile uint8_t  FIOPIN1;
      volatile uint8_t  FIOPIN2;
      volatile uint8_t  FIOPIN3;
    };
  };
  union {
    volatile uint32_t FIOSET;
    struct {
      volatile uint16_t FIOSETL;
      volatile uint16_t FIOSETH;
    };
    struct {
      volatile uint8_t  FIOSET0;
      volatile uint8_t  FIOSET1;
      volatile uint8_t  FIOSET2;
      volatile uint8_t  FIOSET3;
    };
  };
  union {
    volatile  uint32_t FIOCLR;
    struct {
      volatile  uint16_t FIOCLRL;
      volatile  uint16_t FIOCLRH;
    };
    struct {
      volatile  uint8_t  FIOCLR0;
      volatile  uint8_t  FIOCLR1;
      volatile  uint8_t  FIOCLR2;
      volatile  uint8_t  FIOCLR3;
    };
  };
} LPC_GPIO_TypeDef;

 
typedef struct
{
  volatile const  uint32_t IntStatus;
  volatile const  uint32_t IO0IntStatR;
  volatile const  uint32_t IO0IntStatF;
  volatile  uint32_t IO0IntClr;
  volatile uint32_t IO0IntEnR;
  volatile uint32_t IO0IntEnF;
       uint32_t RESERVED0[3];
  volatile const  uint32_t IO2IntStatR;
  volatile const  uint32_t IO2IntStatF;
  volatile  uint32_t IO2IntClr;
  volatile uint32_t IO2IntEnR;
  volatile uint32_t IO2IntEnF;
} LPC_GPIOINT_TypeDef;

 
 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
       uint32_t RESERVED0[2];
  volatile uint32_t EMR;
       uint32_t RESERVED1[12];
  volatile uint32_t CTCR;
} LPC_TIM_TypeDef;

 
 
typedef struct
{
  volatile uint32_t IR;
  volatile uint32_t TCR;
  volatile uint32_t TC;
  volatile uint32_t PR;
  volatile uint32_t PC;
  volatile uint32_t MCR;
  volatile uint32_t MR0;
  volatile uint32_t MR1;
  volatile uint32_t MR2;
  volatile uint32_t MR3;
  volatile uint32_t CCR;
  volatile const  uint32_t CR0;
  volatile const  uint32_t CR1;
  volatile const  uint32_t CR2;
  volatile const  uint32_t CR3;
       uint32_t RESERVED0;
  volatile uint32_t MR4;
  volatile uint32_t MR5;
  volatile uint32_t MR6;
  volatile uint32_t PCR;
  volatile uint32_t LER;
       uint32_t RESERVED1[7];
  volatile uint32_t CTCR;
} LPC_PWM_TypeDef;

 
 
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[7];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED2[7];
  volatile uint8_t  SCR;
       uint8_t  RESERVED3[3];
  volatile uint32_t ACR;
  volatile uint8_t  ICR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  FDR;
       uint8_t  RESERVED5[7];
  volatile uint8_t  TER;
} LPC_UART_TypeDef;

 
typedef struct
{
  union {
  volatile const  uint8_t  RBR;
  volatile  uint8_t  THR;
  volatile uint8_t  DLL;
       uint32_t RESERVED0;
  };
  union {
  volatile uint8_t  DLM;
  volatile uint32_t IER;
  };
  union {
  volatile const  uint32_t IIR;
  volatile  uint8_t  FCR;
  };
  volatile uint8_t  LCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  MCR;
       uint8_t  RESERVED2[3];
  volatile const  uint8_t  LSR;
       uint8_t  RESERVED3[3];
  volatile const  uint8_t  MSR;
       uint8_t  RESERVED4[3];
  volatile uint8_t  SCR;
       uint8_t  RESERVED5[3];
  volatile uint32_t ACR;
       uint32_t RESERVED6;
  volatile uint32_t FDR;
       uint32_t RESERVED7;
  volatile uint8_t  TER;
       uint8_t  RESERVED8[27];
  volatile uint8_t  RS485CTRL;
       uint8_t  RESERVED9[3];
  volatile uint8_t  ADRMATCH;
       uint8_t  RESERVED10[3];
  volatile uint8_t  RS485DLY;
} LPC_UART1_TypeDef;

 
 
typedef struct
{
  volatile uint32_t SPCR;
  volatile const  uint32_t SPSR;
  volatile uint32_t SPDR;
  volatile uint32_t SPCCR;
       uint32_t RESERVED0[3];
  volatile uint32_t SPINT;
} LPC_SPI_TypeDef;

 
 
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile const  uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
} LPC_SSP_TypeDef;

 
 
typedef struct
{
  volatile uint32_t I2CONSET;
  volatile const  uint32_t I2STAT;
  volatile uint32_t I2DAT;
  volatile uint32_t I2ADR0;
  volatile uint32_t I2SCLH;
  volatile uint32_t I2SCLL;
  volatile  uint32_t I2CONCLR;
  volatile uint32_t MMCTRL;
  volatile uint32_t I2ADR1;
  volatile uint32_t I2ADR2;
  volatile uint32_t I2ADR3;
  volatile const  uint32_t I2DATA_BUFFER;
  volatile uint32_t I2MASK0;
  volatile uint32_t I2MASK1;
  volatile uint32_t I2MASK2;
  volatile uint32_t I2MASK3;
} LPC_I2C_TypeDef;

 
 
typedef struct
{
  volatile uint32_t I2SDAO;
  volatile uint32_t I2SDAI;
  volatile  uint32_t I2STXFIFO;
  volatile const  uint32_t I2SRXFIFO;
  volatile const  uint32_t I2SSTATE;
  volatile uint32_t I2SDMA1;
  volatile uint32_t I2SDMA2;
  volatile uint32_t I2SIRQ;
  volatile uint32_t I2STXRATE;
  volatile uint32_t I2SRXRATE;
  volatile uint32_t I2STXBITRATE;
  volatile uint32_t I2SRXBITRATE;
  volatile uint32_t I2STXMODE;
  volatile uint32_t I2SRXMODE;
} LPC_I2S_TypeDef;

 
 
typedef struct
{
  volatile uint32_t RICOMPVAL;
  volatile uint32_t RIMASK;
  volatile uint8_t  RICTRL;
       uint8_t  RESERVED0[3];
  volatile uint32_t RICOUNTER;
} LPC_RIT_TypeDef;

 
 
typedef struct
{
  volatile uint8_t  ILR;
       uint8_t  RESERVED0[7];
  volatile uint8_t  CCR;
       uint8_t  RESERVED1[3];
  volatile uint8_t  CIIR;
       uint8_t  RESERVED2[3];
  volatile uint8_t  AMR;
       uint8_t  RESERVED3[3];
  volatile const  uint32_t CTIME0;
  volatile const  uint32_t CTIME1;
  volatile const  uint32_t CTIME2;
  volatile uint8_t  SEC;
       uint8_t  RESERVED4[3];
  volatile uint8_t  MIN;
       uint8_t  RESERVED5[3];
  volatile uint8_t  HOUR;
       uint8_t  RESERVED6[3];
  volatile uint8_t  DOM;
       uint8_t  RESERVED7[3];
  volatile uint8_t  DOW;
       uint8_t  RESERVED8[3];
  volatile uint16_t DOY;
       uint16_t RESERVED9;
  volatile uint8_t  MONTH;
       uint8_t  RESERVED10[3];
  volatile uint16_t YEAR;
       uint16_t RESERVED11;
  volatile uint32_t CALIBRATION;
  volatile uint32_t GPREG0;
  volatile uint32_t GPREG1;
  volatile uint32_t GPREG2;
  volatile uint32_t GPREG3;
  volatile uint32_t GPREG4;
  volatile uint8_t  RTC_AUXEN;
       uint8_t  RESERVED12[3];
  volatile uint8_t  RTC_AUX;
       uint8_t  RESERVED13[3];
  volatile uint8_t  ALSEC;
       uint8_t  RESERVED14[3];
  volatile uint8_t  ALMIN;
       uint8_t  RESERVED15[3];
  volatile uint8_t  ALHOUR;
       uint8_t  RESERVED16[3];
  volatile uint8_t  ALDOM;
       uint8_t  RESERVED17[3];
  volatile uint8_t  ALDOW;
       uint8_t  RESERVED18[3];
  volatile uint16_t ALDOY;
       uint16_t RESERVED19;
  volatile uint8_t  ALMON;
       uint8_t  RESERVED20[3];
  volatile uint16_t ALYEAR;
       uint16_t RESERVED21;
} LPC_RTC_TypeDef;

 
 
typedef struct
{
  volatile uint8_t  WDMOD;
       uint8_t  RESERVED0[3];
  volatile uint32_t WDTC;
  volatile  uint8_t  WDFEED;
       uint8_t  RESERVED1[3];
  volatile const  uint32_t WDTV;
  volatile uint32_t WDCLKSEL;
} LPC_WDT_TypeDef;

 
 
typedef struct
{
  volatile uint32_t ADCR;
  volatile uint32_t ADGDR;
       uint32_t RESERVED0;
  volatile uint32_t ADINTEN;
  volatile const  uint32_t ADDR0;
  volatile const  uint32_t ADDR1;
  volatile const  uint32_t ADDR2;
  volatile const  uint32_t ADDR3;
  volatile const  uint32_t ADDR4;
  volatile const  uint32_t ADDR5;
  volatile const  uint32_t ADDR6;
  volatile const  uint32_t ADDR7;
  volatile const  uint32_t ADSTAT;
  volatile uint32_t ADTRM;
} LPC_ADC_TypeDef;

 
 
typedef struct
{
  volatile uint32_t DACR;
  volatile uint32_t DACCTRL;
  volatile uint16_t DACCNTVAL;
} LPC_DAC_TypeDef;

 
 
typedef struct
{
  volatile const  uint32_t MCCON;
  volatile  uint32_t MCCON_SET;
  volatile  uint32_t MCCON_CLR;
  volatile const  uint32_t MCCAPCON;
  volatile  uint32_t MCCAPCON_SET;
  volatile  uint32_t MCCAPCON_CLR;
  volatile uint32_t MCTIM0;
  volatile uint32_t MCTIM1;
  volatile uint32_t MCTIM2;
  volatile uint32_t MCPER0;
  volatile uint32_t MCPER1;
  volatile uint32_t MCPER2;
  volatile uint32_t MCPW0;
  volatile uint32_t MCPW1;
  volatile uint32_t MCPW2;
  volatile uint32_t MCDEADTIME;
  volatile uint32_t MCCCP;
  volatile uint32_t MCCR0;
  volatile uint32_t MCCR1;
  volatile uint32_t MCCR2;
  volatile const  uint32_t MCINTEN;
  volatile  uint32_t MCINTEN_SET;
  volatile  uint32_t MCINTEN_CLR;
  volatile const  uint32_t MCCNTCON;
  volatile  uint32_t MCCNTCON_SET;
  volatile  uint32_t MCCNTCON_CLR;
  volatile const  uint32_t MCINTFLAG;
  volatile  uint32_t MCINTFLAG_SET;
  volatile  uint32_t MCINTFLAG_CLR;
  volatile  uint32_t MCCAP_CLR;
} LPC_MCPWM_TypeDef;

 
 
typedef struct
{
  volatile  uint32_t QEICON;
  volatile const  uint32_t QEISTAT;
  volatile uint32_t QEICONF;
  volatile const  uint32_t QEIPOS;
  volatile uint32_t QEIMAXPOS;
  volatile uint32_t CMPOS0;
  volatile uint32_t CMPOS1;
  volatile uint32_t CMPOS2;
  volatile const  uint32_t INXCNT;
  volatile uint32_t INXCMP;
  volatile uint32_t QEILOAD;
  volatile const  uint32_t QEITIME;
  volatile const  uint32_t QEIVEL;
  volatile const  uint32_t QEICAP;
  volatile uint32_t VELCOMP;
  volatile uint32_t FILTER;
       uint32_t RESERVED0[998];
  volatile  uint32_t QEIIEC;
  volatile  uint32_t QEIIES;
  volatile const  uint32_t QEIINTSTAT;
  volatile const  uint32_t QEIIE;
  volatile  uint32_t QEICLR;
  volatile  uint32_t QEISET;
} LPC_QEI_TypeDef;

 
 
typedef struct
{
  volatile uint32_t mask[512];               
} LPC_CANAF_RAM_TypeDef;

 
typedef struct                           
{
  volatile uint32_t AFMR;
  volatile uint32_t SFF_sa;
  volatile uint32_t SFF_GRP_sa;
  volatile uint32_t EFF_sa;
  volatile uint32_t EFF_GRP_sa;
  volatile uint32_t ENDofTable;
  volatile const  uint32_t LUTerrAd;
  volatile const  uint32_t LUTerr;
  volatile uint32_t FCANIE;
  volatile uint32_t FCANIC0;
  volatile uint32_t FCANIC1;
} LPC_CANAF_TypeDef;

 
typedef struct                           
{
  volatile const  uint32_t CANTxSR;
  volatile const  uint32_t CANRxSR;
  volatile const  uint32_t CANMSR;
} LPC_CANCR_TypeDef;

 
typedef struct                           
{
  volatile uint32_t MOD;
  volatile  uint32_t CMR;
  volatile uint32_t GSR;
  volatile const  uint32_t ICR;
  volatile uint32_t IER;
  volatile uint32_t BTR;
  volatile uint32_t EWL;
  volatile const  uint32_t SR;
  volatile uint32_t RFS;
  volatile uint32_t RID;
  volatile uint32_t RDA;
  volatile uint32_t RDB;
  volatile uint32_t TFI1;
  volatile uint32_t TID1;
  volatile uint32_t TDA1;
  volatile uint32_t TDB1;
  volatile uint32_t TFI2;
  volatile uint32_t TID2;
  volatile uint32_t TDA2;
  volatile uint32_t TDB2;
  volatile uint32_t TFI3;
  volatile uint32_t TID3;
  volatile uint32_t TDA3;
  volatile uint32_t TDB3;
} LPC_CAN_TypeDef;

 
 
typedef struct                           
{
  volatile const  uint32_t DMACIntStat;
  volatile const  uint32_t DMACIntTCStat;
  volatile  uint32_t DMACIntTCClear;
  volatile const  uint32_t DMACIntErrStat;
  volatile  uint32_t DMACIntErrClr;
  volatile const  uint32_t DMACRawIntTCStat;
  volatile const  uint32_t DMACRawIntErrStat;
  volatile const  uint32_t DMACEnbldChns;
  volatile uint32_t DMACSoftBReq;
  volatile uint32_t DMACSoftSReq;
  volatile uint32_t DMACSoftLBReq;
  volatile uint32_t DMACSoftLSReq;
  volatile uint32_t DMACConfig;
  volatile uint32_t DMACSync;
} LPC_GPDMA_TypeDef;

 
typedef struct                           
{
  volatile uint32_t DMACCSrcAddr;
  volatile uint32_t DMACCDestAddr;
  volatile uint32_t DMACCLLI;
  volatile uint32_t DMACCControl;
  volatile uint32_t DMACCConfig;
} LPC_GPDMACH_TypeDef;

 
 
typedef struct
{
  volatile const  uint32_t HcRevision;              
  volatile uint32_t HcControl;
  volatile uint32_t HcCommandStatus;
  volatile uint32_t HcInterruptStatus;
  volatile uint32_t HcInterruptEnable;
  volatile uint32_t HcInterruptDisable;
  volatile uint32_t HcHCCA;
  volatile const  uint32_t HcPeriodCurrentED;
  volatile uint32_t HcControlHeadED;
  volatile uint32_t HcControlCurrentED;
  volatile uint32_t HcBulkHeadED;
  volatile uint32_t HcBulkCurrentED;
  volatile const  uint32_t HcDoneHead;
  volatile uint32_t HcFmInterval;
  volatile const  uint32_t HcFmRemaining;
  volatile const  uint32_t HcFmNumber;
  volatile uint32_t HcPeriodicStart;
  volatile uint32_t HcLSTreshold;
  volatile uint32_t HcRhDescriptorA;
  volatile uint32_t HcRhDescriptorB;
  volatile uint32_t HcRhStatus;
  volatile uint32_t HcRhPortStatus1;
  volatile uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  volatile const  uint32_t Module_ID;

  volatile const  uint32_t OTGIntSt;                
  volatile uint32_t OTGIntEn;
  volatile  uint32_t OTGIntSet;
  volatile  uint32_t OTGIntClr;
  volatile uint32_t OTGStCtrl;
  volatile uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  volatile const  uint32_t USBDevIntSt;             
  volatile uint32_t USBDevIntEn;
  volatile  uint32_t USBDevIntClr;
  volatile  uint32_t USBDevIntSet;

  volatile  uint32_t USBCmdCode;              
  volatile const  uint32_t USBCmdData;

  volatile const  uint32_t USBRxData;               
  volatile  uint32_t USBTxData;
  volatile const  uint32_t USBRxPLen;
  volatile  uint32_t USBTxPLen;
  volatile uint32_t USBCtrl;
  volatile  uint32_t USBDevIntPri;

  volatile const  uint32_t USBEpIntSt;              
  volatile uint32_t USBEpIntEn;
  volatile  uint32_t USBEpIntClr;
  volatile  uint32_t USBEpIntSet;
  volatile  uint32_t USBEpIntPri;

  volatile uint32_t USBReEp;                 
  volatile  uint32_t USBEpInd;
  volatile uint32_t USBMaxPSize;

  volatile const  uint32_t USBDMARSt;               
  volatile  uint32_t USBDMARClr;
  volatile  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  volatile uint32_t USBUDCAH;
  volatile const  uint32_t USBEpDMASt;
  volatile  uint32_t USBEpDMAEn;
  volatile  uint32_t USBEpDMADis;
  volatile const  uint32_t USBDMAIntSt;
  volatile uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  volatile const  uint32_t USBEoTIntSt;
  volatile  uint32_t USBEoTIntClr;
  volatile  uint32_t USBEoTIntSet;
  volatile const  uint32_t USBNDDRIntSt;
  volatile  uint32_t USBNDDRIntClr;
  volatile  uint32_t USBNDDRIntSet;
  volatile const  uint32_t USBSysErrIntSt;
  volatile  uint32_t USBSysErrIntClr;
  volatile  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  volatile const  uint32_t I2C_RX;                  
  volatile  uint32_t I2C_TX;
  };
  volatile const  uint32_t I2C_STS;
  volatile uint32_t I2C_CTL;
  volatile uint32_t I2C_CLKHI;
  volatile  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  volatile uint32_t USBClkCtrl;              
  volatile uint32_t OTGClkCtrl;
  };
  union {
  volatile const  uint32_t USBClkSt;
  volatile const  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;

 
 
typedef struct
{
  volatile uint32_t MAC1;                    
  volatile uint32_t MAC2;
  volatile uint32_t IPGT;
  volatile uint32_t IPGR;
  volatile uint32_t CLRT;
  volatile uint32_t MAXF;
  volatile uint32_t SUPP;
  volatile uint32_t TEST;
  volatile uint32_t MCFG;
  volatile uint32_t MCMD;
  volatile uint32_t MADR;
  volatile  uint32_t MWTD;
  volatile const  uint32_t MRDD;
  volatile const  uint32_t MIND;
       uint32_t RESERVED0[2];
  volatile uint32_t SA0;
  volatile uint32_t SA1;
  volatile uint32_t SA2;
       uint32_t RESERVED1[45];
  volatile uint32_t Command;                 
  volatile const  uint32_t Status;
  volatile uint32_t RxDescriptor;
  volatile uint32_t RxStatus;
  volatile uint32_t RxDescriptorNumber;
  volatile const  uint32_t RxProduceIndex;
  volatile uint32_t RxConsumeIndex;
  volatile uint32_t TxDescriptor;
  volatile uint32_t TxStatus;
  volatile uint32_t TxDescriptorNumber;
  volatile uint32_t TxProduceIndex;
  volatile const  uint32_t TxConsumeIndex;
       uint32_t RESERVED2[10];
  volatile const  uint32_t TSV0;
  volatile const  uint32_t TSV1;
  volatile const  uint32_t RSV;
       uint32_t RESERVED3[3];
  volatile uint32_t FlowControlCounter;
  volatile const  uint32_t FlowControlStatus;
       uint32_t RESERVED4[34];
  volatile uint32_t RxFilterCtrl;            
  volatile uint32_t RxFilterWoLStatus;
  volatile uint32_t RxFilterWoLClear;
       uint32_t RESERVED5;
  volatile uint32_t HashFilterL;
  volatile uint32_t HashFilterH;
       uint32_t RESERVED6[882];
  volatile const  uint32_t IntStatus;               
  volatile uint32_t IntEnable;
  volatile  uint32_t IntClear;
  volatile  uint32_t IntSet;
       uint32_t RESERVED7;
  volatile uint32_t PowerDown;
       uint32_t RESERVED8;
  volatile uint32_t Module_ID;
} LPC_EMAC_TypeDef;



#pragma no_anon_unions



 
 
 
 
#line 933 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 954 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 968 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 
#line 981 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"

 






 
 
 
#line 1039 "C:\\Keil_v5\\ARM\\INC\\NXP\\LPC17xx\\LPC17xx.h"



 

#line 44 ".\\Lib\\Drivers\\include\\lpc17xx_can.h"
#line 1 ".\\Lib\\Drivers\\include\\lpc_types.h"


 





























 

 



 




 
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



 






 

 
#line 45 ".\\Lib\\Drivers\\include\\lpc17xx_can.h"






 


 
#line 63 ".\\Lib\\Drivers\\include\\lpc17xx_can.h"



 

 


 

 
 

 
 

 

 

 

 

 

 


 

 
 

 

 

 

 

 

 

 


 

 
 

 

 

 

 

 

 

 

 

 


 

 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 


 

 
 

 

 

 

 

 

 

 

 

 

 


 

 
 

 

 

 

 


 

 
 


 

 
 

 

 

 

 

 

 

 

 

 

 

 

 

 


 

 
 

 

 

 

 


 

 
 

 


 

 
 

 

 

 


 

 
 

 

 

 


 

 
 

 

 

 


 

 
 

 


 

 
 

 

 

 


 

 
 

 

 

 


 

 
 

 


 

 
 

 


 

 
 

 

 

 

 

 


 

 
 

 

 

 

 

 


 

 
 

 

 

 


 

 
 

 

 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 



 
 



 


 


 


 


 


 



 

 


 


 


 



 


 



 





 


 


 
#line 534 ".\\Lib\\Drivers\\include\\lpc17xx_can.h"

 




 




 

 


 

 


 


 
typedef enum {
	STD_ID_FORMAT = 0, 	 
	EXT_ID_FORMAT = 1	 
} CAN_ID_FORMAT_Type;



 
typedef enum {
	FULLCAN_ENTRY = 0,
	EXPLICIT_STANDARD_ENTRY,
	GROUP_STANDARD_ENTRY,
	EXPLICIT_EXTEND_ENTRY,
	GROUP_EXTEND_ENTRY
} AFLUT_ENTRY_Type;



 
typedef enum {
	DATA_FRAME = 0, 	 
	REMOTE_FRAME = 1	 
} CAN_FRAME_Type;



 
typedef enum {
	CANCTRL_GLOBAL_STS = 0,  
	CANCTRL_INT_CAP, 		 
	CANCTRL_ERR_WRN, 		 
	CANCTRL_STS				 
} CAN_CTRL_STS_Type;



 
typedef enum {
	CANCR_TX_STS = 0, 	 
	CANCR_RX_STS, 		 
	CANCR_MS			 
} CAN_CR_STS_Type;



 
typedef enum{
	FULLCAN_IC0,	 
	FULLCAN_IC1	 
}FullCAN_IC_Type;



 
typedef enum {
	CANINT_RIE = 0, 	 
	CANINT_TIE1, 		 
	CANINT_EIE, 		 
	CANINT_DOIE, 		 
	CANINT_WUIE, 		 
	CANINT_EPIE, 		 
	CANINT_ALIE, 		 
	CANINT_BEIE, 		 
	CANINT_IDIE, 		 
	CANINT_TIE2, 		 
	CANINT_TIE3, 		 
	CANINT_FCE			 
} CAN_INT_EN_Type;



 
typedef enum {
	CAN_Normal = 0, 	 
	CAN_AccOff, 		 
	CAN_AccBP, 			 
	CAN_eFCAN			 
} CAN_AFMODE_Type;



 
typedef enum {
	CAN_OPERATING_MODE = 0, 	 
	CAN_RESET_MODE, 			 
	CAN_LISTENONLY_MODE, 		 
	CAN_SELFTEST_MODE, 			 
	CAN_TXPRIORITY_MODE, 		 
	CAN_SLEEP_MODE, 			 
	CAN_RXPOLARITY_MODE, 		 
	CAN_TEST_MODE				 
} CAN_MODE_Type;



 
typedef enum {
	CAN_OK = 1, 				 
	CAN_OBJECTS_FULL_ERROR, 	 
	CAN_FULL_OBJ_NOT_RCV, 		 
	CAN_NO_RECEIVE_DATA, 		 
	CAN_AF_ENTRY_ERROR, 		 
	CAN_CONFLICT_ID_ERROR, 		 
	CAN_ENTRY_NOT_EXIT_ERROR	 
} CAN_ERROR;



 
typedef struct {
	uint8_t RD; 			






 
	uint8_t TD;				






 
} CAN_PinCFG_Type;



 
typedef struct {
	uint32_t id; 			


 
	uint8_t dataA[4]; 		 
	uint8_t dataB[4]; 		 
	uint8_t len; 			


 
	uint8_t format; 		


 
	uint8_t type; 			



 
} CAN_MSG_Type;



 
typedef struct {
	uint8_t controller;		


 
	uint8_t disable;		


 
	uint16_t id_11;			 
} FullCAN_Entry;



 
typedef struct {
	uint8_t controller; 	


 
	uint8_t disable; 		


 
	uint16_t id_11; 		 
} SFF_Entry;



 
typedef struct {
	uint8_t controller1; 	


 
	uint8_t disable1; 		


 
	uint16_t lowerID; 		 
	uint8_t controller2; 	


 
	uint8_t disable2; 		


 
	uint16_t upperID; 		

 
} SFF_GPR_Entry;



 
typedef struct {
	uint8_t controller; 	


 
	uint32_t ID_29; 		 
} EFF_Entry;




 
typedef struct {
	uint8_t controller1; 	


 
	uint8_t controller2; 	


 
	uint32_t lowerEID; 		 
	uint32_t upperEID; 		 
} EFF_GPR_Entry;




 
typedef struct {
	FullCAN_Entry* FullCAN_Sec; 	 
	uint8_t FC_NumEntry;			 
	SFF_Entry* SFF_Sec; 			 
	uint8_t SFF_NumEntry;			 
	SFF_GPR_Entry* SFF_GPR_Sec; 	 
	uint8_t SFF_GPR_NumEntry;		 
	EFF_Entry* EFF_Sec; 			 
	uint8_t EFF_NumEntry;			 
	EFF_GPR_Entry* EFF_GPR_Sec; 	 
	uint8_t EFF_GPR_NumEntry;		 
} AF_SectionDef;



 


 


 

 
void CAN_Init(LPC_CAN_TypeDef *CANx, uint32_t baudrate);
void CAN_DeInit(LPC_CAN_TypeDef *CANx);

 
Status CAN_SendMsg(LPC_CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg);
Status CAN_ReceiveMsg(LPC_CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg);
CAN_ERROR FCAN_ReadObj(LPC_CANAF_TypeDef* CANAFx, CAN_MSG_Type *CAN_Msg);

 
void CAN_ModeConfig(LPC_CAN_TypeDef* CANx, CAN_MODE_Type mode,
		FunctionalState NewState);
void CAN_SetAFMode(LPC_CANAF_TypeDef* CANAFx, CAN_AFMODE_Type AFmode);
void CAN_SetCommand(LPC_CAN_TypeDef* CANx, uint32_t CMRType);

 
CAN_ERROR CAN_SetupAFLUT(LPC_CANAF_TypeDef* CANAFx, AF_SectionDef* AFSection);
CAN_ERROR CAN_LoadFullCANEntry(LPC_CAN_TypeDef* CANx, uint16_t ID);
CAN_ERROR CAN_LoadExplicitEntry(LPC_CAN_TypeDef* CANx, uint32_t ID,
		CAN_ID_FORMAT_Type format);
CAN_ERROR CAN_LoadGroupEntry(LPC_CAN_TypeDef* CANx, uint32_t lowerID,
		uint32_t upperID, CAN_ID_FORMAT_Type format);
CAN_ERROR CAN_RemoveEntry(AFLUT_ENTRY_Type EntryType, uint16_t position);

 
void CAN_IRQCmd(LPC_CAN_TypeDef* CANx, CAN_INT_EN_Type arg, FunctionalState NewState);
uint32_t CAN_IntGetStatus(LPC_CAN_TypeDef* CANx);

 
IntStatus CAN_FullCANIntGetStatus (LPC_CANAF_TypeDef* CANAFx);
uint32_t CAN_FullCANPendGetStatus (LPC_CANAF_TypeDef* CANAFx, FullCAN_IC_Type type);
uint32_t CAN_GetCTRLStatus(LPC_CAN_TypeDef* CANx, CAN_CTRL_STS_Type arg);
uint32_t CAN_GetCRStatus(LPC_CANCR_TypeDef* CANCRx, CAN_CR_STS_Type arg);



 










 

 
#line 39 "Lib\\Drivers\\source\\lpc17xx_can.c"
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_clkpwr.h"


 



























 

 



 




 
#line 44 ".\\Lib\\Drivers\\include\\lpc17xx_clkpwr.h"
#line 45 ".\\Lib\\Drivers\\include\\lpc17xx_clkpwr.h"






 


 



 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 





 
 

 

 





 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 





 
 


 

 
 

 
 

 

 

 


 

 
 
 

 

 

 

 

 

 

 

 


 

 
 

 

 


 

 
 

 

 



 

 
 

 

 

 

 


 

 
 


 

 
 

 

 


 

 
 

 

 


 

 
 

 

 

 

 


 

 
 


 

 
 


 

 
 


 

 
 


 

 
 

 



 

 

 


 

 
 

 

 

 

 

 

 

 

 


 

 
 




 


 


 

void CLKPWR_SetPCLKDiv (uint32_t ClkType, uint32_t DivVal);
uint32_t CLKPWR_GetPCLKSEL (uint32_t ClkType);
uint32_t CLKPWR_GetPCLK (uint32_t ClkType);
void CLKPWR_ConfigPPWR (uint32_t PPType, FunctionalState NewState);
void CLKPWR_Sleep(void);
void CLKPWR_DeepSleep(void);
void CLKPWR_PowerDown(void);
void CLKPWR_DeepPowerDown(void);



 










 

 
#line 40 "Lib\\Drivers\\source\\lpc17xx_can.c"




 
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_libcfg_default.h"


 


























 

 



 




 
#line 43 ".\\Lib\\Drivers\\include\\lpc17xx_libcfg_default.h"


 


 

 

 




 
 

 


 


 


 






 


 


 





 





 


 



 



 


 



 



 


 


 



 


 


 


 


 


 










 







 


 


 


void check_failed(uint8_t *file, uint32_t line);




 





 

 
#line 49 "Lib\\Drivers\\source\\lpc17xx_can.c"





 


 

FunctionalState FULLCAN_ENABLE;


 
uint16_t CANAF_FullCAN_cnt = 0;
uint16_t CANAF_std_cnt = 0;
uint16_t CANAF_gstd_cnt = 0;
uint16_t CANAF_ext_cnt = 0;
uint16_t CANAF_gext_cnt = 0;

 


 

 
static void can_SetBaudrate (LPC_CAN_TypeDef *CANx, uint32_t baudrate);

 






 
static void can_SetBaudrate (LPC_CAN_TypeDef *CANx, uint32_t baudrate)
{
	uint32_t result = 0;
	uint8_t NT, TSEG1, TSEG2, BRFail;
	uint32_t CANPclk = 0;
	uint32_t BRP;
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 91));

	if (CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		CANPclk = CLKPWR_GetPCLK (((uint32_t)(26)));
	}
	else
	{
		CANPclk = CLKPWR_GetPCLK (((uint32_t)(28)));
	}
	result = CANPclk / baudrate;
	



 
	BRFail = 1;
	for(NT=24;NT>0;NT=NT-2)
	{
		if ((result%NT)==0)
		{
			BRP = result / NT - 1;
			NT--;
			TSEG2 = (NT/3) - 1;
			TSEG1 = NT -(NT/3) - 1;
			BRFail = 0;
			break;
		}
	}
	if(BRFail)
		while(1); 
	 
	CANx->MOD = 0x01;
	


 
	CANx->BTR  = (TSEG2<<20)|(TSEG1<<16)|(3<<14)|BRP;
	 
	CANx->MOD = 0;
}
 


 


 

 






 
void CAN_Init(LPC_CAN_TypeDef *CANx, uint32_t baudrate)
{
	uint16_t i;
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 151));

	if(CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		 
		CLKPWR_ConfigPPWR(((uint32_t)(1<<13)), ENABLE);
		 
	}
	else
	{
		 
		CLKPWR_ConfigPPWR(((uint32_t)(1<<14)), ENABLE);
		 
	}
	CLKPWR_SetPCLKDiv (((uint32_t)(26)), ((uint32_t)(2)));
	CLKPWR_SetPCLKDiv (((uint32_t)(28)), ((uint32_t)(2)));
	CLKPWR_SetPCLKDiv (((uint32_t)(30)), ((uint32_t)(2)));

	CANx->MOD = 1; 
	CANx->IER = 0; 
	CANx->GSR = 0;
	 
	
	CANx->CMR = (1<<1)|(1<<2)|(1<<3);
	 
	i = CANx->ICR;
	CANx->MOD = 0;

	
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x01;

	
	for (i = 0; i < 512; i++) {
		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[i] = 0x00;
	}

	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_sa = 0x00;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa = 0x00;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa = 0x00;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa = 0x00;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable = 0x00;

	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00;
	 
	can_SetBaudrate (CANx, baudrate);
}

 





 
void CAN_DeInit(LPC_CAN_TypeDef *CANx)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 207));

	if(CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		 
		CLKPWR_ConfigPPWR(((uint32_t)(1<<13)), DISABLE);
	}
	else
	{
		 
		CLKPWR_ConfigPPWR(((uint32_t)(1<<14)), DISABLE);
	}
}

 









 
CAN_ERROR CAN_SetupAFLUT(LPC_CANAF_TypeDef* CANAFx, AF_SectionDef* AFSection)
{
	uint8_t ctrl1,ctrl2;
	uint8_t dis1, dis2;
	uint16_t SID, ID_temp,i, count = 0;
	uint32_t EID, entry, buf;
	uint16_t lowerSID, upperSID;
	uint32_t lowerEID, upperEID;

	(((((uint32_t*)CANAFx)== ((uint32_t*)((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 241));
	CANAFx->AFMR = 0x01;

 
	if(AFSection->FullCAN_Sec == ((void*) 0))
	{
		FULLCAN_ENABLE = DISABLE;
	}
	else
	{
		FULLCAN_ENABLE = ENABLE;
		for(i=0;i<(AFSection->FC_NumEntry);i++)
		{
			if(count + 1 > 64)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->FullCAN_Sec->controller;
			SID = AFSection->FullCAN_Sec->id_11;
			dis1 = AFSection->FullCAN_Sec->disable;

			((((ctrl1==((uint8_t)(0)))|(ctrl1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 262));
			((((SID>>11)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 263));
			((((dis1==((uint8_t)(0)))|(dis1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 264));
			entry = 0x00; 
			if((CANAF_FullCAN_cnt & 0x00000001)==0)
			{
				if(count!=0x00)
				{
					buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count-1];
					ID_temp = (buf & 0xE7FF); 
					if(ID_temp > ((ctrl1<<13)|SID))
					{
						return CAN_AF_ENTRY_ERROR;
					}
				}
				entry = (ctrl1<<29)|(dis1<<28)|(SID<<16)|(1<<27);
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] &= 0x0000FFFF;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] |= entry;
				CANAF_FullCAN_cnt++;
				if(CANAF_FullCAN_cnt == AFSection->FC_NumEntry) 
					count++;
			}
			else
			{
				buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count];
				ID_temp = (buf >>16) & 0xE7FF;
				if(ID_temp > ((ctrl1<<13)|SID))
				{
					return CAN_AF_ENTRY_ERROR;
				}
				entry = (ctrl1<<13)|(dis1<<12)|(SID<<0)|(1<<11);
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] &= 0xFFFF0000;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count]|= entry;
				count++;
				CANAF_FullCAN_cnt++;
			}
			AFSection->FullCAN_Sec = (FullCAN_Entry *)((uint32_t)(AFSection->FullCAN_Sec)+ sizeof(FullCAN_Entry));
		}
	}

 
	if(AFSection->SFF_Sec != ((void*) 0))
	{
		for(i=0;i<(AFSection->SFF_NumEntry);i++)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->SFF_Sec->controller;
			SID = AFSection->SFF_Sec->id_11;
			dis1 = AFSection->SFF_Sec->disable;

			
			((((ctrl1==((uint8_t)(0)))|(ctrl1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 316));
			((((SID>>11)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 317));
			((((dis1==((uint8_t)(0)))|(dis1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 318));

			entry = 0x00; 
			if((CANAF_std_cnt & 0x00000001)==0)
			{
				if(CANAF_std_cnt !=0 )
				{
					buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count-1];
					ID_temp = (buf & 0xE7FF); 
					if(ID_temp > ((ctrl1<<13)|SID))
					{
						return CAN_AF_ENTRY_ERROR;
					}
				}
				entry = (ctrl1<<29)|(dis1<<28)|(SID<<16);
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] &= 0x0000FFFF;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] |= entry;
				CANAF_std_cnt++;
				if(CANAF_std_cnt == AFSection->SFF_NumEntry)
					count++;
			}
			else
			{
				buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count];
				ID_temp = (buf >>16) & 0xE7FF;
				if(ID_temp > ((ctrl1<<13)|SID))
				{
					return CAN_AF_ENTRY_ERROR;
				}
				entry = (ctrl1<<13)|(dis1<<12)|(SID<<0);
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] &= 0xFFFF0000;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] |= entry;
				count++;
				CANAF_std_cnt++;
			}
			AFSection->SFF_Sec = (SFF_Entry *)((uint32_t)(AFSection->SFF_Sec)+ sizeof(SFF_Entry));
		}
	}

 
	if(AFSection->SFF_GPR_Sec != ((void*) 0))
	{
		for(i=0;i<(AFSection->SFF_GPR_NumEntry);i++)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->SFF_GPR_Sec->controller1;
			ctrl2 = AFSection->SFF_GPR_Sec->controller2;
			dis1 = AFSection->SFF_GPR_Sec->disable1;
			dis2 = AFSection->SFF_GPR_Sec->disable2;
			lowerSID = AFSection->SFF_GPR_Sec->lowerID;
			upperSID = AFSection->SFF_GPR_Sec->upperID;

			 
			((((ctrl1==((uint8_t)(0)))|(ctrl1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 374));
			((((ctrl2==((uint8_t)(0)))|(ctrl2==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 375));
			((((dis1==((uint8_t)(0)))|(dis1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 376));
			((((dis2==((uint8_t)(0)))|(dis2==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 377));
			((((lowerSID>>11)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 378));
			((((upperSID>>11)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 379));

			entry = 0x00;
			if(CANAF_gstd_cnt!=0)
			{
				buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count-1];
				ID_temp = buf & 0xE7FF;
				if((ctrl1 != ctrl2)||(lowerSID > upperSID)||(ID_temp > ((ctrl1<<13)|lowerSID)))
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			entry = (ctrl1 << 29)|(dis1 << 28)|(lowerSID << 16)|  					(ctrl2 << 13)|(dis2 << 12)|(upperSID << 0);

			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] = entry;
			CANAF_gstd_cnt++;
			count++;
			AFSection->SFF_GPR_Sec = (SFF_GPR_Entry *)((uint32_t)(AFSection->SFF_GPR_Sec)+ sizeof(SFF_GPR_Entry));
		}
	}

 
	if(AFSection->EFF_Sec != ((void*) 0))
	{
		for(i=0;i<(AFSection->EFF_NumEntry);i++)
		{
			if(count + 1 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			EID = AFSection->EFF_Sec->ID_29;
			ctrl1 = AFSection->EFF_Sec->controller;

			
			((((EID>>29)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 413));
			((((ctrl1==((uint8_t)(0)))|(ctrl1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 414));

			entry = (ctrl1 << 29)|(EID << 0);
			if(CANAF_ext_cnt != 0)
			{
				buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count-1];

				if(buf > entry)
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count] = entry;
			CANAF_ext_cnt ++;
			count++;
			AFSection->EFF_Sec = (EFF_Entry *)((uint32_t)(AFSection->EFF_Sec)+ sizeof(EFF_Entry));
		}
	}

 
	if(AFSection->EFF_GPR_Sec != ((void*) 0))
	{
		for(i=0;i<(AFSection->EFF_GPR_NumEntry);i++)
		{
			if(count + 2 > 512)
			{
				return CAN_OBJECTS_FULL_ERROR;
			}
			ctrl1 = AFSection->EFF_GPR_Sec->controller1;
			ctrl2 = AFSection->EFF_GPR_Sec->controller2;
			lowerEID = AFSection->EFF_GPR_Sec->lowerEID;
			upperEID = AFSection->EFF_GPR_Sec->upperEID;

			
			((((ctrl1==((uint8_t)(0)))|(ctrl1==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 448));
			((((ctrl2==((uint8_t)(0)))|(ctrl2==((uint8_t)(1))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 449));
			((((lowerEID>>29)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 450));
			((((upperEID>>29)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 451));

			entry = 0x00;
			if(CANAF_gext_cnt != 0)
			{
				buf = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count-1];

				if((ctrl1 != ctrl2) || (lowerEID > upperEID) || (buf > ((ctrl1 << 29)|(lowerEID << 0))))
				{
					return CAN_AF_ENTRY_ERROR;
				}
			}
			entry = (ctrl1 << 29)|(lowerEID << 0);
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count++] = entry;
			entry = (ctrl2 << 29)|(upperEID << 0);
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[count++] = entry;
			CANAF_gext_cnt++;
			AFSection->EFF_GPR_Sec = (EFF_GPR_Entry *)((uint32_t)(AFSection->EFF_GPR_Sec)+ sizeof(EFF_GPR_Entry));
		}
	}
	
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_sa = ((CANAF_FullCAN_cnt + 1)>>1)<<2;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_sa + (((CANAF_std_cnt+1)>>1)<< 2);
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa + (CANAF_gstd_cnt << 2);
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa + (CANAF_ext_cnt << 2);
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa + (CANAF_gext_cnt << 3);

	if(FULLCAN_ENABLE == DISABLE)
	{
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00; 
	}
	else
	{
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
	}
	return CAN_OK;
}
 












 
CAN_ERROR CAN_LoadExplicitEntry(LPC_CAN_TypeDef* CANx, uint32_t id, CAN_ID_FORMAT_Type format)
{
	uint32_t tmp0 = 0;
	uint32_t buf0=0, buf1=0;
	int16_t cnt1=0, cnt2=0, bound1=0, total=0;


	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 509));
	((((format==STD_ID_FORMAT)||(format==EXT_ID_FORMAT))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 510));

	if (CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		tmp0 = 0;
	}
	else if (CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))
	{
		tmp0 = 1;
	}

	 
	total =((CANAF_FullCAN_cnt+1)>>1)+ CANAF_FullCAN_cnt*3 +((CANAF_std_cnt + 1) >> 1)+  			CANAF_gstd_cnt + CANAF_ext_cnt + (CANAF_gext_cnt<<1);

	if (total >= 512){ 
		return CAN_OBJECTS_FULL_ERROR;
	}

	
 
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00000001;

 
 	if(format == STD_ID_FORMAT)
 	{
 		id &= 0x07FF;
 		id |= (tmp0 << 13);  
		
 
		if ((CANAF_std_cnt & 0x0001) == 0)
		{
			cnt1   = ((CANAF_FullCAN_cnt+1)>>1)+((CANAF_std_cnt+1)>>1);
			bound1 = total - cnt1;
			buf0   = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
			while(bound1--)
			{
				cnt1++;
				buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf0;
				buf0 = buf1;
			}
		}
		if (CANAF_std_cnt == 0)
		{
			cnt2 = (CANAF_FullCAN_cnt + 1)>>1;
			 
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] = 0x0000FFFF | (id << 16);
		}
		else if (CANAF_std_cnt == 1)
		{
			cnt2 = (CANAF_FullCAN_cnt + 1)>>1;
			 
			if (((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] >> 16)& 0xE7FF) > id)
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] >> 16) | (id << 16);
			}
			else
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt2] & 0xFFFF0000) | id;
			}
		}
		else
		{
			 
			cnt1 = (CANAF_FullCAN_cnt+1)>>1;
			cnt2 = CANAF_std_cnt;
			bound1 = ((CANAF_FullCAN_cnt+1)>>1)+((CANAF_std_cnt+1)>>1);
			while (cnt1 < bound1)
			{
				 
				if (((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >> 16) & 0xE7FF) > id)
				{
					cnt2 = cnt1 * 2;
					break;
				}

				if ((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] & 0x0000E7FF) > id)
				{
					cnt2 = cnt1 * 2 + 1;
					break;
				}

				cnt1++;
			}
			 
			 

			if (cnt1 == bound1)
			{
				 
				 
				if ((CANAF_std_cnt & 0x0001) == 0)
				{
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]  = 0x0000FFFF | (id << 16);
				}
				 
				else
				{
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]  = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] & 0xFFFF0000) | id;
				}
			}
			else
			{
				buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];  
				if ((cnt2 & 0x0001) == 0)
				{
					 
					buf1 = (id << 16) | (buf0 >> 16);
				}
				else
				{
					 
					buf1 = (buf0 & 0xFFFF0000) | id;
				}
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf1; 
				bound1 = ((CANAF_FullCAN_cnt+1)>>1)+((CANAF_std_cnt+1)>>1)-1;
				 
				while (cnt1 < bound1)
				{
					cnt1++;
					buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
					buf0  = buf1;
				}

				if ((CANAF_std_cnt & 0x0001) == 0)
				{
					 
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1] = (buf0 <<16) |(0x0000FFFF);
				}
			}
		}
		CANAF_std_cnt++;
		
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa +=0x04 ;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     +=0x04 ;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa +=0x04;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable +=0x04;
 	}

 
 	else
 	{
 		 
 		id |= (tmp0) << 29;

 		cnt1 = ((CANAF_FullCAN_cnt+1)>>1)+(((CANAF_std_cnt + 1) >> 1) + CANAF_gstd_cnt);
 		cnt2 = 0;
 		while (cnt2 < CANAF_ext_cnt)
 		{
 			 
 			if (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] > id)
 			{
 				break;
 			}
 			cnt1++; 
			cnt2++;
 		}

 		buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];   
 		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = id;     

 		CANAF_ext_cnt++;

 		bound1 = total;
 		 
 		while (cnt2 < bound1)
 		{
 			cnt1++;
 			cnt2++;
 			buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
 			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf0;
 			buf0 = buf1;
 		}
 		 
 		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa += 4;
 		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable += 4;
 	}
 	if(CANAF_FullCAN_cnt == 0) 
 	{
 		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00;
 	}
 	else
 	{
 		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
 	}

 	return CAN_OK;
}

 









 
CAN_ERROR CAN_LoadFullCANEntry (LPC_CAN_TypeDef* CANx, uint16_t id)
{
	uint32_t ctrl0 = 0;
	uint32_t buf0=0, buf1=0, buf2=0;
	uint32_t tmp0=0, tmp1=0, tmp2=0;
	int16_t cnt1=0, cnt2=0, bound1=0, total=0;

	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 718));

	if (CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		ctrl0 = 0;
	}
	else if (CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))
	{
		ctrl0 = 1;
	}

	 
	total =((CANAF_FullCAN_cnt+1)>>1)+ CANAF_FullCAN_cnt*3 +((CANAF_std_cnt + 1) >> 1)+  			CANAF_gstd_cnt + CANAF_ext_cnt + (CANAF_gext_cnt<<1);

	
	if ((total >=508)||(CANAF_FullCAN_cnt>=64)){
		return CAN_OBJECTS_FULL_ERROR;
	}
	
 
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00000001;

	 
	id &= 0x07FF;
	id |= (ctrl0 << 13) | (1 << 11);  

	
 
	if (((CANAF_FullCAN_cnt & 0x0001) == 0)&&(total!=0))
	{
		
		cnt1   = (CANAF_FullCAN_cnt >> 1);
		bound1 = total;
		buf0   = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];

		while (bound1--)
		{
			cnt1++;
			buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf0;
			buf0 = buf1;
		}
	}
	if (CANAF_FullCAN_cnt == 0)
	{
		 
		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] = 0x0000FFFF | (id << 16);
	}
	else if (CANAF_FullCAN_cnt == 1)
	{
		 
		if (((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] >> 16)& 0xE7FF) > id)
		{
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] >> 16) | (id << 16);
		}
		else
		{
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[0] & 0xFFFF0000) | id;
		}
	}
	else
	{
		 
		cnt1 = 0;
		cnt2 = CANAF_FullCAN_cnt;
		bound1 = (CANAF_FullCAN_cnt - 1) >> 1;
		while (cnt1 <= bound1)
		{
			 
			if (((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >> 16) & 0xE7FF) > (id & 0xE7FF))
			{
				cnt2 = cnt1 * 2;
				break;
			}

			if ((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] & 0x0000E7FF) > (id & 0xE7FF))
			{
				cnt2 = cnt1 * 2 + 1;
				break;
			}

			cnt1++;
		}
		 
		 

		if (cnt1 > bound1)
		{
			 
			 
			if ((CANAF_FullCAN_cnt & 0x0001) == 0)
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]  = 0x0000FFFF | (id << 16);
			}
			 
			else
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]  = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] & 0xFFFF0000) | id;
			}
		}
		else
		{
			buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];  
			if ((cnt2 & 0x0001) == 0)
			{
				 
				buf1 = (id << 16) | (buf0 >> 16);
			}
			else
			{
				 
				buf1 = (buf0 & 0xFFFF0000) | id;
			}
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf1; 
			bound1 = CANAF_FullCAN_cnt >> 1;
			 
			while (cnt1 < bound1)
			{
				cnt1++;
				buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = (buf1 >> 16) | (buf0 << 16);
				buf0  = buf1;
			}

			if ((CANAF_FullCAN_cnt & 0x0001) == 0)
			{
				 
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = (((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] & 0xFFFF0000)
											| (0x0000FFFF);
			}
		}
	}
	
	bound1 = CANAF_FullCAN_cnt - cnt2;
	cnt1 = total - (CANAF_FullCAN_cnt)*3 + cnt2*3 + 1;
	buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
	buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1];
	buf2 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+2];
	((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]=((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1]= ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+2]=0x00;
	cnt1+=3;
	while(bound1--)
	{
		tmp0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
		tmp1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1];
		tmp2 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+2];
		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]= buf0;
		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1]= buf1;
		((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+2]= buf2;
		buf0 = tmp0;
		buf1 = tmp1;
		buf2 = tmp2;
		cnt1+=3;
	}
	CANAF_FullCAN_cnt++;
	
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_sa 	  +=0x04;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa +=0x04 ;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     +=0x04 ;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa +=0x04;
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable +=0x04;

	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
 	return CAN_OK;
}

 












 
CAN_ERROR CAN_LoadGroupEntry(LPC_CAN_TypeDef* CANx, uint32_t lowerID, 		uint32_t upperID, CAN_ID_FORMAT_Type format)

{
	uint16_t tmp = 0;
	uint32_t buf0, buf1, entry1, entry2, LID,UID;
	int16_t cnt1, bound1, total;
	

	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 905));
	((((format==STD_ID_FORMAT)||(format==EXT_ID_FORMAT))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 906));

	if(lowerID > upperID) return CAN_CONFLICT_ID_ERROR;
	if(CANx == ((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))
	{
		tmp = 0;
	}
	else
	{
		tmp = 1;
	}

	total =((CANAF_FullCAN_cnt+1)>>1)+ CANAF_FullCAN_cnt*3 +((CANAF_std_cnt + 1) >> 1)+  			CANAF_gstd_cnt + CANAF_ext_cnt + (CANAF_gext_cnt<<1);


	
 
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00000001;

 
	if(format == STD_ID_FORMAT)
	{
		if ((total >= 512)){
			return CAN_OBJECTS_FULL_ERROR;
		}
		lowerID &=0x7FF; 
		upperID &=0x7FF;
		entry1  = (tmp << 29)|(lowerID << 16)|(tmp << 13)|(upperID << 0);
		cnt1 = ((CANAF_FullCAN_cnt+1)>>1) + ((CANAF_std_cnt + 1) >> 1);

		
		if(CANAF_gstd_cnt == 0)
		{
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
		}
		else
		{
			
			bound1 = ((CANAF_FullCAN_cnt+1)>>1) + ((CANAF_std_cnt + 1) >> 1) + CANAF_gstd_cnt;
			while(cnt1 < bound1)
			{
				
				while((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >> 29)< (entry1 >> 29))
					cnt1++;
				buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				if((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >> 29)> (entry1 >> 29)) 
				{
					
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
					break;
				}
				else 
				{
					LID  = (buf0 >> 16)&0x7FF;
					UID  = buf0 & 0x7FF;
					if (upperID <= LID)
					{
						
						((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
						break;
					}
					else if (lowerID >= UID)
					{
						
						cnt1 ++;
					}
					else
						return CAN_CONFLICT_ID_ERROR;
				}
			}
			if(cnt1 >= bound1)
			{
				
				buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
			}

			
			bound1 = total - cnt1;
			while(bound1--)
			{
				cnt1++;
				buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = buf0;
				buf0 = buf1;
			}
		}
		CANAF_gstd_cnt++;
		
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     +=0x04 ;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa +=0x04;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable +=0x04;
	}


 
	else
	{
		if ((total >= 511)){
			return CAN_OBJECTS_FULL_ERROR;
		}
		lowerID  &= 0x1FFFFFFF; 
		upperID &= 0x1FFFFFFF;
		entry1   = (tmp << 29)|(lowerID << 0);
		entry2   = (tmp << 29)|(upperID << 0);

		cnt1 = ((CANAF_FullCAN_cnt+1)>>1) + ((CANAF_std_cnt + 1) >> 1) + CANAF_gstd_cnt + CANAF_ext_cnt;
		
		if(CANAF_gext_cnt == 0)
		{
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1] = entry2;
		}
		else
		{
			
			bound1 = ((CANAF_FullCAN_cnt+1)>>1) + ((CANAF_std_cnt + 1) >> 1) + CANAF_gstd_cnt 						+ CANAF_ext_cnt + (CANAF_gext_cnt<<1);

			while(cnt1 < bound1)
			{
				while((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >>29)< tmp) 
					cnt1++;
				buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1];
				if((((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] >> 29)> (entry1 >> 29)) 
				{
					
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[++cnt1] = entry2;
					break;
				}
				else 
				{
					LID  = buf0 & 0x1FFFFFFF; 
					UID  = buf1 & 0x1FFFFFFF;
					if (upperID <= LID)
					{
						
						((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1] = entry1;
						((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[++cnt1] = entry2;
						break;
					}
					else if (lowerID >= UID)
					{
						
						cnt1 +=2;
					}
					else
						return CAN_CONFLICT_ID_ERROR;
				}
			}
			if(cnt1 >= bound1)
			{
				
				buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				buf1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]   = entry1;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[++cnt1] = entry2;
			}
			
			bound1 = total - cnt1 + 1;
			cnt1++;
			while(bound1>0)
			{
				entry1 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1];
				entry2 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1]   = buf0;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt1+1] = buf1;
				buf0 = entry1;
				buf1 = entry2;
				cnt1   +=2;
				bound1 -=2;
			}
		}
		CANAF_gext_cnt++;
		
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable +=0x08;
	}
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
 	return CAN_OK;
}

 












 
CAN_ERROR CAN_RemoveEntry(AFLUT_ENTRY_Type EntryType, uint16_t position)
{
	uint16_t cnt, bound, total;
	uint32_t buf0, buf1;
	((((EntryType==FULLCAN_ENTRY)||(EntryType==EXPLICIT_STANDARD_ENTRY)||(EntryType==GROUP_STANDARD_ENTRY)||(EntryType==EXPLICIT_EXTEND_ENTRY) ||(EntryType==GROUP_EXTEND_ENTRY))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1106));
	(((position<512)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1107));

	
 
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00000001;
	total = ((CANAF_FullCAN_cnt+1)>>1)+((CANAF_std_cnt + 1) >> 1) + 			CANAF_gstd_cnt + CANAF_ext_cnt + (CANAF_gext_cnt<<1);



 
	if(EntryType == FULLCAN_ENTRY)
	{
		if((CANAF_FullCAN_cnt==0)||(position >= CANAF_FullCAN_cnt))
		{
			return CAN_ENTRY_NOT_EXIT_ERROR;
		}
		else
		{
			cnt = position >> 1;
			buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
			bound = (CANAF_FullCAN_cnt - position -1)>>1;
			if((position & 0x0001) == 0) 
			{
				while(bound--)
				{
					
					buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1];
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf1 >> 16) | (buf0 << 16);
					buf0  = buf1;
					cnt++;
				}
			}
			else 
			{
				while(bound--)
				{
					
					buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1];
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf0 & 0xFFFF0000)|(buf1 >> 16);
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1] << 16;
					buf0  = buf1<<16;
					cnt++;
				}
			}
			if((CANAF_FullCAN_cnt & 0x0001) == 0)
			{
				if((position & 0x0001)==0)
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf0 << 16) | (0x0000FFFF);
				else
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = buf0 | 0x0000FFFF;
			}
			else
			{
				
				cnt = (CANAF_FullCAN_cnt + 1)>>1;
				bound = total + CANAF_FullCAN_cnt * 3;
				while(bound>cnt)
				{
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
					cnt++;
				}
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1]=0x00;
				
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_sa 	  -=0x04;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa -=0x04 ;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     -=0x04 ;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa -=0x04;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable -=0x04;
			}
			CANAF_FullCAN_cnt--;

			
			
			cnt = total + position * 3;
			bound = (CANAF_FullCAN_cnt - position + 1) * 3;

			while(bound)
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt]=((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+3];;
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1]=((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+4];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+2]=((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+5];
				bound -=3;
				cnt   +=3;
			}
		}
	}

 
	else if(EntryType == EXPLICIT_STANDARD_ENTRY)
	{
		if((CANAF_std_cnt==0)||(position >= CANAF_std_cnt))
		{
			return CAN_ENTRY_NOT_EXIT_ERROR;
		}
		else
		{
			cnt = ((CANAF_FullCAN_cnt+1)>>1)+ (position >> 1);
			buf0 = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
			bound = (CANAF_std_cnt - position - 1)>>1;
			if((position & 0x0001) == 0) 
			{
				while(bound--)
				{
					
					buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1];
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf1 >> 16) | (buf0 << 16);
					buf0  = buf1;
					cnt++;
				}
			}
			else 
			{
				while(bound--)
				{
					
					buf1  = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1];
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf0 & 0xFFFF0000)|(buf1 >> 16);
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1] << 16;
					buf0  = buf1<<16;
					cnt++;
				}
			}
			if((CANAF_std_cnt & 0x0001) == 0)
			{
				if((position & 0x0001)==0)
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = (buf0 << 16) | (0x0000FFFF);
				else
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = buf0 | 0x0000FFFF;
			}
			else
			{
				
				cnt = ((CANAF_FullCAN_cnt + 1)>>1) + ((CANAF_std_cnt + 1) >> 1);
				bound = total + CANAF_FullCAN_cnt * 3;
				while(bound>cnt)
				{
					((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
					cnt++;
				}
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1]=0x00;
				
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->SFF_GRP_sa -=0x04 ;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     -=0x04 ;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa -=0x04;
				((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable -=0x04;
			}
			CANAF_std_cnt--;
		}
	}

 
	else if(EntryType == GROUP_STANDARD_ENTRY)
	{
		if((CANAF_gstd_cnt==0)||(position >= CANAF_gstd_cnt))
		{
			return CAN_ENTRY_NOT_EXIT_ERROR;
		}
		else
		{
			cnt = ((CANAF_FullCAN_cnt + 1)>>1) + ((CANAF_std_cnt + 1) >> 1)+ position + 1;
			bound = total + CANAF_FullCAN_cnt * 3;
			while (cnt<bound)
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
				cnt++;
			}
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1]=0x00;
		}
		CANAF_gstd_cnt--;
		
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_sa     -=0x04;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa -=0x04;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable -=0x04;
	}

 
	else if(EntryType == EXPLICIT_EXTEND_ENTRY)
	{
		if((CANAF_ext_cnt==0)||(position >= CANAF_ext_cnt))
		{
			return CAN_ENTRY_NOT_EXIT_ERROR;
		}
		else
		{
			cnt = ((CANAF_FullCAN_cnt + 1)>>1) + ((CANAF_std_cnt + 1) >> 1)+ CANAF_gstd_cnt + position + 1;
			bound = total + CANAF_FullCAN_cnt * 3;
			while (cnt<bound)
			{
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt];
				cnt++;
			}
			((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt-1]=0x00;
		}
		CANAF_ext_cnt--;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->EFF_GRP_sa -=0x04;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable -=0x04;
	}

 
	else
	{
		if((CANAF_gext_cnt==0)||(position >= CANAF_gext_cnt))
		{
			return CAN_ENTRY_NOT_EXIT_ERROR;
		}
		else
		{
			cnt = total - (CANAF_gext_cnt<<1) + (position<<1);
			bound = total + CANAF_FullCAN_cnt * 3;
			while (cnt<bound)
			{
				
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+2];
				((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+1] = ((LPC_CANAF_RAM_TypeDef *) ((0x40000000UL) + 0x38000))->mask[cnt+3];
				cnt+=2;
			}
		}
		CANAF_gext_cnt--;
		((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable -=0x08;
	}
	((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
	return CAN_OK;
}

 









 
Status CAN_SendMsg (LPC_CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg)
{
	uint32_t data;
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1345));
	((((CAN_Msg->format==STD_ID_FORMAT)||(CAN_Msg->format==EXT_ID_FORMAT))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1346));
	if(CAN_Msg->format==STD_ID_FORMAT)
	{
		((((CAN_Msg->id>>11)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1349));
	}
	else
	{
		((((CAN_Msg->id>>29)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1353));
	}
	((((CAN_Msg->len>>4)==0)) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1355));
	((((CAN_Msg->type==DATA_FRAME)||(CAN_Msg->type==REMOTE_FRAME))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1356));

	
	if (CANx->SR & (1<<2))
	{
		 
		
 
		CANx->TFI1 &= ~0x000F0000;
		CANx->TFI1 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI1 |= (1<<30); 
		}
		else
		{
			CANx->TFI1 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI1 |= (0x80000000); 
		}
		else
		{
			CANx->TFI1 &= ~(0x80000000);
		}

		 
		CANx->TID1 = CAN_Msg->id;

		 
		data = (CAN_Msg->dataA[0])|(((CAN_Msg->dataA[1]))<<8)|((CAN_Msg->dataA[2])<<16)|((CAN_Msg->dataA[3])<<24);
		CANx->TDA1 = data;

		 
		data = (CAN_Msg->dataB[0])|(((CAN_Msg->dataB[1]))<<8)|((CAN_Msg->dataB[2])<<16)|((CAN_Msg->dataB[3])<<24);
		CANx->TDB1 = data;

		  
		 CANx->CMR = 0x21;
		 return SUCCESS;
	}
	
	else if(CANx->SR & (1<<10))
	{
		 
		
 
		CANx->TFI2 &= ~0x000F0000;
		CANx->TFI2 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI2 |= (1<<30); 
		}
		else
		{
			CANx->TFI2 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI2 |= (0x80000000); 
		}
		else
		{
			CANx->TFI2 &= ~(0x80000000);
		}

		 
		CANx->TID2 = CAN_Msg->id;

		 
		data = (CAN_Msg->dataA[0])|(((CAN_Msg->dataA[1]))<<8)|((CAN_Msg->dataA[2])<<16)|((CAN_Msg->dataA[3])<<24);
		CANx->TDA2 = data;

		 
		data = (CAN_Msg->dataB[0])|(((CAN_Msg->dataB[1]))<<8)|((CAN_Msg->dataB[2])<<16)|((CAN_Msg->dataB[3])<<24);
		CANx->TDB2 = data;

		 
		CANx->CMR = 0x41;
		return SUCCESS;
	}
	
	else if (CANx->SR & (1<<18))
	{
		 
		
 
		CANx->TFI3 &= ~0x000F0000;
		CANx->TFI3 |= (CAN_Msg->len)<<16;
		if(CAN_Msg->type == REMOTE_FRAME)
		{
			CANx->TFI3 |= (1<<30); 
		}
		else
		{
			CANx->TFI3 &= ~(1<<30);
		}
		if(CAN_Msg->format == EXT_ID_FORMAT)
		{
			CANx->TFI3 |= (0x80000000); 
		}
		else
		{
			CANx->TFI3 &= ~(0x80000000);
		}

		 
		CANx->TID3 = CAN_Msg->id;

		 
		data = (CAN_Msg->dataA[0])|(((CAN_Msg->dataA[1]))<<8)|((CAN_Msg->dataA[2])<<16)|((CAN_Msg->dataA[3])<<24);
		CANx->TDA3 = data;

		 
		data = (CAN_Msg->dataB[0])|(((CAN_Msg->dataB[1]))<<8)|((CAN_Msg->dataB[2])<<16)|((CAN_Msg->dataB[3])<<24);
		CANx->TDB3 = data;

		 
		CANx->CMR = 0x81;
		return SUCCESS;
	}
	else
	{
		return ERROR;
	}
}

 









 
Status CAN_ReceiveMsg (LPC_CAN_TypeDef *CANx, CAN_MSG_Type *CAN_Msg)
{
	uint32_t data;

	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1499));

	
	if((CANx->SR &0x00000001))
	{
		 
		 
		CAN_Msg->format   = (uint8_t)(((CANx->RFS) & 0x80000000)>>31);
		CAN_Msg->type     = (uint8_t)(((CANx->RFS) & 0x40000000)>>30);
		CAN_Msg->len      = (uint8_t)(((CANx->RFS) & 0x000F0000)>>16);


		 
		CAN_Msg->id = CANx->RID;

		 
		if (CAN_Msg->type == DATA_FRAME)
		{
			 
			data = CANx->RDA;
			*((uint8_t *) &CAN_Msg->dataA[0])= data & 0x000000FF;
			*((uint8_t *) &CAN_Msg->dataA[1])= (data & 0x0000FF00)>>8;;
			*((uint8_t *) &CAN_Msg->dataA[2])= (data & 0x00FF0000)>>16;
			*((uint8_t *) &CAN_Msg->dataA[3])= (data & 0xFF000000)>>24;

			 
			data = CANx->RDB;
			*((uint8_t *) &CAN_Msg->dataB[0])= data & 0x000000FF;
			*((uint8_t *) &CAN_Msg->dataB[1])= (data & 0x0000FF00)>>8;
			*((uint8_t *) &CAN_Msg->dataB[2])= (data & 0x00FF0000)>>16;
			*((uint8_t *) &CAN_Msg->dataB[3])= (data & 0xFF000000)>>24;

		 
		CANx->CMR = 0x04;
		}
		else
		{
			
 
			CANx->CMR = 0x04;  
			return SUCCESS;
		}
	}
	else
	{
		
		return ERROR;
	}
	return SUCCESS;
}

 








 
CAN_ERROR FCAN_ReadObj (LPC_CANAF_TypeDef* CANAFx, CAN_MSG_Type *CAN_Msg)
{
	uint32_t *pSrc, data;
	uint32_t interrut_word, msg_idx, test_bit, head_idx, tail_idx;

	(((((uint32_t*)CANAFx)== ((uint32_t*)((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1565));

	interrut_word = 0;

	if (((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIC0 != 0)
	{
		interrut_word = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIC0;
		head_idx = 0;
		tail_idx = 31;
	}
	else if (((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIC1 != 0)
	{
		interrut_word = ((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIC1;
		head_idx = 32;
		tail_idx = 63;
	}

	if (interrut_word != 0)
	{
		 
		msg_idx = 0;
		for (msg_idx = head_idx; msg_idx <= tail_idx; msg_idx++)
		{
			test_bit = interrut_word & 0x1;
			interrut_word = interrut_word >> 1;

			if (test_bit)
			{
				pSrc = (uint32_t *) (((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->ENDofTable + ((0x40000000UL) + 0x38000) + msg_idx * 12);

	    	 	 
	    	 	if ((*pSrc & 0x03000000L) == 0x03000000L)
	    	 	{
	    	 		 
	    	 		*pSrc &= 0xFCFFFFFF;

	    	 		 
	    	 		pSrc++;
	    	 		 
	    	 		data = *pSrc;
	    			*((uint8_t *) &CAN_Msg->dataA[0])= data & 0x000000FF;
	    			*((uint8_t *) &CAN_Msg->dataA[1])= (data & 0x0000FF00)>>8;
	    			*((uint8_t *) &CAN_Msg->dataA[2])= (data & 0x00FF0000)>>16;
	    			*((uint8_t *) &CAN_Msg->dataA[3])= (data & 0xFF000000)>>24;

	    	 		 
	    	 		pSrc++;
	    	 		 
	    	 		data = *pSrc;
	    			*((uint8_t *) &CAN_Msg->dataB[0])= data & 0x000000FF;
	    			*((uint8_t *) &CAN_Msg->dataB[1])= (data & 0x0000FF00)>>8;
	    			*((uint8_t *) &CAN_Msg->dataB[2])= (data & 0x00FF0000)>>16;
	    			*((uint8_t *) &CAN_Msg->dataB[3])= (data & 0xFF000000)>>24;
	    	 		 
	    	 		pSrc -= 2;

	    	 		CAN_Msg->id = *pSrc & 0x7FF;
	    	 		CAN_Msg->len = (uint8_t) (*pSrc >> 16) & 0x0F;
					CAN_Msg->format = 0; 
					CAN_Msg->type = (uint8_t)(*pSrc >> 30) &0x01;
	    	 		 
	    	 		if ((*pSrc & 0x03000000L) == 0)
	    	 		{
	    	 			return CAN_OK;
	    	 		}
	    	 	}
			}
		}
	}
	return CAN_FULL_OBJ_NOT_RCV;
}
 











 
uint32_t CAN_GetCTRLStatus (LPC_CAN_TypeDef* CANx, CAN_CTRL_STS_Type arg)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1651));
	((((arg==CANCTRL_GLOBAL_STS)||(arg==CANCTRL_INT_CAP) ||(arg==CANCTRL_ERR_WRN)||(arg==CANCTRL_STS))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1652));

	switch (arg)
	{
	case CANCTRL_GLOBAL_STS:
		return CANx->GSR;

	case CANCTRL_INT_CAP:
		return CANx->ICR;

	case CANCTRL_ERR_WRN:
		return CANx->EWL;

	default: 
		return CANx->SR;
	}
}
 








 
uint32_t CAN_GetCRStatus (LPC_CANCR_TypeDef* CANCRx, CAN_CR_STS_Type arg)
{
	(((((uint32_t*)CANCRx)==((uint32_t*)((LPC_CANCR_TypeDef *) ((0x40000000UL) + 0x40000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1681));
	((((arg==CANCR_TX_STS)||(arg==CANCR_RX_STS) ||(arg==CANCR_MS))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1682));

	switch (arg)
	{
	case CANCR_TX_STS:
		return CANCRx->CANTxSR;

	case CANCR_RX_STS:
		return CANCRx->CANRxSR;

	default:	
		return CANCRx->CANMSR;
	}
}
 






















 
void CAN_IRQCmd (LPC_CAN_TypeDef* CANx, CAN_INT_EN_Type arg, FunctionalState NewState)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1722));
	((((arg==CANINT_RIE)||(arg==CANINT_TIE1) ||(arg==CANINT_EIE)||(arg==CANINT_DOIE) ||(arg==CANINT_WUIE)||(arg==CANINT_EPIE) ||(arg==CANINT_ALIE)||(arg==CANINT_BEIE) ||(arg==CANINT_IDIE)||(arg==CANINT_TIE2) ||(arg==CANINT_TIE3)||(arg==CANINT_FCE))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1723));
	((((NewState==DISABLE) || (NewState==ENABLE))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1724));

	if(NewState == ENABLE)
	{
		if(arg==CANINT_FCE)
		{
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x01;
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIE = 0x01;
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x04;
		}
		else
			CANx->IER |= (1 << arg);
	}
	else
	{
		if(arg==CANINT_FCE){
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x01;
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->FCANIE = 0x01;
			((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )->AFMR = 0x00;
		}
		else
			CANx->IER &= ~(1 << arg);
	}
}

 








 
void CAN_SetAFMode (LPC_CANAF_TypeDef* CANAFx, CAN_AFMODE_Type AFMode)
{
	(((((uint32_t*)CANAFx)== ((uint32_t*)((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1761));
	((((AFMode==CAN_Normal)||(AFMode==CAN_AccOff) ||(AFMode==CAN_AccBP)||(AFMode==CAN_eFCAN))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1762));

	switch(AFMode)
	{
	case CAN_Normal:
		CANAFx->AFMR = 0x00;
		break;
	case CAN_AccOff:
		CANAFx->AFMR = 0x01;
		break;
	case CAN_AccBP:
		CANAFx->AFMR = 0x02;
		break;
	case CAN_eFCAN:
		CANAFx->AFMR = 0x04;
		break;
	}
}

 

















 
void CAN_ModeConfig(LPC_CAN_TypeDef* CANx, CAN_MODE_Type mode, FunctionalState NewState)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1802));
	((((mode==CAN_OPERATING_MODE)||(mode==CAN_RESET_MODE) ||(mode==CAN_LISTENONLY_MODE)||(mode==CAN_SELFTEST_MODE) ||(mode==CAN_TXPRIORITY_MODE)||(mode==CAN_SLEEP_MODE) ||(mode==CAN_RXPOLARITY_MODE)||(mode==CAN_TEST_MODE))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1803));
	((((NewState==DISABLE) || (NewState==ENABLE))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1804));

	switch(mode)
	{
	case CAN_OPERATING_MODE:
		CANx->MOD = 0x00;
		break;
	case CAN_RESET_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1));
		else
			CANx->MOD &= ~((uint32_t)(1));
		break;
	case CAN_LISTENONLY_MODE:
		CANx->MOD |=((uint32_t)(1));
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<1));
		else
			CANx->MOD &=~((uint32_t)(1<<1));
		CANx->MOD &=~((uint32_t)(1));
		break;
	case CAN_SELFTEST_MODE:
		CANx->MOD |=((uint32_t)(1));
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<2));
		else
			CANx->MOD &=~((uint32_t)(1<<2));
		CANx->MOD &=~((uint32_t)(1));
		break;
	case CAN_TXPRIORITY_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<3));
		else
			CANx->MOD &=~((uint32_t)(1<<3));
		break;
	case CAN_SLEEP_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<4));
		else
			CANx->MOD &=~((uint32_t)(1<<4));
		break;
	case CAN_RXPOLARITY_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<5));
		else
			CANx->MOD &=~((uint32_t)(1<<5));
		break;
	case CAN_TEST_MODE:
		if(NewState == ENABLE)
			CANx->MOD |=((uint32_t)(1<<7));
		else
			CANx->MOD &=~((uint32_t)(1<<7));
		break;
	}
}
 














 
void CAN_SetCommand(LPC_CAN_TypeDef* CANx, uint32_t CMRType)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1877));
	CANx->CMR |= CMRType;
}

 





 
uint32_t CAN_IntGetStatus(LPC_CAN_TypeDef* CANx)
{
	((((((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x44000) ))) ||(((uint32_t*)CANx)==((uint32_t *)((LPC_CAN_TypeDef *) ((0x40000000UL) + 0x48000) ))))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1890));
	return CANx->ICR;
}

 





 
IntStatus CAN_FullCANIntGetStatus (LPC_CANAF_TypeDef* CANAFx)
{
	(((((uint32_t*)CANAFx)== ((uint32_t*)((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1903));
	if (CANAFx->FCANIE)
		return SET;
	return RESET;
}

 






 
uint32_t CAN_FullCANPendGetStatus(LPC_CANAF_TypeDef* CANAFx, FullCAN_IC_Type type)
{
	(((((uint32_t*)CANAFx)== ((uint32_t*)((LPC_CANAF_TypeDef *) ((0x40000000UL) + 0x3C000) )))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1919));
	((((type==FULLCAN_IC0)||(type==FULLCAN_IC1))) ? (void)0 : check_failed((uint8_t *)"Lib\\Drivers\\source\\lpc17xx_can.c", 1920));
	if (type == FULLCAN_IC0)
		return CANAFx->FCANIC0;
	return CANAFx->FCANIC1;
}
 


 





 

 
