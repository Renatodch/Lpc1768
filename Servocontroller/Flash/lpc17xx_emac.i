#line 1 "Lib\\Drivers\\source\\lpc17xx_emac.c"


 



























 

 


 

 
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"


 



























 

 



 




 
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



 

#line 44 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"
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



 






 

 
#line 45 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"










 


 


 




 








 
 


 


 





 
 

 
#line 103 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 120 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 

 



 



 


 

 
 

 


 

 


 

 

 


 

 


 

 
 


 

 



 

 




 

 






 

 






 

 



 

 


 

 


 

 





 
 
 


 
 

 
#line 241 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 



 

 
#line 268 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 



 

 
#line 294 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 



 

 



 
 

 
#line 319 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 332 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"


 
 

 
#line 348 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 


 
 

 



 

 



 

 
#line 387 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 398 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 410 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"


 

 


 


 

 
#line 432 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 448 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 460 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 474 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"

 

 
#line 485 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"














#line 587 ".\\Lib\\Drivers\\include\\lpc17xx_emac.h"



 


 


 

 



 
typedef struct {
	uint32_t Packet;	 
	uint32_t Ctrl;		 
} RX_Desc;



 
typedef struct {
	uint32_t Info;		 
	uint32_t HashCRC;	 
} RX_Stat;



 
typedef struct {
	uint32_t Packet;	 
	uint32_t Ctrl;		 
} TX_Desc;



 
typedef struct {
   uint32_t Info;		 
} TX_Stat;




 
typedef struct {
	uint32_t ulDataLen;			 
	uint32_t *pbDataBuf;		 
} EMAC_PACKETBUF_Type;



 
typedef struct {
	uint32_t	Mode;						





 
	uint8_t 	*pbEMAC_Addr;				

 
} EMAC_CFG_Type;




 


 


 
 
Status EMAC_Init(EMAC_CFG_Type *EMAC_ConfigStruct);
void EMAC_DeInit(void);

 
int32_t EMAC_CheckPHYStatus(uint32_t ulPHYState);
int32_t EMAC_SetPHYMode(uint32_t ulPHYMode);
int32_t EMAC_UpdatePHYStatus(void);

 
void EMAC_SetHashFilter(uint8_t dstMAC_addr[], FunctionalState NewState);
void EMAC_SetFilterMode(uint32_t ulFilterMode, FunctionalState NewState);

 
void EMAC_WritePacketBuffer(EMAC_PACKETBUF_Type *pDataStruct);
void EMAC_ReadPacketBuffer(EMAC_PACKETBUF_Type *pDataStruct);

 
void EMAC_IntCmd(uint32_t ulIntType, FunctionalState NewState);
IntStatus EMAC_IntGetStatus(uint32_t ulIntType);

 
Bool EMAC_CheckReceiveIndex(void);
Bool EMAC_CheckTransmitIndex(void);
void EMAC_UpdateRxConsumeIndex(void);
void EMAC_UpdateTxProduceIndex(void);

FlagStatus EMAC_CheckReceiveDataStatus(uint32_t ulRxStatType);
uint32_t EMAC_GetReceiveDataSize(void);
FlagStatus EMAC_GetWoLStatus(uint32_t ulWoLMode);



 









 

 
#line 40 "Lib\\Drivers\\source\\lpc17xx_emac.c"
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



 










 

 
#line 41 "Lib\\Drivers\\source\\lpc17xx_emac.c"




 
#line 1 ".\\Lib\\Drivers\\include\\lpc17xx_libcfg_default.h"


 


























 

 



 




 
#line 43 ".\\Lib\\Drivers\\include\\lpc17xx_libcfg_default.h"


 


 

 

 




 
 

 


 


 


 






 


 


 





 





 


 



 



 


 



 



 


 


 



 


 


 


 


 


 










 







 


 


 


void check_failed(uint8_t *file, uint32_t line);




 





 

 
#line 50 "Lib\\Drivers\\source\\lpc17xx_emac.c"





 


 

 
const uint8_t EMAC_clkdiv[] = { 4, 6, 8, 10, 14, 20, 28, 36, 40, 44, 48, 52, 56, 60, 64};

 

 
static RX_Desc Rx_Desc[4];

 

static __align(8) RX_Stat Rx_Stat[4];
#line 77 "Lib\\Drivers\\source\\lpc17xx_emac.c"

 
static TX_Desc Tx_Desc[3];
 
static TX_Stat Tx_Stat[3];

 
 
static uint32_t rx_buf[4][1536>>2];
 
static uint32_t tx_buf[3][1536>>2];



 

 
static void rx_descr_init (void);
static void tx_descr_init (void);
static int32_t write_PHY (uint32_t PhyReg, uint16_t Value);
static int32_t  read_PHY (uint32_t PhyReg);

static void setEmacAddr(uint8_t abStationAddr[]);
static int32_t emac_CRCCalc(uint8_t frame_no_fcs[], int32_t frame_len);


 
 



 
static void rx_descr_init (void)
{
	 
	uint32_t i;

	for (i = 0; i < 4; i++) {
		Rx_Desc[i].Packet  = (uint32_t)&rx_buf[i];
		Rx_Desc[i].Ctrl    = 0x80000000 | (1536 - 1);
		Rx_Stat[i].Info    = 0;
		Rx_Stat[i].HashCRC = 0;
	}

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxDescriptor       = (uint32_t)&Rx_Desc[0];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxStatus           = (uint32_t)&Rx_Stat[0];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxDescriptorNumber = 4 - 1;

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex  = 0;
}


 
 



 
static void tx_descr_init (void) {
	 
	uint32_t i;

	for (i = 0; i < 3; i++) {
		Tx_Desc[i].Packet = (uint32_t)&tx_buf[i];
		Tx_Desc[i].Ctrl   = 0;
		Tx_Stat[i].Info   = 0;
	}

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxDescriptor       = (uint32_t)&Tx_Desc[0];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxStatus           = (uint32_t)&Tx_Stat[0];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxDescriptorNumber = 3 - 1;

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex  = 0;
}


 
 





 
static int32_t write_PHY (uint32_t PhyReg, uint16_t Value)
{
	 
	uint32_t tout;

	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MADR = 0x0100 | PhyReg;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MWTD = Value;

	 
	tout = 0;
	for (tout = 0; tout < 0x00050000; tout++) {
		if ((((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MIND & 0x00000001) == 0) {
			return (0);
		}
	}
	
	return (-1);
}


 
 




 
static int32_t read_PHY (uint32_t PhyReg)
{
	 
	uint32_t tout;

	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MADR = 0x0100 | PhyReg;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MCMD = 0x00000001;

	 
	tout = 0;
	for (tout = 0; tout < 0x00050000; tout++) {
		if ((((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MIND & 0x00000001) == 0) {
			((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MCMD = 0;
			return (((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MRDD);
		}
	}
	
	return (-1);
}

 




 
static void setEmacAddr(uint8_t abStationAddr[])
{
	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SA0 = ((uint32_t)abStationAddr[5] << 8) | (uint32_t)abStationAddr[4];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SA1 = ((uint32_t)abStationAddr[3] << 8) | (uint32_t)abStationAddr[2];
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SA2 = ((uint32_t)abStationAddr[1] << 8) | (uint32_t)abStationAddr[0];
}


 




 
static int32_t emac_CRCCalc(uint8_t frame_no_fcs[], int32_t frame_len)
{
	int i; 		
	int j; 		
	char byte; 	
	int crc; 	
	int q0, q1, q2, q3; 
	crc = 0xFFFFFFFF;
	for (i = 0; i < frame_len; i++) {
		byte = *frame_no_fcs++;
		for (j = 0; j < 2; j++) {
			if (((crc >> 28) ^ (byte >> 3)) & 0x00000001) {
				q3 = 0x04C11DB7;
			} else {
				q3 = 0x00000000;
			}
			if (((crc >> 29) ^ (byte >> 2)) & 0x00000001) {
				q2 = 0x09823B6E;
			} else {
				q2 = 0x00000000;
			}
			if (((crc >> 30) ^ (byte >> 1)) & 0x00000001) {
				q1 = 0x130476DC;
			} else {
				q1 = 0x00000000;
			}
			if (((crc >> 31) ^ (byte >> 0)) & 0x00000001) {
				q0 = 0x2608EDB8;
			} else {
				q0 = 0x00000000;
			}
			crc = (crc << 4) ^ q3 ^ q2 ^ q1 ^ q0;
			byte >>= 4;
		}
	}
	return crc;
}
 


 


 


 

















 
Status EMAC_Init(EMAC_CFG_Type *EMAC_ConfigStruct)
{
	 
	int32_t regv,tout, tmp;

	 
	CLKPWR_ConfigPPWR (((uint32_t)(1<<30)), ENABLE);

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAC1    = 0x00000100 | 0x00000200 | 0x00000400 |
					0x00000800 | 0x00004000 | 0x00008000;

	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->Command = 0x00000008 | 0x00000010 | 0x00000020 | 0x00000040;

	 
	for (tout = 100; tout; tout--);

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAC1 = 0x00000002;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAC2 = 0x00000010 | 0x00000020;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAXF = 1536;
	

 
	tmp = SystemCoreClock / 2500000UL;
	for (tout = 0; tout < sizeof (EMAC_clkdiv); tout++){
		if (EMAC_clkdiv[tout] >= tmp) break;
	}
	tout++;
	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MCFG = ((tout&0x0F)<<2) | 0x00008000;
	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MCFG &= ~(0x00008000);
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->CLRT = (((0x0F&0x0F))|(((0x37&0x3F)<<8)));
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IPGR = ((0x12&0x7F));

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->Command = 0x00000200 | 0x00000040;

	 


	for (tout = 100; tout; tout--);
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SUPP = 0;

	 
	write_PHY (0x00, (1<<15));

	 
	for (tout = 0x100000UL; tout>=0; tout--) {
		regv = read_PHY (0x00);
		if (!(regv & ((1<<15) | (1<<11)))) {
			 
			break;
		}
		if (tout == 0){
			
			return (ERROR);
		}
	}

	
	if (EMAC_SetPHYMode(EMAC_ConfigStruct->Mode) < 0){
		return (ERROR);
	}

	
	setEmacAddr(EMAC_ConfigStruct->pbEMAC_Addr);

	 
	rx_descr_init ();
	tx_descr_init ();

	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxFilterCtrl = 0x00000004 | 0x00000002 | 0x00000020;

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntEnable = 0x00000008 | 0x00000080;

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntClear  = 0xFFFF;

	 
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->Command  |= (0x00000001 | 0x00000002);
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAC1     |= 0x00000001;

	return SUCCESS;
}


 




 
void EMAC_DeInit(void)
{
	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntEnable = 0x00;
	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntClear = (0xFF) | (0x00001000 | 0x00002000);

	 
	CLKPWR_ConfigPPWR (((uint32_t)(1<<30)), DISABLE);
}


 


















 
int32_t EMAC_CheckPHYStatus(uint32_t ulPHYState)
{
	int32_t regv, tmp;

	regv = read_PHY (0x10);
	switch(ulPHYState){
	case (0):
		tmp = (regv & (1<<0)) ? 1 : 0;
		break;
	case (1):
		tmp = (regv & (1<<1)) ? 0 : 1;
		break;
	case (2):
		tmp = (regv & ((1<<14)|(1<<12))) ? 1 : 0;
		break;
#line 458 "Lib\\Drivers\\source\\lpc17xx_emac.c"
	default:
		tmp = -1;
		break;
	}
	return (tmp);
}


 








 
int32_t EMAC_SetPHYMode(uint32_t ulPHYMode)
{
	int32_t id1, id2, tout;

	 
	id1 = read_PHY (0x02);
	id2 = read_PHY (0x03);


	if (((id1 << 16) | (id2 & 0xFFF0)) == 0x20005C90) {
		switch(ulPHYMode){
		case (0):
			write_PHY (0x00, 0x3000);
#line 498 "Lib\\Drivers\\source\\lpc17xx_emac.c"
			 
			for (tout = 0x100000UL; tout>=0; tout--) {
				
			}
			break;
		case (1):
			 
			write_PHY (0x00, 0x0100);
			break;
		case (2):
			 
			write_PHY (0x00, 0x0000);
			break;
		case (3):
			 
			write_PHY (0x00, 0x2100);
			break;
		case (4):
			 
			write_PHY (0x00, 0x2000);
			break;
		default:
			
			return (-1);
		}
	}
	
	else {
		return (-1);
	}

	
	if (EMAC_UpdatePHYStatus() < 0){
		return (-1);
	}

	
	return (0);
}


 








 
int32_t EMAC_UpdatePHYStatus(void)
{
	int32_t regv, tout;

	 

	for (tout = 0x100000UL; tout>=0; tout--) {
		regv = read_PHY (0x10);
		if (regv & (1<<0)) {
			 
			break;
		}
		if (tout == 0){
			
			return (-1);
		}
	}
	 
	if (regv & (1<<2)) {
	 
			((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->MAC2    |= 0x00000001;
			((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->Command |= 0x00000400;
			((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IPGT     = ((0x15&0x7F));
	} else {
		 
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IPGT = ((0x12&0x7F));
	}
	if (regv & (1<<1)) {
	 
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SUPP = 0;
	} else {
		 
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->SUPP = 0x00000100;
	}
#line 616 "Lib\\Drivers\\source\\lpc17xx_emac.c"
	
	return (0);
}


 

















 
void EMAC_SetHashFilter(uint8_t dstMAC_addr[], FunctionalState NewState)
{
	uint32_t *pReg;
	uint32_t tmp;
	int32_t crc;

	
	crc = emac_CRCCalc(dstMAC_addr, 6);
	
	crc = (crc >> 23) & 0x3F;

	pReg = (crc > 31) ? ((uint32_t *)&((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->HashFilterH) 								: ((uint32_t *)&((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->HashFilterL);

	tmp = (crc > 31) ? (crc - 32) : crc;
	if (NewState == ENABLE) {
		(*pReg) |= (1UL << tmp);
	} else {
		(*pReg) &= ~(1UL << tmp);
	}
	
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->Command &= ~0x00000080;
}

 

























 
void EMAC_SetFilterMode(uint32_t ulFilterMode, FunctionalState NewState)
{
	if (NewState == ENABLE){
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxFilterCtrl |= ulFilterMode;
	} else {
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxFilterCtrl &= ~ulFilterMode;
	}
}

 















 
FlagStatus EMAC_GetWoLStatus(uint32_t ulWoLMode)
{
	if (((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxFilterWoLStatus & ulWoLMode) {
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxFilterWoLClear = ulWoLMode;
		return SET;
	} else {
		return RESET;
	}
}


 






 
void EMAC_WritePacketBuffer(EMAC_PACKETBUF_Type *pDataStruct)
{
	uint32_t idx,len;
	uint32_t *sp,*dp;

	idx = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex;
	sp  = (uint32_t *)pDataStruct->pbDataBuf;
	dp  = (uint32_t *)Tx_Desc[idx].Packet;
	 
	for (len = (pDataStruct->ulDataLen + 3) >> 2; len; len--) {
		*dp++ = *sp++;
	}
	Tx_Desc[idx].Ctrl = (pDataStruct->ulDataLen - 1) | (0x80000000 | 0x40000000);
}

 






 
void EMAC_ReadPacketBuffer(EMAC_PACKETBUF_Type *pDataStruct)
{
	uint32_t idx, len;
	uint32_t *dp, *sp;

	idx = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex;
	dp = (uint32_t *)pDataStruct->pbDataBuf;
	sp = (uint32_t *)Rx_Desc[idx].Packet;

	if (pDataStruct->pbDataBuf != ((void*) 0)) {
		for (len = (pDataStruct->ulDataLen + 3) >> 2; len; len--) {
			*dp++ = *sp++;
		}
	}
}

 
















 
void EMAC_IntCmd(uint32_t ulIntType, FunctionalState NewState)
{
	if (NewState == ENABLE) {
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntEnable |= ulIntType;
	} else {
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntEnable &= ~(ulIntType);
	}
}

 















 
IntStatus EMAC_IntGetStatus(uint32_t ulIntType)
{
	if (((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntStatus & ulIntType) {
		((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->IntClear = ulIntType;
		return SET;
	} else {
		return RESET;
	}
}


 








 
Bool EMAC_CheckReceiveIndex(void)
{
	if (((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex != ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxProduceIndex) {
		return TRUE;
	} else {
		return FALSE;
	}
}


 








 
Bool EMAC_CheckTransmitIndex(void)
{
    uint32_t tmp = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxConsumeIndex;
    if (((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex == ( tmp - 1 ))
    {
        return FALSE;
    }
    else if( ( tmp == 0 ) && ( ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex == ( 3 - 1 ) ) )
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}



 

















 
FlagStatus EMAC_CheckReceiveDataStatus(uint32_t ulRxStatType)
{
	uint32_t idx;
	idx = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex;
	return (((Rx_Stat[idx].Info) & ulRxStatType) ? SET : RESET);
}


 




 
uint32_t EMAC_GetReceiveDataSize(void)
{
	uint32_t idx;
	idx =((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex;
	return ((Rx_Stat[idx].Info) & 0x000007FF);
}

 





 
void EMAC_UpdateRxConsumeIndex(void)
{
	
	uint32_t idx = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex;

	 
	if (++idx == 4) idx = 0;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->RxConsumeIndex = idx;
}

 





 
void EMAC_UpdateTxProduceIndex(void)
{
	
	uint32_t idx = ((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex;

	 
	if (++idx == 3) idx = 0;
	((LPC_EMAC_TypeDef *) ((0x50000000UL) + 0x00000) )->TxProduceIndex = idx;
}




 





 

 
