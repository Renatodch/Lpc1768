#line 1 "Lib\\Usb_Drivers\\Src\\usbuser.c"
















 
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



 






 

 
#line 19 "Lib\\Usb_Drivers\\Src\\usbuser.c"

#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
















 

#line 22 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"






typedef __packed union {
#line 35 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint16_t W;

  __packed struct {
#line 44 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
    uint8_t L;
    uint8_t H;
  } WB;



} WORD_BYTE;





 



 





 





 

typedef __packed union _REQUEST_TYPE {
#line 82 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
	__packed struct _BM {
#line 89 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
    uint8_t Recipient : 5;
    uint8_t Type      : 2;
    uint8_t Dir       : 1;
  } BM;



  uint8_t B;
} REQUEST_TYPE;




 
#line 114 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"

 




 



 

typedef __packed struct _USB_SETUP_PACKET {
#line 133 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  REQUEST_TYPE bmRequestType;
  uint8_t         bRequest;
  WORD_BYTE    wValue;
  WORD_BYTE    wIndex;
  uint16_t         wLength;
} USB_SETUP_PACKET;





 
#line 156 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"

 
#line 170 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"

 





 


 




 
#line 201 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"

 

typedef __packed struct _USB_DEVICE_DESCRIPTOR {
#line 211 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t  bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t  idVendor;
  uint16_t  idProduct;
  uint16_t  bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
} USB_DEVICE_DESCRIPTOR;




 

typedef __packed struct _USB_DEVICE_QUALIFIER_DESCRIPTOR {
#line 239 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t  bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint8_t  bNumConfigurations;
  uint8_t  bReserved;
} USB_DEVICE_QUALIFIER_DESCRIPTOR;





typedef __packed struct _USB_CONFIGURATION_DESCRIPTOR {
#line 261 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t  bNumInterfaces;
  uint8_t  bConfigurationValue;
  uint8_t  iConfiguration;
  uint8_t  bmAttributes;
  uint8_t  bMaxPower;
} USB_CONFIGURATION_DESCRIPTOR;




 

typedef __packed struct _USB_INTERFACE_DESCRIPTOR {
#line 283 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bInterfaceNumber;
  uint8_t  bAlternateSetting;
  uint8_t  bNumEndpoints;
  uint8_t  bInterfaceClass;
  uint8_t  bInterfaceSubClass;
  uint8_t  bInterfaceProtocol;
  uint8_t  iInterface;
} USB_INTERFACE_DESCRIPTOR;




 

typedef __packed struct _USB_ENDPOINT_DESCRIPTOR {
#line 306 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t  wMaxPacketSize;
  uint8_t  bInterval;
} USB_ENDPOINT_DESCRIPTOR;




 

typedef __packed struct _USB_STRING_DESCRIPTOR {
#line 326 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t  bString ;
} USB_STRING_DESCRIPTOR;




 

typedef __packed struct _USB_COMMON_DESCRIPTOR {
#line 343 ".\\Lib\\Usb_Drivers\\Inc\\usb.h"
  uint8_t  bLength;
  uint8_t  bDescriptorType;
} USB_COMMON_DESCRIPTOR;






#line 21 "Lib\\Usb_Drivers\\Src\\usbuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"




















 






















































 

#line 84 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





































 

#line 134 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"























 

#line 172 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





 



#line 22 "Lib\\Usb_Drivers\\Src\\usbuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbhw.h"




















 

#line 26 ".\\Lib\\Usb_Drivers\\Inc\\usbhw.h"

 



 
#line 39 ".\\Lib\\Usb_Drivers\\Inc\\usbhw.h"

 



 
#line 53 ".\\Lib\\Usb_Drivers\\Inc\\usbhw.h"

 
#line 63 ".\\Lib\\Usb_Drivers\\Inc\\usbhw.h"

 
typedef struct _USB_DMA_DESCRIPTOR {
  uint32_t BufAdr;                      
  uint16_t  BufLen;                      
  uint16_t  MaxSize;                     
  uint32_t InfoAdr;                     
  union {                            
    struct {
      uint32_t Link   : 1;              
      uint32_t IsoEP  : 1;              
      uint32_t ATLE   : 1;              
      uint32_t Rsrvd  : 5;              
      uint32_t LenPos : 8;              
    } Type;
    uint32_t Val;
  } Cfg;
} USB_DMA_DESCRIPTOR;

 
extern void  USB_Init       (void);
extern void  USB_Connect    (uint32_t  con);
extern void  USB_Reset      (void);
extern void  USB_Suspend    (void);
extern void  USB_Resume     (void);
extern void  USB_WakeUp     (void);
extern void  USB_WakeUpCfg  (uint32_t  cfg);
extern void  USB_SetAddress (uint32_t adr);
extern void  USB_Configure  (uint32_t  cfg);
extern void  USB_ConfigEP   (USB_ENDPOINT_DESCRIPTOR *pEPD);
extern void  USB_DirCtrlEP  (uint32_t dir);
extern void  USB_EnableEP   (uint32_t EPNum);
extern void  USB_DisableEP  (uint32_t EPNum);
extern void  USB_ResetEP    (uint32_t EPNum);
extern void  USB_SetStallEP (uint32_t EPNum);
extern void  USB_ClrStallEP (uint32_t EPNum);
extern void USB_ClearEPBuf  (uint32_t  EPNum);
extern uint32_t USB_ReadEP     (uint32_t EPNum, uint8_t *pData);
extern uint32_t USB_WriteEP    (uint32_t EPNum, uint8_t *pData, uint32_t cnt);
extern uint32_t  USB_DMA_Setup  (uint32_t EPNum, USB_DMA_DESCRIPTOR *pDD);
extern void  USB_DMA_Enable (uint32_t EPNum);
extern void  USB_DMA_Disable(uint32_t EPNum);
extern uint32_t USB_DMA_Status (uint32_t EPNum);
extern uint32_t USB_DMA_BufAdr (uint32_t EPNum);
extern uint32_t USB_DMA_BufCnt (uint32_t EPNum);
extern uint32_t USB_GetFrame   (void);
extern void  USB_IRQHandler (void);


#line 23 "Lib\\Usb_Drivers\\Src\\usbuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbcore.h"
















 





 
typedef struct _USB_EP_DATA {
  uint8_t  *pData;
  uint16_t Count;
} USB_EP_DATA;

 
extern uint16_t USB_DeviceStatus;
extern uint8_t  USB_DeviceAddress;
extern uint8_t  USB_Configuration;
extern uint32_t USB_EndPointMask;
extern uint32_t USB_EndPointHalt;
extern uint32_t USB_EndPointStall;
extern uint8_t  USB_AltSetting[4];

 
extern uint8_t  EP0Buf[8];

 
extern USB_EP_DATA EP0Data;

 
extern USB_SETUP_PACKET SetupPacket;

 
extern void USB_ResetCore (void);



#line 24 "Lib\\Usb_Drivers\\Src\\usbuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbuser.h"
















 





 
extern void USB_Power_Event     (uint32_t power);
extern void USB_Reset_Event     (void);
extern void USB_Suspend_Event   (void);
extern void USB_Resume_Event    (void);
extern void USB_WakeUp_Event    (void);
extern void USB_SOF_Event       (void);
extern void USB_Error_Event     (uint32_t error);

 
#line 46 ".\\Lib\\Usb_Drivers\\Inc\\usbuser.h"

 
extern void (* const USB_P_EP[16])(uint32_t event);

 
extern void USB_EndPoint0  (uint32_t event);
extern void USB_EndPoint1  (uint32_t event);
extern void USB_EndPoint2  (uint32_t event);
extern void USB_EndPoint3  (uint32_t event);
extern void USB_EndPoint4  (uint32_t event);
extern void USB_EndPoint5  (uint32_t event);
extern void USB_EndPoint6  (uint32_t event);
extern void USB_EndPoint7  (uint32_t event);
extern void USB_EndPoint8  (uint32_t event);
extern void USB_EndPoint9  (uint32_t event);
extern void USB_EndPoint10 (uint32_t event);
extern void USB_EndPoint11 (uint32_t event);
extern void USB_EndPoint12 (uint32_t event);
extern void USB_EndPoint13 (uint32_t event);
extern void USB_EndPoint14 (uint32_t event);
extern void USB_EndPoint15 (uint32_t event);

 
extern void USB_Configure_Event (void);
extern void USB_Interface_Event (void);
extern void USB_Feature_Event   (void);


#line 25 "Lib\\Usb_Drivers\\Src\\usbuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\cdcuser.h"
















 




 
extern int CDC_RdOutBuf        (char *buffer, const int *length);
extern int CDC_WrOutBuf        (const char *buffer, int *length);
extern int CDC_OutBufAvailChar (int *availChar);


 



 


 
extern uint32_t CDC_SendEncapsulatedCommand  (void);
extern uint32_t CDC_GetEncapsulatedResponse  (void);
extern uint32_t CDC_SetCommFeature           (unsigned short wFeatureSelector);
extern uint32_t CDC_GetCommFeature           (unsigned short wFeatureSelector);
extern uint32_t CDC_ClearCommFeature         (unsigned short wFeatureSelector);
extern uint32_t CDC_GetLineCoding            (void);
extern uint32_t CDC_SetLineCoding            (void);
extern uint32_t CDC_SetControlLineState      (unsigned short wControlSignalBitmap);
extern uint32_t CDC_SendBreak                (unsigned short wDurationOfBreak);

 
extern void CDC_BulkIn                   (void);
extern void CDC_BulkOut                  (void);

 
extern void CDC_NotificationIn           (void);

 
extern void CDC_Init (char portNum);

 
extern unsigned short CDC_GetSerialState (void);

 
extern unsigned short CDC_DepInEmpty;         



#line 26 "Lib\\Usb_Drivers\\Src\\usbuser.c"






 










 


void USB_Reset_Event (void) {
  USB_ResetCore();
}






 










 










 










 











 










 


void USB_Configure_Event (void) {

  if (USB_Configuration) {                   
     
  }
}






 










 









 
void (* const USB_P_EP[16]) (uint32_t event) = {
  ((0x0007 & (1 << (0))) ? USB_EndPoint0 : ((void*) 0)),
  ((0x0007 & (1 << (1))) ? USB_EndPoint1 : ((void*) 0)),
  ((0x0007 & (1 << (2))) ? USB_EndPoint2 : ((void*) 0)),
  ((0x0007 & (1 << (3))) ? USB_EndPoint3 : ((void*) 0)),
  ((0x0007 & (1 << (4))) ? USB_EndPoint4 : ((void*) 0)),
  ((0x0007 & (1 << (5))) ? USB_EndPoint5 : ((void*) 0)),
  ((0x0007 & (1 << (6))) ? USB_EndPoint6 : ((void*) 0)),
  ((0x0007 & (1 << (7))) ? USB_EndPoint7 : ((void*) 0)),
  ((0x0007 & (1 << (8))) ? USB_EndPoint8 : ((void*) 0)),
  ((0x0007 & (1 << (9))) ? USB_EndPoint9 : ((void*) 0)),
  ((0x0007 & (1 << (10))) ? USB_EndPoint10 : ((void*) 0)),
  ((0x0007 & (1 << (11))) ? USB_EndPoint11 : ((void*) 0)),
  ((0x0007 & (1 << (12))) ? USB_EndPoint12 : ((void*) 0)),
  ((0x0007 & (1 << (13))) ? USB_EndPoint13 : ((void*) 0)),
  ((0x0007 & (1 << (14))) ? USB_EndPoint14 : ((void*) 0)),
  ((0x0007 & (1 << (15))) ? USB_EndPoint15 : ((void*) 0)),
};






 

void USB_EndPoint1 (uint32_t event) {
  uint16_t temp;
  static uint16_t serialState;

  switch (event) {
    case 3:
      temp = CDC_GetSerialState();
      if (serialState != temp) {
         serialState = temp;
         CDC_NotificationIn();             
      }
      break;
  }
}






 

void USB_EndPoint2 (uint32_t event) {

  switch (event) {
    case 2:
      CDC_BulkOut ();                 
      break;
    case 3:
      CDC_BulkIn ();                  
      break;
  }
}






 

void USB_EndPoint3 (uint32_t event) {
}






 

void USB_EndPoint4 (uint32_t event) {
}






 

void USB_EndPoint5 (uint32_t event) {
}






 

void USB_EndPoint6 (uint32_t event) {
}






 

void USB_EndPoint7 (uint32_t event) {
}






 

void USB_EndPoint8 (uint32_t event) {
}






 

void USB_EndPoint9 (uint32_t event) {
}






 

void USB_EndPoint10 (uint32_t event) {
}






 

void USB_EndPoint11 (uint32_t event) {
}






 

void USB_EndPoint12 (uint32_t event) {
}






 

void USB_EndPoint13 (uint32_t event) {
}






 

void USB_EndPoint14 (uint32_t event) {
}






 

void USB_EndPoint15 (uint32_t event) {
}
