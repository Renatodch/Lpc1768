#line 1 "Lib\\Usb_Drivers\\Src\\usbcore.c"






















 
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



 






 

 
#line 25 "Lib\\Usb_Drivers\\Src\\usbcore.c"

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






#line 27 "Lib\\Usb_Drivers\\Src\\usbcore.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"




















 






















































 

#line 84 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





































 

#line 134 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"























 

#line 172 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





 



#line 28 "Lib\\Usb_Drivers\\Src\\usbcore.c"
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


#line 29 "Lib\\Usb_Drivers\\Src\\usbcore.c"
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



#line 30 "Lib\\Usb_Drivers\\Src\\usbcore.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbdesc.h"
















 












extern const uint8_t USB_DeviceDescriptor[];
extern const uint8_t USB_ConfigDescriptor[];
extern const uint8_t USB_StringDescriptor[];


#line 31 "Lib\\Usb_Drivers\\Src\\usbcore.c"
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


#line 32 "Lib\\Usb_Drivers\\Src\\usbcore.c"



















#line 1 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
















 

#line 22 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"






 









#line 45 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"











#line 68 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"








#line 93 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"




#line 132 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"




















#line 160 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"



#line 170 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"




 






typedef __packed struct _CDC_HEADER_DESCRIPTOR{
#line 188 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  uint8_t bFunctionLength;                     
  uint8_t bDescriptorType;                     
  uint8_t bDescriptorSubtype;                  
  uint16_t bcdCDC;                              
} CDC_HEADER_DESCRIPTOR;





typedef __packed struct _CDC_CALL_MANAGEMENT_DESCRIPTOR{
#line 205 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  uint8_t bFunctionLength;                     
  uint8_t bDescriptorType;                     
  uint8_t bDescriptorSubtype;                  
  uint8_t bmCapabilities;                      
  uint8_t bDataInterface;                      
} CDC_CALL_MANAGEMENT_DESCRIPTOR;





typedef __packed struct _CDC_ABSTRACT_CONTROL_MANAGEMENT_DESCRIPTOR{
#line 223 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  uint8_t bFunctionLength;                     
  uint8_t bDescriptorType;                     
  uint8_t bDescriptorSubtype;                  
  uint8_t bmCapabilities;                      
} CDC_ABSTRACT_CONTROL_MANAGEMENT_DESCRIPTOR;





typedef __packed struct _CDC_UNION_DESCRIPTOR{
#line 240 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  uint8_t bFunctionLength;                     
  uint8_t bDescriptorType;                     
  uint8_t bDescriptorSubtype;                  
  uint8_t bMasterInterface;                    
} CDC_UNION_DESCRIPTOR;




typedef __packed struct _CDC_UNION_1SLAVE_DESCRIPTOR{
#line 256 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  CDC_UNION_DESCRIPTOR sUnion;              
  uint8_t                 bSlaveInterfaces[1]; 
} CDC_UNION_1SLAVE_DESCRIPTOR;





typedef __packed struct _CDC_LINE_CODING{
#line 271 ".\\Lib\\Usb_Drivers\\Inc\\cdc.h"
  uint32_t dwDTERate;                          
  uint8_t  bCharFormat;                        
  uint8_t  bParityType;                        
  uint8_t  bDataBits;                          
} CDC_LINE_CODING;




typedef USB_SETUP_PACKET CDC_NOTIFICATION_HEADER;



#line 53 "Lib\\Usb_Drivers\\Src\\usbcore.c"
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



#line 54 "Lib\\Usb_Drivers\\Src\\usbcore.c"









#pragma diag_suppress 111,1441






uint16_t  USB_DeviceStatus;
uint8_t  USB_DeviceAddress;
uint8_t  USB_Configuration;
uint32_t USB_EndPointMask;
uint32_t USB_EndPointHalt;
uint32_t USB_EndPointStall;                          
uint8_t  USB_NumInterfaces;
uint8_t  USB_AltSetting[4];

uint8_t  EP0Buf[8];


USB_EP_DATA EP0Data;

USB_SETUP_PACKET SetupPacket;






 

void USB_ResetCore (void) {

  USB_DeviceStatus  = 0;
  USB_DeviceAddress = 0;
  USB_Configuration = 0;
  USB_EndPointMask  = 0x00010001;
  USB_EndPointHalt  = 0x00000000;
  USB_EndPointStall = 0x00000000;
}






 

void USB_SetupStage (void) {
  USB_ReadEP(0x00, (uint8_t *)&SetupPacket);
}






 

void USB_DataInStage (void) {
  uint32_t cnt;

  if (EP0Data.Count > 8) {
    cnt = 8;
  } else {
    cnt = EP0Data.Count;
  }
  cnt = USB_WriteEP(0x80, EP0Data.pData, cnt);
  EP0Data.pData += cnt;
  EP0Data.Count -= cnt;
}






 

void USB_DataOutStage (void) {
  uint32_t cnt;

  cnt = USB_ReadEP(0x00, EP0Data.pData);
  EP0Data.pData += cnt;
  EP0Data.Count -= cnt;
}






 

void USB_StatusInStage (void) {
  USB_WriteEP(0x80, ((void*) 0), 0);
}






 

void USB_StatusOutStage (void) {
  USB_ReadEP(0x00, EP0Buf);
}






 




__inline uint32_t USB_ReqGetStatus (void) {

  uint32_t n, m;

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:
      EP0Data.pData = (uint8_t *)&USB_DeviceStatus;
      break;
    case 1:
      if ((USB_Configuration != 0) && (SetupPacket.wIndex.WB.L < USB_NumInterfaces)) {
        *((__packed uint16_t *)EP0Buf) = 0;
    	  *((uint16_t *)EP0Buf) = 0;
        EP0Data.pData = EP0Buf;
      } else {
        return (FALSE);
      }
      break;
    case 2:
      n = SetupPacket.wIndex.WB.L & 0x8F;
      m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
      if (((USB_Configuration != 0) || ((n & 0x0F) == 0)) && (USB_EndPointMask & m)) {
        *((__packed uint16_t *)EP0Buf) = (USB_EndPointHalt & m) ? 1 : 0;
    	  *((uint16_t *)EP0Buf) = (USB_EndPointHalt & m) ? 1 : 0;
        EP0Data.pData = EP0Buf;
      } else {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}







 




__inline uint32_t USB_ReqSetClrFeature (uint32_t sc) {

  uint32_t n, m;

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:
      if (SetupPacket.wValue.W == 1) {
        if (sc) {
          USB_WakeUpCfg(TRUE);
          USB_DeviceStatus |=  0x02;
        } else {
          USB_WakeUpCfg(FALSE);
          USB_DeviceStatus &= ~0x02;
        }
      } else {
        return (FALSE);
      }
      break;
    case 1:
      return (FALSE);
    case 2:
      n = SetupPacket.wIndex.WB.L & 0x8F;
      m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
      if ((USB_Configuration != 0) && ((n & 0x0F) != 0) && (USB_EndPointMask & m)) {
        if (SetupPacket.wValue.W == 0) {
          if (sc) {
            USB_SetStallEP(n);
            USB_EndPointHalt |=  m;
          } else {
            if ((USB_EndPointStall & m) != 0) {
              return (TRUE);
            }
            USB_ClrStallEP(n);
#line 267 "Lib\\Usb_Drivers\\Src\\usbcore.c"
            USB_EndPointHalt &= ~m;
          }
        } else {
          return (FALSE);
        }
      } else {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}






 




__inline uint32_t USB_ReqSetAddress (void) {

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:
      USB_DeviceAddress = 0x80 | SetupPacket.wValue.WB.L;
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}






 




__inline uint32_t USB_ReqGetDescriptor (void) {

  uint8_t  *pD;
  uint32_t len, n;

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:
      switch (SetupPacket.wValue.WB.H) {
        case 1:
          EP0Data.pData = (uint8_t *)USB_DeviceDescriptor;
          len = (sizeof(USB_DEVICE_DESCRIPTOR));
          break;
        case 2:
          pD = (uint8_t *)USB_ConfigDescriptor;
          for (n = 0; n != SetupPacket.wValue.WB.L; n++) {
            if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bLength != 0) {
              pD += ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
            }
          }
          if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bLength == 0) {
            return (FALSE);
          }
          EP0Data.pData = pD;
          len = ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
          break;
        case 3:
          pD = (uint8_t *)USB_StringDescriptor;
          for (n = 0; n != SetupPacket.wValue.WB.L; n++) {
            if (((USB_STRING_DESCRIPTOR *)pD)->bLength != 0) {
              pD += ((USB_STRING_DESCRIPTOR *)pD)->bLength;
            }
          }
          if (((USB_STRING_DESCRIPTOR *)pD)->bLength == 0) {
            return (FALSE);
          }
          EP0Data.pData = pD;
          len = ((USB_STRING_DESCRIPTOR *)EP0Data.pData)->bLength;
          break;
        default:
          return (FALSE);
      }
      break;
    case 1:
      switch (SetupPacket.wValue.WB.H) {
#line 376 "Lib\\Usb_Drivers\\Src\\usbcore.c"
        default:
          return (FALSE);
      }

    default:
      return (FALSE);
  }

  if (EP0Data.Count > len) {
    EP0Data.Count = len;
  }

  return (TRUE);
}






 




__inline uint32_t USB_ReqGetConfiguration (void) {

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:
      EP0Data.pData = &USB_Configuration;
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}






 




__inline uint32_t USB_ReqSetConfiguration (void) {

  USB_COMMON_DESCRIPTOR *pD;
  uint32_t alt = 0;
  uint32_t n, m;
  uint32_t tmp;

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 0:

      if (SetupPacket.wValue.WB.L) {
        pD = (USB_COMMON_DESCRIPTOR *)USB_ConfigDescriptor;
        while (pD->bLength) {
          switch (pD->bDescriptorType) {
            case 2:
              if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bConfigurationValue == SetupPacket.wValue.WB.L) {
                USB_Configuration = SetupPacket.wValue.WB.L;
                USB_NumInterfaces = ((USB_CONFIGURATION_DESCRIPTOR *)pD)->bNumInterfaces;
                for (n = 0; n < 4; n++) {
                  USB_AltSetting[n] = 0;
                }
                for (n = 1; n < 16; n++) {
                  if (USB_EndPointMask & (1 << n)) {
                    USB_DisableEP(n);
                  }
                  if (USB_EndPointMask & ((1 << 16) << n)) {
                    USB_DisableEP(n | 0x80);
                  }
                }
                USB_EndPointMask = 0x00010001;
                USB_EndPointHalt = 0x00000000;
                USB_EndPointStall= 0x00000000;
                USB_Configure(TRUE);
                if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bmAttributes & 0x40) {
                  USB_DeviceStatus |=  0x01;
                } else {
                  USB_DeviceStatus &= ~0x01;
                }
              } else {

            	  tmp = (uint32_t)pD;
            	  tmp += ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
            	  pD = (USB_COMMON_DESCRIPTOR *)tmp;
            	  continue;
              }
              break;
            case 4:
              alt = ((USB_INTERFACE_DESCRIPTOR *)pD)->bAlternateSetting;
              break;
            case 5:
              if (alt == 0) {
                n = ((USB_ENDPOINT_DESCRIPTOR *)pD)->bEndpointAddress & 0x8F;
                m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
                USB_EndPointMask |= m;
                USB_ConfigEP((USB_ENDPOINT_DESCRIPTOR *)pD);
                USB_EnableEP(n);
                USB_ResetEP(n);
              }
              break;
          }

			tmp = (uint32_t)pD;
			tmp += pD->bLength;
			pD = (USB_COMMON_DESCRIPTOR *)tmp;
        }
      }
      else {
        USB_Configuration = 0;
        for (n = 1; n < 16; n++) {
          if (USB_EndPointMask & (1 << n)) {
            USB_DisableEP(n);
          }
          if (USB_EndPointMask & ((1 << 16) << n)) {
            USB_DisableEP(n | 0x80);
          }
        }
        USB_EndPointMask  = 0x00010001;
        USB_EndPointHalt  = 0x00000000;
        USB_EndPointStall = 0x00000000;
        USB_Configure(FALSE);
      }

      if (USB_Configuration != SetupPacket.wValue.WB.L) {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}






 




__inline uint32_t USB_ReqGetInterface (void) {

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 1:
      if ((USB_Configuration != 0) && (SetupPacket.wIndex.WB.L < USB_NumInterfaces)) {
        EP0Data.pData = USB_AltSetting + SetupPacket.wIndex.WB.L;
      } else {
        return (FALSE);
      }
      break;
    default:
      return (FALSE);
  }
  return (TRUE);
}






 



__inline uint32_t USB_ReqSetInterface (void) {

  USB_COMMON_DESCRIPTOR *pD;
  uint32_t ifn = 0, alt = 0, old = 0, msk = 0;
  uint32_t n, m;
  uint32_t set;
  uint32_t tmp;

  switch (SetupPacket.bmRequestType.BM.Recipient) {
    case 1:
      if (USB_Configuration == 0) return (FALSE);
      set = FALSE;
      pD  = (USB_COMMON_DESCRIPTOR *)USB_ConfigDescriptor;
      while (pD->bLength) {
        switch (pD->bDescriptorType) {
          case 2:
            if (((USB_CONFIGURATION_DESCRIPTOR *)pD)->bConfigurationValue != USB_Configuration) {

            	tmp = (uint32_t)pD;
            	tmp += ((USB_CONFIGURATION_DESCRIPTOR *)pD)->wTotalLength;
            	pD = (USB_COMMON_DESCRIPTOR *)tmp;

              continue;
            }
            break;
          case 4:
            ifn = ((USB_INTERFACE_DESCRIPTOR *)pD)->bInterfaceNumber;
            alt = ((USB_INTERFACE_DESCRIPTOR *)pD)->bAlternateSetting;
            msk = 0;
            if ((ifn == SetupPacket.wIndex.WB.L) && (alt == SetupPacket.wValue.WB.L)) {
              set = TRUE;
              old = USB_AltSetting[ifn];
              USB_AltSetting[ifn] = (uint8_t)alt;
            }
            break;
          case 5:
            if (ifn == SetupPacket.wIndex.WB.L) {
              n = ((USB_ENDPOINT_DESCRIPTOR *)pD)->bEndpointAddress & 0x8F;
              m = (n & 0x80) ? ((1 << 16) << (n & 0x0F)) : (1 << n);
              if (alt == SetupPacket.wValue.WB.L) {
                USB_EndPointMask |=  m;
                USB_EndPointHalt &= ~m;
                USB_ConfigEP((USB_ENDPOINT_DESCRIPTOR *)pD);
                USB_EnableEP(n);
                USB_ResetEP(n);
                msk |= m;
              }
              else if ((alt == old) && ((msk & m) == 0)) {
                USB_EndPointMask &= ~m;
                USB_EndPointHalt &= ~m;
                USB_DisableEP(n);
              }
            }
           break;
        }

			tmp = (uint32_t)pD;
			tmp += pD->bLength;
			pD = (USB_COMMON_DESCRIPTOR *)tmp;
      }
      break;
    default:
      return (FALSE);
  }

  return (set);
}






 

void USB_EndPoint0 (uint32_t event) {

  switch (event) {
    case 1:
      USB_SetupStage();
      USB_DirCtrlEP(SetupPacket.bmRequestType.BM.Dir);
      EP0Data.Count = SetupPacket.wLength;      
      switch (SetupPacket.bmRequestType.BM.Type) {

        case 0:
          switch (SetupPacket.bRequest) {
            case 0:
              if (!USB_ReqGetStatus()) {
                goto stall_i;
              }
              USB_DataInStage();
              break;

            case 1:
              if (!USB_ReqSetClrFeature(0)) {
                goto stall_i;
              }
              USB_StatusInStage();



              break;

            case 3:
              if (!USB_ReqSetClrFeature(1)) {
                goto stall_i;
              }
              USB_StatusInStage();



              break;

            case 5:
              if (!USB_ReqSetAddress()) {
                goto stall_i;
              }
              USB_StatusInStage();
              break;

            case 6:
              if (!USB_ReqGetDescriptor()) {
                goto stall_i;
              }
              USB_DataInStage();
              break;

            case 7:
   USB_SetStallEP(0x00);             
              EP0Data.Count = 0;
              break;

            case 8:
              if (!USB_ReqGetConfiguration()) {
                goto stall_i;
              }
              USB_DataInStage();
              break;

            case 9:
              if (!USB_ReqSetConfiguration()) {
                goto stall_i;
              }
              USB_StatusInStage();

              USB_Configure_Event();

              break;

            case 10:
              if (!USB_ReqGetInterface()) {
                goto stall_i;
              }
              USB_DataInStage();
              break;

            case 11:
              if (!USB_ReqSetInterface()) {
                goto stall_i;
              }
              USB_StatusInStage();



              break;

            default:
              goto stall_i;
          }
          break;   


        case 1:
          switch (SetupPacket.bmRequestType.BM.Recipient) {

            case 0:
              goto stall_i;                                               

            case 1:
#line 821 "Lib\\Usb_Drivers\\Src\\usbcore.c"
              if ((SetupPacket.wIndex.WB.L == 0)  ||        
                  (SetupPacket.wIndex.WB.L == 1)) {
                switch (SetupPacket.bRequest) {
                  case 0x00:
                    EP0Data.pData = EP0Buf;                               
                    goto setup_class_ok;
                  case 0x01:
                    if (CDC_GetEncapsulatedResponse()) {
                      EP0Data.pData = EP0Buf;                             
                      USB_DataInStage();                                  
                      goto setup_class_ok;
                    }
                    break;
                  case 0x02:
                    EP0Data.pData = EP0Buf;                               
                    goto setup_class_ok;
                  case 0x03:
                    if (CDC_GetCommFeature(SetupPacket.wValue.W)) {
                      EP0Data.pData = EP0Buf;                             
                      USB_DataInStage();                                  
                      goto setup_class_ok;
                    }
                    break;
                  case 0x04:
                    if (CDC_ClearCommFeature(SetupPacket.wValue.W)) {
                      USB_StatusInStage();                                
                      goto setup_class_ok;
                    }
                    break;
                  case 0x20:
                    EP0Data.pData = EP0Buf;                               
                    goto setup_class_ok;
                  case 0x21:
                    if (CDC_GetLineCoding()) {
                      EP0Data.pData = EP0Buf;                             
                      USB_DataInStage();                                  
                      goto setup_class_ok;
                    }
                    break;
                  case 0x22:
                    if (CDC_SetControlLineState(SetupPacket.wValue.W)) {
                      USB_StatusInStage();                                
                      goto setup_class_ok;
                    }
                    break;
                  case 0x23:
                    if (CDC_SendBreak(SetupPacket.wValue.W)) {
                      USB_StatusInStage();                                
                      goto setup_class_ok;
                    }
                    break;
                }
              }

              goto stall_i;                                               
               

            case 2:
#line 899 "Lib\\Usb_Drivers\\Src\\usbcore.c"
              goto stall_i;
               

            default:
              goto stall_i;
          }
setup_class_ok:                                                           
          break;   


#line 945 "Lib\\Usb_Drivers\\Src\\usbcore.c"

        default:
stall_i:  USB_SetStallEP(0x80);
          EP0Data.Count = 0;
          break;
      }
      break;   

    case 2:
      if (SetupPacket.bmRequestType.BM.Dir == 0) {
        if (EP0Data.Count) {                                              
          USB_DataOutStage();                                             
          if (EP0Data.Count == 0) {                                       
            switch (SetupPacket.bmRequestType.BM.Type) {

              case 0:
                goto stall_i;                                             


              case 1:
                switch (SetupPacket.bmRequestType.BM.Recipient) {
                  case 0:
                    goto stall_i;                                         

                  case 1:
#line 1000 "Lib\\Usb_Drivers\\Src\\usbcore.c"
                    if ((SetupPacket.wIndex.WB.L == 0)  ||  
                        (SetupPacket.wIndex.WB.L == 1)) {
                      switch (SetupPacket.bRequest) {
                        case 0x00:
                          if (CDC_SendEncapsulatedCommand()) {
                            USB_StatusInStage();                          
                            goto out_class_ok;
                          }
                          break;
                        case 0x02:
                          if (CDC_SetCommFeature(SetupPacket.wValue.W)) {
                            USB_StatusInStage();                          
                            goto out_class_ok;
                          }
                          break;
                        case 0x20:
                          if (CDC_SetLineCoding()) {
                            USB_StatusInStage();                          
                            goto out_class_ok;
                          }
                          break;
                      }
                    }

                    goto stall_i;
                     

                  case 2:
#line 1041 "Lib\\Usb_Drivers\\Src\\usbcore.c"
                    goto stall_i;
                     

                  default:
                    goto stall_i;
                }
out_class_ok:                                                             
                break;  


#line 1081 "Lib\\Usb_Drivers\\Src\\usbcore.c"

              default:
                goto stall_i;
            }
          }
        }
      } else {
        USB_StatusOutStage();                                             
      }
      break;   

    case 3 :
      if (SetupPacket.bmRequestType.BM.Dir == 1) {
        USB_DataInStage();                                                
      } else {
        if (USB_DeviceAddress & 0x80) {
          USB_DeviceAddress &= 0x7F;
          USB_SetAddress(USB_DeviceAddress);
        }
      }
      break;   

    case 6:
      USB_ClrStallEP(0x00);
      break;

    case 7:
      USB_ClrStallEP(0x80);
      break;

  }
}
