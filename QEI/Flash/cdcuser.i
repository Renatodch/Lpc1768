#line 1 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
















 

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



 






 

 
#line 20 "Lib\\Usb_Drivers\\Src\\cdcuser.c"

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






#line 22 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
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


#line 23 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"




















 






















































 

#line 84 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





































 

#line 134 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"























 

#line 172 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





 



#line 24 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
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



#line 25 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
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



#line 26 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
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



#line 27 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\uart.h"














 





 
extern void  ser_OpenPort  (char portNum);
extern void  ser_ClosePort (char portNum);
extern void  ser_InitPort0  (unsigned long baudrate, unsigned int databits, unsigned int parity, unsigned int stopbits);
extern void  ser_InitPort1  (unsigned long baudrate, unsigned int databits, unsigned int parity, unsigned int stopbits);
extern void  ser_AvailChar (int *availChar);
extern int   ser_Write     (char portNum, const char *buffer, int *length);
extern int   ser_Read      (char *buffer, const int *length);
extern void  ser_LineState (unsigned short *lineState);

#line 28 "Lib\\Usb_Drivers\\Src\\cdcuser.c"


unsigned char BulkBufIn  [64];            
unsigned char BulkBufOut [64];            
unsigned char NotificationBuf [10];

CDC_LINE_CODING CDC_LineCoding  = {9600, 0, 0, 8};
unsigned short  CDC_SerialState = 0x0000;
unsigned short  CDC_DepInEmpty  = 1;                   




 
 

                                                       


 
#line 54 "Lib\\Usb_Drivers\\Src\\cdcuser.c"



typedef struct __CDC_BUF_T {
  unsigned char data[(64)];
  unsigned int wrIdx;
  unsigned int rdIdx;
} CDC_BUF_T;

volatile CDC_BUF_T  CDC_OutBuf;                                 



 
int CDC_RdOutBuf (char *buffer, const int *length) {
  int bytesToRead, bytesRead;

   
  bytesToRead = *length;
  bytesToRead = (bytesToRead < (*length)) ? bytesToRead : (*length);
  bytesRead = bytesToRead;


  

  while (bytesToRead--) {
    *buffer++ = (CDC_OutBuf . data[((64)-1ul) & CDC_OutBuf . rdIdx++]);
  }
  return (bytesRead);
}



 
int CDC_WrOutBuf (const char *buffer, int *length) {
  int bytesToWrite, bytesWritten;

  
  bytesToWrite = *length;
  bytesWritten = bytesToWrite;


  

  while (bytesToWrite) {
      (CDC_OutBuf . data[((64)-1ul) & CDC_OutBuf . wrIdx++] = (*buffer++));           
      bytesToWrite--;
  }

  return (bytesWritten);
}



 
int CDC_OutBufAvailChar (int *availChar) {

  *availChar = (((64)-1ul) & (CDC_OutBuf . wrIdx - CDC_OutBuf . rdIdx));

  return (0);
}
 







 
void CDC_Init (char portNum ) {

  if ( portNum == 0 )
  {
	ser_OpenPort (0);
	ser_InitPort0 (CDC_LineCoding.dwDTERate,
                CDC_LineCoding.bDataBits,
                CDC_LineCoding.bParityType,
                CDC_LineCoding.bCharFormat);
  }
  else
  {
	ser_OpenPort (1);
	ser_InitPort1 (CDC_LineCoding.dwDTERate,
                CDC_LineCoding.bDataBits,
                CDC_LineCoding.bParityType,
                CDC_LineCoding.bCharFormat);
  }
  CDC_DepInEmpty  = 1;
  CDC_SerialState = CDC_GetSerialState();

  (CDC_OutBuf . rdIdx = CDC_OutBuf . wrIdx = 0);
}







 
uint32_t CDC_SendEncapsulatedCommand (void) {

  return (TRUE);
}







 
uint32_t CDC_GetEncapsulatedResponse (void) {

   
  return (TRUE);
}







 
uint32_t CDC_SetCommFeature (unsigned short wFeatureSelector) {

   
  return (TRUE);
}







 
uint32_t CDC_GetCommFeature (unsigned short wFeatureSelector) {

   
  return (TRUE);
}







 
uint32_t CDC_ClearCommFeature (unsigned short wFeatureSelector) {

   
  return (TRUE);
}







 
uint32_t CDC_SetLineCoding (void) {

  CDC_LineCoding.dwDTERate   =   (EP0Buf[0] <<  0)
                               | (EP0Buf[1] <<  8)
                               | (EP0Buf[2] << 16)
                               | (EP0Buf[3] << 24);
  CDC_LineCoding.bCharFormat =  EP0Buf[4];
  CDC_LineCoding.bParityType =  EP0Buf[5];
  CDC_LineCoding.bDataBits   =  EP0Buf[6];


  ser_ClosePort(1);
  ser_OpenPort (1);
  ser_InitPort1 (CDC_LineCoding.dwDTERate,
                CDC_LineCoding.bDataBits,
                CDC_LineCoding.bParityType,
                CDC_LineCoding.bCharFormat);
#line 244 "Lib\\Usb_Drivers\\Src\\cdcuser.c"
  return (TRUE);
}







 
uint32_t CDC_GetLineCoding (void) {

  EP0Buf[0] = (CDC_LineCoding.dwDTERate >>  0) & 0xFF;
  EP0Buf[1] = (CDC_LineCoding.dwDTERate >>  8) & 0xFF;
  EP0Buf[2] = (CDC_LineCoding.dwDTERate >> 16) & 0xFF;
  EP0Buf[3] = (CDC_LineCoding.dwDTERate >> 24) & 0xFF;
  EP0Buf[4] =  CDC_LineCoding.bCharFormat;
  EP0Buf[5] =  CDC_LineCoding.bParityType;
  EP0Buf[6] =  CDC_LineCoding.bDataBits;

  return (TRUE);
}







 
uint32_t CDC_SetControlLineState (unsigned short wControlSignalBitmap) {

   
  return (TRUE);
}









 
uint32_t CDC_SendBreak (unsigned short wDurationOfBreak) {

   
  return (TRUE);
}






 
void CDC_BulkIn(void) {
  int numBytesRead, numBytesAvail;

  ser_AvailChar (&numBytesAvail);

  

  numBytesRead = ser_Read ((char *)&BulkBufIn[0], &numBytesAvail);

  
  if (numBytesRead > 0) {
	USB_WriteEP (0x82, &BulkBufIn[0], numBytesRead);
  }
  else {
    CDC_DepInEmpty = 1;
  }
}






 
void CDC_BulkOut(void) {
  int numBytesRead;

  
  numBytesRead = USB_ReadEP(0x02, &BulkBufOut[0]);

  

  
  CDC_WrOutBuf ((char *)&BulkBufOut[0], &numBytesRead);

}






 
unsigned short CDC_GetSerialState (void) {
  unsigned short temp;

  CDC_SerialState = 0;
  ser_LineState (&temp);

  if (temp & 0x8000)  CDC_SerialState |= (1 << 0);
  if (temp & 0x2000)  CDC_SerialState |= (1 << 1);
  if (temp & 0x0010)  CDC_SerialState |= (1 << 2);
  if (temp & 0x4000)  CDC_SerialState |= (1 << 3);
  if (temp & 0x0008)  CDC_SerialState |= (1 << 4);
  if (temp & 0x0004)  CDC_SerialState |= (1 << 5);
  if (temp & 0x0002)  CDC_SerialState |= (1 << 6);

  return (CDC_SerialState);
}




 
void CDC_NotificationIn (void) {

  NotificationBuf[0] = 0xA1;                           
  NotificationBuf[1] = 0x20;  
  NotificationBuf[2] = 0x00;                           
  NotificationBuf[3] = 0x00;
  NotificationBuf[4] = 0x00;                           
  NotificationBuf[5] = 0x00;
  NotificationBuf[6] = 0x02;                           
  NotificationBuf[7] = 0x00;
  NotificationBuf[8] = (CDC_SerialState >>  0) & 0xFF; 
  NotificationBuf[9] = (CDC_SerialState >>  8) & 0xFF;

  USB_WriteEP (0x81, &NotificationBuf[0], 10);   
}
