#line 1 "Lib\\Usb_Drivers\\Src\\usbdesc.c"




















 
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



 






 

 
#line 23 "Lib\\Usb_Drivers\\Src\\usbdesc.c"
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






#line 24 "Lib\\Usb_Drivers\\Src\\usbdesc.c"
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



#line 25 "Lib\\Usb_Drivers\\Src\\usbdesc.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"




















 






















































 

#line 84 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





































 

#line 134 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"























 

#line 172 ".\\Lib\\Usb_Drivers\\Inc\\usbcfg.h"





 



#line 26 "Lib\\Usb_Drivers\\Src\\usbdesc.c"
#line 1 ".\\Lib\\Usb_Drivers\\Inc\\usbdesc.h"
















 












extern const uint8_t USB_DeviceDescriptor[];
extern const uint8_t USB_ConfigDescriptor[];
extern const uint8_t USB_StringDescriptor[];


#line 27 "Lib\\Usb_Drivers\\Src\\usbdesc.c"


 
const uint8_t USB_DeviceDescriptor[] = {
  (sizeof(USB_DEVICE_DESCRIPTOR)),               
  1,         
  (0x0200 & 0xFF),((0x0200 >> 8) & 0xFF),              
  0x02,    
  0x00,                               
  0x00,                               
  8,                    
  (0x1FC9 & 0xFF),((0x1FC9 >> 8) & 0xFF),                      
  (0x2002 & 0xFF),((0x2002 >> 8) & 0xFF),                      
  (0x0100 & 0xFF),((0x0100 >> 8) & 0xFF),             
  0x01,                               
  0x02,                               
  0x03,                               
  0x01                                
};

 
 
const uint8_t USB_ConfigDescriptor[] = {
 
  (sizeof(USB_CONFIGURATION_DESCRIPTOR)),        
  2,  
  (1*(sizeof(USB_CONFIGURATION_DESCRIPTOR)) + 1*(sizeof(USB_INTERFACE_DESCRIPTOR)) + 0x0013 + 1*(sizeof(USB_ENDPOINT_DESCRIPTOR)) + 1*(sizeof(USB_INTERFACE_DESCRIPTOR)) + 2*(sizeof(USB_ENDPOINT_DESCRIPTOR)) & 0xFF),((1*(sizeof(USB_CONFIGURATION_DESCRIPTOR)) + 1*(sizeof(USB_INTERFACE_DESCRIPTOR)) + 0x0013 + 1*(sizeof(USB_ENDPOINT_DESCRIPTOR)) + 1*(sizeof(USB_INTERFACE_DESCRIPTOR)) + 2*(sizeof(USB_ENDPOINT_DESCRIPTOR)) >> 8) & 0xFF),
#line 61 "Lib\\Usb_Drivers\\Src\\usbdesc.c"
  0x02,                               
  0x01,                               
  0x00,                               
  0x80          
 ,
  ((100)/2),           
 
  (sizeof(USB_INTERFACE_DESCRIPTOR)),            
  4,      
  0,                    
  0x00,                               
  0x01,                               
  0x02,  
  0x02,         
  0x00,                               
  0x5E,                               
 
  0x05,                               
  0x24,                   
  0x00,                         
  (0x0110 & 0xFF),((0x0110 >> 8) & 0xFF),          
 
  0x05,                               
  0x24,                   
  0x01,                
  0x01,                               
  0x01,                               
 
  0x04,                               
  0x24,                   
  0x02,    
  0x02,                               
 
  0x05,                               
  0x24,                   
  0x06,                          
  0,                    
  1,                    
              
  (sizeof(USB_ENDPOINT_DESCRIPTOR)),             
  5,       
  ((1) | 0x80),                 
  0x03,        
  (0x0010 & 0xFF),((0x0010 >> 8) & 0xFF),                      
  0x02,                       
 
  (sizeof(USB_INTERFACE_DESCRIPTOR)),            
  4,      
  1,                    
  0x00,                               
  0x02,                               
  0x0A,           
  0x00,                               
  0x00,                               
  0x5E,                               
 
  (sizeof(USB_ENDPOINT_DESCRIPTOR)),             
  5,       
  ((2) | 0x00),                
  0x02,             
  (64 & 0xFF),((64 >> 8) & 0xFF),             
  0x00,                               
 
  (sizeof(USB_ENDPOINT_DESCRIPTOR)),             
  5,       
  ((2) | 0x80),                 
  0x02,             
  (64 & 0xFF),((64 >> 8) & 0xFF),             
  0x00,                               
 
  0                                   
};




 
const uint8_t USB_StringDescriptor[] = {
 
  0x04,                               
  3,         
  (0x0409 & 0xFF),((0x0409 >> 8) & 0xFF),       
 
  (13*2 + 2),                         
  3,         
  'N',0,
  'X',0,
  'P',0,
  ' ',0,
  'S',0,
  'E',0,
  'M',0,
  'I',0,
  'C',0,
  'O',0,
  'N',0,
  'D',0,
  ' ',0,
 
  (17*2 + 2),                         
  3,         
  'N',0,
  'X',0,
  'P',0,
  ' ',0,
  'L',0,
  'P',0,
  'C',0,
  '1',0,
  '7',0,
  'x',0,
  'x',0,
  ' ',0,
  'V',0,
  'C',0,
  'O',0,
  'M',0,
  ' ',0,
 
  (12*2 + 2),                         
  3,         
  'D',0,
  'E',0,
  'M',0,
  'O',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
  '0',0,
 
  ( 4*2 + 2),                         
  3,         
  'V',0,
  'C',0,
  'O',0,
  'M',0,
};
