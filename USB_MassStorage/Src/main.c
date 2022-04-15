#include "LPC17xx.h"
#include "RTL.h"
#include "rl_usb.h"                   
#include "File_Config.h"
#include "stdio.h"                    
#include "ctype.h"                 
#include "string.h" 

char in_line[160]; 

enum {BACKSPACE = 0x08, 
      LF        = 0x0A, 
      CR        = 0x0D, 
      CNTLQ     = 0x11, 
      CNTLS     = 0x13, 
      ESC       = 0x1B, 
      DEL       = 0x7F
};

void DeInitUart1 (void)
{
	LPC_SC->PCONP &= ~(1<<4);
	LPC_PINCON->PINSEL4 &= ~(3 << 0);
	LPC_PINCON->PINSEL4 &= ~(3 << 2);
}

void InitUart1_9600 (void)
{
	DeInitUart1 ();

	LPC_SC->PCONP |= (1<<4);

	LPC_PINCON->PINSEL4 |= (2 << 0);                
	LPC_PINCON->PINSEL4 |= (2 << 2);         

	LPC_UART1->LCR    = 0x83;                        
	LPC_UART1->DLL    = 104;                          
	LPC_UART1->FDR    = 0x21;                        
	LPC_UART1->DLM    = 0;                          
	LPC_UART1->LCR    = 0x03;                         
}

int UART_putChar (uint8_t c) 
{
	while (!(LPC_UART1->LSR & 0x20));
	return (LPC_UART1->THR = c);
}	


void UART_putString (char *s) 
{
	while (*s != 0) 
	{
		UART_putChar(*(uint8_t *)s++);
	}
}

int SER_getChar (void) 
{ 
  while (!(LPC_UART1->LSR & 0x01));
  return (LPC_UART1->RBR);
}

int getkey (void) {

  return (SER_getChar());
}

char *get_entry (char *cp, char **pNext) {
  char *sp, lfn = 0, sep_ch = ' ';

  if (cp == NULL) {                           
    *pNext = cp;
    return (cp);
  }

  for ( ; *cp == ' ' || *cp == '\"'; cp++) {  
    if (*cp == '\"') { sep_ch = '\"'; lfn = 1; }
    *cp = 0;
  }
 
  for (sp = cp; *sp != CR && *sp != LF; sp++) {
    if ( lfn && *sp == '\"') break;
    if (!lfn && *sp == ' ' ) break;
  }

  for ( ; *sp == sep_ch || *sp == CR || *sp == LF; sp++) {
    *sp = 0;
    if ( lfn && *sp == sep_ch) { sp ++; break; }
  }

  *pNext = (*sp) ? sp : NULL;                
  return (cp);
}


void cmd_format (char *par) {
  char *label,*next,*opt;
  char arg[20];
  U32 retv;

  label = get_entry (par, &next);
  if (label == NULL) {
    label = "KEIL";
  }
  strcpy (arg, label);
  opt = get_entry (next, &next);
  if (opt != NULL) {
    if ((strcmp (opt, "/FAT32") == 0) ||(strcmp (opt, "/fat32") == 0)) {
      strcat (arg, "/FAT32");
    }
  }
  UART_putString ("\nFormat Flash Mass Storage Device? [Y/N]\n");
  retv = getkey();
  if (retv == 'y' || retv == 'Y') {
    /* Format the MSD with Label "KEIL". "*/
    if (fformat (arg) == 0) {
      UART_putString ("Mass Storage Device Formatted.\n");
      UART_putString ("Mass Storage Device Label is:  ");
          UART_putString (label);
          UART_putString ("\n\n");
    }
    else {
      UART_putString ("Formatting failed.\n");
    }
  }
}

void init_msd (void) {
  U32 retv;
  MMCFG mmcfg;
  char outBuf[26];
  
  while ((retv = finit ()) != 0) {            
    if (retv == 1) {
      UART_putString ("\nMSD Init Failed");
      UART_putString ("\nInsert Mass Storage Device and press key...\n");
      getkey ();
    }
    else {
      UART_putString ("\nMass Storage Device is Unformatted");
      strcpy (&in_line[0], "KEIL\r\n");
      cmd_format (&in_line[0]);
    }
  }

  mmc_read_config (&mmcfg);

  outBuf[0] = 0;
  sprintf (&outBuf[0], "BlockNr:0x%08X", mmcfg.blocknr);
  UART_putString (outBuf);
  UART_putString ("\n");
  sprintf (&outBuf[0], "RdLen:  0x%04X", mmcfg.read_blen);
  UART_putString (outBuf);
  UART_putString ("\n");
  sprintf(&outBuf[0], "WrLen:  0x%04X", mmcfg.write_blen);
  UART_putString (outBuf);
  UART_putString ("\n");
}



int main (void)
{
	SystemInit ();
	InitUart1_9600 ();
	UART_putString ("\n\nStartting ....\n\n");
	init_msd ();
	UART_putString ("\nMass Storage Device is Unformatted");
	cmd_format ("FLASHDISK\r\n");
}

