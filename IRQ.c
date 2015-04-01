/******************************************************************************/
/* IRQ.C: IRQ Handler                                                         */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools.                    */
/* Copyright (c) 2005-2006 Keil Software. All rights reserved.                */
/* This software may only be used under the terms of a valid, current,        */
/* end user licence from KEIL for a compatible version of KEIL software       */
/* development tools. Nothing else gives you the right to use this software.  */
/******************************************************************************/

#include <91x_lib.h>
#include <RTL.h>


short Potentiometer,SlideSensor;
os_mbx_declare (MailBox, 16);
U32 mpool[16*(2*sizeof(U32))/4 + 3]; 

unsigned char AD_in_progress;           /* AD conversion in progress flag     */

__irq void ADC_IRQ_Handler (void) {     /* AD converter interrupt routine     */
	U32 *mptr;
	
	Potentiometer = ADC->DR0 & 0x03FF;    /* AD value for global usage (10 bit) */	
	SlideSensor = ADC->DR1 & 0x03FF;          /* AD value for global usage (10 bit) */	//dr4.1

	os_mbx_init(MailBox, sizeof (MailBox));
	mptr = _alloc_box (mpool);
	mptr[0] = Potentiometer;
	mptr[1] = SlideSensor;
	os_mbx_send (MailBox, mptr, 0xffff);
	
  ADC->CR &= 0xFFFE;                    /* Clear STR bit (Start Conversion)   */
  ADC->CR &= 0x7FFF;                    /* Clear End of Conversion flag       */

  VIC0->VAR = 0;                        /* Acknowledge Interrupt              */  
  VIC1->VAR = 0;

  AD_in_progress = 0;                   /* Clear flag, as AD conv finished    */
}
