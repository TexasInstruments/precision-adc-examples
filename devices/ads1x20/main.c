/* --COPYRIGHT--,BSD
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * Main.c
 *
 */
/******************************************************************************
// ADS1220/MSP430x552x Demo - ADS1220 to MSP430 communication via SPI  
//                    
//
// Description: Use of the MSP430 USCI A0 peripheral for setting up and 
//    communicating to the ADS1220 24-bit ADC.
//    
//    
//                                                   
//                 MSP430x552x
//             ------------------                        
//         /|\|                  |                       
//          | |                  |                       
//          --|RST           P3.4|<-- MISO (DOUT)           
//            |                  |                                         
//            |              P3.3|--> MOSI (DIN)
//            |                  |  
//            |              P2.7|--> SCLK
//            |                  | 
//            |              P2.6|<-- INT (DRDY) 
//            |                  | 
//            |              P1.2|--> CS 
//
//   
//******************************************************************************/
/*/+-----------------------------------------------------------------------------+
//|  Source: main.c, v1.0                                           |
//------------------------------------------------------------------------------+*/   
#include <msp430f5528.h>
#include "ADS1220.h"
/* Function declarations */
void Init_StartUp(void);
void SPIinit(void);
/* Global variable */
int dFlag = 0;
/*----------------------------------------------------------------------------+
| Main Routine                                                                |
+----------------------------------------------------------------------------*/
VOID main(VOID)
{
    signed long tData;
	WDTCTL = WDTPW + WDTHOLD;	    // Stop watchdog timer
	Init_StartUp();                 // Initialize device
	ADS1220Init();					// Initializes the SPI port pins as well as control
    ADS1220Config();				// Set base configuration for ADS1x20 device
    while(1)
    {
        /* Add specifc command for reading and writing ADS1220 here */
		/* dFlag is set in the interrupt service routine when DRDY triggers end
		   of conversion */
		if (dflag)						/* check if new data is available */
		{
			tData = ADS1220ReadData();	/* get the data from the ADS1220 */
			dFlag=0;
		}
		/* other routines could be added here, such as change the mux setting */
    }  /* end of while(1) */
} /* end of main() */

/*----------------------------------------------------------------------------+
| System Initialization Routines                                              |
+----------------------------------------------------------------------------*/

/* Initializes the clocks.  Starts the DCO at USB_MCLK_FREQ (the CPU freq set with the Desc 
   Tool), using the REFO as the FLL reference.  Configures the high-freq crystal, but 
   doesn't start it yet.  Takes some special actions for F563x/663x.   */
VOID Init_Clock(VOID)
{
    if (USB_PLL_XT == 2)
    {
        /* Enable XT2 pins */
        P5SEL |= 0x0C;                      
        P5DIR |= BIT4+BIT5; 
        UCSCTL1 = DCORSEL_6;
        UCSCTL2 = FLLD_1+1;
        UCSCTL3 = SELREF__XT2CLK;
        UCSCTL6 = XT2DRIVE1;
        /* Start the FLL, which will drive MCLK (not the crystal)
          Init_FLL(USB_MCLK_FREQ/1000, USB_MCLK_FREQ/32768);  
          Init_FLL(USB_MCLK_FREQ/1000, USB_MCLK_FREQ/12000000); */
		UCSCTL4  = SELA_4 + SELS_4 + SELM_4;      /* ACLK=TX2  SMCLK=TX2   MCLK=TX2 */
		P2SEL |= BIT2;
		P2DIR |= BIT2;
		__bis_SR_register(GIE+OSCOFF);
    }
    else
    {
        /* Enable XT1 pins */
        P5SEL |= 0x10;                    
        /* Use the REFO oscillator as the FLL reference, and also for ACLK */
        UCSCTL3 = SELREF__REFOCLK;             
        UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK); 
        /* Start the FLL, which will drive MCLK (not the crystal) */
        Init_FLL(USB_MCLK_FREQ/1000, USB_MCLK_FREQ/32768); /* set FLL (DCOCLK) */
    }
}
/*----------------------------------------------------------------------------*/
VOID Init_Ports(VOID)
{
	/* Initialization of ports all unused pins as outputs with low-level */
	/* set all ports  to low on all pins */
	P1OUT 	= 	0x04;
	P1DIR 	=	0xFF;
	P2OUT	= 	0x00;
	P2DIR	|=	0x3F;
	P3OUT	= 	0x00;
	P3DIR	|=	0x07;
	P4OUT	=	0x00;
	P4DIR	|=	0xFF;
	P5OUT	=	0x00;
	P5DIR	|=	0x33;
	P6OUT	=	0x00;
	P6DIR	=	0xFF; 
}
/*----------------------------------------------------------------------------*/
/* Sets the USCI SPI peripheral to use A0 */
void SPIinit(void)
{
	UCA0CTL1 |= UCSWRST;						/* Hold peripheral in reset */
	UCA0CTL0 = UCMST + UCSYNC + UCMSB;			/* SPI master, synchronous */
	UCA0CTL1 = UCSSEL_2 + UCSWRST;				/* Use SMCLK for bit rate clock and keep in reset */
	UCA0BR0 = 12;								/* SMCLK/12 = SCLK (2MHz) */
	UCA0CTL1 &= ~UCSWRST; 						/* Release peripheral for use */
}
/*/----------------------------------------------------------------------------*/
VOID Init_StartUp(VOID)
{
    __disable_interrupt();               /* Disable global interrupts */
    Init_Ports();                        /* Init ports (do first ports because clocks do change ports) */
    SetVCore(3);                         /* USB core requires the VCore set to 1.8 volt, independ of CPU clock frequency */
    Init_Clock();
    SPIinit();
    __enable_interrupt();                /* enable global interrupts */
}
#pragma vector = UNMI_VECTOR
__interrupt VOID UNMI_ISR(VOID)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
    case SYSUNIV_NONE:
      __no_operation();
      break;
    case SYSUNIV_NMIIFG:
      __no_operation();
      break;
    case SYSUNIV_OFIFG:
      UCSCTL7 &= ~(DCOFFG+0+0+0); 			/* Clear OSC flaut Flags fault flags */
      SFRIFG1 &= ~OFIFG;                    /* Clear OFIFG fault flag */
      break;
    case SYSUNIV_ACCVIFG:
      __no_operation();
      break;
    case SYSUNIV_BUSIFG:
      /* If bus error occured - the cleaning of flag and re-initializing of USB is required. */
      SYSBERRIV = 0;            /* clear bus error flag */ 
    }
}
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{
	switch(__even_in_range(P2IV,16))
	{
		case 0: break;
		case 14:
				dFlag=1;
				break;
		default:break;	
	}	
}
/*----------------------------------------------------------------------------+
| End of source file                                                          |
+----------------------------------------------------------------------------*/
/*------------------------ Nothing Below This Line --------------------------*/
