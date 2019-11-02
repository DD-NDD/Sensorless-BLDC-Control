/*
 * File:   main.c
 * Author: Duy Ngo
 *
 * Created on October 29, 2019, 7:44 PM
 */

#include <xc.h>
#include "confibit.h"
#include "defs.h"
#include <uart.h>
#include "debug.h"
#include "stdio.h"
#include "init_PWM.h"
#include "Motor_isr.h"
#include "slow_event.h"
#include "init_ADC.h"
#include "TuningInterface.h"
#include "medium_event.h"
#include "dsp.h"
#include "IIR_Filter.h"
#include "BEMF_filter.h"
//------------------------------------------------------------------------------
//UART
/* Received data is stored in array Buf  */
char Buf[80];

char * Receivedddata = Buf;
void __attribute__((__interrupt__,no_auto_psv)) _U1TXInterrupt(void)

{  

   IFS0bits.U1TXIF = 0;

} 

/* This is UART1 receive ISR */
void __attribute__((__interrupt__,no_auto_psv)) _U1RXInterrupt(void)
{

    IFS0bits.U1RXIF = 0;
/* Read the receive buffer till atleast one or more character can be read */ 
    while( DataRdyUART1())
    {

        ( *( Receivedddata)++) = ReadUART1();

    } 
} 
//------------------------------------------------------------------------------
void DelayNmSec(unsigned int N);
//--Main------------------------------------------------------------------------
int main(void) 
{
    LATE = 0x0000;
    TRISE = 0xFFC0;
  	LATC = 0;
  	LATD = 0;
 	
 	// Setup Timer1 for 1:8 prescale.  Timer1 is used to measure
 	// the period between zero crossings.  Enable Timer 1.
 	T1CON = 0x8010;
 	
 	// Timer2 and Timer 3 will be the commutation timers.  They
	//  have the same prescaler (1:8) as Timer 1.  Turn on Timer 2
	//  but not Timer 3  
 	T2CON = 0x0010;			// 1:8 prescaler
	TMR2 = 0;				// clear Timer 2
	IFS0bits.T2IF = 0;		// Disable the Timer 2 interrupt
	IEC0bits.T2IE = 0;
	T2CONbits.TON = 1; 		// Turn on Timer 2

	T3CON = 0x0010;			// 1:8 prescaler
	TMR3 = 0;				// clear Timer 3
	IFS0bits.T3IF = 0;		// Disable the Timer 3 interrupt
	IEC0bits.T3IE = 0;

	T4CON = 0x8000;    			// turn on timer 4
    if(init_UART() == 1)
    {
        printf("UART INIT OK\r\n");
    }			
    if(initPWM() == 1)
    {
        printf("MCPWM INIT OK\r\n");
    }
    Init_ADC();
    IIRTransposeFilterInit( &BEMF_phaseA_Filter );
	IIRTransposeFilterInit( &BEMF_phaseB_Filter );
	IIRTransposeFilterInit( &BEMF_phaseC_Filter );
    RunMode = MOTOR_OFF;
    while(1)
    {
		if(ControlFlags.MediumEventFlag)
			MediumEvent();
		if(ControlFlags.SlowEventFlag)
			SlowEvent();
        //printf("pot  = %lf\r\n",pot);
    }
    return 0;
}
void DelayNmSec(unsigned int N)
{
    unsigned int j;
    while(N--)
 	for(j=0;j < MILLISEC;j++);
}