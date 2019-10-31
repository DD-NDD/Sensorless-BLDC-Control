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
//------------------------------------------------------------------------------
//UART
/* Received data is stored in array Buf  */
char Buf[80];

char * Receivedddata = Buf;

/* This is UART1 transmit ISR */
void Init_UART(void)
{
    
    //CloseUART1();
    //OpenUART1(0x8400, 0x0400, BRGVAL);
}
void __attribute__((__interrupt__,auto_psv)) _U1TXInterrupt(void)

{  

   IFS0bits.U1TXIF = 0;

} 

/* This is UART1 receive ISR */

void __attribute__((__interrupt__,auto_psv)) _U1RXInterrupt(void)
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
    
//    U1BRG  = BRGVAL;
//	U1MODE = 0x8400; /* Reset UART to 8-n-1, alt pins, and enable */
//	U1STA  = 0x0400; /* Reset status register and enable TX */
    if(init_UART() == 1)
    {
        printf("BLDC SENSORLESS CONTROLER\r\n");
        printf("ngodinhduy011@gmail.com\r\n");
        printf("UART INIT OK\r\n");
    }
    T2CON = 0x0010;			// 1:8 prescaler
	TMR2 = 0;				// clear Timer 2
	IFS0bits.T2IF = 0;		// Disable the Timer 2 interrupt
	IEC0bits.T2IE = 1;
	T2CONbits.TON = 1; 		// Turn on Timer 2
    printf("Timer 2 INIT OK\r\n");
    if(initPWM() == 1)
    {
        printf("MCPWM INIT OK\r\n");
    }
    
    //PWMCON1 = 0x0777;
    RunMode = SENSORLESS_RUNNING;
    while(1)
    {
        //SlowEvent();
    }
    return 0;
}
void DelayNmSec(unsigned int N)
{
    unsigned int j;
    while(N--)
 	for(j=0;j < MILLISEC;j++);
}