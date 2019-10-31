/*
 * File:   main.c
 * Author: Duy Ngo
 *
 * Created on October 29, 2019, 7:44 PM
 */

#include "defs.h"
#include "uart.h"
void DelayNmSec(unsigned int N);
int main(void) {
    LATE = 0x0000;
    TRISE = 0xFFC0;
    TRISCbits.TRISC13 = 0; /* Setup for UART1 TX */
	TRISCbits.TRISC14 = 1; /* Setup for UART1 RX */
    while(1)
    {
    }
    return 0;
}
void DelayNmSec(unsigned int N)
{
    unsigned int j;
    while(N--)
 	for(j=0;j < MILLISEC;j++);
}