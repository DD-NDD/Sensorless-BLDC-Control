/* 
 * File:   defs.h
 * Author: Duy Ngo
 *
 * Created on October 31, 2019, 10:53 AM
 */

#ifndef DEFS_H
#define	DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif
#include "p30F4011.h"  
#define XTFREQ          7372800         //On-board Crystal frequency
#define PLLMODE         16               //On-chip PLL setting
#define FCY             XTFREQ*PLLMODE/4        //Instruction Cycle Frequency
#define MILLISEC        FCY/29491
#define BAUDRATE         115200       
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 
// pwm frequency
#define		FPWM	20000
#define	FULL_DUTY (2*FCY/FPWM)
    
#define lock1  (unsigned int)((unsigned long)50)*FULL_DUTY/100;
// Timer 1,2, and 3 prescaler
#define TMR1_PRESCALER 8UL

// Specify the maximum number of degrees of phase advance
#define MAX_PHASE_ADVANCE 30

// Filter phase delay is the delay in the filtered signal compared to the actual signal
// FILTER_PHASE_DELAY is equal to the Group Delay (as a function of filtered signal frequency)
//  multiplied by Fcy divided by the Timer 1 prescaler.
// Assuming a maximum speed of 100000 electrical revs per minute the frequency we are concerned 
//  with is 1666 revs per second.  This is the frequency of the filtered signal this speed
// Refering to the group delay plot in the dsPIC Filter design tool the delay at 1333 Hz
//  is approx 90 us.  At a frequency of 0 the phase delay is 85us.  We can ignore the 5us
//  and just account for the worst case scenerio
// FILTER_PHASE_DELAY = 90us*FCY/TMR1_PRESCALER
#define INVERSE_GROUP_DELAY 11111 // 1/(90us)  
#define FILTER_PHASE_DELAY FCY/TMR1_PRESCALER/INVERSE_GROUP_DELAY

// time through ADC interrupt divided by prescaler
#define PROCESSING_DELAY 410/TMR1_PRESCALER

// time through ADC interrupt divided by prescaler for high speed mode
#define PROCESSING_DELAY_HS 270/TMR1_PRESCALER

//ADC Configuration Values for low and high speed modes
#define ADCON2_LOW_SPEED 0x0410     // channel scan for CH0, MUX A only, 5 conversions per interrupt, Vrefs are AVdd and AVss
#define ADCON2_HIGH_SPEED 0x0408    // channel scan for CH0, MUX A only, 3 conversions per interrupt, Vrefs are AVdd and AVss
#define ADCSSL_LOW_SPEED 0x01C5     // scan AN0, AN2, AN6, AN7 and AN8
#define ADCSSL_HIGH_SPEED 0x0045    // scan AN0, AN2, AN6
// Define the ADC result buffers for the analog pins
#define POTBUF	ADCBUF1		//AN2
#define VBUSBUF	ADCBUF0		//AN0
#define VPHABUF	ADCBUF2		//AN3
#define VPHBBUF	ADCBUF3		//AN4
#define VPHCBUF ADCBUF4		//AN5

// PID Speed Control Loop enable.  
// Uncomment this define statement if the user desires that the PID Speed Control Loop be enabled
#define PID_SPEED_LOOP
#ifdef	__cplusplus
}
#endif

#endif	/* DEFS_H */

