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


// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT_PLL16           // Primary Oscillator Mode (XT w/PLL 16x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV45          // Brown Out Voltage (4.5V)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define FCY  29491200			//FRC w/PLL x16
#define MILLISEC FCY/29491		//1 mSec delay constant
#define FPWM 20000
#define TYC (1.0/FCY)
#define dDeadTimeSec 0.000002
#define dDeadTime (int)(dDeadTimeSec*TYC) // Dead time in dTcys 

#define BAUDRATE 9600
#define BRG     (FCY/(16*BAUDRATE))-1
#include <xc.h>

#ifdef	__cplusplus
}
#endif

#endif	/* DEFS_H */

