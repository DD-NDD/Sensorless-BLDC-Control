/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <xc.h>
// Config directive to set configuration bits
//_FOSC(CSW_FSCM_OFF & XT_PLL16);
//_FWDT(WDT_OFF);
//_FBORPOR(PBOR_ON & BORV_20 & PWRT_64 & MCLR_EN);
#pragma config FPR = XT_PLL16           // Primary Oscillator Mode (XT w/PLL 16x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV27          // Brown Out Voltage (2.7V)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)


