#include "defs.h"

#define 	PERIOD	FCY/FPWM
#define		DEAD_TIME	.000002          // in seconds
#define		DT_COUNT	DEAD_TIME*FCY    // in PWM counts

// PWM Fault protection options
// Load FLTACON below with one of thes options
#define CYCLE_BY_CYCLE_PROTECTION 0x0087;		// PWM1L/H, PWM2L/H, PWM3L/H have cycle by cycle current limiting enabled
#define FAULT_CAUSES_PWM_SHUTDOWN 0x0007;       // An over-current fault shuts down PWM1L/H, PWM2L/H, PWM3L/H   
#define NO_FAULT_PROTECTION       0x0000;       // No over-current protection

int initPWM(void)
{
	OVDCON = 0;
	
	PTPER = PERIOD;		// set PWM period
	PWMCON1 = 0x0777;	// Enable PWM channels

	DTCON1 = DT_COUNT;			// 2 us deadtime
	
	FLTACON = NO_FAULT_PROTECTION;	// PWM1L/H, PWM2L/H, PWM3L/H have no current limiting enabled -- it's taken care of in hardware
	
	PDC1 = lock1;			// Set all PWM duty cycles initially to zero
	PDC2 = lock1;			
	PDC3 = lock1;
	
	SEVTCMP = 0;		// no special event trigger
	
	PTCON = 0x8000;		// 
	
	IFS2bits.PWMIF = 0;	// enable the PWM interrupt
	IEC2bits.PWMIE = 1;
    return 1;
}