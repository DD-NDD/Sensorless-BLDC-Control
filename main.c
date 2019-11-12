/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"
#include "medium_event.h"
#include "slow_event.h"
#include "setup.h"
#include "user_interface.h"
/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/*********************************************************************/
extern void process_parameters(void);



extern void process_parameters(void);

int16_t main()
{
	unsigned char i;
	unsigned int j;

	run_state=INITIALIZING;
	// Set up peripherals - see setup.c for source code
	setup_ports();
	// Note interrupts must not be running during setup_motor_pwms()
	// as config registers are written to set firing polarity
	setup_motor_pwms();
	setup_adc();
	setup_qei();
	setup_timers();

	// Reset Power Module using delay due to screen initialization 
	// to ensure don't violate min pulse width of 2us at 12C671 PIC
	// in the power module
		
//	FAULT_RESET=TRUE;
	IFS2bits.FLTAIF=0;

	// Enable All types ofMath Error Traps
	INTCON1bits.OVATE=TRUE;
	INTCON1bits.OVBTE=TRUE;
	INTCON1bits.COVTE=TRUE;
	// Make the FAULT A Interrupt highest priority=7
	// Leave all others at the default of 4
	IPC10bits.FLTAIP=7;
	// Enable interrupts
	// NB T2 used for Output Compare is not enabled
	// here but only when commutation is required.
	// The same goes for T1 which is used as a guard
	// timer to catch a missed zero crossing event.
	IEC2bits.PWMIE = 1;
	IEC2bits.FLTAIE=1;
	IEC0bits.ADIE = 1;
	// Set ACQUIRE2 flag high to force capture of all three
	// phase voltage channels so offsets can be calibrated
	// out in process_parameters
	control_flags.ACQUIRE2=TRUE;
	for(j=0;j<10000;j++);
	control_flags.ACQUIRE2=FALSE;

	process_parameters();	

	// Main background loop starts here		
	while(1)
	{
		ClrWdt();
		medium_event_handler();								
		slow_event_handler();
	}	
}
