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
#include "uart.h"
#include "stdio.h"
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
	// Thiet lap cac ngoai vi
	setup_ports();
	// Luu y ngat khong duoc chay trong setup_motor_pwms();
	setup_motor_pwms();
	setup_adc();
	//setup_qei();
	setup_timers();
    UART1_Initialize();
//	FAULT_RESET=TRUE;
	IFS2bits.FLTAIF=0;

	// Kich hoat cac bay loi
	INTCON1bits.OVATE=TRUE;
	INTCON1bits.OVBTE=TRUE;
	INTCON1bits.COVTE=TRUE;
	// Dat FAULT A uu tien cao nhat priority=7
	// Tat ca con lai dat mac dinh la 4
	IPC10bits.FLTAIP=7;
	// Kich hoat ngat
	// NB T2 duoc su dung cho so sanh dau ra khong duoc bat
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
    printf(" SETUP OKE\r\n");
    unsigned int temprpm;
	while(1)
	{
		ClrWdt();
		medium_event_handler();								
		slow_event_handler();
            
	}	
}
