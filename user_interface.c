#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
/***********************************************************************
 *	Code Description
 *  
 *  This file contains all the code for implementing the user interface.
 *  The user interface functions are called from the slow event handler,
 *  which runs every 100 msec.
 *  Different screens are called depending on the operating state of the
 *  software.  The process_switches() function uses the debounced push
 *  buttons and determines the length of time the button has been 
 *  pressed for fast incrementing of the parameter. It also moves the 
 *  system between the various states according to the push buttons.
 *  The save_parameter() function reads an image of the Flash parameters
 *  into a RAM image, modifies the changed parameter, then writes the
 *  block back to Flash.
 *  The process_parameters() function does any scaling or units
 *  conversion that is required for the stored parameters.
 **********************************************************************/
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

/*********************************************************************/
 struct parameter_data 
{
    	unsigned int min;		//minimum allowed value 
    	unsigned int max;    //maximum allowed value

    	//unsigned int med;		//the medium rate parameter increment
		//unsigned int fast;	//the fast rate parameter increment
    	char *line1_msg;     //line 1 parameter screen message 
		char *quick_msg;	//abbriviation for message 
		//char *units_msg;		//units message used for RHS line 2
    }; 

 const struct parameter_data parameter_data [] =	{
// Min, Max, Line1 msg, Quick msg 
{0,1,				"DIRECTION","DD"},//0
{0,3,				"CONTROL MODE","CM"},//1
{0,32767,		"Lock Pos.1  Time","LP1"},//2
{0,32767,		"Lock Pos.2  Time","LP2"},//3
{0,100,			"Lock Pos.1 Dem. ","LP1D"},//4
{0,100,			"Lock Pos.2 Dem. ","LP2D"},//5
{1,9999,		"Ramp Start Speed","RSS"},//6
{1,9999,		"Ramp End Speed","RES"},//7
{0,100,				"Ramp Start Dem. ","RSD"},//8
{0,100,				"Ramp End Dem. ","RED"},//9
{1,32767,		"Ramp Duration   ","RD"},//10
{1,32767,		"Phase Adv. Enable Spd","PAES"},//11
{1,32767,		"Phase Adv. Slope","PAS"},//12
{1,32767,		"Stall Time Limit","STL"},//13
{1,32767,		"Over Speed Limit","OSL"},//14
{1,32767,		"Over Volts Limit","OVL"},//15
{1,32767,		"Over Current Lim","OCL"},//16
{0,32767,		"Current P Gain  ","CKP"},//17
{0,32767,		"Current I Gain  ","CKI"},//18
{0,32767,		"Current D Gain  ","CKD"},//19
{0,9999,		"Speed P Gain    ","SKP"},//20
{0,9999,		"Speed I Gain    ","SKI"},//21
{0,32767,		"Voltage Demand  ","VD"},//22
{0,32767,		"Volts P Gain    ","VKP"},//23
{0,32767,		"Volts I Gain    ","VKI"},//24
{2,48,				"No. Motor Poles ","P"},//25
{1,3670,		"Current Scale X ","CSX"},//26
{1,6550,		"Current Scale / ","CSD"},//27
{1,3670,		"Volts Scale X   ","VSX"},//28
{1,6550,		"Volts Scale /   ","VSD"},//29
{1,100,			"Tolerance Check ","TC"},//30
{0,1,				"Auto Re-acquire ","ARA"},//31
{0,255,				"Blanking Count  ","BC"},//32
{0,255,				"Zero X Level Thd","ZXL"},//33
{1,1023,			"Acquire Threshld","AT"},//34
{0,255,				"Acquire Level Td","AL"},//35
{0,32767,		"Rotation Timeout","RT"},//36
{1,100,			"Pot / for Duty  ","PDD"},//37
{1,100,			"Pot / for Currnt","PDC"},//38
{1,64,				"Pot X for Speed ","PXS"},//39
{0,1,				"Starting Control","SC"},//40
{0,100,			"Windmilling Dem.","WD"},//41
{1,32767,		"Braking Ramp T  ","BRT"},//42
{0,1,				"Acquire Method  ","AM"},//43
{1,9999,		"ZeroX Enable Spd","ZXES"},//44
}; 


volatile unsigned int user_parameters_RAM[64]=
{	
	//BACKWARDS,
	FORWARDS,
	OPEN_VOLTS,	// Speed Control Mode - See defs.h 
	30,				// First Lock Position Time in units of medium event (10ms) 
	30,				// Second Lock Position Time in units of medium event (10ms) 
	30,				// % Demand Used For Lock Position 1
	30,				// % Demand Used For Lock Position 2			
	100,			// Starting Speed for Ramp / RPM 
	1000,			// Finish Speed for Ramp / RPM	 toc do ket thuc
	58,				// % Demand Used At Start of Ramp
	58,				// % Demand Used At End of Ramp	
	3000,				// Duration of starting ramp in units of medium event (10ms) 30
	500,			// Phase Advance Start Speed in RPM (1500 default) 
	25,				// Phase Advance Slope in 1/1000th elec. degree / RPM	
	100,			// Stall Time Limit in units of medium event (10ms) 
	1000,			// Over Speed Trip in RPM (3500 default)	
	500,			//	Over Voltage Trip in 1/10ths V
	100,			// Over Current Limit in 1/10ths A 
	900,			// Current Loop P Gain 
	100,			// Current Loop I Gain 
	0,				// Current Loop D Gain 
	6000,			// Speed Loop P Gain 
	100,				// Speed Loop I Gain 
	490,			// Voltage Demand for Brake Chopper in 1/10ths V 
	10000,			// Voltage Loop P Gain 
	5000,			// Voltage Loop I Gain 
	10,				// Number of Motor Poles 
	100,			// Current Scaling X - see below 100
	539,			// Current Scaling / - see below 539
	240,			// Voltage Scaling X - see below 240
	1024,			// Voltage Scaling / - see below 1024
	90,				// % Tolerance used for Lost Check - see below
	1,				// Auto-reaquire if lost ON/OFF
	3,				// Blanking Length used for zero X in no of ADC samples
	2,				// No. of samples required > or < VDC/2 before zero X checked for
	12,  			// ADC value used for rising edge detect for acquisition routine
	6,				// No of samples of VPH before rising edge detect done in acquisition
	5,				// Rotation Timeout in units of medium event (10ms) 5
	1,				// Divisor used to scale pot ADC reading to Duty Cycle
	8,				// Divisor used to scale pot ADC reading to I Dem in ADC
	3,				// X Used to Scale pot ADC reading to wdemand in rpm
	1,				// 1=Use Voltage Control for Starting, 0=Use Current Control
	20,				// % Demand Used For Windmilling Braking
	1,				// Duration of braking ramp in units of medium event (10ms)
	0,				// Use Method 1 (=0) or Method 2 (=1) for acquisition of position
	400};			// Speed at Which ZeroX detection enabled when using method1 acqusition

// Current and Voltage Scaling X and / Parameters
// These are used to scale ADC readings into Amps or Volts as follows: 
// If you get 12.87 A/V for ibus or 12.87 V/V for vdc then
// X = 100 and / = 1287 is one possible combination

// For LV Power Module Use The Following values: 
// VX=100 V/=1305 i.e scaling is 13.05 V/V		 
// If LK11&12 Open 	IX=100 I/=539 i.e scaling is 5.39A/V 
// If LK11&12 Closed	IX=10  I/=119 i.e.scaling is 11.9A/V 

// For HV Power Module Use The Following values: 
// VX=10  V/910 i.e. scaling is 91.0 V/V 
// If LK11&12 Open	IX=100 I/=108 i.e scaling is 1.08A/V 
// if LK11&12 Closed IX=100 I/=239 i.e scaling is 2.39A/V 

// Tolerance for Lost Check Parameter 
// Every 60 electrical degrees a new zero crossing event should
// be detected. In order to determine if the system is lost, 
// a check is carried out comparing the time elapsed since the 
// last zero crossing and the one before that. 
// If the tolerance parameter is set to 25% then up to 25%
// variation between the two times is considered to be acceptable.
// If the system fails the check, then the system is lost.
// Some natural variation in the times is to be expected even at
// a constant speed due to timer and ADC resolution as well as 
// motor asymmetry. Speed changes will also result in variation.
// It is suggested that the parameter is not set to less than 25%
// as otherwise the system may be determined to lost just due to
// normal variations.
/*********************************************************************/
struct interface_flags {
				unsigned START_BUTTON : 1;
				unsigned UNUSED : 15;
};

struct interface_flags interface_flags;




void process_switches(void)
{
	static unsigned char initializing_timer=20;
	static unsigned char previous_valid_switch_states=0xFF;
	static unsigned char key_hold_timer=0;
	static unsigned int param_increment=1;

	if (initializing_timer==1)	run_state=STANDBY;
	if (initializing_timer) initializing_timer--;



	if (((previous_valid_switch_states & 0x08)==FALSE) && (valid_switch_states & 0x08))
		interface_flags.START_BUTTON=TRUE;
	else
		interface_flags.START_BUTTON=FALSE;

	previous_valid_switch_states=valid_switch_states;


	switch(run_state)
	{
		case INITIALIZING: break;

		case STANDBY:	if (interface_flags.START_BUTTON)
							{
								DISABLE_INTERRUPTS;
                                printf(" STANDBY\r\n");
								control_flags2.ROTATION_CHECK=TRUE;
								control_flags2.RETRY_FLAG=FALSE;
								control_flags.LOCK1=FALSE;
								control_flags.LOCK2=FALSE;
								control_flags.RAMP=FALSE;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								control_flags.ACQUIRE1=FALSE;
								control_flags.DIR=user_parameters_RAM[0];
								ENABLE_INTERRUPTS;
								run_state=STARTING;
								fault_count = 0;
							}
							
							
							break;

		case STARTING:	if (interface_flags.START_BUTTON)
							{
								DISABLE_FIRING;
                                printf("STARTTING\r\n");
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								run_state=STANDBY;
							}
							break;

		case RUNNING: 	if (interface_flags.START_BUTTON)
							{
								DISABLE_FIRING;
                                printf("RUNNNING\r\n");
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								run_state=STANDBY;
							}
							
							break;
		case FAULT:		if (interface_flags.START_BUTTON)
							{
								trip_state=NO_TRIP;
								DISABLE_INTERRUPTS;
                                printf("FAULT\r\n");
								control_flags.LOCK1=FALSE;
								control_flags.LOCK2=FALSE;
								control_flags.RAMP=FALSE;
								control_flags.SENSORLESS=FALSE;
								control_flags.ACQUIRE1=FALSE;
								control_flags.ACQUIRE2=FALSE;
								IEC0bits.T1IE=FALSE;
								IEC0bits.T2IE=FALSE;
								ENABLE_INTERRUPTS;
								period_measurement=1000;
								//FAULT_RESET=TRUE;
								//FAULT_RESET=FALSE;
								run_state=STANDBY;
							}
							
							break;
		default:			break;
	}

	return;
}

// This function does a simple switch debounce by not
// updating the global variable valid_switch_states
// unless all 4 push buttons have been in the same
// state for 3 calls of the function
void debounce_switches(void)
{
	static unsigned char oldest_switch_states=0;
	static unsigned char previous_switch_states=0;
	unsigned char switch_states;

	// The four push buttons are on PORTG6-9 but have pull
	// up resistors making a logic 0 equal to a button press
	// So we complement and shift them down to be aligned to 
	//	the bottom which will also effectively mask off all other bits.

	if (!PORTDbits.RD0)
		switch_states = 0x8;
	else switch_states = 0x0;
	if (switch_states!=previous_switch_states)
	{
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
		return;
	}

	if (previous_switch_states != oldest_switch_states)
	{
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
	}
	else
	{
		valid_switch_states=switch_states;
		oldest_switch_states=previous_switch_states;
		previous_switch_states=switch_states;
	}
	return;
}



	






