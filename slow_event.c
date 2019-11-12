/**********************************************************************
 *                                                                     *
 *                        Software License Agreement                   *
 *                                                                     *
 *    The software supplied herewith by Microchip Technology           *
 *    Incorporated (the "Company") for its dsPIC controller            *
 *    is intended and supplied to you, the Company's customer,         *
 *    for use solely and exclusively on Microchip dsPIC                *
 *    products. The software is owned by the Company and/or its        *
 *    supplier, and is protected under applicable copyright laws. All  *
 *    rights are reserved. Any use in violation of the foregoing       *
 *    restrictions may subject the user to criminal sanctions under    *
 *    applicable laws, as well as to civil liability for the breach of *
 *    the terms and conditions of this license.                        *
 *                                                                     *
 *    THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO           *
 *    WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING,    *
 *    BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND    *
 *    FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE     *
 *    COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,  *
 *    INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.  *
 *                                                                     *
  **********************************************************************/

 /**********************************************************************
 *                                                                     * 
 *    Author: Smart Power Soutions, LLP                                * 
 *                                                                     *
 *    Filename:       slow_event.c     	                             *
 *    Date:           7/6/05                                        *
 *    File Version:   5.00                                             *
 *    Project:        53                                               *
 *    Drawing:        2                                                *
 *                                                                     *
 *    Tools used:    MPLAB C30 Compiler v 1.30                      *
 *                                                                     *
 *    Linker File:    p30f3010.gld                                   *
 *                                                                     *
 *                                                                     *
 ***********************************************************************
 *	Code Description
 *  
 *  This is the code for the slow event handler, which is executed 
 *  every 100 msec in this application.  The slow event handler updates
 *  the display screens, processes button input,and stores system
 *  parameters into user Flash.
 *  
 **********************************************************************/

#include <xc.h>
#include "hardware.h"
#include "defs.h"
#include "extern_globals.h"

void slow_event_handler(void);
void process_parameters(void);
extern void process_switches(void);

void slow_event_handler(void)
{
	// SLOW_EVENT_RATE is set to 100ms in defs.h
	if (slow_event_count > SLOW_EVENT_RATE) 
	{
		// Provide simple filtering of feedback values
		// for display purposes only
		filtered_vdc=(filtered_vdc+vdc)/2;
		filtered_pot=(filtered_pot+pot)/2;
		filtered_rpm=(filtered_rpm+rpm)/2;
		filtered_ibus=(filtered_ibus+ibus)/2;
		
		// If the software is in standby, call process_parameters
		// to updated all calculated parameters.
		if( run_state == STANDBY ) 
			process_parameters();

		process_switches();
		slow_event_count=0;
	}
	return;
}

// This function does the necessary calculations when a parameter
// value changes. Some parameters values are used directly, others
// form the basis for other variables but these need to be calculated.
void process_parameters(void)
{
	unsigned long ltemp;

	// If a value is missing from this switch statement this implies the
	// user parameter is used directly.
	// Note that parameters that affect other variables should also be in
	// this list e.g.if Voltage scaling changes, voltage demand and trips
	// need to be recalculated.
	
	// Save direction of motor rotation
	control_flags.DIR=user_parameters_RAM[0];

	// Get value reported for ibus
	// At this stage assuming ibus=0 and have first valid sample
	// and so use this one for offset
	ibus_offset=ibus;
	ltemp=((unsigned long)user_parameters_RAM[16])*ADC_GAIN;
	ltemp*=((unsigned long)user_parameters_RAM[26]);
	current_trip=(ltemp/(user_parameters_RAM[27]*10))+ibus_offset;

	// If using voltage control for starting
	if (user_parameters_RAM[40])
	{
		hold1_demand=(unsigned int)((unsigned long)user_parameters_RAM[4])*FULL_DUTY/100;
		hold2_demand=(unsigned int)((unsigned long)user_parameters_RAM[5])*FULL_DUTY/100;
		ramp_start_demand=(unsigned int)((unsigned long)user_parameters_RAM[8])*FULL_DUTY/100;
		ramp_end_demand=(unsigned int)((unsigned long)user_parameters_RAM[9])*FULL_DUTY/100;
	}
	else //Using current control assume scaling is in % of trip
	{
		ltemp=(unsigned long)(current_trip-ibus_offset);
		hold1_demand=(unsigned int)((ltemp*(unsigned long)user_parameters_RAM[4])/100);
		hold2_demand=(unsigned int)((ltemp*(unsigned long)user_parameters_RAM[5])/100);
		ramp_start_demand=(unsigned int)((ltemp*(unsigned long)user_parameters_RAM[8])/100);
		ramp_end_demand=(unsigned int)((ltemp*(unsigned long)user_parameters_RAM[9])/100);
	}	
	ramp_demand_delta = (signed int)(ramp_end_demand-ramp_start_demand);
	
	// Calculate step rates in units of TIMER2 from 
	// user parameters in RPM ensuring no overflows occur
	ltemp=((unsigned long)user_parameters_RAM[6]*(unsigned long)user_parameters_RAM[25])/20;
	// This check ensures that ramp_start_rate is calculated with no overflow
	if ((COUNTER_RATE/ltemp) > 65535)
	{	
		ramp_start_rate=65535;
		ramp_start_speed=COUNTER_RATE*20/(65535*(unsigned long)user_parameters_RAM[25]);
	}
	else
	{
		ramp_start_rate=(unsigned int)(COUNTER_RATE/ltemp);
		ramp_start_speed=user_parameters_RAM[6];
	}
	ltemp=((unsigned long)user_parameters_RAM[7]*(unsigned long)user_parameters_RAM[25])/20;
	// This check ensures that ramp_end_rate is calculated with no overflow
	if ((COUNTER_RATE/ltemp) > 65535)	
		ramp_end_rate=65535;
	else
		ramp_end_rate=(unsigned int)(COUNTER_RATE/ltemp);

	ramp_speed_delta=(int)(user_parameters_RAM[7]-user_parameters_RAM[6]);
	// Also calculate step rate at which zero X detection enabled when using
	// acquistion method 1
	ltemp=((unsigned long)user_parameters_RAM[44]*(unsigned long)user_parameters_RAM[25])/20;
	if ((COUNTER_RATE/ltemp) > 65535)
		acquire1_enable_rate=65535;
	else
		acquire1_enable_rate=(unsigned int)(COUNTER_RATE/ltemp);

	// ramp time is used to hold user_parameters[10] so that
	// it can be directly referenced in some inline assembly 
	ramp_time=user_parameters_RAM[10];
	iloop_p_gain=(int)user_parameters_RAM[17];
	iloop_i_gain=(int)user_parameters_RAM[18];
	iloop_d_gain=(int)user_parameters_RAM[19];
	wloop_p_gain=(int)user_parameters_RAM[20];
	wloop_i_gain=(int)user_parameters_RAM[21];
	vloop_p_gain=(int)user_parameters_RAM[23];
	vloop_i_gain=(int)user_parameters_RAM[24];
		
	// Now calculate the current limits used when in CLOSED_CURRENT
	// speed control as 95% of the current trip levels
	pos_current_limit=(long)(current_trip-ibus_offset)*95/100;
	pos_current_limit*=16384L;
	neg_current_limit=-1*pos_current_limit;

	ltemp=((unsigned long)user_parameters_RAM[28])*ADC_GAIN;
	voltage_trip=(ltemp*(unsigned long)user_parameters_RAM[15])/((unsigned long)user_parameters_RAM[29]*10);
	voltage_demand=(ltemp*(unsigned long)user_parameters_RAM[22])/((unsigned long)user_parameters_RAM[29]*10);

	upper_tol=100+user_parameters_RAM[30];
	lower_tol=100-user_parameters_RAM[30];

	// Calculate acquision (method2) threshold values based
	// on user parameter and ADC value read during initialization.
	// This compensates for offset voltages due to ADC and power module.
	vph_red_threshold=vph_red+user_parameters_RAM[34];
	vph_yellow_threshold=vph_yellow+user_parameters_RAM[34];
	vph_blue_threshold=vph_blue+user_parameters_RAM[34];

	return;

}

