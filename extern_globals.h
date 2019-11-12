/* 
 * File:   extern_globals.h
 * Author: Duy Ngo
 *
 * Created on November 12, 2019, 10:37 AM
 */

#ifndef EXTERN_GLOBALS_H
#define	EXTERN_GLOBALS_H
extern volatile unsigned char run_state;
extern volatile unsigned char trip_state;

extern unsigned char sector;

struct control_flags {		
				unsigned TORQUE_LIMIT: 1;
				unsigned VOLTS_LIMIT	: 1;
				unsigned DIR		: 1;
				unsigned ADCCONFIG: 1;
				unsigned ZERO_CROSS : 1;
				unsigned LEVEL		: 1;
				unsigned ACQUIRE1	: 1;
				unsigned SWAP	: 1;
				unsigned SENSORLESS : 1;
				unsigned	ACQUIRE2	: 1;
				unsigned ACQUIRE2_RED : 1;
				unsigned ACQUIRE2_YELLOW : 1;
				unsigned ACQUIRE2_BLUE : 1;
				unsigned LOCK1 : 1;
				unsigned LOCK2	: 1;
				unsigned RAMP	: 1;
};

extern volatile struct control_flags control_flags;

struct control_flags2	{
				unsigned AUTO_REACQUIRE	:	1;
				unsigned ROTATION_CHECK	:	1;
				unsigned RETRY_FLAG		:	1;
				unsigned D_TERM_DISABLE	:	1;
				unsigned ACQUIRE1_REQUEST :1;
				unsigned FALLING_EDGE	:	1;
				unsigned UNUSED			:	10;
								};

extern volatile struct control_flags2 control_flags2;

extern unsigned int hold1_demand;	
extern unsigned int hold2_demand;

extern unsigned int ramp_start_demand;
extern unsigned int ramp_end_demand;
extern unsigned int ramp_start_speed;
extern unsigned int ramp_speed_delta;
extern unsigned int ramp_start_rate;
extern unsigned int ramp_end_rate;
extern unsigned int ramp_time;
extern int ramp_rate_delta;
extern int ramp_demand_delta;
extern unsigned int acquire1_enable_rate;
									
extern unsigned int current_trip;
extern unsigned int voltage_trip;
extern unsigned int speed_trip;
extern volatile unsigned int user_parameters_RAM[64];
extern volatile unsigned int slow_event_count;
extern volatile unsigned char medium_speed_event_count;

extern unsigned int starting_timer;
extern int wloop_p_gain;						
extern int wloop_i_gain;						
extern int iloop_p_gain;						
extern int iloop_i_gain;
extern int iloop_d_gain;						
extern int vloop_p_gain;						
extern int vloop_i_gain;								

extern int voltage_demand;

extern unsigned char valid_switch_states;

// These are globals so they can be referenced inside inline asm
extern volatile int stemp;											
extern volatile unsigned int new_speed;


extern unsigned int vdc;
extern unsigned int ibus;
extern unsigned int vph;
extern unsigned int vph_red;
extern unsigned int vph_yellow;
extern unsigned int vph_blue;
extern unsigned int vph_red_threshold;
extern unsigned int vph_yellow_threshold;
extern unsigned int vph_blue_threshold;
extern unsigned int pot;

extern unsigned int previous_timestamps[];
extern unsigned int edge_deltas[];

extern unsigned int adc_channel_config;

extern volatile unsigned int new_PR2;

extern int current_demand;

extern long pos_duty_limit;
extern long neg_duty_limit;

extern unsigned int ibus_offset;

extern unsigned int period_measurement;

extern unsigned int rpm;

extern unsigned int speed_demand;

extern unsigned int current_scaling;

extern long pos_current_limit;
extern long neg_current_limit;

extern unsigned int stall_counter;

extern unsigned int latest_delta;
extern unsigned int previous_delta;
extern volatile unsigned int check_value;
extern int commutation_time;
extern unsigned int upper_tol;
extern unsigned int lower_tol;

extern volatile unsigned char check_counter;
extern volatile unsigned char acquire_counter;
extern unsigned int phase_advance;

extern unsigned int filtered_rpm;
extern unsigned int filtered_vdc;
extern unsigned int filtered_ibus;
extern unsigned int filtered_pot;

extern unsigned char valid_switch_state;

extern unsigned char param;
extern unsigned int new_param_value;

extern unsigned int fault_count;

#endif	/* EXTERN_GLOBALS_H */

