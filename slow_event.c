#include "defs.h"
#include "hardware.h"
#include "Motor_isr.h"
#include "stdio.h"

void SlowEvent(void)
{
volatile struct {
		unsigned Button_S2:1;
		unsigned Button_S3:1;

		unsigned :14;
} DebounceFlags;

	ControlFlags.SlowEventFlag = 0;

// Check button presses

	if(BUTTON_S2 && !DebounceFlags.Button_S2) 
	{
		ControlFlags.RunMotor = 0;
		DebounceFlags.Button_S2 = 1;

		RunMode = SENSORLESS_RUNNING;
        printf("RunMode = SENSORLESS_RUNNING \r\n");
	}

	if(BUTTON_S3)
	{
		RunMode = MOTOR_OFF;
		ControlFlags.RunMotor = 0;
		SensorlessStartState = 0;
		DebounceFlags.Button_S3 = 1;
        printf("MOTOR OFF\r\n");
	}

	if (DebounceFlags.Button_S2)
	{
		if (!BUTTON_S2) DebounceFlags.Button_S2 = 0;
	}


	if (DebounceFlags.Button_S3)
	{
		if (!BUTTON_S3) DebounceFlags.Button_S3 = 0;
	}
	
}