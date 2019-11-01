#include "defs.h"
#include "Motor_isr.h"
#define		STATE0 	0x0600		// A top, B bottom, C BEMF
#define		STATE1 	0x1200		// A top, C bottom, B BEMF		 
#define		STATE2 	0x1800		// B top, C bottom, A BEMF
#define		STATE3 	0x0900		// B top, A bottom, C BEMF
#define		STATE4 	0x2100		// C top, A bottom, B BEMF
#define  	STATE5  0x2400		// C top, B bottom, A BEMF
// ----------------------Function prototypes ---------------------------
void CheckZeroCrossing(void);
unsigned int ThirtyDegreeTimeAverage(void);

// ----------------- Global Variable Declarations ----------------------
unsigned int pot;		    // stores the potentiometer value
unsigned int HalltoSector[8] = {-1,2,4,3,0,1,5,-1};
unsigned int SectortoState[6] = {STATE0,STATE1,STATE2,STATE3,STATE4,STATE5};
unsigned int HallState;		// Hall effect sensor state
int Sector = 0;				// Sector (0 - 5)
int OldSector;				// Stores old sector

int vpha, vphb, vphc, vbus;  // stores the ADC result for phase A, B, and C and the bus voltage measurement

unsigned int Timer1TimeoutCntr = 0;
unsigned int BlankingCount = 3;
unsigned int phase_advance = 0;
unsigned int SpeedPtr;
unsigned int RunMode = 0;
unsigned int SensorlessStartState = 0;
unsigned int MediumEventCounter = 0;
unsigned int SlowEventCounter = 0;
long accumulator_c;
int signal_average;
unsigned long Speed = 0;
int vpha_filtered_sample;
int vphb_filtered_sample;
int vphc_filtered_sample;
unsigned int NextSectorState;
unsigned int SixtyDegreeTime[6];
unsigned int OneEightyDegreeTime[16];

volatile struct ControlFlags ControlFlags;

unsigned int pos_ptr = 0;
unsigned int TMR4Save;

/*---------------------------------------------------------------------
  Function Name: T2Interrupt
  Description:   T2 Interrupt Handler
  Inputs:        None
  Returns:       None
-----------------------------------------------------------------------*/
void __attribute__((__interrupt__,no_auto_psv)) _T2Interrupt( void )  // TMR 2 is never turned on in Hall Mode
{
	TMR2 = 0;			   			// clear TMR2
	IFS0bits.T2IF = 0;

	if (RunMode == SENSORLESS_RUNNING)	
	{
		Sector++;						// Increment Sector (there are 6 total)
		if(Sector > 5) Sector = 0;
		Commutate(Sector);				// Change the PWM output for next sector
	}
}
void Commutate(unsigned int sector)
{
	OVDCON = SectortoState[sector];			// Change PWM phase 
	OldSector = sector;						// Keep track of last sector (for error checking elsewhere)
	//BlankingCount = 6;
}
