/*
 * File:   main.c
 * Author: Duy Ngo
 *
 * Created on October 29, 2019, 7:44 PM
 */



#define FCY  29491200			//FRC w/PLL x16
#define MILLISEC FCY/29491		//1 mSec delay constant
#define FPWM 16000
#define TYC (1.0/FCY)
#define dDeadTimeSec 0.000002
#define dDeadTime (int)(dDeadTimeSec*TYC) // Dead time in dTcys 
#include "xc.h"

const unsigned int PWM_STATE[]	=	{0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};
void InitTMR3(void);
void DelayNmSec(unsigned int N);
void InitMCPWM(void);

int main(void) {
    LATE = 0x0000;
    TRISE = 0xFFC0;
    //InitTMR3();
    InitMCPWM();
    unsigned char i = 0;
    PWMCON1 = 0x0777;
    while(1)
    {
        for(i = 0; i<6;i++)
		{            
            switch(i)
            {
                case 0: OVDCON=PWM_STATE[1]; break;
                case 1: OVDCON=PWM_STATE[2]; break;
                case 2: OVDCON=PWM_STATE[3]; break;
                case 3: OVDCON=PWM_STATE[4]; break;
                case 4: OVDCON=PWM_STATE[5]; break;
                case 5: OVDCON=PWM_STATE[6]; break;
            }
            if(i == 5) i = 0;
            DelayNmSec(3);
		}
    }
    return 0;
}
void InitMCPWM(void)
{
    PTPER = FCY/FPWM - 1;
    PWMCON1 = 0x0700; // disable PWMs
    DTCON1 = dDeadTime; // Dead time
    OVDCON = 0x0000; // allow control using OVD
    PDC1 = 00; // init PWM 1, 2 and 3 to 100
    PDC2 = 400;
    PDC3 = 400;
    SEVTCMP = PTPER; // special trigger is 16 period values
    PWMCON2 = 0x0F00; // 16 postscale values
    PTCON = 0x8000; // start PWM
}
void InitTMR3(void)
{
    return;
}
void DelayNmSec(unsigned int N)
{
    unsigned int j;
    while(N--)
 	for(j=0;j < MILLISEC;j++);
}