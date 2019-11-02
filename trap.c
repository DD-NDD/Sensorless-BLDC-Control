#include "defs.h"
void __attribute__((__interrupt__,no_auto_psv)) _OscillatorFail(void);
void __attribute__((__interrupt__,no_auto_psv)) _AddressError(void);
void __attribute__((__interrupt__,no_auto_psv)) _StackError(void);
void __attribute__((__interrupt__,no_auto_psv)) _MathError(void);
void __attribute__((__interrupt__,no_auto_psv)) _AltOscillatorFail(void);
void __attribute__((__interrupt__,no_auto_psv)) _AltAddressError(void);
void __attribute__((__interrupt__,no_auto_psv)) _AltStackError(void);
void __attribute__((__interrupt__,no_auto_psv)) _AltMathError(void);
void __attribute__((__interrupt__,no_auto_psv)) _OscillatorFail(void)
{
        INTCON1bits.OSCFAIL = 0;        //Clear the trap flag
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _AddressError(void)
{
        INTCON1bits.ADDRERR = 0;        //Clear the trap flag
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _StackError(void)
{
        INTCON1bits.STKERR = 0;         //Clear the trap flag
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _MathError(void)
{
        INTCON1bits.MATHERR = 0;        //Clear the trap flag
        while (1);
}

/* Alternate Exception Vector handlers:
   These routines are used if INTCON2bits.ALTIVT = 1.
   All trap service routines in this file simply ensure that device
   continuously executes code within the trap service routine. Users
   may modify the basic framework provided here to suit to the needs
   of their application. */

void __attribute__((__interrupt__,no_auto_psv)) _AltOscillatorFail(void)
{
        INTCON1bits.OSCFAIL = 0;
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _AltAddressError(void)
{
        INTCON1bits.ADDRERR = 0;
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _AltStackError(void)
{
        INTCON1bits.STKERR = 0;
        while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _AltMathError(void)
{
        INTCON1bits.MATHERR = 0;
        while (1);
}

