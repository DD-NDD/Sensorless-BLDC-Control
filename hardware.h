/******************************************************************************/
/* System Level #define Macros                                                */
/******************************************************************************/

/* TODO Define system operating frequency */

/* Microcontroller MIPs (FCY) */
#define SYS_FREQ        7370000L
#define FCY             SYS_FREQ/4

/******************************************************************************/
/* System Function Prototypes                                                 */
/******************************************************************************/

// Also declare some useful shortcuts
//#define DISABLE_FIRING PWM_OUTPUT_DISABLE=1
//#define ENABLE_FIRING PWM_OUTPUT_DISABLE=0
#define DISABLE_FIRING PWMCON1 = 0x0700
#define ENABLE_FIRING  PWMCON1 = 0x0777

//This is the RS232_TX line
#define RS232_TX LATFbits.LATF3

//This is the RS232_RX line
#define RS232_RX PORTFbits.RF2