#include "conf.h"
#include "uart1.h"
#define XTFREQ          7372800
#define PLLMODE         16
#define FCY             XTFREQ*PLLMODE/4
#define MILLISEC        FCY/29491
#define BAUDRATE         115200       
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 
int main(void)
{
    UART1_Initialize();
    while (1)
    {
        // Add your application code
    }

    return 1;
}