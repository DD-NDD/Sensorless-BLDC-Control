#include "defs.h"
#include <uart.h>
int init_UART( void )
{
    unsigned int U1MODEvalue;
    unsigned int U1STAvalue; 
    ConfigIntUART1(UART_RX_INT_EN & UART_RX_INT_PR6 & 

                   UART_TX_INT_DIS & UART_TX_INT_PR2);
        U1MODEvalue = UART_EN & UART_IDLE_CON & UART_ALTRX_ALTTX &

                  UART_DIS_WAKE & UART_EN_LOOPBACK  &

                  UART_EN_ABAUD & UART_NO_PAR_8BIT  &

                  UART_1STOPBIT;

    U1STAvalue  = UART_INT_TX_BUF_EMPTY  &  

                  UART_TX_PIN_NORMAL &

                  UART_TX_ENABLE & UART_INT_RX_3_4_FUL &

                  UART_ADR_DETECT_DIS &

                  UART_RX_OVERRUN_CLEAR;

    OpenUART1(U1MODEvalue, U1STAvalue, BRGVAL);
    return 1;
}
