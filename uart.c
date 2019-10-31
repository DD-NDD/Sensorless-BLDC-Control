/**********************************************************************
* © 2012 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Incorporated ("Microchip") retains all ownership and 
* intellectual property rights in the code accompanying this message and in all 
* derivatives hereto.  You may use this code, and any derivatives created by 
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes 
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
******************************************************************************/
#include "uart.h"
#ifdef _U1RXIF

/*********************************************************************
* Function Name     : CloseUART2                                     *
* Description       : This function disables the UART and clears the *
*                     interrupt enable and flag bits                 *
* Parameters        : None                                           *
* Return Value      : None                                           *
*********************************************************************/

void CloseUART1(void)
{  
    U1MODEbits.UARTEN = 0;
	
    IEC0bits.U1RXIE = 0;
    IEC0bits.U1TXIE = 0;
	
    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
}


/*********************************************************************
* Function Name     : DataRdyUart2                                   *
* Description       : This function checks whether there is any data *
*                     that can be read from the input buffer, by     *
*                     checking URXDA bit                             *
* Parameters        : None                                           *
* Return Value      : char if any data available in buffer           *
*********************************************************************/

char DataRdyUART1(void)
{
    return(U1STAbits.URXDA);
}


/*********************************************************************
* Function Name     : OpenUART2                                      *
* Description       : This function configures the UART mode,        *
*                     UART Interrupt modes and the Baud Rate         *
* Parameters        : unsigned int config1 operation setting         *
*                     unsigned int config2 TX & RX interrupt modes   *
*                     unsigned int ubrg baud rate setting            *
* Return Value      : None                                           *      
*********************************************************************/

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
    U1BRG  = ubrg;       /* baud rate */
    U1MODE = config1;    /* operation settings */
    U1STA = config2;     /* TX & RX interrupt modes */
}


/***************************************************************************
* Function Name     : ReadUART2                                            *
* Description       : This function returns the contents of UxRXREG buffer *
* Parameters        : None                                                 *  
* Return Value      : unsigned int value from UxRXREG receive buffer       * 
***************************************************************************/

unsigned int ReadUART1(void)
{
    if(U1MODEbits.PDSEL == 3)
        return (U1RXREG);
    else
        return (U1RXREG & 0xFF);
}


/*************************************************************************
* Function Name     : BusyUART2                                          *
* Description       : This returns status whether the transmission       *  
*                     is in progress or not, by checking Status bit TRMT *
* Parameters        : None                                               *
* Return Value      : Info about whether transmission is in progress.    *
*************************************************************************/

char BusyUART1(void)
{  
    return(!U1STAbits.TRMT);
}


/**********************************************************************
* Function Name     : ConfigIntUART2                                  *
* Description       : This function sets priority for  RX and TX      *
*                     interrupt and enable/disables the interrupt     *  
* Parameters        : unsigned int config enable/disable and priority *
* Return Value      : None                                            *
**********************************************************************/

void ConfigIntUART1(unsigned int config)
{
    /* clear IF flags */
    _U1RXIF = 0;
    _U1TXIF = 0;

    /* set priority */
    _U1RXIP = 0x0007 & config;
    _U1TXIP = (0x0070 & config) >> 4;

    /* enable/disable interrupt */
    _U1RXIE = (0x0008 & config) >> 3;
    _U1TXIE = (0x0080 & config) >> 7;
}


/******************************************************************************
* Function Name     : getsUART2                                               *
* Description       : This function gets a string of data of specified length * 
*                     if available in the UxRXREG buffer into the buffer      *
*                     specified.                                              *
* Parameters        : unsigned int length the length expected                 *
*                     unsigned int *buffer  the received data to be           * 
*                                  recorded to this array                     *
*                     unsigned int uart_data_wait timeout value               *
* Return Value      : unsigned int number of data bytes yet to be received    * 
******************************************************************************/

unsigned int getsUART1(unsigned int length,unsigned int *buffer,
                       unsigned int uart_data_wait)

{
    int wait = 0;
    char *temp_ptr = (char *) buffer;

    while(length)                         /* read till length is 0 */
    {
        while(!DataRdyUART1())
        {
            if(wait < uart_data_wait)
                wait++ ;                  /*wait for more data */
            else
                return(length);           /*Time out- Return words/bytes to be read */
        }
        wait=0;
        if(U1MODEbits.PDSEL == 3)         /* check if TX/RX is 8bits or 9bits */
            *buffer++ = U1RXREG;          /* data word from HW buffer to SW buffer */
	else
            *temp_ptr++ = U1RXREG & 0xFF; /* data byte from HW buffer to SW buffer */

        length--;
    }

    return(length);                       /* number of data yet to be received i.e.,0 */
}


/***************************************************************************
* Function Name     : putsUART2                                            *
* Description       : This function puts the data string to be transmitted *
*                     into the transmit buffer (till NULL character)       * 
* Parameters        : unsigned int * address of the string buffer to be    *
*                     transmitted                                          *
* Return Value      : None                                                 *  
***************************************************************************/

void putsUART1(unsigned int *buffer)
{
    char * temp_ptr = (char *) buffer;

    /* transmit till NULL character is encountered */

//    RTDM_Transmitting = 1;
    if(U1MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer != '\0') 
        {
 //           while(U1STAbits.UTXBF); /* wait if the buffer is full */
		while(U1STAbits.TRMT == 0);/* wait if the buffer is not empty */
            U1TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
    else
    {
        while(*temp_ptr != '\0')
        {
  //          while(U1STAbits.UTXBF);  /* wait if the buffer is full */
		while(U1STAbits.TRMT == 0);/* wait if the buffer is not empty */
            U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
//	RTDM_Transmitting = 0;
}


/*********************************************************************
* Function Name     : WriteUART2                                     *
* Description       : This function writes data into the UxTXREG,    *
* Parameters        : unsigned int data the data to be written       *
* Return Value      : None                                           *
*********************************************************************/

void WriteUART1(unsigned int data)
{
    if(U1MODEbits.PDSEL == 3)
        U1TXREG = data;
    else
        U1TXREG = data & 0xFF;  
}

#else
#warning "Does not build on this target"
#endif
