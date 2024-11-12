// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * UART3.c
 *
 * This file includes subroutine to configure UART3 Module
 * 
 * This file is intended for the dsPIC33CDVL64MC106 on the Engine Cooling Fan
 * Reference Design board.
 * 
 * Component: HAL - UART3
 * 
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">
/*******************************************************************************
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries
 * 
 * Subject to your compliance with these terms, you may use Microchip software
 * and any derivatives exclusively with Microchip products. You are responsible
 * for complying with 3rd party license terms applicable to your use of 3rd
 * party software (including open source software) that may accompany Microchip
 * software. SOFTWARE IS ?AS IS.? NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR
 * STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF
 * NON-INFRINGEMENT, MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN
 * NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S TOTAL LIABILITY ON ALL CLAIMS
 * RELATED TO THE SOFTWARE WILL NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID
 * DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Header Files ">

#include <xc.h>

#include <stdint.h>
#include <stdbool.h>
#include "uart3.h"

// </editor-fold> 

/* RX/TX interrupt function pointers */
static void (*UART3_RxCompleteHandler)(void);
static void (*UART3_TxCompleteHandler)(void);

// <editor-fold defaultstate="collapsed" desc="FUNCTION DECLARATIONS ">

void UART3_Initialize(void);

// </editor-fold> 

void UART3_RxCompleteCallbackAssign(void (*handler)(void))
{
    UART3_RxCompleteHandler = handler;
}

void UART3_TxCompleteCallbackAssign(void (*handler)(void))
{
    UART3_TxCompleteHandler = handler;
}

/**
 * Function to configure UART3 Module
 * @param None.
 * @return None.
 * @example
 * <code>
 * status = UART3_Initialize();
 * </code>
 */
void UART3_Initialize (void)
{
    /** Initialize UARTx Configuration Register */
    U3MODE = 0;
    /** UART Enable bit
        1 = UART is ready to transmit and receive
        0 = UART state machine, FIFO Buffer Pointers and counters are reset */
    U3MODEbits.UARTEN = 0;
    /** UART Stop in Idle Mode bit
        1 = Discontinues module operation when device enters Idle mode
        0 = Continues module operation in Idle mode */
    U3MODEbits.USIDL = 0;
    /** Wake-up Enable bit
        1 = Module will continue to sample the RX pin 
        0 = RX pin is not monitored nor rising edge detected */
    U3MODEbits.WAKE  = 0;
    /** Receive Break Interrupt Mode bit
        1 = RXBKIF flag when a minimum of 23 (DMX)/11 (asynchronous or LIN/J2602
            low bit periods are detected
        0 = RXBKIF flag when the Break makes a low-to-high transition after 
            being low for at least 23/11 bit periods*/
    U3MODEbits.RXBIMD = 0;
    /** Send Break Software Override bit - Overrides the TX Data Line:
        1 = Makes the TX line active ('0' when UTXINV = 0, '1'when UTXINV = 1)
        0 = TX line is driven by the shifter */
    U3MODEbits.BRKOVR = 0;
    /** UART Transmit Break bit
        1 = Sends Sync Break on next transmission;
        0 = Sync Break transmission is disabled or has completed */
    U3MODEbits.UTXBRK = 0;
    /** High Baud Rate Select bit
        1 = High Speed: Baud rate is baud clock/4
        0 = Low Speed: Baud rate is baud clock/16 */
    U3MODEbits.BRGH = 0;
    /** Auto-Baud Detect Enable bit (read-only when MOD<3:0> = 1xxx)
        1 = Enables baud rate measurement on the next character ? 
            requires reception of a Sync field (55h);
        0 = Baud rate measurement is disabled or has completed */
    U3MODEbits.ABAUD = 0;
    /** UART Transmit Enable bit
        1 = Transmit enabled ? except during Auto-Baud Detection
        0 = Transmit disabled ? all transmit counters, pointers and 
            state machines are reset; TX buffer is not flushed, status bits 
            are not reset */
    U3MODEbits.UTXEN = 1;
    /** UART Receive Enable bit
        1 = Receive enabled ? except during Auto-Baud Detection
        0 = Receive disabled ? all receive counters, pointers and state machines
            are reset; RX buffer is not flushed, status bits are not reset */
    U3MODEbits.URXEN = 1;
    /** UART Mode bits
        1111 = Smart card 
        1110 = IrDA�(2)
        1100 = LIN Master/Slave
        1011 = LIN Slave-Only
        1010 = DMX(2)
        1001 = DALI gear(2)
        1000 = DALI device(2)
        0100 = Asynchronous 9-bit UART with address detect, address
        0011 = Asynchronous 8-bit UART without address detect,even parity 
        0010 = Asynchronous 8-bit UART without address detect, odd parity
        0001 = Asynchronous 7-bit UART
        0000 = Asynchronous 8-bit UART */
    U3MODEbits.MOD = 0;
    
    /** Initialize UARTx Configuration Register High */
    U3MODEH =  0;
    /** Run During Sleep Enable bit
        1 = UART BRG clock runs during Sleep
        0 = UART BRG clock is turned off during Sleep */
    U3MODEHbits.SLPEN = 0;
    /** UART Running Status bit
        1 = UART clock request is active 
        0 = UART clock request is not active */
    U3MODEHbits.ACTIVE = 0;
    
    /** Baud clock generation mode select bit
        1 = Uses fractional baud rate generation 
        0 = uses legacy divide-by-x counter for baud clock generation */
    U3MODEHbits.BCLKMOD = 0;
    
    /** Baud Clock Source Selection bits 
        0b11 = AFVCO/3,0b10 = FOSC ,0b01 = Reserved ,0b00 = FOSC/2 */
    U3MODEHbits.BCLKSEL = 0;
    /** UART Half-Duplex Selection Mode bit
        1 = Half-Duplex mode: UxTX is driven high when transmitting and 
            low when TX is Idle
        0 = Full-Duplex mode: UxTX is driven high at all times 
            when both UARTEN and UTXEN are set */
    U3MODEHbits.HALFDPLX = 0;
    /** Run During Overflow Condition Mode bit
        1 = When an Overflow Error (OERR) condition is detected, the RX shifter
             continues to run so as to remain synchronized with incoming RX data;
             data is not transferred to UxRXREG when it is full 
        0 = When an Overflow Error (OERR) condition is detected, 
            the RX shifter stops accepting new data (Legacy mode) */
    U3MODEHbits.RUNOVF = 0;
    /** UART Receive Polarity bit
        1 = Inverts RX polarity; Idle state is low
        0 = Input is not inverted; Idle state is high */
    U3MODEHbits.URXINV = 0;
    /** Number of Stop Bits Selection bits
        11 = 2 Stop bits sent, 1 checked at receive
        10 = 2 Stop bits sent, 2 checked at receive
        01 = 1.5 Stop bits sent, 1.5 checked at receive
        00 = 1 Stop bit sent, 1 checked at receive */
    U3MODEHbits.STSEL = 0;
    /** Enable Legacy Checksum (C0) Transmit and Receive bit
        1 = Checksum Mode 1 (enhanced LIN checksum in LIN mode;)
        0 = Checksum Mode 0 (legacy LIN checksum in LIN mode;
            not used in all other modes)*/
    U3MODEHbits.C0EN = 0;
    /** UART Transmit Polarity bit
        1 = Inverts TX polarity; TX is low in Idle state
        0 = Output data is not inverted; TX output is high in Idle state */
    U3MODEHbits.UTXINV = 0;
    /** Flow Control Enable bits (Only valid when MOD = 0xxx)
        10 = RTS-CTS (For TX-side) / DCD-DTR (For RX-side) Hardware Flow Control
        01 = XON/XOFF Software Flow Control
        00 = Flow Control Off */
    U3MODEHbits.FLO = 0;

    /** Initialize UARTx Status Register */
    U3STA = 0;
    /** Transmit Shifter Empty Interrupt Enable bit
        1 = Interrupt is enabled, 0 = Interrupt is disabled */
    U3STAbits.TXMTIE = 0;
    /** Parity Error Interrupt Enable bit
        1 = Interrupt is enabled , 0 = Interrupt is disabled */
    U3STAbits.PERIE = 0;
    /** Auto-Baud Rate Acquisition Interrupt Enable bit
        1 = Interrupt is enabled, 0 = Interrupt is disabled */
    U3STAbits.ABDOVE = 0;
    /** Checksum Error Interrupt Enable bit
        1 = Interrupt is enabled , 0 = Interrupt is disabled */
    U3STAbits.CERIE = 0;
    /** Framing Error Interrupt Enable bit
        1 = Interrupt is enabled, 0 = Interrupt is disabled */
    U3STAbits.FERIE = 0;
    /** Receive Break Interrupt Enable bit
        1 = Interrupt is enabled, 0 = Interrupt is disabled */
    U3STAbits.RXBKIE = 0;
    /** Receive Buffer Overflow Interrupt Enable bit
        1 = Interrupt is enabled , 0 = Interrupt is disabled */
    U3STAbits.OERIE = 0;
    /** Transmit Collision Interrupt Enable bit
        1 = Interrupt is enabled, 0 = Interrupt is disabled */
    U3STAbits.TXCIE = 0;
    /** Transmit Shifter Empty Interrupt Flag bit
        1 = Transmit Shift Register (TSR) is empty 
        0 = Transmit Shift Register is not empty */
    U3STAbits.TRMT = 0;
    /** Parity Error/Address Received/Forward Frame Interrupt Flag bit
        LIN and Parity Modes:
        1 = Parity error detected, 0 = No parity error detected
        DALI Device Mode:
        1 = Forward frame data, 0 = Back frame data
        Address Mode:
        1 = Address received, 0 = No address detected
        All Other Modes: Not used. */
    U3STAbits.PERR = 0;
    /** Auto-Baud Rate Acquisition Interrupt Flag bit 
        1 = BRG rolled over during the auto-baud rate acquisition sequence
        0 = BRG has not rolled over during the auto-baud rate acquisition 
            sequence*/
    U3STAbits.ABDOVF = 0;
    /** Checksum Error Interrupt Flag bit 
        1 = Checksum error , 0 = No checksum error*/
    U3STAbits.CERIF = 0;
    /** Framing Error Interrupt Flag bit
        1 = Framing Error, 0 = No framing error */
    U3STAbits.FERR = 0;
    /** Receive Break Interrupt Flag bit 
        1 = A Break was received, 0 = No Break was detected */
    U3STAbits.RXBKIF = 0;
    /** Receive Buffer Overflow Interrupt Flag bit (must be cleared by software)
        1 = Receive buffer has overflowed
        0 = Receive buffer has not overflowed */
    U3STAbits.OERR = 0;
    /** Transmit Collision Interrupt Flag bit (must be cleared by software)
        1 = Transmitted word is not equal to the received word
        0 = Transmitted word is equal to the received word */
    U3STAbits.TXCIF = 0;

    /** Initialize UARTx Status High Register */
    U3STAH = 0;
    /** UART Transmit Interrupt Select bits
        111 = Sets TX interrupt when there is 1 empty slot left in the buffer
        ....
        000 = Sets TX interrupt when there are 8 empty slots in the buffer;
              TX buffer is empty */
    U3STAHbits.UTXISEL = 0;
    /** UART Receive Interrupt Select bits
        111 = Triggers receive interrupt when there are 8 words in the buffer;
              RX buffer is full
        ...
        000 = Triggers RX interrupt when there is 1 word or more in the buffer*/
    U3STAHbits.URXISEL = 0;
    /** TX Write Transmit Error Status bit (must be cleared by software)
        LIN and Parity Modes:
        1 = A new byte was written when the buffer was full or when P2<8:0> = 0 
        0 = No error
        Address Detect Mode:
        1 = A new byte was written when the buffer was full or 
            to P1<8:0> when P1x was full 
        0 = No error
        Other Modes:
        1 = A new byte was written when the buffer was full 
        0 = No error */
    U3STAHbits.TXWRE = 0;
    /** Stop Bit Detection Mode bit
        1 = Triggers RXIF at the end of the last Stop bit
        0 = Triggers RXIF in the middle of the first (
            or second, depending on the STSEL<1:0> setting) Stop bit */
    U3STAHbits.STPMD = 0;
    /** UART TX Buffer Empty Status bit
        1 = Transmit buffer is empty; writing ?1? when UTXEN = 0 will reset 
            the TX FIFO Pointers and counters
        0 = Transmit buffer is not empty */
    U3STAHbits.UTXBE = 1;
    /** UART TX Buffer Full Status bit
        1 = Transmit buffer is full
        0 = Transmit buffer is not full */
    U3STAHbits.UTXBF = 0;
    /** Receive Idle bit
        1 = UART RX line is in the Idle state
        0 = UART RX line is receiving something */
    U3STAHbits.RIDLE = 0;
    /** UART in XON Mode bit
        Only valid when FLO<1:0> control bits are set to XON/XOFF mode.
        1 = UART has received XON
        0 = UART has not received XON or XOFF was received */
    U3STAHbits.XON = 0;
    /** UART RX Buffer Empty Status bit
        1 = Receive buffer is empty; writing ?1? when URXEN = 0 will reset 
            the RX FIFO Pointers and counters
        0 = Receive buffer is not empty */
    U3STAHbits.URXBE = 1;
    /** UART RX Buffer Full Status bit
        1 = Receive buffer is full
        0 = Receive buffer is not full */
    U3STAHbits.URXBF = 0;
    
    /** Initialize UARTx Baud Rate Register bits<15:0> =  BRG<15:0>  */
    U3BRG = 650; // baudrate(650) = 9600, baudrate(324) = 19.2k

    /** Initialize UARTx Baud Rate High Register bits<3:0> =  BRG<19:16> */
    U3BRGH = 0;

    /** Initialize UARTx Receive Buffer Register */
    U3RXREG = 0;
    /** Received Character Data bits <7-0>*/
    U3RXREGbits.RXREG = 0;

    /** UARTx Transmit Buffer Register */
    U3TXREG =  0;
    /** Last Byte Indicator for Smart Card Support bit */
    U3TXREGbits.LAST = 0;
    /** Transmitted Character Data bits <7-0>
        If the buffer is full, further writes to the buffer are ignored. */
    U3TXREGbits.TXREG = 0;
    
    /** Initialize UARTx Timing Parameter 1 Register */
    U3P1 = 0;
    /** Initialize UARTx Timing Parameter 2 Register */
    U3P2 = 0;
    /** Initialize UARTx Timing Parameter 3 Register */
    U3P3 = 0;
    /** Initialize UARTx Timing Parameter 3 Register High */
    U3P3H = 0;
    /** Initialize UARTx Transmit Checksum Register */
    U3TXCHK = 0;
    /** Initialize UARTx Receive Checksum Register */
    U3RXCHK = 0;
    /** Initialize UARTx Smart Card Configuration High Register */
    U3SCCON = 0;
    /** Initialize UxSCINT: UARTx Smart Card Interrupt Register */
    U3SCINT =  0;

    /** Initialize UARTx Interrupt Register */
    U3INT = 0;
    /** Wake-up Interrupt Flag bit
        1 = Sets when WAKE = 1 and RX makes a 1-to-0 transition; t
        0 = WAKE is not enabled or WAKE is enabled, 
            but no wake-up event has occurred */
    U3INTbits.WUIF = 0;
    /** Auto-Baud Completed Interrupt Flag bit
        1 = Sets when ABD sequence makes the final 1-to-0 transition; 
        0 = ABAUD is not enabled or ABAUD is enabled 
            but auto-baud has not completed */
    U3INTbits.ABDIF = 0;
    /** Auto-Baud Completed Interrupt Enable Flag bit
        1 = Allows ABDIF to set an event interrupt
        0 = ABDIF does not set an event interrupt */
    U3INTbits.ABDIE = 0;
    
    /** UART Running Status bit
        1 = UART clock request is active 
        0 = UART clock request is not active */
    U3MODEHbits.ACTIVE = 0;
    
    /** UART Enable bit
        1 = UART is ready to transmit and receive
        0 = UART state machine, FIFO Buffer Pointers and counters are reset */
    U3MODEbits.UARTEN = 0;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3TXInterrupt(void)
{
    if (UART3_TxCompleteHandler) {
        (*UART3_TxCompleteHandler)();
    }
    IFS3bits.U3TXIF = 0;
}


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U3RXInterrupt(void)
{
    if (UART3_RxCompleteHandler) {
        (*UART3_RxCompleteHandler)();
    }
    IFS3bits.U3RXIF = 0;
}