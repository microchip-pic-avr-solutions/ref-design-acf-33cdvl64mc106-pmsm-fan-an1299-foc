// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * UART3.h
 *
 * This header file lists interface functions - to configure and enable UART3 
 * module and its features
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

#ifndef __UART3_H
#define __UART3_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#include <xc.h>

#include <stdint.h>
#include <stdbool.h>

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
                
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
     
        
/**
 * Enables and initializes UART3 with a default configuration:
 * - Continues module operation in Idle mode
 * - IrDA encoder and decoder are disabled
 * - UxRTS pin is in Flow Control mode
 * - UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLKx pins are
 *   controlled by PORT latches
 * - No wake-up is enabled
 * - Loop back mode is disabled
 * - Baud rate measurement is disabled
 * - UxRX Idle state is '1'
 * - BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
 * - 8-bit data, no parity
 * - One Stop bit.
 * This function does not configure the baud rate for UART3, however, it does
 * clear interrupt flags and disable interrupts for UART3 before initializing.
 * Summary: Enables and initializes UART3 with a default configuration.
 * @example
 * <code>
 * UART3_Initialize(void);
 * </code>
 */        
extern void UART3_Initialize(void);

/**
 * Configures how many bytes must be in the RX buffer before an RX interrupt
 * is generated. (watermark = 0 for 1 byte in the RX buffer)
 * @param watermark number of bytes
 */
inline static void UART3_RXSetWatermark(uint16_t watermark) { U3STAHbits.URXISEL = watermark; }

/**
 * Configures how much room must be in the TX buffer before an interrupt is
 * generated. (watermark = 0 for empty TX buffer)
 * @param watermark number of bytes
 */
inline static void UART3_TXSetWatermark(uint16_t watermark) { U3STAHbits.UTXISEL = watermark; }

/**
 * Clears the RX FIFO buffer.
 */
inline static void UART3_RXClearBuffer(void)
{
    uint16_t URXENstate = U3MODEbits.URXEN;
    U3MODEbits.URXEN = 0;
    U3STAHbits.URXBE = 1;
    U3MODEbits.URXEN = URXENstate;
}

/**
 * Clears the TX FIFO buffer.
 */
inline static void UART3_TXClearBuffer(void)
{
    uint16_t UTXENstate = U3MODEbits.UTXEN;
    U3MODEbits.UTXEN = 0;
    U3STAHbits.UTXBE = 1;
    U3MODEbits.UTXEN = UTXENstate;
}

/** 
 * Enable transmitter (TX) interrupt.
 */
inline static void UART3_EnableTransmitterInterrupt(void)
{
    IFS3bits.U3TXIF = 0;
    IEC3bits.U3TXIE = 1;
    IPC14bits.U3TXIP = 5;
}

/** 
 * Disable transmitter (TX) interrupt.
 */
inline static void UART3_DisableTransmitterInterrupt(void)
{
    IEC3bits.U3TXIE = 0;
    IFS3bits.U3TXIF = 0;
    IPC14bits.U3TXIP = 0;
}

/** 
 * Enable receiver (RX) interrupt.
 */
inline static void UART3_EnableReceiverInterrupt(void)
{
    IFS3bits.U3RXIF = 0;
    IPC14bits.U3RXIP = 5;
    IEC3bits.U3RXIE = 1;
}

/** 
 * Disable receiver (RX) interrupt.
 */
inline static void UART3_DisableReceiverInterrupt(void)
{
    IFS3bits.U3RXIF = 0;
    IEC3bits.U3RXIE = 0;
    IPC14bits.U3RXIP = 0;
}

/**
 * Assign custom callback to UART3 receive complete interrupt.
 * @param custom handler
 */
void UART3_RxCompleteCallbackAssign(void (*)(void));

/**
 * Assign custom callback to UART3 transfer complete interrupt.
 * @param custom handler
 */
void UART3_TxCompleteCallbackAssign(void (*)(void));

/**
  Section: Driver Interface
 */

/**
 * Clears UART3 transmit interrupt request flag.
 * Summary: Clears UART3 transmit interrupt request flag.
 * @example
 * <code>
 * UART3_InterruptTransmitFlagClear();
 * </code>
 */
inline static void UART3_InterruptTransmitFlagClear(void) {_U3TXIF = 0; }

/**
 * Clears UART3 receive interrupt request flag.
 * Summary: Clears UART3 receive interrupt request flag.
 * @example
 * <code>
 * UART3_InterruptReceiveFlagClear();
 * </code>
 */
inline static void UART3_InterruptReceiveFlagClear(void) {_U3RXIF = 0; }

/**
 * Enables UART3 transmit interrupt.
 * Summary: Enables UART3 transmit interrupt.
 * @example
 * <code>
 * UART3_InterruptTransmitEnable();
 * </code>
 */
inline static void UART3_InterruptTransmitEnable(void) {_U3TXIE = 1; }

/**
 * Disables UART3 transmit interrupt.
 * Summary: Disables UART3 transmit interrupt.
 * @example
 * <code>
 * UART3_InterruptTransmitDisable();
 * </code>
 */
inline static void UART3_InterruptTransmitDisable(void) {_U3TXIE = 0; }

/**
 * Enables UART3 receive interrupt.
 * Summary: Enables UART3 receive interrupt.
 * @example
 * <code>
 * UART3_InterruptReceiveEnable();
 * </code>
 */
inline static void UART3_InterruptReceiveEnable(void) {_U3RXIE = 1; }

/**
 * Disables UART3 receive interrupt.
 * Summary: Disables UART3 receive interrupt.
 * @example
 * <code>
 * UART3_InterruptReceiveDisable();
 * </code>
 */
inline static void UART3_InterruptReceiveDisable(void) {_U3RXIE = 0; }

/**
 * Configures UART3 module to operate in standard baud rate mode 
 *(i.e. 16x baud clock). 
 * Baud Rate = FREQ_UART_CLK (BCLKSEL)/ (16*(BRG +1))
 * @example
 * <code>
 * UART3_SpeedModeStandard();
 * </code>
 */
inline static void UART3_SpeedModeStandard(void) {U3MODEbits.BRGH = 0; }

/**
 * Configures UART3 module to operate in High-speed baud rate mode 
 * (i.e. 4x baud clock).
 * Baud Rate = FREQ_UART_CLK (BCLKSEL) / (4*(BRG +1))
 * @example
 * <code>
 * UART3_SpeedModeHighSpeed();
 * </code>
 */
inline static void UART3_SpeedModeHighSpeed(void) {U3MODEbits.BRGH = 1; }

/**
 * Configures the baud rate divider for UART3 module.
 * Baud Rate = FREQ_UART_CLK (BCLKSEL)/ (16*(BRG +1)),if BRGH = 0, BCLKMOD = 0
 * Baud Rate = FREQ_UART_CLK (BCLKSEL)/ (4*(BRG +1)),if BRGH = 0, BCLKMOD = 0
 * @example
 * <code>
 * UART3_BaudRateDividerSet(500);
 * </code>
 */
inline static void UART3_BaudRateDividerSet(uint16_t baudRateDivider)
{
    U3BRG = baudRateDivider;
}

/**
 * Disables UART3 module.
 * Summary: Disables UART3 module.
 * @example
 * <code>
 * UART3_ModuleDisable();
 * </code>
 */
inline static void UART3_ModuleDisable(void) 
{
    U3MODEbits.UARTEN = 0;
}

/**
 * Enables UART3 module.
 * Summary: Enables UART3 module.
 * @example
 * <code>
 * UART3_ModuleEnable();
 * </code>
 */
inline static void UART3_ModuleEnable(void) 
{
    U3MODEbits.UARTEN = 1;
}

/**
 * Enables UART3 module transmit mode.
 * @example
 * <code>
 * UART3_TransmitModeEnable();
 * </code>
 */
inline static void UART3_TransmitModeEnable(void) {U3MODEbits.UTXEN = 1; }

/**
 * Disables UART3 module transmit mode.
 * @example
 * <code>
 * UART3_TransmitModeDisable();
 * </code>
 */
inline static void UART3_TransmitModeDisable(void) {U3MODEbits.UTXEN= 0; }

/**
 * Gets the status of UART3 Receive Buffer Data Available flag
 * @return status of UART3 Receive Buffer Data Available flag; 
 * 1 = Receive buffer has data, 0 = Receive buffer is empty 
 * @example
 * <code>
 * status = UART3_IsReceiveBufferDataReady();
 * </code>
 */

inline static bool UART3_IsReceiveBufferDataReady(void)
{
    return(!U3STAHbits.URXBE) ;
}

/**
 * Gets the status of UART3 Receive Buffer Overrun Error Status flag
 * @return status of UART3 Receive Buffer Overrun Error Status flag; 
 * 1 = Receive buffer has overflowed, 0 = Receive buffer has not overflowed
 * @example
 * <code>
 * status = UART3_IsReceiveBufferOverFlowDetected();
 * </code>
 */
inline static bool UART3_IsReceiveBufferOverFlowDetected(void)
{
    return(U3STAbits.OERR) ;
}

/**
 * Gets the status of UART3 Framing Error Status flag
 * @return status of UART3 Framing Error Status flag; 
 * 1 = Framing error has been detected for the character at the top of 
 *     the receive FIFO
 * 0 = Framing error has not been detected
 * @example
 * <code>
 * status = UART3_IsFrameErrorDetected();
 * </code>
 */
inline static bool UART3_IsFrameErrorDetected(void)
{
    return(U3STAbits.FERR) ;
}
/**
 * Gets the status of UART3 Parity Error Status flag
 * @return status of UART3 Parity Error Status flag; 
 * 1 = Parity error has been detected for the character 
 *     at the top of the receive FIFO
 * 0 = Parity error has not been detected
 * @example
 * <code>
 * status = UART3_IsParityErrorDetected();
 * </code>
 */
inline static bool UART3_IsParityErrorDetected(void)
{
    return(U3STAbits.PERR) ;
}
/**
 * Gets the status of UART3 Receiver is Idle Status Flag
 * @return status of UART3 Receiver is Idle Status flag; 
 * 1 = Receiver is Idle
 * 0 = Receiver is Active
 * @example
 * <code>
 * status = UART3_IsReceiverIdle();
 * </code>
 */
inline static bool UART3_IsReceiverIdle(void)
{
    return(U3STAHbits.RIDLE) ;
}
/**
 * Gets the status of UART3 Transmit Shift Register Empty Status Flag
 * @return status of UART3 Transmit Shift Register Empty Status flag; 
 * 1 = Transmit Shift Register is empty and transmit buffer is empty
 * 0 = Transmit Shift Register is not empty, a transmission is in progress 
 *     or queued
 * @example
 * <code>
 * status = UART3_IsTransmissionComplete();
 * </code>
 */
inline static bool UART3_IsTransmissionComplete(void)
{
    return(U3STAbits.TRMT) ;
}

/**
 * Gets the status of UART3 transmit buffer full flag
 * @return status of UART3 transmit buffer full flag; 1 = Transmit buffer is
 *  full, 0 = Transmit buffer is not full
 * @example
 * <code>
 * status = UART3_StatusBufferFullTransmitGet();
 * </code>
 */
inline static bool UART3_StatusBufferFullTransmitGet(void)
{
    return U3STAHbits.UTXBF;
}

/**
 * Gets the transmitter and receiver status of UART3. The returned status may
 * contain a value with more than one of the bits specified in the
 * UART3_STATUS enumeration set. The caller should perform an "AND" with the
 * bit of interest and verify if the result is non-zero to verify the 
 * desired status bit.
 * @return UART3_STATUS value describing the current status of the transfer.
 * @example
 * <code>
 * status = UART3_StatusGet();
 * </code>
 */
inline static uint16_t UART3_StatusGet(void)
{
    return U3STA;
}

/**
 * Clears the Receive Buffer Overrun Error Status bit for UART3. If this bit
 * was previously set, then calling this function will reset the receiver buffer
 * and the U3RSR to an empty state.
 * @example
 * <code>
 * UART3_ReceiveBufferOverrunErrorFlagClear();
 * </code>
 */
inline static void UART3_ReceiveBufferOverrunErrorFlagClear(void)
{
    U3STAbits.OERR = 0;
}

/**
 * Writes a 16-bit data word to UART3 transmit register.
 * @param data data to be transmitted on UART3
 * @example
 * <code>
 * UART3_DataWrite(txdata);
 * </code>
 */
inline static void UART3_DataWrite(uint16_t data)
{
    U3TXREGbits.TXREG =(uint8_t)data;
}

/**
 * Reads a 16-bit data word from the UART3 receive register.
 * @return data read from the UART3 receive register
 * @example
 * <code>
 * rxdata = UART3_DataRead();
 * </code>
 */
inline static uint16_t UART3_DataRead(void)
{
    return U3RXREG;
}
/**
 * Clears the Receive Buffer Overrun Error Status bit for UART3. If this bit
 * was previously set, then calling this function will reset the receiver buffer
 * and the U3RSR to an empty state.
 * @example
 * <code>
 * UART3_ReceiveBufferOverrunErrorFlagClear();
 * </code>
 */
inline static void UART3_ReceiveBufferEmpty(void)
{
    uint16_t data;
    if(U3STAHbits.URXBE == 0)
    {
        while(UART3_IsReceiveBufferDataReady()) 
        {
            data = UART3_DataRead() ;
        }
        U3STAHbits.URXBE = 1;
    }
}

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __UART3_H
    