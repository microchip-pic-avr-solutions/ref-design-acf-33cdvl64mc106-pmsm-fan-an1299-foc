/**
 * lin_types.h
 *
 * This header file lists interface types for the LIN module.
 * 
 * This file is intended for the dsPIC33CDVL64MC106 on the Engine Cooling Fan
 * Reference Design board. The firmware is designed with the assumption that the
 * dsPIC33CDVL64MC106 is the Responder node, and that a LIN frame contains 4
 * bytes of data (not including the break, sync, PID, and checksum bytes).
 * 
 * Component: HAL - LIN
 * 
 */

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

#ifndef LIN_TYPES_H
#define	LIN_TYPES_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
#define LIN_FRAME_LENGTH    4 // Number of data bytes in a LIN frame (excluding break, sync, PID, and checksum bytes)

/** Number of bytes in LIN RX queue.
 */
#define LIN_RXQUEUE_LENGTH  5*LIN_FRAME_LENGTH+1 // 5 LIN frames (+1 byte for detecting end of queue buffer)
    
/** Number of bytes in LIN TX queue.
 */
#define LIN_TXQUEUE_LENGTH  LIN_FRAME_LENGTH+1 // 1 LIN frame (+1 byte for detecting end of queue buffer)

/** 
 * UART watermark for triggering RX interrupt after receiving first 3 bytes of
 * frame.
 */
#define LIN_WATERMARK_FRAME_START  2

/**
 * UART watermark for triggering RX interrupt after receiving LIN frame
 * + checksum.
 */
#define LIN_WATERMARK_FRAME_DATA  LIN_FRAME_LENGTH

/**
 * Protected ID (PID) byte (without parity bits) indicating the commander node
 * is not expecting a response.
 */
#define LIN_PID_RECEIVE   0x04

/**
 * Protected ID (PID) byte (without parity bits) indicating the commander node
 * is expecting a response.
 */
#define LIN_PID_TRANSMIT  0x05

/** 
 * LIN RX state machine names.
 */
typedef enum tagLIN_RXSTATE {
    LIN_RXSTATE_DISCONNECTED = 0,
    LIN_RXSTATE_WAITING      = 1,
    LIN_RXSTATE_RECEIVING    = 2,
    LIN_RXSTATE_TRANSMITTING = 3,
    LIN_RXSTATE_ERROR        = 4
} LIN_RXSTATE;

/**
 * LIN queue state.
 */
typedef struct tagLIN_QUEUE {
    uint8_t *p_head;      /** Pointer to the head of the queue. */
    uint8_t *p_tail;      /** Pointer to the tail of the queue. */
    uint8_t *p_start;     /** Pointer to the start of the buffer. */
    uint8_t *p_end;       /** Pointer to the end of the buffer. */
    uint8_t count;        /** Number of bytes in the queue. */
    uint8_t length;       /** Length of buffer used for queue. */
} LIN_QUEUE;

/**
 * LIN checksum state. 
 */
typedef struct tagLIN_CHECKSUM
{
    uint8_t received;                          /** Received checksum byte. */
    uint8_t calculated;                        /** Calculated checksum for verifying frame integrity. */
    uint8_t buffer[LIN_FRAME_LENGTH];          /** Buffer for storing bytes for checksum calculation. */
} LIN_CHECKSUM;

/**
 * LIN frame state.
 */
typedef struct tagLIN_FRAME
{
    uint8_t break_byte;     /** Break byte (should always be 0). */
    uint8_t sync;           /** Sync byte (should always be 0x55). */
    struct {
        uint8_t received;   /** Received protected ID (PID) byte. */
        uint8_t calculated; /** Calculated PID for checking parity. */
    } pid;
    LIN_CHECKSUM checksum;  /** LIN Checksum state. */
} LIN_FRAME;

/**
 * LIN transmit (TX) state.
 */
typedef struct tagLIN_TX
{
    volatile bool frame_requested;   /** Is data frame requested by commander? */
    uint16_t frame_num;              /** Number of frames transmitted. */
    LIN_QUEUE queue;                 /** TX data queue. */
} LIN_TX;

/**
 * LIN receive (RX) state. 
 */
typedef struct tagLIN_RX
{
    uint16_t state;            /** State of RX state machine. */
    uint16_t interrupt_num;    /** Number of RX interrupts. */
    uint16_t frame_num;        /** Number of frames received without errors. */
    volatile bool frame_ready; /** Status of received frame (is frame ready to be read by main code?). */
    LIN_QUEUE queue;           /** RX data queue. */
} LIN_RX;

/**
 * LIN node state.
 */
typedef struct tagLIN_NODE
{
    bool enabled;                /** LIN module status. */
    uint16_t pid_error_num;      /** Number of rejected frames due to parity error. */
    uint16_t checksum_error_num; /** Number of rejected frames due to checksum error. */
    LIN_FRAME frame;             /** LIN frame state. */
    LIN_RX rx;                   /** LIN RX state. */
    LIN_TX tx;                   /** LIN TX state. */
} LIN_NODE;

#ifdef	__cplusplus
}
#endif

#endif	/* LIN_TYPES_H */

