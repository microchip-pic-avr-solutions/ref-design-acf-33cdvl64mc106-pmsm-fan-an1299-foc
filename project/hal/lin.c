/**
 * lin.c
 *
 * This file includes function definitions for the LIN module.
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

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "lin.h"

LIN_NODE lin;                           /** LIN node state. */
uint8_t rxBuffer[LIN_RXQUEUE_LENGTH];   /** RX queue buffer. */
uint8_t txBuffer[LIN_TXQUEUE_LENGTH];   /** TX queue buffer. */

/**
 * Reset queue state.
 * @param p_queue queue state
 */
inline static void LIN_QueueReset(LIN_QUEUE *p_queue)
{
    p_queue->p_head = p_queue->p_start;
    p_queue->p_tail = p_queue->p_start;
    p_queue->count = 0;
}

/**
 * Initialize queue state.
 * @param p_queue queue state
 * @param p_buffer start of buffer memory used for queue
 * @param length number of bytes of buffer memory
 */
inline static void LIN_QueueInit(LIN_QUEUE *p_queue, uint8_t *p_buffer, uint8_t length)
{
    p_queue->p_start = p_buffer;
    p_queue->length = length - 1;
    p_queue->p_end = p_buffer + length - 1;
    LIN_QueueReset(p_queue);
}

/**
 * Is the queue empty?
 * @param p_queue queue state
 * @return empty = 1, not empty = 0
 */
inline static bool LIN_QueueEmpty(LIN_QUEUE *p_queue)
{
    return p_queue->count == 0;
}

/**
 * Is the queue full?
 * @param p_queue queue state
 * @return full = 1, not full = 0
 */
inline static bool LIN_QueueFull(LIN_QUEUE *p_queue)
{
    return p_queue->count == p_queue->length;
}

/**
 * Get number of bytes in queue.
 * @param p_queue queue state
 * @return number of bytes in queue
 */
inline static uint8_t LIN_QueueGetCount(LIN_QUEUE *p_queue)
{
    return p_queue->count;
}

/**
 * Push byte to queue.
 * @param p_queue queue state
 * @param byte byte to push to queue
 */
inline static void LIN_QueuePush(LIN_QUEUE *p_queue, uint8_t byte)
{
    if (LIN_QueueFull(p_queue)) {
        return;
    }
    *p_queue->p_head++ = byte;
    if (p_queue->p_head == p_queue->p_end) {
        p_queue->p_head = p_queue->p_start;
    }
    p_queue->count++;
}

/**
 * Pop byte from queue.
 * @param p_queue queue state
 * @return byte popped from queue
 */
inline static uint8_t LIN_QueuePop(LIN_QUEUE *p_queue)
{
    if (LIN_QueueEmpty(p_queue)) {
        return 0;
    }
    uint8_t data = *p_queue->p_tail++;
    if (p_queue->p_tail == p_queue->p_end) {
        p_queue->p_tail = p_queue->p_start;
    }
    p_queue->count--;
    return data;
}

/**
 * Check for parity mismatch. See DS70005288 pg. 42 for details.
 * @param pid_rcv PID to check for parity mismatch
 */
inline static uint8_t LIN_CalculatePID(uint8_t pid_rcv)
{
    uint8_t pid = 0x3F&pid_rcv;
    uint8_t id0 = pid & 0x01;
    uint8_t id1 = (pid & 0x02) >> 1;
    uint8_t id2 = (pid & 0x04) >> 2;
    uint8_t id3 = (pid & 0x08) >> 3;
    uint8_t id4 = (pid & 0x10) >> 4;
    uint8_t id5 = (pid & 0x20) >> 5;
    uint8_t p0 = id0^id1^id2^id4;
    uint8_t p1 = ~(id1^id3^id4^id5);
    return pid+(p0<<6)+(p1<<7);
}

/**
 * Calculate checksum. See DS70005288 pg. 43 for details.
 * @param p_queue checksum queue
 */
inline static void LIN_CalculateChecksum(LIN_CHECKSUM *p_checksum)
{
    union {
        struct {
            uint8_t lsb;
            uint8_t msb;
        } uint16;
        uint16_t word;
    } checksum;
    checksum.word = 0;
    uint8_t index;
    for (index = 0; index < LIN_FRAME_LENGTH; index++) {
        checksum.word += p_checksum->buffer[index];
        if (checksum.uint16.msb) {
            checksum.word++;
            checksum.uint16.msb = 0;
        }
    }
    p_checksum->calculated = ~checksum.uint16.lsb;
}

/**
 * Executes actions in the LIN_RXSTATE_DISCONNECTED state.
 * @param p_lin LIN node state
 */
inline static void LIN_RxStateDisconnected(LIN_NODE *p_lin)
{
    // no action yet
}

/**
 * Executes actions in the LIN_RXSTATE_WAITING state.
 * @param p_lin LIN node state
 */
inline static void LIN_RxStateWaiting(LIN_NODE *p_lin)
{
    /* Read break, sync, and pid bytes into LIN frame. Get ready for next RX
     * interrupt and check PID byte for parity error. */
    p_lin->frame.break_byte   = UART3_DataRead();
    p_lin->frame.sync         = UART3_DataRead();
    p_lin->frame.pid.received = UART3_DataRead();
    UART3_RXClearBuffer();
    UART3_RXSetWatermark(LIN_WATERMARK_FRAME_DATA);
    p_lin->frame.pid.calculated = LIN_CalculatePID(p_lin->frame.pid.received);
    if (p_lin->frame.pid.received != p_lin->frame.pid.calculated) {
        /* Parity error, handle in the rx error state. */
        p_lin->pid_error_num++;
        p_lin->rx.state = LIN_RXSTATE_ERROR;
        return;
    }
    
    /* Is the commander node expecting to receive bytes? */
    uint8_t pid = 0x07&p_lin->frame.pid.received;
    if (pid == LIN_PID_TRANSMIT) {
        p_lin->tx.frame_requested = true;
        p_lin->rx.state = LIN_RXSTATE_TRANSMITTING;
    }
    /* Is the commander node transmitting bytes? */
    else if (pid == LIN_PID_RECEIVE) {
        p_lin->rx.state = LIN_RXSTATE_RECEIVING;
    }
    /* Invalid PID byte for this node. */
    else {
        p_lin->rx.state = LIN_RXSTATE_ERROR;
    }
}

/**
 * Executes actions in the LIN_RXSTATE_RECEIVING state.
 * @param p_lin LIN node state
 */
inline static void LIN_RxStateReceiving(LIN_NODE *p_lin)
{
    /* If the application hasn't processed the most recent frame (highly
     * unlikely), then clear the buffer and prepare for the next RX interrupt. */
    if (p_lin->rx.frame_ready) {
        UART3_RXClearBuffer();
        UART3_RXSetWatermark(LIN_WATERMARK_FRAME_START);
        p_lin->rx.state = LIN_RXSTATE_WAITING;
        return;
    }
    
    /* Move frame into checksum buffer. */
    p_lin->frame.checksum.buffer[0] = UART3_DataRead();
    p_lin->frame.checksum.buffer[1] = UART3_DataRead();
    p_lin->frame.checksum.buffer[2] = UART3_DataRead();
    p_lin->frame.checksum.buffer[3] = UART3_DataRead();
    p_lin->frame.checksum.received  = UART3_DataRead();
    
    /* Prepare for next RX interrupt. */
    UART3_RXClearBuffer();
    UART3_RXSetWatermark(LIN_WATERMARK_FRAME_START);
    p_lin->rx.frame_ready = true;
    p_lin->rx.state = LIN_RXSTATE_WAITING;
}

/**
 * Executes actions in the LIN_RXSTATE_TRANSMITTING state.
 * @param p_lin LIN node state
 */
inline static void LIN_RxStateTransmitting(LIN_NODE *p_lin)
{
    /* On a transmit, the bytes we transmit will be read in by the UART RX
     * buffer. To account for this, we need to clear the buffer after we receive
     * the bytes we transmit. */
    UART3_RXClearBuffer();
    UART3_RXSetWatermark(LIN_WATERMARK_FRAME_START);
    p_lin->rx.state = LIN_RXSTATE_WAITING;
}

/**
 * Executes actions in the LIN_RXSTATE_ERROR state.
 * @param p_lin LIN node state
 */
inline static void LIN_RxStateError(LIN_NODE *p_lin)
{
    /* Clear the UART RX buffer, get ready to receive the next frame. Error
     * handling routines for specific errors can be added here in the future. */
    UART3_RXClearBuffer();
    UART3_RXSetWatermark(LIN_WATERMARK_FRAME_START);
    p_lin->rx.state = LIN_RXSTATE_WAITING;
}

/**
 * LIN RX interrupt callback function to be called in UART RX interrupt. This
 * function executes the LIN RX state machine.
 */
void LIN_RxInterruptCallback(void)
{
    const uint16_t state = lin.rx.state;
    switch (state)
    {
        case LIN_RXSTATE_WAITING:
            LIN_RxStateWaiting(&lin);
            break;
        case LIN_RXSTATE_RECEIVING:
            LIN_RxStateReceiving(&lin);
            break;
        case LIN_RXSTATE_TRANSMITTING:
            LIN_RxStateTransmitting(&lin);
            break;
        case LIN_RXSTATE_ERROR:
            LIN_RxStateError(&lin);
            break;
        case LIN_RXSTATE_DISCONNECTED:
            LIN_RxStateDisconnected(&lin);
            break;
        default:
            break;
    }
    lin.rx.interrupt_num++;
}

/**
 * Executes actions when data bytes are requested by commander node.
 * @param p_lin LIN node state
 */
inline static void LIN_ApplicationOnTransmitFrame(LIN_NODE *p_lin)
{
    /* Calculate checksum. */
    p_lin->frame.checksum.buffer[0] = LIN_QueuePop(&p_lin->tx.queue);
    p_lin->frame.checksum.buffer[1] = LIN_QueuePop(&p_lin->tx.queue);
    p_lin->frame.checksum.buffer[2] = LIN_QueuePop(&p_lin->tx.queue);
    p_lin->frame.checksum.buffer[3] = LIN_QueuePop(&p_lin->tx.queue);
    LIN_CalculateChecksum(&p_lin->frame.checksum);
    
    /* Transmit frame. */
    UART3_DataWrite(p_lin->frame.checksum.buffer[0]);
    UART3_DataWrite(p_lin->frame.checksum.buffer[1]);
    UART3_DataWrite(p_lin->frame.checksum.buffer[2]);
    UART3_DataWrite(p_lin->frame.checksum.buffer[3]);
    UART3_DataWrite(p_lin->frame.checksum.calculated);
    p_lin->tx.frame_num++;
    p_lin->tx.frame_requested = false;
}

/**
 * Executes actions when a frame is received.
 * @param p_lin LIN node state
 */
inline static void LIN_ApplicationOnReceiveFrame(LIN_NODE *p_lin)
{
    /* Calculate checksum. Don't move data to RX queue if there is a checksum
     * error. */
    LIN_CalculateChecksum(&p_lin->frame.checksum);
    if (p_lin->frame.checksum.received != p_lin->frame.checksum.calculated) {
        p_lin->checksum_error_num++;
        p_lin->rx.frame_ready = false;
        return;
    }
    LIN_QueuePush(&p_lin->rx.queue, p_lin->frame.checksum.buffer[0]);
    LIN_QueuePush(&p_lin->rx.queue, p_lin->frame.checksum.buffer[1]);
    LIN_QueuePush(&p_lin->rx.queue, p_lin->frame.checksum.buffer[2]);
    LIN_QueuePush(&p_lin->rx.queue, p_lin->frame.checksum.buffer[3]);
    p_lin->rx.frame_num++;
    p_lin->rx.frame_ready = false;
}

void LIN_ApplicationStepMain(LIN_NODE *p_lin)
{
    if (p_lin->tx.frame_requested && !LIN_QueueEmpty(&p_lin->tx.queue)) {
        LIN_ApplicationOnTransmitFrame(p_lin);
    }
    
    if (p_lin->rx.frame_ready) {
        LIN_ApplicationOnReceiveFrame(p_lin);
    }
}

bool LIN_TxQueueEmpty(LIN_NODE *p_lin)
{
    return LIN_QueueEmpty(&p_lin->tx.queue);
}

void LIN_TxQueuePush(LIN_NODE *p_lin, uint8_t data)
{
    LIN_QueuePush(&p_lin->tx.queue, data);
}

bool LIN_RxQueueFull(LIN_NODE *p_lin)
{
    return LIN_QueueFull(&p_lin->rx.queue);
}

uint8_t LIN_RxQueueGetCount(LIN_NODE *p_lin)
{
    return LIN_QueueGetCount(&p_lin->rx.queue);
}

bool LIN_RxFrameReady(LIN_NODE *p_lin)
{
    return LIN_QueueGetCount(&p_lin->rx.queue) >= LIN_FRAME_LENGTH;
}

uint8_t LIN_RxQueuePop(LIN_NODE *p_lin)
{
    return LIN_QueuePop(&p_lin->rx.queue);
}

void LIN_Enable(LIN_NODE *p_lin)
{
    p_lin->rx.state = LIN_RXSTATE_WAITING;
    LIN_QueueReset(&p_lin->rx.queue);
    LIN_QueueReset(&p_lin->tx.queue);
    p_lin->rx.frame_ready = false;
    p_lin->rx.frame_num = 0;
    p_lin->tx.frame_requested = false;
    p_lin->tx.frame_num = 0;
    UART3_RXClearBuffer();
    UART3_TXClearBuffer();
    UART3_RXSetWatermark(LIN_WATERMARK_FRAME_START);
    UART3_EnableReceiverInterrupt();
    UART3_ModuleEnable();
    LIN_EN = 1;
    p_lin->enabled = true;
}

void LIN_Disable(LIN_NODE *p_lin)
{
    LIN_EN = 0;
    UART3_ModuleDisable();
    UART3_DisableReceiverInterrupt();
    p_lin->rx.state = LIN_RXSTATE_DISCONNECTED;
    p_lin->enabled = false;
}

void LIN_InitNode(LIN_NODE *p_lin)
{
    LIN_EN = 0;
    p_lin->enabled = false;
    p_lin->rx.state = LIN_RXSTATE_DISCONNECTED;
    p_lin->rx.interrupt_num = 0;
    p_lin->pid_error_num = 0;
    p_lin->checksum_error_num = 0;
    LIN_QueueInit(&p_lin->rx.queue, &rxBuffer[0], LIN_RXQUEUE_LENGTH);
    LIN_QueueInit(&p_lin->tx.queue, &txBuffer[0], LIN_TXQUEUE_LENGTH);
    UART3_RxCompleteCallbackAssign(&LIN_RxInterruptCallback);
    UART3_Initialize();
}