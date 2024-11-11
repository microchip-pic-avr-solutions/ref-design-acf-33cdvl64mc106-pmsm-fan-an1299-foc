/**
 * lin.h
 *
 * This header file defines the interface of the LIN module - to configure and 
 * manage LIN communication.
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

#ifndef LIN_H
#define	LIN_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "lin_types.h"
#include "uart3.h"
#include "port_config.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Is the LIN transceiver enabled?
 * @param p_lin LIN node state
 * @return LIN transceiver status (not connected = false, connected = true)
 */
inline static bool LIN_IsEnabled(LIN_NODE *p_lin)
{
    return p_lin->enabled;
}

/**
 * Has a response been requested by the commander node?
 * @param p_lin LIN node state
 * @return yes = 1, no = 0
 */
inline static bool LIN_TxFrameRequested(LIN_NODE *p_lin)
{
    return p_lin->tx.frame_requested;
}

/**
 * Is the TX queue empty?
 * @param p_lin LIN node state
 * @return empty = 1, not empty = 0
 */
bool LIN_TxQueueEmpty(LIN_NODE *p_lin);

/**
 * Push a byte to the TX queue.
 * @param p_lin LIN node state
 * @param data byte to push to TX queue
 */
void LIN_TxQueuePush(LIN_NODE *p_lin, uint8_t data);

/**
 * Does the RX queue have a full frame of data?
 * @param p_lin LIN node state
 * @return full = 1, not full = 0
 */
bool LIN_RxQueueFull(LIN_NODE *p_lin);

/**
 * Get number of bytes in the RX queue.
 * @param p_lin LIN node state
 * @return number of bytes in queue
 */
uint8_t LIN_RxQueueGetCount(LIN_NODE *p_lin);

/**
 * Is a complete frame in the RX queue?
 * @param p_lin LIN node state
 * @return yes = 1, no = 0
 */
bool LIN_RxFrameReady(LIN_NODE *p_lin);

/**
 * Get one byte from the RX queue.
 * @param p_lin LIN node state
 * @return byte from the RX queue
 */
uint8_t LIN_RxQueuePop(LIN_NODE *p_lin);

/**
 * Handles tasks outside of RX interrupts. Should be called in the main loop.
 * @param p_lin LIN node state
 */
void LIN_ApplicationStepMain(LIN_NODE *);

/**
 * Enable LIN transceiver and connect to LIN bus.
 * @param p_lin LIN node state
 */
void LIN_Enable(LIN_NODE *);

/**
 * Disable LIN transceiver.
 * @param p_lin LIN node state
 */
void LIN_Disable(LIN_NODE *);

/**
 * Initialize LIN node.
 * @param p_lin LIN node state
 */
void LIN_InitNode(LIN_NODE *);

#ifdef	__cplusplus
}
#endif

#endif	/* LIN_H */

