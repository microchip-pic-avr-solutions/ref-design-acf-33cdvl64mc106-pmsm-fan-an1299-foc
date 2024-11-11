// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file brake.c
 *
 * @brief This module implements generic load state machine.
 *
 * Component: FAN LOAD
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>
#include <libq.h>

#include "brake.h"
#include "motor_control_declarations.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_BrakeInit(MCAPP_BRAKE_T *);
void MCAPP_Brake(MCAPP_BRAKE_T *);

// </editor-fold>

/**
* <B> Function: void MCAPP_BrakeInit(MCAPP_BRAKE_T *)  </B>
*
* @brief Brake initialization.
*
* @param Pointer to the data structure containing brake parameters.
* @return none.
* @example
* <CODE> MCAPP_BrakeInit(&brake); </CODE>
*
*/
void MCAPP_BrakeInit(MCAPP_BRAKE_T *pBrake)
{
    pBrake->status = 0;
    pBrake->minBrakeCount = 0;
    pBrake->minCurrentHoldCount = 0;
    pBrake->brakeOnCount = 0;
}

/**
* <B> Function: void MCAPP_Brake(MCAPP_BRAKE_T *)  </B>
*
* @brief Brake Function - Provides braking duration.
*
* @param Pointer to the data structure containing brake parameters.
* @return none.
* @example
* <CODE> MCAPP_Brake(&brake); </CODE>
*
*/
void MCAPP_Brake(MCAPP_BRAKE_T *pBrake)
{
    pBrake->brakeOnCount++;
    if (pBrake->brakeOnCount >= pBrake->brakeOnCountMax)
    {
        pBrake->brakeOnCount = 0;
        pBrake->status = 1;
    }
}