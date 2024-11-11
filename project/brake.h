// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file brake.h
 *
 * @brief This module initializes data structure variable type definitions of 
 * data structure
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

#ifndef BRAKING_H
#define	BRAKING_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">


#include "general.h"
#include "userparms.h"

// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">

// </editor-fold>
    
typedef struct
{
    int16_t
        qIMag,                  /* Magnitude of current */
        *pIa,                   /* Pointer to IAlpha */
        *pIb,                   /* Pointer to IBeta */
        qIAlphaSqr,             /* Ialpha squared */
        qIBetaSqr,              /* Ibeta squared */
        qIThreshold,            /* Current threshold for braking */
        minBrakeCount,          /* Counter for minimum time for braking */
        minCurrentHoldCount,    /* Counter for holding the current 
                                 * below min value */
        status;                 /* Flag to indicate braking is complete */
    int32_t
        brakeOnCount,
        brakeOnCountMax;
}MCAPP_BRAKE_T;

void MCAPP_BrakeInit(MCAPP_BRAKE_T *);
void MCAPP_Brake(MCAPP_BRAKE_T *);

#ifdef	__cplusplus
}
#endif

#endif	/* BRAKING_H */

