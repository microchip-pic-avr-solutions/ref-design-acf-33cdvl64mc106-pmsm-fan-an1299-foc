// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file windmill.h
 *
 * @brief This module initializes data structure variable type definitions of 
 * data structure and enumerations
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



#ifndef WINDMILL_H
#define	WINDMILL_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">


#include "general.h"
#include "userparms.h"
#include "board_service.h" 
#include "measure.h"
#include "pwm.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">
#define PHASE_VOLTAGE_OFFSET            950
#define WINDMILL_BEMF_CONST             (float)(NOMINAL_KFI *MC1_PEAK_SPEED_RPM) /(float)(1000*(float)MC1_PEAK_VOLTAGE)
#define WINDMILL_BEMF_CONST_Q15         Q15(WINDMILL_BEMF_CONST)
#define WINDMILL_MIN_BEMF_PU            (int16_t)( WINDMILL_BEMF_CONST * WINDMILL_MIN_SPEED_PU / 1.73)    
#define WINDMILL_MIN_BEMF                WINDMILL_MIN_BEMF_PU + PHASE_VOLTAGE_OFFSET
#define WINDMILL_LOW_BEMF_COUNT_MAX     (int16_t)((60*(int32_t)(PWMFREQUENCY_HZ))/(MC1_BEMF_SAMPLING_FACTOR * WINDMILL_MIN_SPEED * POLE_PAIRS))

#define WINDMILL_HIGH_BEMF_COUNT_MAX    (int16_t)((60*(int32_t)(PWMFREQUENCY_HZ))/(int32_t)(MC1_BEMF_SAMPLING_FACTOR * (int32_t)NOMINAL_SPEED_RPM * POLE_PAIRS * 4))

#define WINDMILL_ZERO_CROSS_PERSIST     (int16_t)((60*(int32_t)(PWMFREQUENCY_HZ))/(int32_t)(MC1_BEMF_SAMPLING_FACTOR * (int32_t)MAXIMUM_SPEED_RPM * POLE_PAIRS *5))

#define WINDMILL_CYCLES_MAX             3


// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="ENUMERATED CONSTANTS ">
    
    /* Windmill state Enumeration */
typedef enum
{
    WM_INIT = 0,                    /* Initialization of 
                                     * Windmill State Machine */
    WM_CHECK_WINDMILL = 1,          /* Check if motor is windmilling */
            
    WM_CALCULATE_WINDMILL_SPEED = 2,/* Check the direction of windmilling */
            
    WM_CHECK_WINDMILL_DIRECTION = 3,/* Calculate windmilling speed */
            
    WM_CALCULATE_CURRENT_REF = 4,   /* Calculate the initialization 
                                     * current reference for closed loop */
    WM_CALCULATE_VOLTAGE_REF = 5,   /* Calculate the initialization 
                                     * voltage reference for closed loop */
    WM_CALCULATE_MOD_INDEX_REF = 6, /* Calculate the initialization
                                     * modulation index for closed loop */
    WM_WAIT_FOR_COMPLETE = 7,       /* Wait for transition point */
            
    WM_COMPLETE = 8,                /* Windmilling is complete */
            
}WINDMILL_STATE_VARS;

    /* Windmill direction Enumeration */
typedef enum
{
    WM_DIRECTION_UNIDENTIFIED = 0,  /* Unidentified direction */
    WM_DIRECTION_STOP = 1,          /* Motor is stopped */
    WM_DIRECTION_REVERSE = 2,       /* Motor is spinning in reverse direction */
    WM_DIRECTION_FORWARD = 3,       /* Motor is spinning in forward direction */
}WINDMILL_DIRN_VARS;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">

typedef struct
{
    int16_t
        *pVa,                   /* Pointer for phase A phase voltage */
        *pVb,                   /* Pointer for phase B phase voltage */
        *pVdc,                  /* Pointer for dc bus voltage */
        *pMeasureStatus,        /* Status of measurement available */
        qMaxSpeedFactor,        /* Maximum speed to peak speed ratio */
        Va,                     /* Phase A voltage */    
        Vb,                     /* Phase B voltage */
        Vdc,                    /* DC bus voltage */
        Vab,                    /* Line to line voltage */
        prevVab,                /* Previous value of line to line */
        direction,              /* Direction of windmilling */
        qSpeed,                 /* Rotor speed detected during windmilling */
        qMinimumSpeed,          /* Minimum speed for windmilling */
        cycles,                 /* Number of cycles for windmilling */
        qIqRef,                 /* Q axis current reference */
        qVelRef,                /* Speed reference */
        OmegaDtGain,            /* Scaling for inductive drop */
        rsIqs,                  /* Q-axis resistive voltage drop  */
        omegaFdr,               /* BEMF */
        omegaLqs,               /* Q-axis reactance */
        omegaLqsIqs,            /* Q-axis inductive voltage drop */
        vds,                    /* D-axis voltage reference */
        vqs,                    /* Q-axis voltage reference */
        mds,                    /* D-axis modulation index reference */
        mqs,                    /* Q-axis modulation index reference */
        periodCountsCycle,      /* Samples in one BEMF cycle */
        periodCountsCycleMin,   /* Minimum number of samples in cycle */
        periodCounts,           /* BEMF Sample counter */
        bemfLowCounts,          /* Counter for low BEMF */
        bemfHighCounts,         /* Counter for sufficient BEMF */
        zeroCrossCountStart,    /* Start the persistence counter */
        zeroCrossCounts,        /* Persistence for Zero Cross counter */
        state,                  /* Windmilling state */
        status;                 /* Windmilling is complete status */
    
    int32_t
        mdSum,                  /* Integrator accumulator for Vd */
        mqSum,                  /* Integrator accumulator for Vq */
        IqSum,                  /* Integrator accumulator for Iq */
        VelSum,                 /* Integrator accumulator for speed */
        thetaSum,               /* Integrator accumulator for angle */
        speedCalcNum;           /* Numerator for speed calculation */
     
}MCAPP_WINDMILL_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_WindMillInit(MCAPP_WINDMILL_T *);
void MCAPP_WindMill(MCAPP_WINDMILL_T *);

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* WINDMILL_H */

