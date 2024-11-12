// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file ipd.h
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



#ifndef IPD_H
#define	IPD_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">

/* No of times IPD cycle repeated */
#define IPD_CYCLES_MAX      1
/* Total number of active vectors */  
#define NUM_VECTORS         6
/* IPD Pulse ON Time duration in number of PWM Cycles */
#define IPD_ONTIME_TCY      1
/* IPD Pulse OFF Time duration in number of PWM Cycles */
#define IPD_OFFTTIME_TCY    10

// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="ENUMERATED CONSTANTS">
typedef enum
{
    IPD_INIT = 0,           /* Initialize IPD */
    IPD_ON = 1,             /* Pulses are turned on */
    IPD_OFF = 2,            /* Pulses are turned off */
    IPD_CALC_MAX_CURR = 3,  /* Calculate the maximum current */
    IPD_COMPLETE = 4,       /* IPD is complete */
}IPD_STATE_VARS;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">

typedef struct
{
    int16_t
        state,                      /* IPD State */
        status,
        motorTimeConstant,
        onTime,
        offTime,
        sixtyDegrees,
        onCounter,                     /* Pulse turn on counter */
        offCounter,                    /* Pulse turn off counter */        
        cycles,                     /* Number of cycles of IPD */
        *pBusCurrent,                    /* Bus current */
        startVoltageVector,                     /* Rotor position */
        initAngle,                  /* Initial rotor angle */
        maxCurrent,                    /* Maximum current */
        maxCurrentIndex,                /* Current index at max current */
        currentIndex,                   /* Current index */
        voltageVector,
        nextCurrentIndex,               /* Current Index after max current */
        prevCurrentIndex,               /* Current Index before max current */
        busCurrentVector[NUM_VECTORS];   /* Bus Current vector */
    
    /* Function pointers for motor inputs */    
    void (*MCAPP_HALSetVoltageVector) (int16_t);
    void (*MCAPP_HALDisablePWM) (void);
    
}MCAPP_IPD_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_IpdInit(MCAPP_IPD_T *);
void MCAPP_Ipd(MCAPP_IPD_T *);

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* IPD_H */

