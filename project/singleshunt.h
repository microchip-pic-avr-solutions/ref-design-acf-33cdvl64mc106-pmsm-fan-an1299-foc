// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fan.h
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

#ifndef __SINGLESHUNT_H
#define	__SINGLESHUNT_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h> 
#include "clock.h"
#include "motor_control_noinline.h"
    
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">

/*  Critical Minimum window in seconds to measure current through single shunt*/
#define SSTCRITINSEC	3.0E-6
/* Single shunt algorithm defines *2 because is same resolution as PDCx registers */
#define SSTCRIT         (uint16_t)(SSTCRITINSEC*FCY*2)
/* Single shunt algorithm defines *2 because is same resolution as PDCx registers */
#define SS_SAMPLE_DELAY  175

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">
 /* Description:
    This structure will host parameters related to single shunt
 */
typedef struct
{
    int16_t T1;
    int16_t T2;
    int16_t T7;
    int16_t Ta1;
    int16_t Ta2;
    int16_t Tb1;
    int16_t Tb2;
    int16_t Tc1;
    int16_t Tc2;
    int16_t sectorSVM;			// Variable indicating the active sector in which space
                                // vector modulation is in. The Sector value is used to
                                // identify which duty cycle registers to be modified
                                // in order to measure current through single shunt
    int16_t tcrit;			// variable used to create minimum window to measure
                                // current through single shunt resistor when enabled
                                // Define minimum window in UserParms.h, SSTCRITINSEC
    int16_t tDelaySample;		// variable used to create a delay for single shunt
                                // current measurement. Delay is defined in seconds in
                                // the actual dead time definition in UserParms.h,
                                // DEADTIMESEC
    
    int16_t Ia;				/* Reconstructed value for Ia */
    int16_t Ib;				/* Reconstructed value for Ib */
    int16_t Ic;				/* Reconstructed value for Ic */
    int16_t Ibus1;
    int16_t Ibus2;			
    int16_t trigger1;       /* This variable holds the first trigger value
                               to be stored in TRIG1 register for
                               A/D conversion. The converted value will be used
                               for current reconstruction later on. */
    int16_t trigger2;       /* Variable holding the second trigger value to
                               be stored in TRIG1 register to trigger
                               A/D conversion */
    int16_t adcSamplePoint;
    MC_DUTYCYCLEOUT_T pwmDutycycle1;
    MC_DUTYCYCLEOUT_T pwmDutycycle2;
    
} SINGLE_SHUNT_PARM_T;

typedef enum tagSSADCSAMPLE_STATE
{ 
    SS_SAMPLE_BUS1 = 0,     /* Bus Current Sample1 */
    SS_SAMPLE_BUS2 = 1,     /* Bus Current Sample2 */   
     
}SSADCSAMPLE_STATE;

extern SINGLE_SHUNT_PARM_T singleShuntParam;    

// </editor-fold>
   
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

uint16_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *pABC,
                                                     uint16_t iPwmPeriod,
                                                     SINGLE_SHUNT_PARM_T *);
void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *);
void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *);
void ResetSingleShuntSamplePoint(SINGLE_SHUNT_PARM_T *);
    
// </editor-fold>
    
#ifdef	__cplusplus
}
#endif

#endif	/* SINGLESHUNT_H */
