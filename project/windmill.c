// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file windmill.c
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

#include "windmill.h"
#include "estim.h"
#include "hal/board_service.h"
#include "hal/measure.h"

// </editor-fold>
uint16_t Check_Windmill_min_backemf,Check_low_count,Check_high_count,Check_zero_cross,Check_Speed_Calc;
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_WindMillInit(MCAPP_WINDMILL_T *);
void MCAPP_WindMill(MCAPP_WINDMILL_T *);
static int16_t MCAPP_BEMFPositiveZeroCrossDetect(MCAPP_WINDMILL_T *);
// </editor-fold>
int16_t Trigger_Check =0;
/**
* <B> Function: void MCAPP_WindMillInit(MCAPP_WINDMILL_T *)  </B>
*
* @brief WIndmill initialization.
*
* @param Pointer to the data structure containing windmill parameters.
* @return none.
* @example
* <CODE> MCAPP_WindMillInit(&windmill); </CODE>
*
*/
void MCAPP_WindMillInit(MCAPP_WINDMILL_T *pWindmill)
{
   
    
    /* Initialize counters and speed */
    pWindmill->periodCounts = 0;
    pWindmill->bemfLowCounts = 0;
    pWindmill->bemfHighCounts = 0;
    pWindmill->zeroCrossCounts = 0;
    pWindmill->qSpeed = 0;
    pWindmill->qVelRef = 0;
    pWindmill->cycles = 0;
    pWindmill->periodCountsCycle = 0;
    pWindmill->direction = WM_DIRECTION_UNIDENTIFIED;
    pWindmill->status = 0;
    Trigger_Check =0;
}
/**
* <B> Function: void MCAPP_BEMFPositiveZeroCrossDetect(MCAPP_WINDMILL_T *)  </B>
*
* @brief Back EMF Zero Cross Detection.
*
* @param Pointer to the data structure containing windmill parameters.
* @return status of zero cross detection.
* @example
* <CODE> MCAPP_BEMFPositiveZeroCrossDetect(&windmill); </CODE>
*
*/
static int16_t MCAPP_BEMFPositiveZeroCrossDetect(MCAPP_WINDMILL_T *pWindmill)
{
    int16_t zeroCrossDetect;
    
    if((pWindmill->prevVab <= 0) && 
            (pWindmill->Vab > 0) &&
            (pWindmill->zeroCrossCountStart == 0))
    {
        zeroCrossDetect = 1;
        pWindmill->zeroCrossCountStart = 1;
    }
    else
    {
        zeroCrossDetect = 0;
    }
    
    if(pWindmill->zeroCrossCountStart == 1)
    {
        pWindmill->zeroCrossCounts++;
        if(pWindmill->zeroCrossCounts >= WINDMILL_ZERO_CROSS_PERSIST)
        {
            pWindmill->zeroCrossCounts = 0;
            pWindmill->zeroCrossCountStart = 0;
        }
    }
    
    return zeroCrossDetect;
}
/**
* <B> Function: void MCAPP_WindMill(MCAPP_WINDMILL_T *)  </B>
*
* @brief Windmill state machine.
*
* @param Pointer to the data structure containing windmill parameters.
* @return none.
* @example
* <CODE> MCAPP_WindMill(&windmill); </CODE>
*
*/
void MCAPP_WindMill(MCAPP_WINDMILL_T *pWindmill)
{
    int16_t tmp;
    int32_t tmp_iqref;
    int16_t tmp_dirCheck_bemf;
    
    
    if(*pWindmill->pMeasureStatus == 1)
    {
        *pWindmill->pMeasureStatus = 0;
        
        pWindmill->prevVab = pWindmill->Vab;
        pWindmill->Vab = *pWindmill->pVa - *pWindmill->pVb;

        pWindmill->Va = *pWindmill->pVa;
        pWindmill->Vb = *pWindmill->pVb;

        /* Windmilling State Machine */
        switch(pWindmill->state)
        {
            case WM_INIT:
                MCAPP_WindMillInit(pWindmill);
                pWindmill->state = WM_CHECK_WINDMILL;

            break;

            case WM_CHECK_WINDMILL:
                /* Check if motor is windmilling */
                if(*pWindmill->pVa < WINDMILL_MIN_BEMF)
                {
                    /* BEMF is too low */
                    pWindmill->bemfLowCounts++;
                    pWindmill->bemfHighCounts = 0;

                    if(pWindmill->bemfLowCounts >= 
                            WINDMILL_LOW_BEMF_COUNT_MAX)
                    {
                        /* BEMF is too low for significant time. 
                         * Motor not windmilling */
                        pWindmill->direction = WM_DIRECTION_STOP;
                        pWindmill->state = WM_COMPLETE;
                    }
                }
                else
                {
                    /* BEMF has become big enough */
                    pWindmill->bemfHighCounts++;
                    pWindmill->bemfLowCounts = 0;
                    
                    if(pWindmill->bemfHighCounts >= 
                            WINDMILL_HIGH_BEMF_COUNT_MAX)
                    {
                        pWindmill->state = WM_CALCULATE_WINDMILL_SPEED;
                    }
                }
            break;

            case WM_CALCULATE_WINDMILL_SPEED:
                pWindmill->periodCounts++;

                if(MCAPP_BEMFPositiveZeroCrossDetect(pWindmill))
                {
                    /* Positive zero crossing of AB detected */

                    pWindmill->cycles++;
                    
                    pWindmill->periodCountsCycle = pWindmill->periodCounts;
                    pWindmill->periodCounts = 0;
                    
                    if(pWindmill->periodCountsCycle < 
                            pWindmill->periodCountsCycleMin)
                    {
                        pWindmill->periodCountsCycle = 
                                pWindmill->periodCountsCycleMin;
                    }
                    pWindmill->qVelRef = 
                            (__builtin_divsd(pWindmill->speedCalcNum, 
                            pWindmill->periodCountsCycle));
                    
                }
                if(pWindmill->cycles >= WINDMILL_CYCLES_MAX)
                {                
                    if(pWindmill->qVelRef < pWindmill->qMinimumSpeed)
                    {
                        pWindmill->direction = WM_DIRECTION_STOP;
                        pWindmill->state = WM_COMPLETE;
                    }
                    else
                    {
                        pWindmill->qSpeed = pWindmill->qVelRef;
                        
                        pWindmill->state = WM_CHECK_WINDMILL_DIRECTION;
                    }
                }
            break;

            case WM_CHECK_WINDMILL_DIRECTION:

                /* Motor is windmilling. Determine direction */
                if(MCAPP_BEMFPositiveZeroCrossDetect(pWindmill))
                {
                     tmp_dirCheck_bemf= (__builtin_mulss(pWindmill->qSpeed, 
                                        WINDMILL_BEMF_CONST_Q15))>>15;
                    /* Zero crossing of BEMF_AB is detected. Possible when
                     * BEMF_A = BEMF_B = 0, or BEMF_A = BEMF_B != 0
                     * If BEMF_A = BEMF_B = 0, it is forward direction
                     * If BEMF_A = BEMF_B != 0, it is in reverse direction */
                    if(*pWindmill->pVa < ((tmp_dirCheck_bemf>>2)+ PHASE_VOLTAGE_OFFSET))
                    {
                        pWindmill->direction = WM_DIRECTION_FORWARD;
                        pWindmill->state = WM_CALCULATE_CURRENT_REF;
                    }
                    else
                    {
                        if(*pWindmill->pVa > ((tmp_dirCheck_bemf>>1)+ PHASE_VOLTAGE_OFFSET))
                        {
                            pWindmill->direction = WM_DIRECTION_REVERSE;
                            
                            pWindmill->state = WM_COMPLETE;
                        }
                        else
                        {
                            pWindmill->direction = WM_DIRECTION_UNIDENTIFIED;
                            pWindmill->state = WM_COMPLETE;
                        }
                    }
                }
            break;

            case WM_CALCULATE_CURRENT_REF:
                
                tmp = (int16_t)(__builtin_mulss(pWindmill->qSpeed, 
                                        pWindmill->qSpeed) >> 15);
                tmp_iqref = __builtin_mulss(tmp, 
                                        motorParm.qRatedPeakCurrent);
                tmp = (int16_t)(__builtin_mulss((int16_t)Q15_NOMINALSPEED, 
                                        (int16_t)Q15_NOMINALSPEED) >> 15);
                pWindmill->qIqRef = (int16_t)(__builtin_divsd(tmp_iqref,tmp));

                pWindmill->IqSum = ((int32_t)pWindmill->qIqRef) << 16;
                pWindmill->VelSum = ((int32_t)pWindmill->qSpeed) << 15;

                pWindmill->state = WM_CALCULATE_VOLTAGE_REF;
            break;

            case WM_CALCULATE_VOLTAGE_REF:
                /* rsIqs in Vbase/2^15 scaling */
                pWindmill->rsIqs = (int16_t)(__builtin_mulss(pWindmill->qIqRef, 
                                        motorParm.qRs) >> motorParm.qRsScale);

                /* omegaLqs in NORM_LS_SCALE */
                pWindmill->omegaLqs = 
                                (int16_t)(__builtin_mulss(pWindmill->qVelRef, 
                                                    motorParm.qLsDt) >> 15);
                pWindmill->omegaLqs = 
                                (int16_t)(__builtin_mulss(pWindmill->omegaLqs, 
                                                pWindmill->OmegaDtGain) >> 15);

                /* omegaLqsIqs in Vbase/2^15 scaling */
                pWindmill->omegaLqsIqs = 
                        (int16_t)(__builtin_mulss(pWindmill->omegaLqs, 
                            pWindmill->qIqRef) >> motorParm.qLsDtScale);

                /* omegaFdr in Vbase/2^15 scaling */
                pWindmill->omegaFdr = 
                        (int16_t)((__builtin_divsd((__builtin_mulss(pWindmill->qVelRef,32767)), 
                                    motorParm.qInvKFiBase)) >> 
                                       motorParm.qInvKFiBaseScale);

                pWindmill->vds = -pWindmill->omegaLqsIqs;
                pWindmill->vqs = pWindmill->rsIqs + pWindmill->omegaFdr;

                pWindmill->state = WM_CALCULATE_MOD_INDEX_REF;
            break;

            case WM_CALCULATE_MOD_INDEX_REF:
                pWindmill->mds = (__builtin_divf(pWindmill->vds,measureInputs.dcBusVoltage ));

                pWindmill->mqs = (__builtin_divf(pWindmill->vqs,measureInputs.dcBusVoltage ));

                pWindmill->mdSum = ((int32_t)pWindmill->mds) << 16;
                pWindmill->mqSum = ((int32_t)pWindmill->mqs) << 16;

                pWindmill->state = WM_WAIT_FOR_COMPLETE;
            break;

            case WM_WAIT_FOR_COMPLETE:
                    if(MCAPP_BEMFPositiveZeroCrossDetect(pWindmill))
                    {
                        pWindmill->state = WM_COMPLETE;
                    }
            break;

            case WM_COMPLETE:
                pWindmill->status = 1;
            break;

            default:
                pWindmill->state = WM_INIT;
            break;
        }
    }
}