// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fan.c
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

#include "fan.h"
#include <libq.h>
#include "motor_control_noinline.h"
#include "estim.h"
#include "control.h"
#include "userparms.h"
#include "control.h"
#include "singleshunt.h"

// </editor-fold>
extern int16_t Trigger_Check;
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_FanStateMachine (MCAPP_FAN_T *);
void MCAPP_FanInit (MCAPP_FAN_T *);
void MCAPP_FanConfig (MCAPP_FAN_T *);
void MCAPP_MC1LoadStartTransition (MCAPP_FAN_T *);
void MCAPP_BrakeCountMaxLimit (MCAPP_FAN_T *);
// </editor-fold>

/**
* <B> Function: void MCAPP_FanConfig(MCAPP_FAN_T *)  </B>
*
* @brief Function to configure and initialize variables in Fan structure.
*
* @param Pointer to the data structure containing Fan Load parameters.
* @return none.
* @example
* <CODE> MCAPP_FanConfig(&fan); </CODE>
*
*/
void MCAPP_FanConfig (MCAPP_FAN_T *pFan)
{
    pFan->enableWindmill = WINDMILL_FUNCTIONALITY;
    pFan->enableBrake = BRAKE_FUNCTIONALITY;
    pFan->enableIPD = IPD_FUNCTIONALITY;
    
    pFan->pWindmill = &pFan->windMill;
    pFan->pBrake = &pFan->brake;
    pFan->pIpd = &pFan->ipd;
    
    pFan->MCAPP_HALSetVoltageVector = HAL_MC1SetVoltageVector;
    
    /* Windmill Configuration */
    pFan->windMill.pVa  = &measureInputs.measurePhaseVolt.Va;
    pFan->windMill.pVb  = &measureInputs.measurePhaseVolt.Vb;
    pFan->windMill.pVdc = &measureInputs.dcBusVoltage;
    
    pFan->windMill.OmegaDtGain = MC1_WBASE_DT_LSGAIN;

    pFan->windMill.qMinimumSpeed = WINDMILL_MIN_SPEED_PU;
    

    pFan->windMill.pMeasureStatus = &measureInputs.measurePhaseVolt.status;

    pFan->windMill.speedCalcNum = (int32_t)((32768*(int32_t)(PWMFREQUENCY_HZ))/
                            (MC1_BEMF_SAMPLING_FACTOR*POLE_PAIRS*(int32_t)(MC1_PEAK_SPEED_RPM)))*60;

    pFan->windMill.periodCountsCycleMin = 
                (int16_t)(__builtin_divsd(pFan->windMill.speedCalcNum, 
                            Q15_MAXIMUMSPEED));

    pFan->windMill.thetaSum =  (int32_t)(FIVE_PI_BY_SIX<<15);
    
    /* Brake Configuration */
    pFan->brake.brakeOnCountMax = BRAKE_ON_COUNT_LOW;
    /* IPD Configuration*/
    pFan->ipd.onTime = IPD_ONTIME_TCY;
    
    pFan->ipd.offTime = IPD_OFFTTIME_TCY;
    
    pFan->ipd.sixtyDegrees = PI_BY_THREE;
    
    pFan->pIpd->pBusCurrent = &measureInputs.current.Ibus;
    
    pFan->ipd.MCAPP_HALSetVoltageVector = HAL_MC1SetVoltageVector;
    pFan->ipd.MCAPP_HALDisablePWM = DisablePWMOutputs;
}

/**
* <B> Function: void MCAPP_FanInit(MCAPP_FAN_T *)  </B>
*
* @brief Fan initialization.
*
* @param Pointer to the data structure containing load parameters.
* @return none.
* @example
* <CODE> MCAPP_FanInit(&fan); </CODE>
*
*/
void MCAPP_FanInit (MCAPP_FAN_T *pFan)
{
    MCAPP_WindMillInit(pFan->pWindmill);
    MCAPP_BrakeInit(pFan->pBrake);
    MCAPP_IpdInit(pFan->pIpd);
    
    pFan->pWindmill->state = WM_CHECK_WINDMILL;
    
    pFan->pIpd->state = IPD_INIT;
    
    pFan->startStatus = FAN_START_GENERIC;
    
    if(pFan->enableWindmill == 1)
    {
        pFan->state = FAN_WINDMILL_CHECK;
    }
    else if(pFan->enableBrake == 1)
    {
        pFan->state = FAN_BRAKE;
    }
    else if(pFan->enableIPD == 1)
    {
        pFan->state = FAN_IPD;
    }
    else
    {
        pFan->state = FAN_RUN;
    }
}

/**
* <B> Function: void MCAPP_FanStateMachine(MCAPP_FAN_T *)  </B>
*
* @brief Fan state machine.
*
* @param Pointer to the data structure containing load parameters.
* @return none.
* @example
* <CODE> MCAPP_FanStateMachine(&fan); </CODE>
*
*/
void MCAPP_FanStateMachine (MCAPP_FAN_T *pFan)
{
    MCAPP_WINDMILL_T *pWindmill = pFan->pWindmill;
    MCAPP_BRAKE_T *pBrake = pFan->pBrake;
    MCAPP_IPD_T *pIpd = pFan->pIpd;
    
    switch(pFan->state)
    {            
        case FAN_WINDMILL_CHECK:
            MCAPP_WindMill(pWindmill);
            
            if(pWindmill->status)
            {
                switch(pWindmill->direction)
                {
                    case WM_DIRECTION_UNIDENTIFIED:
                        if(pFan->enableBrake == 1)
                        {
                            pFan->state = FAN_BRAKE;
                        }
                        else if(pFan->enableIPD == 1)
                        {
                            pFan->state = FAN_IPD;
                        }
                        else
                        {
                            pFan->state = FAN_RUN;
                        }
                        pFan->startStatus = FAN_START_GENERIC;
                        MCAPP_BrakeCountMaxLimit (pFan);
                    break;
                    case WM_DIRECTION_STOP:
                        if(pFan->enableBrake == 1)
                        {
                            pFan->state = FAN_BRAKE;
                        }
                        else if(pFan->enableIPD == 1)
                        {
                            pFan->state = FAN_IPD;
                        }
                        else
                        {
                            pFan->state = FAN_RUN;
                        }
                        pFan->startStatus = FAN_START_GENERIC;
                        MCAPP_BrakeCountMaxLimit (pFan);
                    break;
                    case WM_DIRECTION_REVERSE:
                        if(pFan->enableBrake == 1)
                        {
                            pFan->state = FAN_BRAKE;
                        }
                        else if(pFan->enableIPD == 1)
                        {
                            pFan->state = FAN_IPD;
                        }
                        else
                        {
                            pFan->state = FAN_RUN;
                        }
                        pFan->startStatus = FAN_START_GENERIC;
                        MCAPP_BrakeCountMaxLimit (pFan);
                        
                    break;
                    case WM_DIRECTION_FORWARD:
                        pFan->state = FAN_RUN;
                        pFan->startStatus = FAN_START_WINDMILL;

                    break;
                    
                    default:
                        pWindmill->direction = WM_DIRECTION_UNIDENTIFIED;
                    break;
                }
            }            
        break;
            
        case FAN_BRAKE:
            pFan->MCAPP_HALSetVoltageVector(0);
            MCAPP_Brake(pBrake);

            if(pBrake->status)
            {
                if(pFan->enableIPD == 1)
                {
                    pFan->state = FAN_IPD;
                }
                else
                {
                    pFan->state = FAN_RUN;
                }
            }
        break;
        
        case FAN_IPD:
            MCAPP_Ipd(pIpd);
            
            if(pFan->pIpd->status)
            {
                pFan->startStatus = FAN_START_IPD;
                pFan->state = FAN_RUN;
            }
            
        break;
            
        case FAN_RUN:
        break;
            
        case FAN_STOP:
        break;
            
        case FAN_FAULT:            
        break;
            
        default:
            pFan->state = FAN_FAULT;
        break;
    }
}

/**
* <B> Function: void MCAPP_IsFanReadyToStart(MCAPP_FAN_T *)  
* </B>
*
* @brief Load Ready to Start Interface Function.
*
* @param Pointer to the data structure containing load parameters.
* @return Ready to start Flag.
* @example
* <CODE> MCAPP_IsFanReadyToStart(&fan); </CODE>
*
*/
int16_t MCAPP_IsFanReadyToStart (MCAPP_FAN_T *pFan)
{
    int16_t status;
    
    if(pFan->state == FAN_RUN)
    {
        status = 1;        
    }
    else
    {
        status = 0;
    }
    
    return status;
}

/**
* <B> Function: void MCAPP_MC1LoadStartTransition (MCAPP_FAN_T *pLoad)  
* </B>
*
* @brief Load Ready to Start Interface Function.
*
* @param Pointer to the data structure containing load parameters.
* @return Ready to start Flag.
* @example
* <CODE> MCAPP_MC1LoadStartTransition(&fan); </CODE>
*
*/
void MCAPP_MC1LoadStartTransition(MCAPP_FAN_T *pLoad)
{
    MCAPP_WINDMILL_T *pWindmill = pLoad->pWindmill;
    MCAPP_IPD_T *pIpd = pLoad->pIpd;
    
    switch(pLoad->startStatus)
    {
        case FAN_START_GENERIC:
            ctrlParm.focState = FOC_RTR_LOCK;
            break;
            
        case FAN_START_WINDMILL:  
            uGF.bits.OpenLoop = 0;
            uGF.bits.ChangeMode = 0;
            ctrlParm.qVelRef = pWindmill->qVelRef;
            ctrlParm.qVdRef = 0;
            ctrlParm.qVqRef = pWindmill->qIqRef;
            idq.q = ctrlParm.qVqRef;
            idq.d = 0;
            vdq.q = pWindmill->vqs;
            vdq.d = pWindmill->vds;
            piInputId.piState.integrator  = pWindmill->mdSum;
            piInputIq.piState.integrator = pWindmill->mqSum;
            piInputOmega.piState.integrator = pWindmill->IqSum;

            estimator.qVelEstimStateVar = pWindmill->VelSum;
            estimator.qRhoStateVar = pWindmill->thetaSum;
            MC_CalculateSineCosine_Assembly_Ram(FIVE_PI_BY_SIX,&sincosTheta);
            MC_TransformParkInverse_Assembly(&idq,&sincosTheta,&ialphabeta);
            MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);
            estimator.qRhoOffset = 0;
            estimator.qEsdStateVar  = 0;
            estimator.qEsqStateVar = ((int32_t)pWindmill->omegaFdr <<15);
            ctrlParm.focState = FOC_CLOSE_LOOP;
        break;
            
        case FAN_START_IPD:
            thetaElectricalOpenLoop = 
                    (int16_t)pIpd->initAngle;            
            ctrlParm.focState = FOC_OPEN_LOOP;
            motorStartUpData.startupLock = LOCK_TIME;
        break;
            
        default:
            pLoad->startStatus = FAN_START_GENERIC;

    }
}
/**
* <B> Function: void MCAPP_BrakeCountMaxLimit(MCAPP_FAN_T *)  </B>
*
* @brief Function to limit the braking time.
*
* @param Pointer to the data structure containing Fan Load parameters.
* @return none.
* @example
* <CODE> MCAPP_BrakeCountMaxLimit(&fan); </CODE>
*
*/
void MCAPP_BrakeCountMaxLimit (MCAPP_FAN_T *pFan)
{
    MCAPP_WINDMILL_T *pWindmill = pFan->pWindmill;
    MCAPP_BRAKE_T *pBrake = pFan->pBrake;
    
    if(pWindmill->qVelRef < ((pWindmill->qMinimumSpeed) >>2))
    {
       pBrake->brakeOnCountMax = BRAKE_ON_COUNT_LOW; 
    }   
    else
    {
       pBrake->brakeOnCountMax = BRAKE_ON_COUNT_HIGH;
    }
}