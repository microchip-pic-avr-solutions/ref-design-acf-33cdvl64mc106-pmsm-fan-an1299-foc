// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file ipd.c
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

#include "ipd.h"
#include "pwm.h"

// </editor-fold>

/**
* <B> Function: void MCAPP_IpdInit(MCAPP_IPD_T *)  </B>
*
* @brief IPD initialization.
*
* @param Pointer to the data structure containing IPD parameters.
* @return none.
* @example
* <CODE> MCAPP_IpdInit(&ipd); </CODE>
*
*/
void MCAPP_IpdInit(MCAPP_IPD_T *pIpd)
{
    pIpd->onCounter = 0;
    pIpd->offCounter = 0;
    pIpd->cycles = 0;
    pIpd->status = 0;
    
    pIpd->maxCurrent = 0;
    pIpd->maxCurrentIndex = 0;
    
    pIpd->currentIndex = 0; 
}

/**
* <B> Function: void MCAPP_IpdIndexIncrement(int16_t *)  </B>
*
* @brief function to increment index to the IPD vectors.
*
* @param Pointer to the data structure containing IPD vector.
* @return none.
* @example
* <CODE> MCAPP_IpdIndexIncrement(&index); </CODE>
*
*/
void MCAPP_IpdIndexIncrement(int16_t *pIndex)
{
    /* Increments index by 1, wrapping around after NUM_VECTORS */
    *pIndex = *pIndex + 1;
    if(*pIndex >= NUM_VECTORS)
    {
        *pIndex = 0;
    }
}

/**
* <B> Function: void MCAPP_IpdIndexDecrement(int16_t *)  </B>
*
* @brief function to decrement index to the IPD vectors.
*
* @param Pointer to the data structure containing IPD vector.
* @return none.
* @example
* <CODE> MCAPP_IpdIndexDecrement(&index); </CODE>
*
*/
void MCAPP_IpdIndexDecrement(int16_t *pIndex)
{
    /* Decrements index by 1, wrapping around after 0 */
    *pIndex = *pIndex - 1;
    if(*pIndex < 0)
    {
        *pIndex = NUM_VECTORS - 1;
    }
}

/**
* <B> Function: void MCAPP_Ipd(MCAPP_IPD_T *)  </B>
*
* @brief IPD state machine.
*
* @param Pointer to the IPD data structure.
* @return none.
* @example
* <CODE> MCAPP_Ipd(&ipd); </CODE>
*
*/
void MCAPP_Ipd(MCAPP_IPD_T *pIpd)
{
    int16_t busCurrent = *pIpd->pBusCurrent;
    
    switch(pIpd->state)
    {
        case IPD_INIT:
            MCAPP_IpdInit(pIpd);
            
            if(pIpd->cycles >= IPD_CYCLES_MAX)
            {
                pIpd->state = IPD_COMPLETE;
            }
            else
            {
                pIpd->voltageVector = 1;
                pIpd->MCAPP_HALSetVoltageVector(pIpd->voltageVector);
                pIpd->state = IPD_ON;
            }    
            
        break;
            
        case IPD_ON:
            pIpd->onCounter++;
            
            if(pIpd->onCounter >= pIpd->onTime)
            {
                pIpd->onCounter = 0;
                pIpd->MCAPP_HALDisablePWM();
                
                pIpd->busCurrentVector[pIpd->currentIndex] = busCurrent;
                
                pIpd->state = IPD_OFF;
            }
        break;
            
        case IPD_OFF:
            pIpd->offCounter++;
            
            if(pIpd->offCounter >= pIpd->offTime)
            {
                pIpd->offCounter = 0;
                
                if(pIpd->voltageVector < NUM_VECTORS)
                {
                    pIpd->voltageVector++;
                }
                else
                {
                    pIpd->voltageVector = 1;
                    pIpd->cycles++;
                }
                
                pIpd->currentIndex = pIpd->voltageVector - 1;
                
                if(pIpd->cycles < IPD_CYCLES_MAX)
                {
                    pIpd->MCAPP_HALSetVoltageVector(pIpd->voltageVector);
                    pIpd->onCounter = 0;
                    pIpd->state = IPD_ON;
                }
                else
                {
                    pIpd->currentIndex = 0;
                    pIpd->maxCurrent = 0;
                    pIpd->maxCurrentIndex = 0;

                    pIpd->state = IPD_CALC_MAX_CURR;
                }
                
            }
        break;
        
        case IPD_CALC_MAX_CURR:
            
            if(pIpd->currentIndex < NUM_VECTORS)
            {
                if(pIpd->busCurrentVector[pIpd->currentIndex] >= 
                        pIpd->maxCurrent)
                {
                    pIpd->maxCurrent = 
                                pIpd->busCurrentVector[pIpd->currentIndex];
                    
                    pIpd->maxCurrentIndex = pIpd->currentIndex;
                    pIpd->nextCurrentIndex = pIpd->currentIndex;
                    pIpd->prevCurrentIndex = pIpd->currentIndex;
                }
            }
            else
            {
                MCAPP_IpdIndexIncrement(&(pIpd->nextCurrentIndex));
                MCAPP_IpdIndexDecrement(&(pIpd->prevCurrentIndex));
                
                if(pIpd->busCurrentVector[pIpd->nextCurrentIndex] > 
                        pIpd->busCurrentVector[pIpd->prevCurrentIndex])
                {
                    pIpd->startVoltageVector = pIpd->nextCurrentIndex;                    
                }
                else
                {
                    pIpd->startVoltageVector = pIpd->maxCurrentIndex;
                }
                
                pIpd->initAngle = (pIpd->startVoltageVector * pIpd->sixtyDegrees)
                             - (pIpd->sixtyDegrees>>1);
                pIpd->state = IPD_COMPLETE;
            }
            pIpd->currentIndex++;
        break;
        
        case IPD_COMPLETE:
            pIpd->status = 1;
        break;
        
        default:
            pIpd->state = IPD_INIT;
        break;
    }
}