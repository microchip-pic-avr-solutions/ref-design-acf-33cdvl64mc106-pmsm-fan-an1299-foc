// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file fan_types.h
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



#ifndef FAN_TYPES_H
#define	FAN_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>
#include <stdbool.h>
    
#include "windmill.h"
#include "brake.h"
#include "ipd.h"
    
// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="ENUMERATED CONSTANTS ">

typedef enum
{
    FAN_WINDMILL_CHECK = 1,         /* Check for windmill */
    FAN_BRAKE = 2,                  /* Brake the fan */
    FAN_IPD = 3,                    /* Detect initial position */
    FAN_RUN = 4,                    /* Fan is running */
    FAN_STOP = 5,                   /* Fan is stopped */
    FAN_FAULT = 6,                  /* Motor is in Fault mode */

}MCAPP_FAN_STATE_T;

typedef enum
{
    FAN_START_GENERIC = 0,
    FAN_START_WINDMILL = 1,
    FAN_START_IPD = 2,
}MCAPP_FAN_START_STATUS;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    int16_t
        state,
        startStatus,
        enableWindmill,
        enableBrake,
        enableIPD;
    
    MCAPP_WINDMILL_T
        windMill;
    
    MCAPP_BRAKE_T
        brake;
    
    MCAPP_IPD_T
        ipd;
    
    MCAPP_WINDMILL_T
        *pWindmill;
    
    MCAPP_BRAKE_T
        *pBrake;
    
    MCAPP_IPD_T
        *pIpd;
    
    /* Function pointers for motor inputs */    
    void (*MCAPP_HALSetVoltageVector) (int16_t);
    
}MCAPP_FAN_T;

// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* FAN_TYPES_H */

