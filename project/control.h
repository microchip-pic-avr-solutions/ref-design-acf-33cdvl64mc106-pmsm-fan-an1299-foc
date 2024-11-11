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

#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>

typedef enum
{
    FOC_INIT = 0,              /* Initialize control parameters */
    FOC_RTR_LOCK = 1,          /* Rotor Lock */
    FOC_OPEN_LOOP = 2,         /* Open Loop */
    FOC_CLOSE_LOOP = 3,        /* Closed Loop */
    FOC_FAULT = 4,             /* Motor is in Fault */

}FOC_CONTROL_STATE_T;    
/* Control Parameter data type

  Description:
    This structure will host parameters related to application control
    parameters.
 */
typedef struct
{
    /* Reference velocity */
    int16_t   qVelRef;
    /* Vd flux reference value */
    int16_t   qVdRef;
    /* Vq torque reference value */
    int16_t   qVqRef;
    /* Ramp for speed reference value for Acceleration */
    int16_t   qRefRampUp;
    /* Ramp for speed reference value for Deceleration*/
    int16_t   qRefRampDown;
    /* Speed of the ramp */
    int16_t   qDiff;
    /* Target Speed*/
    int16_t  qtargetSpeed;
    /* The Speed Control Loop will be executed only every speedRampCount*/
    int16_t   speedRampCount;  
    /*foc state*/
    int16_t focState;
    /* Speed reference */
    int16_t qAnRef;
} CTRL_PARM_T;
/* Motor Parameter data type

  Description:
    This structure will host parameters related to motor parameters.
*/
typedef struct
{
    /* Start up ramp in open loop. */
    uint32_t startupRamp;
    /* Start up Ramp Count*/
    uint16_t startupRampCount;
    /* counter that is incremented in CalculateParkAngle() up to LOCK_TIME,*/
    uint16_t startupLock;
    /* Start up ramp increment */
    uint16_t tuningAddRampup;	
    uint16_t tuningDelayRampup;
    uint16_t startUpRampCount;
} MOTOR_STARTUP_DATA_T;

typedef struct
{
    /* stop Delay Count */
    uint32_t stopDelayCount;
    /* stop Delay Count Limit*/
    uint32_t stopDelayCountLimit;
} MOTOR_STOP_DATA_T;

/* General system flag data type

  Description:
    This structure will host parameters related to application system flags.
 */
typedef union
{
    struct
    {
        /* Run motor indication */
        unsigned RunMotor:1;
        /* Open loop/closed loop indication */
        unsigned OpenLoop:1;
        /* Mode changed indication - from open to closed loop */
        unsigned ChangeMode:1;
        /* Speed doubled indication */
        unsigned ChangeSpeed:1;
        /* Stop Command*/
        unsigned StopCommand:1;
       /* Unused bits */
        unsigned    :11;
    } bits;
    uint16_t Word;
} UGF_T;

extern CTRL_PARM_T ctrlParm;
extern MOTOR_STARTUP_DATA_T motorStartUpData;
extern MOTOR_STOP_DATA_T motorStopData;
extern MC_ALPHABETA_T valphabeta,ialphabeta;
extern MC_ALPHABETA_T valphabeta,ialphabeta;
extern MC_SINCOS_T sincosTheta;
extern MC_DQ_T vdq,idq;
extern MC_DUTYCYCLEOUT_T pwmDutycycle;
extern MC_ABC_T   vabc,iabc;
extern UGF_T uGF;
extern MC_PIPARMIN_T piInputIq;
extern MC_PIPARMOUT_T piOutputIq;
extern MC_PIPARMIN_T piInputId;
extern MC_PIPARMOUT_T piOutputId;
extern MC_PIPARMIN_T piInputOmega;
extern MC_PIPARMOUT_T piOutputOmega;
extern int16_t thetaElectricalOpenLoop;

#ifdef __cplusplus
}
#endif

#endif /* __CONTORL_H */
