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

#include <libq.h>
#include "motor_control_noinline.h"
#include "userparms.h"
#include "estim.h"
#include "control.h"
#include "measure.h"

#define DECIMATE_NOMINAL_SPEED    NORM_VALUE(NOMINAL_SPEED_RPM/10,MC1_PEAK_SPEED_RPM)

/** Variables */
ESTIM_PARM_T estimator;
MOTOR_ESTIM_PARM_T motorParm;
MC_ALPHABETA_T bemfAlphaBeta;
MC_DQ_T bemfdq;
MC_SINCOS_T sincosThetaEstimator;

// *****************************************************************************

/* Function:
    Estim()

  Summary:
    Motor speed and angle estimator

  Description:
    Estimation of the speed of the motor and field angle based on inverter
    voltages and motor currents.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Estim(void) 
{
    int32_t tempint;
    uint16_t index = (estimator.qDiCounter - 3)&0x0003;

    /* dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
       For lower speed the granularity of difference is higher - the
       difference is made between 2 sampled values @ 8 ADC ISR cycles */

        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (estimator.qDIalpha > estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = estimator.qDIlimitLS;
        }
        if (estimator.qDIalpha<-estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitLS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIalpha) >> NORM_LSDTBASE_SCALE_SHIFT);

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR cycle
           the following limitation assures a limitation per low speed - up to
           the nominal speed */
        if (estimator.qDIbeta > estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = estimator.qDIlimitLS;
        }
        if (estimator.qDIbeta<-estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitLS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIbeta) >> NORM_LSDTBASE_SCALE_SHIFT);

   
    /* Update  LastIalpha and LastIbeta */
    estimator.qDiCounter = (estimator.qDiCounter + 1) & 0x0003;
    estimator.qLastIalphaHS[estimator.qDiCounter] = ialphabeta.alpha;
    estimator.qLastIbetaHS[estimator.qDiCounter] = ialphabeta.beta;

    estimator.qValpha = (int16_t) (__builtin_mulss(valphabeta.alpha, 
                                  measureInputs.dcBusVoltage) >> 15);
    
    estimator.qVbeta = (int16_t) (__builtin_mulss(valphabeta.beta, 
                                  measureInputs.dcBusVoltage) >> 15);
    /* Stator voltage equations
     Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
     BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt */
    
    bemfAlphaBeta.alpha =  estimator.qValpha -
                        (int16_t) (__builtin_mulss(motorParm.qRs, 
                                  ialphabeta.alpha) >> NORM_RS_SCALE_SHIFT) -
                        estimator.qVIndalpha;

    /* The multiplication between the Rs and Ialpha was shifted by 14 instead
       of 15 because the Rs value normalized exceeded Q15 range, so it was
       divided by 2 immediately after the normalization - in userparms.h */

    /* Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
       BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt */
    bemfAlphaBeta.beta =   estimator.qVbeta -
                        (int16_t) (__builtin_mulss(motorParm.qRs,
                                 ialphabeta.beta) >> NORM_RS_SCALE_SHIFT) -
                        estimator.qVIndbeta;

    /* The multiplication between the Rs and Ibeta was shifted by 14 instead of 15
     because the Rs value normalized exceeded Q15 range, so it was divided by 2
     immediately after the normalization - in userparms.h */
    MC_CalculateSineCosine_Assembly_Ram((estimator.qRho),
                                        &sincosThetaEstimator);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&bemfAlphaBeta, &sincosThetaEstimator, &bemfdq);

    /* Filter first order for Esd and Esq
       EsdFilter = 1/TFilterd * Integral{ (Esd-EsdFilter).dt } */
    tempint = (int16_t) (bemfdq.d - estimator.qEsdf);
    estimator.qEsdStateVar += __builtin_mulss(tempint, estimator.qKfilterEsdq);
    estimator.qEsdf = (int16_t) (estimator.qEsdStateVar >> 15);

    tempint = (int16_t) (bemfdq.q - estimator.qEsqf);
    estimator.qEsqStateVar += __builtin_mulss(tempint, estimator.qKfilterEsdq);
    estimator.qEsqf = (int16_t) (estimator.qEsqStateVar >> 15);

    /* OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf)
       For stability the condition for low speed */
    if (_Q15abs(estimator.qVelEstim) > DECIMATE_NOMINAL_SPEED) 
    {
        if (estimator.qEsqf > 0) 
        {
            tempint = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr =  (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        } 
        else 
        {
            tempint = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        }
    }        
    /* if estimator speed<10% => condition VelRef<>0 */
    else 
    {
        if (estimator.qVelEstim > 0) 
        {
            tempint = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        } 
        else 
        {
            tempint = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        }
    }
    /* The result of the calculation above is shifted left by one because
       initial value of InvKfi was shifted by 2 after normalizing -
       assuring that extended range of the variable is possible in the
       lookup table the initial value of InvKfi is defined in userparms.h */
    estimator.qOmegaMr = estimator.qOmegaMr << NORM_INVKFIBASE_SCALE;
    
    /* the integral of the angle is the estimated angle */
    estimator.qRhoStateVar += __builtin_mulss(estimator.qOmegaMr,
                                estimator.qDeltaT);
    estimator.qRho = (int16_t) (estimator.qRhoStateVar >> 15);


    /* The estimated speed is a filter value of the above calculated OmegaMr.
       The filter implementation is the same as for BEMF d-q components
       filtering */
    tempint = (int16_t) (estimator.qOmegaMr - estimator.qVelEstim);
    estimator.qVelEstimStateVar += __builtin_mulss(tempint,
                                    estimator.qVelEstimFilterK);
    estimator.qVelEstim = (int16_t) (estimator.qVelEstimStateVar >> 15);

}
// *****************************************************************************

/* Function:
    InitEstimParm ()

  Summary:
    Initializes Motor speed and angle estimator parameters

  Description:
    Initialization of the parameters of the estimator.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitEstimParm(void) 
{
    /* Constants are defined in usreparms.h */

    motorParm.qLsDtBase = NORM_LSDTBASE;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = NORM_RS;
    motorParm.qRsScale = NORM_RS_SCALE_SHIFT; 
    motorParm.qLsDtScale = 15- NORM_LSDTBASE_SCALE;

    motorParm.qInvKFiBase = NORM_INVKFIBASE;
    motorParm.qInvKFi = motorParm.qInvKFiBase;
    motorParm.qInvKFiBaseScale = NORM_INVKFIBASE_SCALE;
    motorParm.qRatedPeakCurrent = Q15_NOMINAL_PEAK_CURRENT;
    
    estimator.qRhoStateVar = 0;
    estimator.qOmegaMr = 0;
    estimator.qDiCounter = 0;
    estimator.qEsdStateVar = 0;
    estimator.qEsqStateVar = 0;

    estimator.qDIlimitHS = D_ILIMIT_HS;
    estimator.qDIlimitLS = D_ILIMIT_LS;

    estimator.qKfilterEsdq = KFILTER_ESDQ;
    estimator.qVelEstimFilterK = KFILTER_VELESTIM;

    estimator.qDeltaT = NORM_DELTAT;
    estimator.qRhoOffset = INITOFFSET_TRANS_OPEN_CLSD;

}
