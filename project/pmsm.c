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
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include <libq.h>      
#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "estim.h"

#include "clock.h"
#include "pwm.h"
#include "adc.h"
#include "port_config.h"
#include "delay.h"
#include "board_service.h"
#include "diagnostics.h"
#include "lin.h"
#include "uart2.h"
#include "singleshunt.h"
#include "measure.h"
#include "hardware_access_functions.h"
#include "fan.h"
#include "windmill.h"

UGF_T uGF;
uint8_t motorFaultCode;
int16_t Q15_Speed_Command, Speed_Command_RPM;

#ifdef LIN_CONTROL
extern LIN_NODE lin;
uint16_t LIN_Start_Stop_Command;
enum {
    LIN_START_COMMAND = 0x0ECF,
    LIN_STOP_COMMAND  = 0xDEAD
};
#else
int16_t X2C_Start_Stop_Command;
#endif

enum {
    MOTOR_NO_FAULT          = 0,
    MOTOR_OVERCURRENT_FAULT = 1,
    MOTOR_DRIVER_FAULT      = 2
};

CTRL_PARM_T ctrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData;
MOTOR_STOP_DATA_T motorStopData;

int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;

MCAPP_FAN_T fan;
MCAPP_FAN_T *pFan = &fan;

volatile uint16_t adcDataBuffer;
MCAPP_MEASURE_T measureInputs;
uint16_t loadStartReadyCheckFlag = 0;
int16_t Actual_Speed_RPM;

void InitControlParameters(void);
void DoControl( void );
void CalculateParkAngle(void);
void ResetParmeters(void);
void MCParametersInit();

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    InitOscillator();

    SetupGPIOPorts();
    CN_PortDEnable();
    /* Sets required fan operations and configurations*/
    MCAPP_FanConfig(pFan);
    /* Initialize Peripherals */
    InitPeripherals();
    DiagnosticsInit();
    
#ifdef LIN_CONTROL
    LIN_InitNode(&lin);
    LIN_Enable(&lin);
#endif
    
    BoardServiceInit();
    MCParametersInit();
    CORCONbits.SATA = 0;
    while(1)
    {        
        /* Reset parameters used for running motor*/
        ResetParmeters();
        while(systemState != SYSTEM_READY)
        {
            BoardService();
            DiagnosticsStepMain();
#ifdef LIN_CONTROL
            LIN_ApplicationStepMain(&lin);
#endif
        }
        while(1)
        {
            ResetSingleShuntSamplePoint(&singleShuntParam); 
            
            DiagnosticsStepMain();
            BoardService();
            
#ifdef LIN_CONTROL
            if (LIN_TxFrameRequested(&lin)) 
            {
                uint8_t byte1 = motorFaultCode;
                uint8_t byte2 = (uint8_t)uGF.Word;
                uint8_t byte3 = (uint8_t)(Actual_Speed_RPM >> 8);
                uint8_t byte4 = (uint8_t)Actual_Speed_RPM;
                LIN_TxQueuePush(&lin, byte1);
                LIN_TxQueuePush(&lin, byte2);
                LIN_TxQueuePush(&lin, byte3);
                LIN_TxQueuePush(&lin, byte4);
            }
            
            LIN_ApplicationStepMain(&lin);
            
            if (LIN_RxFrameReady(&lin)) 
            {
                /* We are always expecting 4 bytes */
                uint8_t byte1 = LIN_RxQueuePop(&lin);
                uint8_t byte2 = LIN_RxQueuePop(&lin); 
                uint8_t byte3 = LIN_RxQueuePop(&lin); 
                uint8_t byte4 = LIN_RxQueuePop(&lin);
                LIN_Start_Stop_Command = (byte1 << 8) | byte2;
                Speed_Command_RPM = (byte3 << 8) | byte4;
            }
            
            if(LIN_Start_Stop_Command == LIN_START_COMMAND)
            {
                if (uGF.bits.RunMotor == 0)
                {
                    uGF.bits.RunMotor = 1;
                }
                if(uGF.bits.StopCommand == true)
                {
                    uGF.bits.StopCommand = false; 
                }
            } 
            else if (LIN_Start_Stop_Command == LIN_STOP_COMMAND)
            {
                if (uGF.bits.RunMotor == 1)
                {
                    uGF.bits.StopCommand = true;                   
                }
            }
#else
            if(X2C_Start_Stop_Command)
            {
                if(uGF.bits.RunMotor == true)
                {
                    if(uGF.bits.StopCommand == true)
                    {
                       uGF.bits.StopCommand = false; 
                    }
                    else
                    {
                        uGF.bits.StopCommand = true;
                    }
                }
                else
                {
                    ResetParmeters();
                    uGF.bits.RunMotor = true;  
                }
                X2C_Start_Stop_Command = false;
            }
#endif

        }

    } // End of Main loop
    // should never get here
    while(1){}
}
// *****************************************************************************
/* Function:
    ResetParmsA()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
    
#ifdef SINGLE_SHUNT
    /* Initialize Single Shunt Related parameters */
    SingleShunt_InitializeParameters(&singleShuntParam);
    PWM_TRIGA = ADC_SAMPLING_POINT;
    PWM_TRIGB = LOOPTIME_TCY>>2;
    PWM_TRIGC = LOOPTIME_TCY>>1;
    PWM_PHASE3 = MIN_DUTY;
    PWM_PHASE2 = MIN_DUTY;
    PWM_PHASE1 = MIN_DUTY;
#else
    PWM_TRIGA = ADC_SAMPLING_POINT;
#endif
    /* Re initialize the duty cycle to minimum value */
    PWM_PDC3 = MIN_DUTY;
    PWM_PDC2 = MIN_DUTY;
    PWM_PDC1 = MIN_DUTY;
    
    DisablePWMOutputs();
#ifdef LIN_CONTROL
    /* LIN variables */
    LIN_Start_Stop_Command = LIN_STOP_COMMAND;
#else
    /* X2C variables */
    X2C_Start_Stop_Command = false;
#endif
    Actual_Speed_RPM = 0;
    Speed_Command_RPM = 0;
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;  
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change speed */
    uGF.bits.ChangeSpeed = 0;
    /* Change mode */
    uGF.bits.ChangeMode = 1;
    /* Stop Command*/
    uGF.bits.StopCommand = 0;
    
    loadStartReadyCheckFlag = 0;
    
    /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    InitEstimParm();
    /* Initialize measurement parameters */
    MCAPP_MeasureCurrentInit(&measureInputs);
    /* Initialize fan parameters */
    MCAPP_FanInit(pFan);

    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables for sqrt calculation of q reference */
    volatile int16_t temp_qref_pow_q15;
    
    if(uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if(uGF.bits.ChangeMode)
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = 0;

            /* Reinitialize variables for initial speed ramp */
            motorStartUpData.startupLock = 0;
            motorStartUpData.startupRamp = 0;
            #ifdef TUNING
                motorStartUpData.tuningAddRampup = 0;
                motorStartUpData.tuningDelayRampup = 0;
            #endif
        }

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d = piOutputId.out;
         /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(SQR_MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax =_Q15sqrt(temp_qref_pow_q15);
        piInputIq.piState.outMin = - piInputIq.piState.outMax;    
        /* PI control for Q */
        /* Speed reference */
        ctrlParm.qVelRef = Q15_CURRENT_REF_OPENLOOP;
        /* q current reference is equal to the velocity reference 
         while d current reference is equal to 0
        for maximum startup torque, set the q current to maximum acceptable 
        value represents the maximum peak value */
        ctrlParm.qVqRef = ctrlParm.qVelRef;
        piInputIq.inMeasure = idq.q;
        piInputIq.inReference = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;

    }
    else
    /* Closed Loop Vector Control */
    {
        ctrlParm.qtargetSpeed = Q15_Speed_Command;
        if(ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.qtargetSpeed;
            /* Speed Ref Ramp */
            if (ctrlParm.qDiff < 0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRampUp;
                /* If difference less than half of ref ramp, set reference
                    directly from the pot */
                if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRampUp << 1))
                {
                    ctrlParm.qVelRef = ctrlParm.qtargetSpeed;
                }
            }
            else
            {
                /* Same as above for speed decrease */
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRampDown;
                /* If difference less than half of ref ramp, set reference
                    directly from the pot */
                if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRampDown << 1))
                {
                    ctrlParm.qVelRef = ctrlParm.qtargetSpeed;
                }
            }

            ctrlParm.speedRampCount = 0;
        }

        if( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            piInputOmega.piState.integrator = (int32_t)ctrlParm.qVqRef << 15;
            ctrlParm.qVelRef = Q15_END_SPEED_RPM;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
            piInputOmega.inMeasure = estimator.qVelEstim;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
            Actual_Speed_RPM=(__builtin_mulss(piInputOmega.inMeasure, 
                                        MC1_PEAK_SPEED_RPM))>>15;
        #else
            ctrlParm.qVqRef = ctrlParm.qVelRef;
        #endif
        
        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        ctrlParm.qVdRef=IDREF_BASESPEED;

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

        /* Dynamic d-q adjustment
         with d component priority 
         vq=sqrt (vs^2 - vd^2) 
        limit vq maximum to the one resulting from the calculation above */
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(SQR_MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax = _Q15sqrt(temp_qref_pow_q15);
        piInputIq.piState.outMin = - piInputIq.piState.outMax;
        /* PI control for Q */
        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
}
// *****************************************************************************
/* Function:
   _ADCInterrupt()

  Summary:
   _ADCInterrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{  
#ifdef SINGLE_SHUNT 
    if(IFS4bits.PWM1IF ==1)
    {
        singleShuntParam.adcSamplePoint = 0;
        IFS4bits.PWM1IF = 0;
    }    
    /* If single shunt algorithm is enabled, two ADC interrupts will be
     serviced every PWM period in order to sample current twice and
     be able to reconstruct the three phases */

    switch(singleShuntParam.adcSamplePoint)
    {
        case SS_SAMPLE_BUS1:
            /*Set Trigger to measure BusCurrent Second sample during PWM 
              Timer is counting up*/
            singleShuntParam.adcSamplePoint = 1;  
            /* Ibus is measured and offset removed from measurement*/
            singleShuntParam.Ibus1 = (int16_t)(ADCBUF_INV_A_IBUS) - 
                                            measureInputs.current.offsetIbus;                        
        break;

        case SS_SAMPLE_BUS2:
            /*Set Trigger to measure BusCurrent first sample during PWM 
              Timer is counting up*/
            PWM_TRIGA = ADC_SAMPLING_POINT;
            singleShuntParam.adcSamplePoint = 0;
            /* this interrupt corresponds to the second trigger and 
                save second current measured*/
            /* Ibus is measured and offset removed from measurement*/
            singleShuntParam.Ibus2 = (int16_t)(ADCBUF_INV_A_IBUS) - 
                                            measureInputs.current.offsetIbus;
        break;

        default:
        break;  
    }
#endif
    if(uGF.bits.RunMotor)
    {
        
         
        if(singleShuntParam.adcSamplePoint == 0)
        {
            measureInputs.current.Ia = ADCBUF_INV_A_IPHASE1;
            measureInputs.current.Ib = ADCBUF_INV_A_IPHASE2;
            measureInputs.current.Ibus = ADCBUF_INV_A_IBUS;
            MCAPP_MeasureCurrentCalibrate(&measureInputs);
#ifdef SINGLE_SHUNT
                
            /* Reconstruct Phase currents from Bus Current*/                
            SingleShunt_PhaseCurrentReconstruction(&singleShuntParam);
            iabc.a = singleShuntParam.Ia;
            iabc.b = singleShuntParam.Ib;
#else

            iabc.a = measureInputs.current.Ia;
            iabc.b = measureInputs.current.Ib;
#endif
        if (loadStartReadyCheckFlag == 0)
            {
                HAL_MC1MotorInputsRead(&measureInputs);
                
                MCAPP_FanStateMachine(pFan);

                if(MCAPP_IsFanReadyToStart(pFan))
                {
                    /* Load is ready, start the motor */
                    EnablePWMOutputs();

                    MCAPP_MC1LoadStartTransition(pFan);

                    loadStartReadyCheckFlag = 1;
                }
            }
        else
            {   
                /*Motor deceleration control while stopping*/
                if(uGF.bits.StopCommand == 1)
                {
                    Speed_Command_RPM = END_SPEED_RPM;
                    if(estimator.qVelEstim < (Q15_END_SPEED_RPM <<1))
                    {
                        uGF.bits.StopCommand = 0;
                        motorStopData.stopDelayCount = 0;
                        ResetParmeters();
                    }                   
                }
                /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
                MC_TransformClarke_Assembly(&iabc,&ialphabeta);
                MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

                /* Speed and field angle estimation */
                Estim();
                /* Calculate control values */
                DoControl();
                /* Calculate qAngle */
                CalculateParkAngle();
                /* if open loop */
                if(uGF.bits.OpenLoop == 1)
                {
                    /* the angle is given by park parameter */
                    thetaElectrical = thetaElectricalOpenLoop;
                }
                else
                {
                    /* if closed loop, angle generated by estimator */
                    thetaElectrical = estimator.qRho + estimator.qRhoOffset;
                }
                MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);
                MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

                MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
                
#ifdef  SINGLE_SHUNT
                SingleShunt_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,&singleShuntParam);

                PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,&singleShuntParam.pwmDutycycle2);
#else
                MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                        &pwmDutycycle);
                PWMDutyCycleSet(&pwmDutycycle);
#endif
            }    
        }
    }
    else
    {
        PWM_TRIGA = ADC_SAMPLING_POINT;
#ifdef SINGLE_SHUNT
        PWM_TRIGB = LOOPTIME_TCY>>2;
        PWM_TRIGC = LOOPTIME_TCY>>1;
        singleShuntParam.pwmDutycycle1.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle1 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,
                &singleShuntParam.pwmDutycycle2);
#else
        pwmDutycycle.dutycycle3 = MIN_DUTY;
        pwmDutycycle.dutycycle2 = MIN_DUTY;
        pwmDutycycle.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSet(&pwmDutycycle);
#endif

    } 
    
    if(singleShuntParam.adcSamplePoint == 0)
    {
        if(uGF.bits.RunMotor == 0)
        {
            measureInputs.current.Ia = ADCBUF_INV_A_IPHASE1;
            measureInputs.current.Ib = ADCBUF_INV_A_IPHASE2; 
            measureInputs.current.Ibus = ADCBUF_INV_A_IBUS;
        }
        if(MCAPP_MeasureCurrentOffsetStatus(&measureInputs) == 0)
        {
            MCAPP_MeasureCurrentOffset(&measureInputs);
        }
        else
        {
            BoardServiceStepIsr(); 
        }

        if(Speed_Command_RPM >= MAXIMUM_SPEED_RPM)
        {
            Speed_Command_RPM = MAXIMUM_SPEED_RPM;
        }
        if(Speed_Command_RPM <= END_SPEED_RPM)
        {
            Speed_Command_RPM = END_SPEED_RPM;
        }
        Q15_Speed_Command = NORM_VALUE(Speed_Command_RPM,MC1_PEAK_SPEED_RPM);
        measureInputs.dcBusVoltage = (int16_t)( ADCBUF_VBUS_A);
               
        DiagnosticsStepIsr();
    }

    /* Read ADC Buffet to Clear Flag */
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    ClearADCIF();   
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if(uGF.bits.OpenLoop)
    {
        /* begin with the lock sequence, for field alignment */
        if (motorStartUpData.startupLock < LOCK_TIME)
        {
            motorStartUpData.startupLock += 1;
        }
        /* Then ramp up till the end speed /4 */
        else if (motorStartUpData.startupRamp < (END_SPEED))
        {
            motorStartUpData.startUpRampCount++;
            if(motorStartUpData.startUpRampCount >= OPENLOPP_SPEEDREFRAMP_COUNT)
            {   
                motorStartUpData.startUpRampCount = 0;
                motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
            }
        }
        /* Switch to closed loop */
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                uGF.bits.ChangeMode = 1;
                uGF.bits.OpenLoop = 0;
                estimator.qRhoOffset = thetaElectricalOpenLoop - estimator.qRho;
            #endif
        }
        /* The angle set depends on startup ramp */
        thetaElectricalOpenLoop += (int16_t)(motorStartUpData.startupRamp >> 
                                            STARTUPRAMP_THETA_OPENLOOP_SCALER);

    }
    /* Switched to closed loop */
    else 
    {
        /* In closed loop slowly decrease the offset add to the estimated angle */
        if(estimator.qRhoOffset > 0)
        {
            estimator.qRhoOffset--;
        }
        else if(estimator.qRhoOffset < 0)
        {
            estimator.qRhoOffset++;
        }
    }
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{
    
    ctrlParm.qRefRampUp = SPEEDREFRAMP_UP;
    ctrlParm.qRefRampDown = SPEEDREFRAMP_DOWN;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;
 
    /* PI - Id Current Control */
    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = 0;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}

void MCParametersInit()
{
    measureInputs.measurePhaseVolt.samplingFactor = MC1_BEMF_SAMPLING_FACTOR;
    motorStopData.stopDelayCountLimit = STOPDELALYCOUNTMAX;
    motorFaultCode = MOTOR_NO_FAULT;
}

void __attribute__((__interrupt__,no_auto_psv)) _PWMInterrupt()
{
    ResetParmeters();
    ClearPWMPCIFault();
    motorFaultCode |= MOTOR_OVERCURRENT_FAULT;
    ClearPWMIF(); 
}
void __attribute__ ((interrupt, no_auto_psv)) _CNDInterrupt(void)
{
    if(BSP_LATCH_GATE_DRIVER_A_FAULT == true)
    {
        motorFaultCode |= MOTOR_DRIVER_FAULT;
        HAL_Board_FaultClear();
    }
    CN_PortDInterruptFlagClear();
}