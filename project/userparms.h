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

#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <xc.h>
#include "general.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
        


/* Definition for tuning - if active the speed reference is a ramp with a 
constant slope. The slope is determined by TUNING_DELAY_RAMPUP constant.
 the software ramp implementing the speed increase has a constant slope, 
 adjusted by the delay TUNING_DELAY_RAMPUP when the speed is incremented.
 The potentiometer speed reference is overwritten. The speed is          
 increased from 0 up to the END_SPEED_RPM in open loop – with the speed  
 increase typical to open loop, the transition to closed loop is done    
 and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
#undef TUNING

/* if TUNING was define, then the ramp speed is needed: */
#ifdef TUNING
    /* the smaller the value, the quicker the ramp */
    #define TUNING_DELAY_RAMPUP   0xF      
#endif


/* open loop continuous functioning */
/* closed loop transition disabled  */
#undef OPEN_LOOP_FUNCTIONING

/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE

#define  ENABLE      1
#define  DISABLE     0    
    
#define  WINDMILL_FUNCTIONALITY    ENABLE
#define  BRAKE_FUNCTIONALITY       ENABLE
#define  IPD_FUNCTIONALITY         ENABLE
    
/* FOC with single shunt is enabled */
/* undef to work with dual Shunt  */    
#define SINGLE_SHUNT     
    
/* undef to use x2cscope, define to use LIN communication for controlling 
 * ECF board. */
#define LIN_CONTROL
    
/**********Update the Board parameters as per the board********/
/** Board Parameters */
#define     MC1_PEAK_VOLTAGE            36
#define     MC1_PEAK_CURRENT            50    
#define     MC1_PEAK_SPEED_RPM          4000  

  
/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */

/* Motor's number of pole pairs */
#define POLE_PAIRS 4
/* Nominal speed of the motor in RPM */
#define NOMINAL_SPEED_RPM    2700
/* Maximum speed of the motor in RPM - given by the motor's manufacturer */
#define MAXIMUM_SPEED_RPM    2700 
/* Motor Rated Phase Current Peak in Amps*/
#define NOMINAL_PEAK_CURRENT    30

/* Voltage L-L Peak Inverse Kfi value in float */
#define NOMINAL_KFI (float)(3.62)

/* The following values are given in the xls attached file */
/* normalized ls/dt value */
#define NORM_LSDTBASE 7262
#define NORM_LSDTBASE_SCALE 3    /* 2^NORM_LSDTBASE_SCALE is the scaling */
#define NORM_LSDTBASE_SCALE_SHIFT   (15- NORM_LSDTBASE_SCALE + 2)   
/* normalized rs value */
#define NORM_RS  2050
#define NORM_RS_SCALE       0   /* 2^NORM_RS_SCALE is the scaling */ 
#define NORM_RS_SCALE_SHIFT   (15 - NORM_RS_SCALE)   
/* the calculation of Rs gives a value exceeding the Q15 range so,
 the normalized value is further divided by 2 to fit the 32768 limit
 this is taken care in the estim.c where the value is implied
 normalized inv kfi at base speed */
#define NORM_INVKFIBASE  5092
#define NORM_INVKFIBASE_SCALE	4   /* 2^NORM_INVKFIBASE_SCALE is
                                            the scaling */    
/* the calculation of InvKfi gives a value which not exceed the Q15 limit
   to assure that an increase of the term with 5 is possible in the lookup table
   for high flux weakening the normalized is initially divided by 2
   this is taken care in the estim.c where the value is implied
   normalized dt value */
#define NORM_DELTAT  874

/* Limitation constants */
/* di = i(t1)-i(t2) limitation high speed limitation, for dt 50us 
 the value can be taken from attached xls file */
#define D_ILIMIT_HS 590
/* low speed limitation, for dt 8*50us */
#define D_ILIMIT_LS 2359

/**********************  support xls file definitions end *********************/
/*Over Current Protection*/
#define DC_BUS_OVER_CURRENT_THRESHOLD 35

/* Open loop startup constants */
#define CURRENT_REF_OPENLOOP      10    

/* The following values depends on the PWM frequency,lock time is the time 
   needed for motor's poles alignment before the open loop speed ramp up */
/* This number is: 20,000 is 1 second. */
#define LOCK_TIME 0

/* Open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 1
/* The Open loop speed increments every  OPENLOPP_SPEEDREFRAMP_COUNT */ 
#define OPENLOPP_SPEEDREFRAMP_COUNT   2    
/* Open loop speed ramp up end value Value in RPM*/
#define END_SPEED_RPM 300     
/* Open loop angle scaling Constant - This corresponds to 1024(2^10)
   Scaling down motorStartUpData.startupRamp to thetaElectricalOpenLoop   */
#define STARTUPRAMP_THETA_OPENLOOP_SCALER       10 
/* In case of the potentiometer speed reference, a reference ramp
is needed for assuring the motor can follow the reference imposed /
minimum value accepted */
#define SPEEDREFRAMP_UP   Q15(0.00003)
#define SPEEDREFRAMP_DOWN   Q15(0.00002)
/* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
#define SPEEDREFRAMP_COUNT   16//8  

/* Filters constants definitions  */
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 1200
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 164
/* Estimated speed filter constant */
#define KFILTER_VELESTIM 2*374

/* initial offset added to estimated value, when transitioning from open loop 
 to closed loop the value represents 45deg and should satisfy both open loop and
 closed loop functioning normally this value should not be modified, but in 
 case of fine tuning of the transition, depending on the load or the 
 rotor moment of inertia */
#define INITOFFSET_TRANS_OPEN_CLSD  0
/* motor speed down from full speed to openloop during stop command
PWM_FREQUENCY * closedloop ramp time (in secs) */    
#define STOPDELALYCOUNTMAX  (uint32_t)(120000)  

/************* PI controllers tuning values - ***********/     
/* D Control Loop Coefficients */
#define D_CURRCNTR_PTERM       4500//Q15(0.005)
#define D_CURRCNTR_ITERM       500//Q15(0.0003)
#define D_CURRCNTR_CTERM       Q15(0.999)
#define D_CURRCNTR_OUTMAX      0x7FFF

/* Q Control Loop Coefficients */
#define Q_CURRCNTR_PTERM       4500//Q15(0.005)
#define Q_CURRCNTR_ITERM       500//Q15(0.0003)
#define Q_CURRCNTR_CTERM       Q15(0.999)
#define Q_CURRCNTR_OUTMAX      0x7FFF

/* Velocity Control Loop Coefficients */
#define SPEEDCNTR_PTERM        6550//Q15(0.2)
#define SPEEDCNTR_ITERM        5//Q15(0.0001)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       0x7FFF
 
/* Open loop q current setup - */
#define Q15_CURRENT_REF_OPENLOOP NORM_VALUE(CURRENT_REF_OPENLOOP,MC1_PEAK_CURRENT)

/* Specify Over Current Limit - DC BUS */
#define Q15_DC_BUS_OVER_CURRENT_THRESHOLD NORM_VALUE(DC_BUS_OVER_CURRENT_THRESHOLD,MC1_PEAK_CURRENT)
    
#define Q15_END_SPEED_RPM  NORM_VALUE(END_SPEED_RPM,MC1_PEAK_SPEED_RPM)   
/* Maximum motor speed converted into electrical speed */
#define Q15_MAXIMUMSPEED NORM_VALUE(MAXIMUM_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* Nominal motor speed converted into electrical speed */
#define Q15_NOMINALSPEED NORM_VALUE(NOMINAL_SPEED_RPM,MC1_PEAK_SPEED_RPM)
/* Motor Rated Phase Current Peak in Q15 Format */
#define Q15_NOMINAL_PEAK_CURRENT NORM_VALUE(NOMINAL_PEAK_CURRENT,MC1_PEAK_CURRENT)  
/* End speed converted to fit the startup ramp */
#define END_SPEED ((uint32_t)Q15_END_SPEED_RPM * NORM_DELTAT/32768)*1024

/* Maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*POLE_PAIRS
/* Nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*POLE_PAIRS

/* Fraction of dc link voltage (Ma)(to set the limit for current controllers PI Output */
#ifdef SINGLE_SHUNT
        #define MAX_VOLTAGE_VECTOR                      0.86
#else
        #define MAX_VOLTAGE_VECTOR                      0.96
#endif

/* Squared modulation index*/
#define SQR_MAX_VOLTAGE_VECTOR            (MAX_VOLTAGE_VECTOR * MAX_VOLTAGE_VECTOR)
/******************************** Field Weakening *****************************/
/* Field Weakening constant for constant torque range 
   Flux reference value */
#define IDREF_BASESPEED         0   

    //********Windmill code configurations******************************
/* WBASE_DT_LSGAIN = pi * NORM_DETLAT */
#define MC1_WBASE_DT_LSGAIN     WBASE_DT_LSGAIN(NORM_DELTAT)
//The minimum windmill speed should always be greater or equal to END_SPEED_RPM
#define WINDMILL_MIN_SPEED         END_SPEED_RPM //Mechanical RPM

#define WINDMILL_MIN_SPEED_PU      NORM_VALUE(WINDMILL_MIN_SPEED,MC1_PEAK_SPEED_RPM) 
//********Brake code configurations****************************** 
/* Braking time in terms of PWM Cycles:20,000 counts = 1 second braking*/
#define BRAKE_ON_COUNT_LOW           50000
#define BRAKE_ON_COUNT_HIGH          90000


#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
