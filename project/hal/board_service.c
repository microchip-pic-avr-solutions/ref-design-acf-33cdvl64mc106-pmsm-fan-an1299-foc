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

#include <stdint.h>
#include <stdbool.h>
#include "port_config.h"
#include "board_service.h"
#include "userparms.h"
#include "adc.h"
#include "pwm.h"
#include "cmp.h"
#include "hardware_access_functions.h"

/** Maintains runtime state of Board_Service() or Board_Configure() functions */
volatile uint16_t runtimeState = 0;
volatile uint16_t systemState = 0;
BUTTON_T buttonStartStop;
BUTTON_T buttonSpeedHalfDouble;

uint16_t boardServiceISRCounter = 0;

void DisablePWMOutputs(void);
void EnablePWMOutputs(void);
void ClearPWMPCIFault(void);
void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
bool IsPressed_Button1(void);
bool IsPressed_Button2(void);
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);


static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

bool IsPressed_Button1(void)
{
    if(buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool IsPressed_Button2(void)
{
    if(buttonSpeedHalfDouble.status)
    {
        buttonSpeedHalfDouble.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        switch(systemState)
        {
            case SYSTEM_INITIALIZATION: 
                runtimeState = HAL_Board_Configure();
                if (driverStatus0Data > 0)
                {
                    systemState = SYSTEM_ERROR;
                }
                if (driverStatus1Data > 0)
                {
                    systemState = SYSTEM_ERROR;
                }                
                if (runtimeState == BOARD_READY)
                {
                    systemState = SYSTEM_READY;
                }
            break;
            case SYSTEM_READY:
                runtimeState = HAL_Board_Service();
                if (runtimeState == BOARD_ERROR)
                {

                }
            break;
            case SYSTEM_ERROR:

            break;
        }
        boardServiceISRCounter = 0;
    }
}
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false;

    buttonSpeedHalfDouble.state = BUTTON_NOT_PRESSED;
    buttonSpeedHalfDouble.debounceCount = 0;
    buttonSpeedHalfDouble.state = false;

}
// *****************************************************************************
/* Function:
    Init_Peripherals()

  Summary:
    Routine initializes controller peripherals

  Description:
    Routine to initialize Peripherals used for Inverter Control

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitPeripherals(void)
{        
    uint16_t cmpReference = 0;
    CMP_Initialize();
    CMP1_ModuleEnable(true);
    cmpReference = (uint16_t)(__builtin_mulss(Q15_DC_BUS_OVER_CURRENT_THRESHOLD,2047)>>15);
    cmpReference = cmpReference + 2048; 
    CMP1_ReferenceSet(cmpReference);
    InitializeADCs();
    
    InitPWMGenerators();
    
    /* Make sure ADC does not generate interrupt while initializing parameters*/
    DisableADCInterrupt();
}
/**
 * Disable the PWM channels assigned for Inverter #A by overriding them to low state.
 * @example
 * <code>
 * DisablePWMOutputsInverterA();
 * </code>
 */
void DisablePWMOutputs(void)
{
    /** Set Override Data on all PWM outputs */
    // 0b00 = State for PWM3H,L, if Override is Enabled
    PG3IOCONLbits.OVRDAT = 0;
    // 0b00 = State for PWM2H,L, if Override is Enabled
    PG2IOCONLbits.OVRDAT = 0; 
    // 0b00 = State for PWM1H,L, if Override is Enabled
    PG1IOCONLbits.OVRDAT = 0;  
    
    // 1 = OVRDAT<1> provides data for output on PWM3H
    PG3IOCONLbits.OVRENH = 1; 
    // 1 = OVRDAT<0> provides data for output on PWM3L
    PG3IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM2H
    PG2IOCONLbits.OVRENH = 1;
    // 1 = OVRDAT<0> provides data for output on PWM2L
    PG2IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM1H
    PG1IOCONLbits.OVRENH = 1;  
    // 1 = OVRDAT<0> provides data for output on PWM1L
    PG1IOCONLbits.OVRENL = 1;     
}

/**
 * Enable the PWM channels assigned for Inverter #A by removing Override.
 * @example
 * <code>
 * EnablePWMOutputsInverterA();
 * </code>
 */
void EnablePWMOutputs(void)
{    
    // 0 = PWM Generator provides data for the PWM3H pin
    PG3IOCONLbits.OVRENH = 0; 
    // 0 = PWM Generator provides data for the PWM3L pin
    PG3IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM2H pin
    PG2IOCONLbits.OVRENH = 0;
    // 0 = PWM Generator provides data for the PWM2L pin
    PG2IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM1H pin
    PG1IOCONLbits.OVRENH = 0;  
    // 0 = PWM Generator provides data for the PWM1L pin
    PG1IOCONLbits.OVRENL = 0;     
}

void ClearPWMPCIFault(void)
{
    
    PG1FPCILbits.SWTERM = 1;
    PG2FPCILbits.SWTERM = 1;
    PG3FPCILbits.SWTERM = 1;
}

void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));  
    PWM_PDC3 = pPwmDutycycle->dutycycle3;
    PWM_PDC2 = pPwmDutycycle->dutycycle2;
    PWM_PDC1 = pPwmDutycycle->dutycycle1;
}
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *pPwmDutycycle1,MC_DUTYCYCLEOUT_T *pPwmDutycycle2)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle1,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));
    
    PWM_PHASE3 = pPwmDutycycle1->dutycycle3 + (DEADTIME>>1);
    PWM_PHASE2 = pPwmDutycycle1->dutycycle2 + (DEADTIME>>1);
    PWM_PHASE1 = pPwmDutycycle1->dutycycle1 + (DEADTIME>>1);
    
    pwmDutyCycleLimitCheck(pPwmDutycycle2,(DEADTIME>>1),(LOOPTIME_TCY - (DEADTIME>>1)));
    
    PWM_PDC3 = pPwmDutycycle2->dutycycle3 - (DEADTIME>>1);
    PWM_PDC2 = pPwmDutycycle2->dutycycle2 - (DEADTIME>>1);
    PWM_PDC1 = pPwmDutycycle2->dutycycle1 - (DEADTIME>>1);
}
void pwmDutyCycleLimitCheck (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint16_t min,uint16_t max)
{
    if (pPwmDutycycle->dutycycle1 < min)
    {
        pPwmDutycycle->dutycycle1 = min;
    }
    else if (pPwmDutycycle->dutycycle1 > max)
    {
        pPwmDutycycle->dutycycle1 = max;
    }
    
    if (pPwmDutycycle->dutycycle2 < min)
    {
        pPwmDutycycle->dutycycle2 = min;
    }
    else if (pPwmDutycycle->dutycycle2 > max)
    {
        pPwmDutycycle->dutycycle2 = max;
    }
    
    if (pPwmDutycycle->dutycycle3 < min)
    {
        pPwmDutycycle->dutycycle3 = min;
    }
    else if (pPwmDutycycle->dutycycle3 > max)
    {
        pPwmDutycycle->dutycycle3 = max;
    }
}

void HAL_MC1SetVoltageVector(int16_t vector)
{
    /* Overrides PWM based on vector number in the order of c-b-a */
    
    switch(vector)
    {
        case 0:
            /* c-b-a :: 0-0-0 */
            PG3IOCONL = PWM_BOT_ON;
            PG2IOCONL = PWM_BOT_ON;
            PG1IOCONL = PWM_BOT_ON;
        break;
        
        case 1:
            /* c-b-a :: 0-0-1 */
            PG3IOCONL = PWM_BOT_ON;
            PG2IOCONL = PWM_BOT_ON;
            PG1IOCONL = PWM_TOP_ON;
        break;
        
        case 2:
            /* c-b-a :: 0-1-1 */
            PG3IOCONL = PWM_BOT_ON;
            PG2IOCONL = PWM_TOP_ON;
            PG1IOCONL = PWM_TOP_ON;
        break;
        
        case 3:
            /* c-b-a :: 0-1-0 */
            PG3IOCONL = PWM_BOT_ON;
            PG2IOCONL = PWM_TOP_ON;
            PG1IOCONL = PWM_BOT_ON;
        break;
        
        case 4:
            /* c-b-a :: 1-1-0 */
            PG3IOCONL = PWM_TOP_ON;
            PG2IOCONL = PWM_TOP_ON;
            PG1IOCONL = PWM_BOT_ON;
        break;
        
        case 5:
            /* c-b-a :: 1-0-0 */
            PG3IOCONL = PWM_TOP_ON;
            PG2IOCONL = PWM_BOT_ON;
            PG1IOCONL = PWM_BOT_ON;
        break;
        
        case 6:
             /* c-b-a :: 1-0-1 */
            PG3IOCONL = PWM_TOP_ON;
            PG2IOCONL = PWM_BOT_ON;
            PG1IOCONL = PWM_TOP_ON;
        break;
        
        case 7:
            /* c-b-a :: 1-1-1 */
            PG3IOCONL = PWM_TOP_ON;
            PG2IOCONL = PWM_TOP_ON;
            PG1IOCONL = PWM_TOP_ON;
        break;

        default:
            vector = 0;
        break;
    }
}