/*******************************************************************************
  ADC Configuration Routine source File

  File Name:
    adc.c

  Summary:
    This file includes subroutine for initializing ADC Cores of Controller

  Description:
    Definitions in the file are for dsPIC33CDVL64MC106 on Motor Control 
    Development board from Microchip

*******************************************************************************/
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "adc.h"
#include "userparms.h"

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void InitializeADCs(void);
// *****************************************************************************
/* Function:
    void InitializeADCs (void)

  Summary:
    Routine to configure ADC Module

  Description:
    Function to configure ADC Cores
  
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitializeADCs (void)
{
    ADCON1L = 0;
    /* ADC Module is turned OFF before configuring it */
    /* ADC Enable bit 1 = ADC module is enabled 0 = ADC module is off         */
    ADCON1Lbits.ADON = 0;
    /* ADC Stop in Idle Mode bit 0 = Continues module operation in Idle mode  */
    ADCON1Lbits.ADSIDL = 0;

    ADCON1H = 0;
    /* Shared ADC Core Resolution Selection bits
       0b11 = 12-bit ; 0b10 = 10-bit ; 0b01 = 8-bit ; 0b00 = 6-bit resolution */
    ADCON1Hbits.SHRRES = 3;
    /* Fractional Data Output Format bit 1 = Fractional ; 0 = Integer         */
    ADCON1Hbits.FORM = 1;

    ADCON2L = 0;
    /* Shared ADC Core Input Clock Divider bits
       These bits determine the number of TCORESRC (Source Clock Periods) 
       for one shared TADCORE (Core Clock Period).
       1111111 = 254 Source Clock Periods
        ???
       0000010 = 4 Source Clock Periods
       0000001 = 2 Source Clock Periods
       0000000 = 2 Source Clock Periods
     */
    ADCON2Lbits.SHRADCS = 0;
    /* EIEN: Early Interrupts Enable bit
       1 = The early interrupt feature is enabled for the input channel 
           interrupts ,when the EISTATx flag is set
       0 = The individual interrupts are generated when conversion is done ,
           when the ANxRDY flag is set*/
    ADCON2Lbits.EIEN = 0;

    ADCON2H = 0;
    /* Shared ADC Core Sample Time Selection bits 
       These bits specify the number of shared ADC Core Clock Periods (TADCORE) 
       for the shared ADC core sample time.
       Ranges from 2 to 1025 TADCORE
       if SHRSAMC = 15 ,then Sampling time is 17 TADCORE */
    ADCON2Hbits.SHRSAMC = 15;

    ADCON3L  = 0;
    /* ADC Reference Voltage Selection bits 
       0b000 = VREFH is AVDD and VREFL is  AVSS */
    ADCON3Lbits.REFSEL = 0;

    ADCON3H = 0;

    /* Shared ADC Core Enable bit 1 = Shared ADC core is enabled 
       0 = Shared ADC core is disabled  */
    /* Shared ADC Core is Disabled prior to configuration */
    ADCON3Hbits.SHREN = 0;
    /* ADC Module Clock Source Selection bits 
       0b11 = FVCO/4;0b10 = AFVCODIV ;0b01 = FOSC ; 0b00 = FP(Peripheral Clock) */
    ADCON3Hbits.CLKSEL = 0;
    /* ADC Module Clock Source Divider bits (1 to 64)
       The divider forms a TCORESRC clock used by all ADC cores (shared and 
       dedicated) from the TSRC ADC module clock selected by the CLKSEL<2:0> .
       000000 = 1 Source Clock Periods */
    ADCON3Hbits.CLKDIV = 0;
    


    /* Configuring ADC INPUT MODE CONTROL REGISTER bits 
       Output Data Sign for Corresponding Analog Inputs bits
       1 = Channel output data is signed
       0 = Channel output data is unsigned    */
    /*ADMOD0L configures Output Data Sign for Analog inputs  AN0 to AN7 */
    ADMOD0L = 0x0000;
    ADMOD0Lbits.SIGN0 = 1;  /*Ibus*/
    ADMOD0Lbits.SIGN1 = 1;  /*Ia*/
    ADMOD0Lbits.SIGN4 = 1;	/*Ib*/
   
    /*ADMOD0H configures Output Data Sign for Analog inputs  AN8 to AN15 */
    ADMOD0H = 0;   
	ADMOD0Hbits.SIGN10 = 0;  /*Vb*/
	ADMOD0Hbits.SIGN11 = 0;  /*Va*/
	ADMOD0Hbits.SIGN12 = 0;  /*VBUS*/
    
    ADMOD1L = 0;
    
    /* Ensuring all interrupts are disabled and Status Flags are cleared */
    ADIEL = 0;
    ADIEH = 0;
    ADSTATL = 0;
    ADSTATH = 0;
    ADEIEL  = 0;
    ADEIEH  = 0;
    ADEISTATL = 0;
    ADEISTATH = 0;
 
    ADCON5H = 0;
    /* Shared ADC Core Ready Common Interrupt Enable bit
       0 = Common interrupt is disabled for an ADC core ready event*/
    ADCON5Hbits.SHRCIE = 0;
    /* ADC Dedicated Core x Power-up Delay bits
       These bits determine the power-up delay in the number of the 
       Core Source Clock Periods (TCORESRC) for all ADC cores.
       0b1011 = 2048 Source Clock Periods */
    ADCON5Hbits.WARMTIME  = 0b1111 ;                                         
    
    /* ADC Enable bit 1 = ADC module is enabled 0 = ADC module is off         */
    /* Turn on ADC Module */
    ADCON1Lbits.ADON      = 1 ;  
    
    ADCON5L = 0;

    /* Turn on analog power for shared core */
    ADCON5Lbits.SHRPWR    = 1 ;
    while(ADCON5Lbits.SHRRDY == 0);
    /* Shared ADC Core Enable bit 1 = Shared ADC core is enabled 
       0 = Shared ADC core is disabled  */
    /* Shared ADC Core is Enabled  */
    ADCON3Hbits.SHREN     = 1 ;
    
    /* Setup ADC Interrupts for reading and processing converted results */
    /* Common Interrupt Enable bits
       1 = Common and individual interrupts are enabled for analog channel
       0 = Common and individual interrupts are disabled for analog channel*/
   
 
    
#ifdef SINGLE_SHUNT    
     _IE0        = 1 ;
    /* Clear ADC interrupt flag */
    _ADCAN0IF    = 0 ;  
    /* Set ADC interrupt priority IPL 7  */ 
    _ADCAN0IP   = 7 ;  
    /* Disable the AN0 interrupt  */
    _ADCAN0IE    = 0 ;
     
#else         
    _IE1        = 1 ;
    /* Clear ADC interrupt flag */
    _ADCAN1IF    = 0 ;  
    /* Set ADC interrupt priority IPL 7  */ 
    _ADCAN1IP   = 7 ;  
    /* Disable the AN1 interrupt  */
    _ADCAN1IE    = 0 ; 
#endif
    
    /* Trigger Source Selection for Corresponding Analog Inputs bits 
     *  00101 = PMW1 Trigger 2
        00100 = PMW1 Trigger 1
        00001 = Common software trigger
        00000 = No trigger is enabled  */
    

#ifdef SINGLE_SHUNT
    /* Trigger Source for Analog Input #0(Ibus)  = 0b0101 */
    ADTRIG0Lbits.TRGSRC0 = 0x5;
#else
    /* Trigger Source for Analog Input #0(Ibus)  = 0b0100 */
    ADTRIG0Lbits.TRGSRC0 = 0x4;
    /* Trigger Source for Analog Input #1(Ia)  = 0b0100 */
    ADTRIG0Lbits.TRGSRC1 = 0x4;
    /* Trigger Source for Analog Input #4(Ib)  = 0b0100 */
    ADTRIG1Lbits.TRGSRC4 = 0x4;  
#endif
    /* Trigger Source for Analog Input #10(Vb)  = 0b0100 */
    ADTRIG2Hbits.TRGSRC10 = 0x4;
    /* Trigger Source for Analog Input #11(Va)  = 0b0100 */
    ADTRIG2Hbits.TRGSRC11 = 0x4;
    /* Trigger Source for Analog Input #12(Vbus)  = 0b0100 */
    ADTRIG3Lbits.TRGSRC12 = 0x4;
    
}
