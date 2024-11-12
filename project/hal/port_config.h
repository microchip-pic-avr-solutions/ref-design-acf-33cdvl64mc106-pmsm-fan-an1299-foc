/*******************************************************************************
  Hardware specific routine definition and interfaces Header File

  File Name:
    port_config.h

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

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

#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

#include <xc.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// Digital I/O definitions
// Push button Switches
    
// SW1 :  (For any GPIO Configuration in future)
#define SW1                   0
        
// SW1 :  Used as START/STOP button of Motor
#define BUTTON_START_STOP        SW1

// LIN transceiver enable pin
#define LIN_EN                  LATCbits.LATC10

// Clear CN interrupt         
//
inline static void CN_PortDInterruptFlagClear (void)
{
    uint16_t bufferPORTD;
    
    bufferPORTD = PORTD;
    CNFDbits.CNFD1 = 0;
    _CNDIF = 0 ;
}
inline static void CN_PortDEnable(void){CNCONDbits.ON = 1;}         
inline static void CN_PortDDisable(void){CNCONDbits.ON = 0;}
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void MapGPIOHWFunction(void);
void SetupGPIOPorts(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PORTCONFIG_H


