// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * cmp.h
 *
 * This header file lists interface functions - configuring and enabling the 
 * Comparator Modules and its output
 * 
 * Definitions in this file are for dsPIC33CDVL64MC106.
 * 
 * Component: HAL - CMP
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

#ifndef __CMP_H
#define __CMP_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
                
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
            
void CMP_Initialize(void);

void CMP1_ModuleEnable(bool);
void CMP1_ReferenceSet(uint16_t );

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __CMP_H
    