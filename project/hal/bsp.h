/**
  @File Name:
    bsp.h

  @Summary:
    This module provides board support functions.

  @Description:
    This module provides board support functions.
 */
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

#ifndef __BSP_H
#define __BSP_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t bspUnassignedTestPoint;

/**
 * Section: Peripheral Interface
 * Ready for use in application: No
 * This section contains hardware-specific information with macros defined
 * to provide hardware-abstracted-access to GPIO pins and their board 
 * functions. It is not recommended to use the macros defined in this section
 * directly in the application since their names have specific dependency
 * on hardware features that may or may not be available across 
 * all hardware platforms.
 */
#define BSP_LATCH_GATE_DRIVER_A_CE        LATCbits.LATC13
#define BSP_LATCH_GATE_DRIVER_A_CE_PORT   &LATC
#define BSP_LATCH_GATE_DRIVER_A_CE_BIT    13

#define BSP_LATCH_GATE_DRIVER_A_FAULT     CNFDbits.CNFD1

/**
 * Section: Application Interface
 * Ready for use in application: Yes
 * This section provides hardware access interface with generic names
 * that can be used in the application/framework and have no dependency on
 * hardware feature naming convention. Update this section as required
 * by the end-application. These generic hardware access macros can be
 * redefined to match their functionality.
 */

#define BSP_DIAGNOSTICS_UARTMODULE                    1
#define BSP_GATE_DRIVER_INTERFACE_A_UARTMODULE        2

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */


/**
 End of File
 */