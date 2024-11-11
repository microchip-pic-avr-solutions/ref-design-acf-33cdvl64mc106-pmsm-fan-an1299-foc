/**
  @File Name:
    hardware_access_functions.h

  @Summary:
    This module provides hardware access function support.

  @Description:
    This module provides hardware access function support.
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

#ifndef __HAF_H
#define __HAF_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "uart2.h"
#include "bsp.h"
#include "delay.h"
#include "hardware_access_functions_types.h"
#include "hardware_access_functions_params.h"

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif


/**
  Section: Hardware Access Functions
 */     

/**
 * Configures the board and initializes any state depenendent data that is used
 * in the HAL. This function needs to be called once every 1ms.
 * Summary: Configures the board and initializes any state depenendent data that is used in the HAL.
 * @example
 * <code>
 * while (HAL_Board_Configure()==BOARD_NOT_READY);
 * <code>
 */
HAL_BOARD_STATUS HAL_Board_Configure(void);

/**
 * Runs the board service routines. This function needs to be called once every 1ms.
 * Summary: Runs the board service routines.
 * @example
 * <code>
 * if (HAL_Board_Service()!=BOARD_READY)
 * {
 *     // do something
 * }
 * <code>
 */
HAL_BOARD_STATUS HAL_Board_Service(void);

/**
 * This function can  be called to clear the fault.
 * Summary: Clears Gate Driver Fault Status
 * @example
 * <code>
 * HAL_Board_Board_FaultClear();
 * <code>
 */
void HAL_Board_FaultClear(void);
/**
 * This function can  be called to clear the fault.
 * Summary: Clears Gate Driver Fault Status
 * @example
 * <code>
 * HAL_Board_Board_FaultClear();
 * <code>
 */
void HAL_Board_FaultSet(void);
/**
 * This function can  be called to request auto baud sequence.
 * Summary: Requests the auto baud sequence 
 * @example
 * <code>
 * HAL_Board_AutoBaudRequest();
 * <code>
 */
void HAL_Board_AutoBaudRequest(void);

extern  GATE_DRIVER_OBJ inverterGateDriver[BSP_GATE_DRIVER_INSTANCE_COUNT];
extern uint8_t driverStatus0Data,driverStatus1Data;
#ifdef __cplusplus
}
#endif

#endif /* __HAF_H */


/**
 End of File
 */