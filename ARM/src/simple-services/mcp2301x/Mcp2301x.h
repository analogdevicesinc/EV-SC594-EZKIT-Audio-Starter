/**
 * Copyright (c) 2025 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/***********************************************************************************
**
** MODULE TITLE:
**     Mcp2301x.h
**
** MODULE FUNCTION:
**     This is the public interface header for the MCP2301X driver component.
**     This file is required and should NOT be modified by the user. 
**
**     The purpose of this module is to provide a modular and scalable interface to 
**     the Mcp2301x I2C to GPIO extender IC. This module provides GPIO only
**     (read/write only) interfacing and only supports I2C as the interface to the IC 
**     even where SPI may be supported. Interrupt handling is NOT supported by 
**     this module. 
**
************************************************************************************
*/

#ifndef MCP2301X_H_
#define MCP2301X_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "Mcp2301x_cfg.h"

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/
 
typedef enum
{
   E_MCP2301X_STATUS_OK,
   E_MCP2301X_STATUS_NOT_INITALIZED,
   E_MCP2301X_STATUS_INVALID_DEVICE_ID,
   E_MCP2301X_STATUS_INVALID_PORT_ID,
   E_MCP2301X_STATUS_INVALID_PIN_ID,
   E_MCP2301X_STATUS_NULL_PTR,
   E_MCP2301X_STATUS_CFGIF_ERROR,
   E_MCP2301X_STATUS_ALREADY_INITIALIZED,
   E_MCP2301X_STATUS_ALREADY_UNINITIALIZED,
   E_MCP2301X_STATUS_NO_SYSTEM_MAP,
   E_MCP2301X_STATUS_PIN_NOT_FOUND,
}T_MCP2301X_STATUS;

typedef enum
{
    E_MCP2301X_PORT_A,
    E_MCP2301X_PORT_B,
    E_MCP2301X_PORT_MAX
}T_MCP2301X_PORT;

typedef enum
{
    E_MCP2301X_PIN_0,
    E_MCP2301X_PIN_1,
    E_MCP2301X_PIN_2,
    E_MCP2301X_PIN_3,
    E_MCP2301X_PIN_4,
    E_MCP2301X_PIN_5,
    E_MCP2301X_PIN_6,
    E_MCP2301X_PIN_7,
    E_MCP2301X_PIN_MAX
}T_MCP2301X_PIN;

typedef struct
{
    int32_t s32PinId;
    T_MCP2301X_PORT ePort;
    T_MCP2301X_PIN  ePin;
}T_MCP2301X_SYSTEM_MAP;

typedef struct 
{
    uint16_t u16PinDirMask;
    uint16_t u16PullUpEnMask;
    uint16_t u16PinInitValueMask;
    uint16_t u16PinMapLength;
    const T_MCP2301X_SYSTEM_MAP * const psPinMap;
}T_MCP2301X_DEVICE_CONFIG;


/***********************************************************************************
 *  GLOBAL PROTOTYPES
 **********************************************************************************/

/******************************************************************************
 *   FUNCTION:     Mcp2301x_Init
 *   DESCRIPTION:  This function resets the Mcp2301x device to it's default state
 *                 and initializes the device according to the device settings
 *                 in the asMcp2301xConfigRegistry. 
 *              
 *                 This function is REQUIRED to be called during project initialization
 *                 to use this module. It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific device in the registry to initialize and
 *                             configure
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
extern T_MCP2301X_STATUS Mcp2301x_Init(const T_MCP2301X_DEVICE_ID eDeviceId);

/******************************************************************************
 *   FUNCTION:     Mcp2301x_ReadPin
 *   DESCRIPTION:  This function reads the level on the requested hardware ICs
 *                 GPIO port/pin. 
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific hardware device to read from
 *                 ePort     - The specific GPIO port (A/B) to read from
 *                 ePin      - The specic pin whose value is to be read
 *   OUTPUT(S):    pbPinVal  - A pointer to port/pin value read from the device 
 *   RETURN VALUE: T_MCP2301X_STATUS      
 ******************************************************************************/
extern T_MCP2301X_STATUS Mcp2301x_ReadPin(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, bool * const pbPinVal);

/******************************************************************************
 *   FUNCTION:     Mcp2301x_WritePin
 *   DESCRIPTION:  This function writes a 1 or a 0 (high/low) to a specific
 *                 hardware ICs GPIO port/pin.
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The specific hardware device to write to
 *                 ePort     - The specific GPIO port (A/B) to write to
 *                 ePin      - The specic pin whose value is to be written
 *                 bPinVal   - The value to write on the pin
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
extern T_MCP2301X_STATUS Mcp2301x_WritePin(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, const bool bPinVal);

/******************************************************************************
 *   FUNCTION:     Mcp2301x_GetPortPin
 *   DESCRIPTION:  This retrieves the hardware port and pin configuration defined
 *                 in the asMcp2301xConfigRegistry table. This can be useful for 
 *                 applications that have an upper level abstraction for handling
 *                 multiple I2C to GPIO expanders in the system. This allows
 *                 the system level configuration to be maintained at the hardware
 *                 layer and keep a common upper layer for handling all of the 
 *                 pins in the system. This API is optional for users who 
 *                 wish to use this module by itself. Users may mark the psPinMap
 *                 parameter in the asMcp2301xConfigRegistry as NULL in this case and 
 *                 this API should NOT be called. 
 *  
 *                 This function is NOT REQUIRED to be used. It is NOT thread/context
 *                 safe. Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The device whose hardware port and pin is to be
 *                             retrieved. 
 *                 s32PinId  - The system ID representing the port and pin for 
 *                             the specific device.
 *   OUTPUT(S):    pePort    - A pointer to the port representing the system
 *                             pin.
 *                 pePin     - A pointer to the pin representing the system 
 *                             pin. 
 *   RETURN VALUE: T_MCP2301X_STATUS      
 ******************************************************************************/
extern T_MCP2301X_STATUS Mcp2301x_GetPortPin(const T_MCP2301X_DEVICE_ID eDeviceId, int32_t s32PinId, T_MCP2301X_PORT * const pePort, T_MCP2301X_PIN * const pePin);

/******************************************************************************
 *   FUNCTION:     Mcp2301x_DeInit
 *   DESCRIPTION:  This function resets the requested device to it's specific
 *                 POR reset values. 
 *              
 *                 This function is NOT REQUIRED to be called if system
 *                 deinitialization is not required. 
 *                 It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     eDeviceId - The device to reset      
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
extern T_MCP2301X_STATUS Mcp2301x_DeInit(const T_MCP2301X_DEVICE_ID eDeviceId);

#endif /* MCP2301X_H_ */
