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
** MODULE TITLE:
**     ss.h
**
** MODULE FUNCTION:
**     This is the public interface header for the soft switch component.
**     This file is required and should NOT be modified by the user. 
**
************************************************************************************
*/

#ifndef SS_H_
#define SS_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "ss_cfg.h"

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

typedef void (*SS_INIT)(int deviceId);
typedef void (*SS_DEINIT)(int deviceId);
typedef int (*SS_GET)(int deviceId, int portId, int pinId, bool *value);
typedef int (*SS_SET)(int deviceId, int portId, int pinId, bool value);
typedef int (*SS_GET_PORTPIN)(int deviceId, int sysPinId, int * portId, int * pinId);

/* 
 * These are the possible return values for this module:
 * 
 *  E_SS_STATUS_OK    - The API call completed successfully 
 *  E_SS_STATUS_ERROR - The API call completed with an error
 */
typedef enum
{
   E_SS_STATUS_OK,
   E_SS_STATUS_ERROR,
}T_SS_STATUS;

/*
 * This structure defines the function pointers required by
 * this module, whose prototypes are defined above. 
 */
typedef struct 
{
    SS_INIT         pfSSInit;
    SS_DEINIT       pfSSDeInit;
    SS_GET          pfSSGet;
    SS_SET          pfSSSet;
    SS_GET_PORTPIN  pfSSGetPortPin;
}T_SS_HW_FP;

/*
 * This is the format of the soft-switch system
 * configuration. Each system pin is tied to a 
 * specific hardware device and thus also tied
 * to a specific set of hardware specific function
 * prototoypes. 
 * 
 * ePinId   - This is one of the system pins defined in 
 *            ss_cfg.h
 * deviceId - This is the hardware device ID, typically
 *            defined by the hardware IC software module. 
 * psHwFP   - A pointer to the function pointers for the 
 *            specific hardware soft-switch device. 
 */
typedef struct
{
    SS_PIN_ID    ePinId;
    int          deviceId;
    T_SS_HW_FP * psHwFP;
}T_SS_PIN_CONFIG;

/***********************************************************************************
 *  GLOBAL PROTOTYPES
 **********************************************************************************/

/******************************************************************************
 *   FUNCTION:     softswitch_init   
 *   DESCRIPTION:  This function verifies the user system soft-switch table
 *                 and performs system soft-switch initialization based
 *                 on the hardware devices provided.
 *              
 *                 This function is REQUIRED to be called during project initialization
 *                 to use this module. It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     psPinConfig - Pointer to the user configuration which defines
 *                               the system soft-switch device configuration. 
 *                               This configuration MUST be sized to the last 
 *                               entry in SS_PIN_ID defined in ss_cfg.h. Blanks
 *                               are NOT allowed in this table.     
 *   OUTPUT(S):    None  
 *   RETURN VALUE: None      
 ******************************************************************************/
extern void softswitch_init(T_SS_PIN_CONFIG * psPinConfig);

/******************************************************************************
 *   FUNCTION:     softswitch_deinit   
 *   DESCRIPTION:  This function uninitializes any system soft-switch devices 
 *                 and clears any internal module paramters.
 *              
 *                 This function is REQUIRED to be called during project initialization
 *                 to use this module. It is NOT thread/context safe.
 *                 Do NOT call this function from an ISR context. 
 *   INPUT(S):     None      
 *   OUTPUT(S):    None  
 *   RETURN VALUE: None      
 ******************************************************************************/
extern void softswitch_deinit(void);

/******************************************************************************
 *   FUNCTION:     softswitch_get   
 *   DESCRIPTION:  This function retrieves the logical pin value from the 
 *                 softswitches hardware device tied to the system pin
 *                 provided
 *   INPUT(S):     ePinId - System pin whose value is to be retrieved      
 *   OUTPUT(S):    bValue - Retrieved logical value
 *   RETURN VALUE: E_SS_STATUS_OK if successful, E_SS_STATUS_ERROR otherwise      
 ******************************************************************************/
extern T_SS_STATUS softswitch_get(SS_PIN_ID ePinId, bool * pbValue);

/******************************************************************************
 *   FUNCTION:     softswitch_set   
 *   DESCRIPTION:  This function sets the softswitch pin to the value provided
 *   INPUT(S):     ePinId - Pin whose value is to be set
 *                 bValue - Value to set
 *   OUTPUT(S):    None  
 *   RETURN VALUE: E_SS_STATUS_OK if successful, E_SS_STATUS_ERROR otherwise         
 ******************************************************************************/
extern T_SS_STATUS softswitch_set(SS_PIN_ID ePinId, bool bValue);

#endif /* SS_H_ */
