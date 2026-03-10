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
**     ss.c
**
** MODULE FUNCTION:
**     This is the core implementation for the soft switch module. 
**     This file is required and should NOT be modified by the user. 
**
************************************************************************************
*/

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/

#include "ss.h"

/*******************************************************************************
 *  CONFIGURATION VERIFICATION
 ******************************************************************************/

  /* None */

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

 /* None */

/*******************************************************************************
 *  MODULE VARIABLES
 ******************************************************************************/
 
static bool bInitialized = false;
static T_SS_HW_FP asHwFPLocal[SS_PIN_ID_MAX];
static T_SS_PIN_CONFIG asPinConfigLocal[SS_PIN_ID_MAX];

/***********************************************************************************
 *  FUNCTION PROTOTYPES
 **********************************************************************************/
 
static void ss_reset_config(void);
static T_SS_STATUS ss_verify_config(T_SS_PIN_CONFIG * psPinConfig);
static T_SS_STATUS ss_verify_inputs(SS_PIN_ID ePinId, bool * bValue);
static T_SS_STATUS ss_verify_inputs(SS_PIN_ID ePinId, bool * pbValue);

/******************************************************************************
 *   DESCRIPTION:   Refer to ss.h for detailed description.
 ******************************************************************************/
void softswitch_init(T_SS_PIN_CONFIG * psPinConfig)
{
    uint8_t u8PinIdx;
    uint8_t u8TableIdx;
    T_SS_STATUS eStatus;
    
    /* Local Inits */
    u8PinIdx = 0U;
    
    /* Reset local config table */
    ss_reset_config();
    
    if(psPinConfig != NULL)
    {
        do
        {
            eStatus = ss_verify_config(&psPinConfig[u8PinIdx]);
            if(eStatus == E_SS_STATUS_OK)
            {
                /* Save configuration - Ensuring the table is in order of the enumeration defined in ss_cfg.h */
                u8TableIdx = (uint8_t)psPinConfig[u8PinIdx].ePinId;
                if(asPinConfigLocal[u8TableIdx].ePinId == SS_PIN_ID_MAX)
                {
                    asPinConfigLocal[u8TableIdx].ePinId                 = psPinConfig[u8PinIdx].ePinId;
                    asPinConfigLocal[u8TableIdx].deviceId               = psPinConfig[u8PinIdx].deviceId;
                    asPinConfigLocal[u8TableIdx].psHwFP->pfSSInit       = psPinConfig[u8PinIdx].psHwFP->pfSSInit;
                    asPinConfigLocal[u8TableIdx].psHwFP->pfSSDeInit     = psPinConfig[u8PinIdx].psHwFP->pfSSDeInit;
                    asPinConfigLocal[u8TableIdx].psHwFP->pfSSGet        = psPinConfig[u8PinIdx].psHwFP->pfSSGet;
                    asPinConfigLocal[u8TableIdx].psHwFP->pfSSSet        = psPinConfig[u8PinIdx].psHwFP->pfSSSet;
                    asPinConfigLocal[u8TableIdx].psHwFP->pfSSGetPortPin = psPinConfig[u8PinIdx].psHwFP->pfSSGetPortPin;
                }
                else
                {
                    eStatus = E_SS_STATUS_ERROR;
                }
            }
            u8PinIdx++;
        }while((u8PinIdx < (uint8_t)SS_PIN_ID_MAX) && (eStatus == E_SS_STATUS_OK));
        
        if(eStatus == E_SS_STATUS_OK)
        {
            for(u8PinIdx = 0U; u8PinIdx < SS_PIN_ID_MAX; u8PinIdx++)
            {
                asPinConfigLocal[u8PinIdx].psHwFP->pfSSInit(asPinConfigLocal[u8PinIdx].deviceId);
            }
            
            bInitialized = true;
        }
        else
        {
            ss_reset_config();
        }
    }
}

/******************************************************************************
 *   DESCRIPTION:   Refer to ss.h for detailed description.
 ******************************************************************************/
void softswitch_deinit(void)
{
    uint8_t u8PinIdx;
    
    for(u8PinIdx = 0U; u8PinIdx < SS_PIN_ID_MAX; u8PinIdx++)
    {
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSDeInit(asPinConfigLocal[u8PinIdx].deviceId);
    }
    
    ss_reset_config();
    
    bInitialized = false;
}

/******************************************************************************
 *   DESCRIPTION:   Refer to ss.h for detailed description.
 ******************************************************************************/
T_SS_STATUS softswitch_get(SS_PIN_ID ePinId, bool * pbValue)
{
    int  pinNum;
    int  portNum;
    int  deviceId;
    T_SS_STATUS eStatus;
    
    eStatus = ss_verify_inputs(ePinId, pbValue);
    
    if(eStatus == E_SS_STATUS_OK)
    {
        deviceId = asPinConfigLocal[ePinId].deviceId;
        eStatus  = (T_SS_STATUS)asPinConfigLocal[ePinId].psHwFP->pfSSGetPortPin(deviceId, (int)ePinId, &portNum, &pinNum);
        if(eStatus == E_SS_STATUS_OK)
        {
            eStatus = (T_SS_STATUS)asPinConfigLocal[ePinId].psHwFP->pfSSGet(deviceId, portNum, pinNum, pbValue);
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to ss.h for detailed description.
 ******************************************************************************/
T_SS_STATUS softswitch_set(SS_PIN_ID ePinId, bool bValue)
{
    int  pinNum;
    int  portNum;
    int  deviceId;
    T_SS_STATUS eStatus;
    
    eStatus = ss_verify_inputs(ePinId, &bValue);
    
    if(eStatus == E_SS_STATUS_OK)
    {
        deviceId = asPinConfigLocal[ePinId].deviceId;
        eStatus  = (T_SS_STATUS)asPinConfigLocal[ePinId].psHwFP->pfSSGetPortPin(deviceId, (int)ePinId, &portNum, &pinNum);
        if(eStatus == E_SS_STATUS_OK)
        {
            eStatus = (T_SS_STATUS)asPinConfigLocal[ePinId].psHwFP->pfSSSet(deviceId, portNum, pinNum, bValue);
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     ss_verify_inputs   
 *   DESCRIPTION:  This function verifies common input parameters
 *   INPUT(S):     ePinId - System pin ID to verify
 *                 bValue - User pointer to confirm not NULL 
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_SS_STATUS      
 ******************************************************************************/
static T_SS_STATUS ss_verify_inputs(SS_PIN_ID ePinId, bool * bValue)
{
    T_SS_STATUS eStatus;
    
    /* Local Inits */
    eStatus = E_SS_STATUS_ERROR;
    
    if(bInitialized == true)
    {
        if(ePinId < SS_PIN_ID_MAX)
        {
            if(bValue != NULL)
            {
                eStatus = E_SS_STATUS_OK;
            }
        }
    }

    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     ss_reset_config   
 *   DESCRIPTION:  This function resets the local pin configuration table
 *   INPUT(S):     None      
 *   OUTPUT(S):    None  
 *   RETURN VALUE: None      
 ******************************************************************************/
static void ss_reset_config(void)
{
    uint8_t u8PinIdx;
    
    for(u8PinIdx = 0U; u8PinIdx < SS_PIN_ID_MAX; u8PinIdx++)
    {
        asPinConfigLocal[u8PinIdx].ePinId                 = SS_PIN_ID_MAX;
        asPinConfigLocal[u8PinIdx].deviceId               = 0;
        asPinConfigLocal[u8PinIdx].psHwFP                 = (T_SS_HW_FP *)&asHwFPLocal[u8PinIdx];
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSInit       = NULL;
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSDeInit     = NULL;
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSGet        = NULL;
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSSet        = NULL;
        asPinConfigLocal[u8PinIdx].psHwFP->pfSSGetPortPin = NULL;
    }
}

/******************************************************************************
 *   FUNCTION:     ss_verify_config   
 *   DESCRIPTION:  This function verifies the user configuration table for
 *                 valid entries
 *   INPUT(S):     psPinConfig - Pointer to user's configuration table      
 *   OUTPUT(S):    None  
 *   RETURN VALUE: T_SS_STATUS      
 ******************************************************************************/
static T_SS_STATUS ss_verify_config(T_SS_PIN_CONFIG * psPinConfig)
{
    T_SS_STATUS eStatus;

    /* Local Inits */
    eStatus = E_SS_STATUS_ERROR;

    if(psPinConfig->ePinId < SS_PIN_ID_MAX)
    {
        if(psPinConfig->deviceId >= 0)
        {
            if(psPinConfig->psHwFP != NULL)
            {
                if(psPinConfig->psHwFP->pfSSInit != NULL)
                {
                    if(psPinConfig->psHwFP->pfSSGet != NULL)
                    {
                        if(psPinConfig->psHwFP->pfSSSet != NULL)
                        {
                            if(psPinConfig->psHwFP->pfSSDeInit != NULL)
                            { 
                                if(psPinConfig->psHwFP->pfSSGetPortPin != NULL)
                                {
                                    eStatus = E_SS_STATUS_OK;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return(eStatus);
}

