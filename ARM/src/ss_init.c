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

#include "ss.h"
#include "ss_init.h"
#include "Mcp2301x.h"

/* 
 * HW Specific Prototypes - Note in this case there is only one set
 * of HW specific API as all available soft switches use the same
 * hardware IC. If more ICs are added, or changed, those 
 * prototypes and specific implementations should be handled below
 * and each pins should be correspondingly mapped to that specific
 * HW implementation via the T_SS_HW_FP structure.
 */
static void ss_hw_init(int deviceId);
static void ss_hw_deinit(int deviceId);
static int ss_hw_set(int deviceId, int portId, int pinId, bool value);
static int ss_hw_get(int deviceId, int portId, int pinId, bool *value);
static int ss_hw_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId);

static const T_SS_HW_FP sSSFP = 
{
    .pfSSInit       = ss_hw_init, 
    .pfSSDeInit     = ss_hw_deinit, 
    .pfSSGet        = ss_hw_get, 
    .pfSSSet        = ss_hw_set, 
    .pfSSGetPortPin = ss_hw_get_portpin
};

/* 
 * Hardware Configuration for all available pins on all soft switches -
 * Noting that anything that is switchable due to hardware differences in 
 * SOM or EZKIT revisions of the same type are updated during initialization 
 * below.
 */
static T_SS_PIN_CONFIG asSSPinConfigRegistry[SS_PIN_ID_MAX] = 
{
   /* Carrier */
   {.ePinId = SS_PIN_ID_nADAU1979_EN,      .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nADAU_1962_EN,     .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nADAU_RESET,       .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nCAN_EN,           .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nFTDI_USB_EN,      .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nMicroSD_SPI,      .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_PUSHBUTTON_EN,     .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_EEPROM_EN,         .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nGIGe_RESET,       .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nETH1_RESET,       .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nETH1_EN,          .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nMLB_EN,           .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_AUDIO_JACK_SEL,    .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nSPDIF_OPTICAL_EN, .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nSPDIF_DIGITAL_EN, .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_OCTAL_SPI_CS_EN,   .deviceId = -1,                               .psHwFP = (T_SS_HW_FP *)&sSSFP},
   /* SOM */
   {.ePinId = SS_PIN_ID_nOSPIFLASH_CS_EN,  .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nUART0_FLOW_EN,    .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nUART0_EN,         .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nSPID2_D3_EN,      .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_nSPI2FLASH_CS_EN,  .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_LED4,              .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_LED2,              .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
   {.ePinId = SS_PIN_ID_LED5,              .deviceId = (int)E_MCP2301X_DEVICE_SC594_SOM, .psHwFP = (T_SS_HW_FP *)&sSSFP},
};

bool ss_get(APP_CONTEXT *context, int pinId, bool *value)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;

    if(softswitch_get(pinId, value) == E_SS_STATUS_OK)
    {
        bSuccess = true;
    }

    return(bSuccess);
}

bool ss_set(APP_CONTEXT *context, int pinId, bool value)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;

    if(softswitch_set(pinId, value) == E_SS_STATUS_OK)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

void ss_init(APP_CONTEXT *context)
{
    uint8_t u8PinIdx;
    int     hwDeviceId;
    
    if(context->SoMCRRVersion == SOMCRR_REV_D)
    {
        hwDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_REV_D;
    }
    else if(context->SoMCRRVersion == SOMCRR_REV_A)
    {
        hwDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_REV_A;
    }
    else
    {
        hwDeviceId = (int)E_MCP2301X_DEVICE_EZKIT_HW_PROBE;
    }
    
    for(u8PinIdx = 0U; u8PinIdx < SS_PIN_ID_MAX; u8PinIdx++)
    {
        if((u8PinIdx >= SS_PIN_ID_nADAU1979_EN) && (u8PinIdx < SS_PIN_ID_nOSPIFLASH_CS_EN))
        {
            asSSPinConfigRegistry[u8PinIdx].deviceId = hwDeviceId;
        }
    }
    
    softswitch_init((T_SS_PIN_CONFIG *)&asSSPinConfigRegistry);
}

void ss_deinit(APP_CONTEXT *context)
{
    softswitch_deinit();
}

/******************************************************************************
 *   Static Helpers
 ******************************************************************************/
static void ss_hw_init(int deviceId)
{
    (void)Mcp2301x_Init(deviceId);
}

static void ss_hw_deinit(int deviceId)
{
    (void)Mcp2301x_DeInit(deviceId);
}

static int ss_hw_get(int deviceId, int portId, int pinId, bool *value)
{
    return((int)Mcp2301x_ReadPin(deviceId, portId, pinId, value));
}

static int ss_hw_set(int deviceId, int portId, int pinId, bool value)
{
    return((int)Mcp2301x_WritePin(deviceId, portId, pinId, value));
}

static int ss_hw_get_portpin(int deviceId, int sysPinId, int * portId, int * pinId)
{
    return((int)Mcp2301x_GetPortPin(deviceId, sysPinId, (T_MCP2301X_PORT * const)portId, (T_MCP2301X_PIN * const)pinId));
}
