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
**     Mcp2301x.c
**
** MODULE FUNCTION:
**     This is the core implementation for the MCP2301X module. 
**     This file is required and should NOT be modified by the user. 
**
************************************************************************************
*/

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/

#include "Mcp2301x.h"
#include "Mcp2301x_Private.h"

/*******************************************************************************
 *  CONFIGURATION VERIFICATION
 ******************************************************************************/

#if !defined(C_MCP2301X_RUN_TIME_ERROR_CHECK)
    #error "Missing C_MCP2301X_RUN_TIME_ERROR_CHECK definition in Mcp2301x_cfg.h!"
#elif (C_MCP2301X_RUN_TIME_ERROR_CHECK > C_MCP2301X_RUN_TIME_ERROR_CHECK_ENABLED)
    #error "Invalid configuration setting for C_MCP2301X_RUN_TIME_ERROR_CHECK. See Mcp2301x_cfg.h for details!"
#endif

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

#define C_MCP2301X_NUM_REGS   (22U)
#define C_NUM_IOCON_BANK_REGS (4U)

/* These are the possible addresses of the bank registers.
 * Note that if the bank is currently swapped, there is no
 * harm in setting the non-bank registers to 0 as 0 is the 
 * expected POR settings anyway.
 */
#define C_IOCONA_BANK0_ADDR   (0x0AU)
#define C_IOCONB_BANK0_ADDR   (0x0BU)
#define C_IOCONA_BANK1_ADDR   (0x05U)
#define C_IOCONB_BANK1_ADDR   (0x15U)
#define C_IOCON_BANK_MASK     (0x80U)

/* Configurable registers */
#define C_IODIRA_ADDR         (0x00U)
#define C_GPPUA_ADDR          (0x0CU)
#define C_GPIOA_ADDR          (0x12U)
#define C_GPIOB_ADDR          (0x13U)
#define C_REG_LOWER_MASK      (0x00FFU)
#define C_REG_UPPER_MASK      (0xFF00U)  

/*******************************************************************************
 *  MODULE VARIABLES
 ******************************************************************************/
 
static bool abInitialized[E_MCP2301X_DEVICE_ID_MAX]     = {false};
static const uint8_t au8PortAddr[E_MCP2301X_PORT_MAX]   = {C_GPIOA_ADDR, C_GPIOB_ADDR};
static const uint8_t au8BankAddr[C_NUM_IOCON_BANK_REGS] = {C_IOCONA_BANK0_ADDR, C_IOCONB_BANK0_ADDR, C_IOCONA_BANK1_ADDR, C_IOCONB_BANK1_ADDR};

/***********************************************************************************
 *  FUNCTION PROTOTYPES
 **********************************************************************************/

static T_MCP2301X_STATUS Mcp2301x_ResetBank(const T_MCP2301X_DEVICE_ID eDeviceId);
static T_MCP2301X_STATUS Mcp2301x_ResetDevice(const T_MCP2301X_DEVICE_ID eDeviceId);
static T_MCP2301X_STATUS Mcp2301x_ConfigureDevice(const T_MCP2301X_DEVICE_ID eDeviceId);
static T_MCP2301X_STATUS Mcp2301x_VerifyInputs(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, const bool * const pbPinVal);

/******************************************************************************
 *   DESCRIPTION:   Refer to Mcp2301x.h for detailed description.
 ******************************************************************************/
T_MCP2301X_STATUS Mcp2301x_Init(const T_MCP2301X_DEVICE_ID eDeviceId)
{
    T_MCP2301X_STATUS eStatus;
    
    if(eDeviceId < E_MCP2301X_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == false)
        {
            /* Ensure Bank option is set to paired mode (BANK=0) */
            eStatus = Mcp2301x_ResetBank(eDeviceId);
            
            /* Reset the registers to their default values */
            if(eStatus == E_MCP2301X_STATUS_OK)
            {
                eStatus = Mcp2301x_ResetDevice(eDeviceId);
                
                /* Configure the device */
                if(eStatus == E_MCP2301X_STATUS_OK)
                {
                    eStatus = Mcp2301x_ConfigureDevice(eDeviceId);
                    if(eStatus == E_MCP2301X_STATUS_OK)
                    {
                        abInitialized[eDeviceId] = true;
                    }
                }
            }
        }
        else
        {
            eStatus = E_MCP2301X_STATUS_ALREADY_INITIALIZED;
            Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_ALREADY_INITIALIZED);
        }
    }
    else
    {
        eStatus = E_MCP2301X_STATUS_INVALID_DEVICE_ID;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Mcp2301x.h for detailed description.
 ******************************************************************************/
T_MCP2301X_STATUS Mcp2301x_DeInit(const T_MCP2301X_DEVICE_ID eDeviceId)
{
    T_MCP2301X_STATUS eStatus;
    
    if(eDeviceId < E_MCP2301X_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            eStatus = Mcp2301x_ResetDevice(eDeviceId);
            
            if(eStatus == E_MCP2301X_STATUS_OK)
            {
                abInitialized[eDeviceId] = false;
            }
        }
        else
        {
            eStatus = E_MCP2301X_STATUS_ALREADY_UNINITIALIZED;
            Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_ALREADY_UNINITIALIZED);
        }
    }
    else
    {
        eStatus = E_MCP2301X_STATUS_INVALID_DEVICE_ID;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Mcp2301x.h for detailed description.
 ******************************************************************************/
T_MCP2301X_STATUS Mcp2301x_ReadPin(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, bool * const pbPinVal)
{
    T_MCP2301X_STATUS eStatus;
    uint8_t           au8ReadBuff[1];
    uint8_t           au8WriteBuff[1];
    
    eStatus = Mcp2301x_VerifyInputs(eDeviceId, ePort, ePin, pbPinVal);
    
    if(eStatus == E_MCP2301X_STATUS_OK)
    {
        au8WriteBuff[0] = au8PortAddr[ePort];
        if(Mcp2301x_CfgIf_I2cRead(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff), au8ReadBuff, sizeof(au8ReadBuff)) == true)
        {
            *pbPinVal = (bool)((au8ReadBuff[0] & (1U << ((uint8_t)ePin))) >> (uint8_t)ePin);
        }
        else
        {
            eStatus = E_MCP2301X_STATUS_CFGIF_ERROR;
            Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_CFGIF_ERROR);
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Mcp2301x.h for detailed description.
 ******************************************************************************/
T_MCP2301X_STATUS Mcp2301x_WritePin(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, const bool bPinVal)
{
    T_MCP2301X_STATUS eStatus;
    uint8_t           u8PinMask;
    uint8_t           au8ReadBuff[1];
    uint8_t           au8WriteBuff[2];
    
    eStatus = Mcp2301x_VerifyInputs(eDeviceId, ePort, ePin, &bPinVal);
    
    if(eStatus == E_MCP2301X_STATUS_OK)
    {
        /* Read back data first to mask with requested value */
        au8WriteBuff[0] = au8PortAddr[ePort];
        if(Mcp2301x_CfgIf_I2cRead(eDeviceId, au8WriteBuff, 1U, au8ReadBuff, 1U) == true)
        {
            if(bPinVal == true)
            {
                u8PinMask = au8ReadBuff[0] | (1U << (uint8_t)ePin);
            }
            else
            {
                u8PinMask = au8ReadBuff[0] & (~(1U << (uint8_t)ePin));
            }
            
            /* Write the data */
            au8WriteBuff[0] = au8PortAddr[ePort];
            au8WriteBuff[1] = u8PinMask;
            if(Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) == false)
            {
                eStatus = E_MCP2301X_STATUS_CFGIF_ERROR;
                Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_CFGIF_ERROR);
            }
        }
    }
    
    return(eStatus);
}

/******************************************************************************
 *   DESCRIPTION:   Refer to Mcp2301x.h for detailed description.
 ******************************************************************************/
T_MCP2301X_STATUS Mcp2301x_GetPortPin(const T_MCP2301X_DEVICE_ID eDeviceId, int32_t s32PinId, T_MCP2301X_PORT * const pePort, T_MCP2301X_PIN * const pePin)
{
    bool bPinFound;
    uint16_t u16PinMapIdx;
    uint16_t u16PinMapLength;
    T_MCP2301X_STATUS eStatus;
    
    if(eDeviceId < E_MCP2301X_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            /* Search for a valid port and pin setting from the system map */
            if(asMcp2301xConfigRegistry[eDeviceId].psPinMap != NULL)
            {
                u16PinMapIdx    = 0U;
                bPinFound       = false;
                u16PinMapLength = asMcp2301xConfigRegistry[eDeviceId].u16PinMapLength;
                do
                {
                    if(s32PinId == asMcp2301xConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].s32PinId)
                    {
                        bPinFound = true;
                        *pePort   = asMcp2301xConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].ePort;
                        *pePin    = asMcp2301xConfigRegistry[eDeviceId].psPinMap[u16PinMapIdx].ePin;
                        eStatus   = E_MCP2301X_STATUS_OK;
                    }
                    u16PinMapIdx++;
                }while((bPinFound == false) && (u16PinMapIdx < u16PinMapLength));

                if(bPinFound == false)
                {
                    eStatus = E_MCP2301X_STATUS_PIN_NOT_FOUND;
                    Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_PIN_NOT_FOUND);
                }
            }
            else
            {
                eStatus = E_MCP2301X_STATUS_NO_SYSTEM_MAP;
                Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_NO_SYSTEM_MAP);
            }
        }
        else
        {
            eStatus = E_MCP2301X_STATUS_NOT_INITALIZED;
            Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_NOT_INITALIZED);
        }
    }
    else
    {
        eStatus = E_MCP2301X_STATUS_INVALID_DEVICE_ID;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_DEVICE_ID);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Mcp2301x_ConfigureDevice
 *   DESCRIPTION:  Writes the device configurations for the specific hardware
 *                 device
 *   INPUT(S):     eDeviceId - Device to configure
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
static T_MCP2301X_STATUS Mcp2301x_ConfigureDevice(const T_MCP2301X_DEVICE_ID eDeviceId)
{
    T_MCP2301X_STATUS eStatus;
    bool              bSuccess;
    uint8_t           au8WriteBuff[3];
    
    /* Local Inits */
    eStatus = E_MCP2301X_STATUS_OK;
        
    /* 
     * Write the device configuration - setting final direction in/out last to avoid 
     * spurious behavior on the outputs when changing the pins state/characteristics.
     */
    au8WriteBuff[0] = C_GPPUA_ADDR;
    au8WriteBuff[1] = (uint8_t)(asMcp2301xConfigRegistry[eDeviceId].u16PullUpEnMask & C_REG_LOWER_MASK);
    au8WriteBuff[2] = (uint8_t)((asMcp2301xConfigRegistry[eDeviceId].u16PullUpEnMask & C_REG_UPPER_MASK) >> 8U);
    bSuccess        = Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff));
    
    au8WriteBuff[0] = C_GPIOA_ADDR;
    au8WriteBuff[1] = (uint8_t)(asMcp2301xConfigRegistry[eDeviceId].u16PinInitValueMask & C_REG_LOWER_MASK);
    au8WriteBuff[2] = (uint8_t)((asMcp2301xConfigRegistry[eDeviceId].u16PinInitValueMask & C_REG_UPPER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;

    au8WriteBuff[0] = C_IODIRA_ADDR;
    au8WriteBuff[1] = (uint8_t)(asMcp2301xConfigRegistry[eDeviceId].u16PinDirMask & C_REG_LOWER_MASK);
    au8WriteBuff[2] = (uint8_t)((asMcp2301xConfigRegistry[eDeviceId].u16PinDirMask & C_REG_UPPER_MASK) >> 8U);
    bSuccess        = (bSuccess == true) ? Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff)) : false;
    
    if(bSuccess == false)
    {
        eStatus = E_MCP2301X_STATUS_CFGIF_ERROR;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_CFGIF_ERROR);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Mcp2301x_ResetBank
 *   DESCRIPTION:  Resets device bank parameters to the known default setting
 *   INPUT(S):     eDeviceId - Device whose bank is to be reset
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
static T_MCP2301X_STATUS Mcp2301x_ResetBank(const T_MCP2301X_DEVICE_ID eDeviceId)
{
    T_MCP2301X_STATUS eStatus;
    bool              bSuccess;
    uint8_t           u8BankAddrIdx;
    uint8_t           au8WriteBuff[2];
    
    /* Local Inits */
    eStatus       = E_MCP2301X_STATUS_OK;
    u8BankAddrIdx = 0U;
    
    do
    {
        /* Ensure all possibilites of BANK are set to 0 */
        au8WriteBuff[0] = au8BankAddr[u8BankAddrIdx];
        au8WriteBuff[1] = 0x00U;
        bSuccess        = Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff));
        u8BankAddrIdx++;
    }while((bSuccess == true) && (u8BankAddrIdx < C_NUM_IOCON_BANK_REGS));
    
    if(bSuccess == false)
    {
        eStatus = E_MCP2301X_STATUS_CFGIF_ERROR;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_CFGIF_ERROR);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Mcp2301x_ResetDevice
 *   DESCRIPTION:  Resets device to POR values
 *   INPUT(S):     eDeviceId - Device to reset
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
static T_MCP2301X_STATUS Mcp2301x_ResetDevice(const T_MCP2301X_DEVICE_ID eDeviceId)
{
    T_MCP2301X_STATUS eStatus;
    bool              bSuccess;
    uint8_t           u8RegAddr;
    uint8_t           u8RegValue;
    uint8_t           au8WriteBuff[2];
    
    /* Local Inits */
    eStatus   = E_MCP2301X_STATUS_OK;
    u8RegAddr = 0U;
    
    do
    {
        u8RegValue      = ((u8RegAddr == 0U) || (u8RegAddr == 1U)) ? 0xFFU : 0U;
        au8WriteBuff[0] = u8RegAddr;
        au8WriteBuff[1] = u8RegValue;
        bSuccess        = Mcp2301x_CfgIf_I2cWrite(eDeviceId, au8WriteBuff, sizeof(au8WriteBuff));
        u8RegAddr++;
    }while((bSuccess == true) && (u8RegAddr < C_MCP2301X_NUM_REGS));
    
    if(bSuccess == false)
    {
        eStatus = E_MCP2301X_STATUS_CFGIF_ERROR;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_CFGIF_ERROR);
    }
    
    return(eStatus);
}

/******************************************************************************
 *   FUNCTION:     Mcp2301x_VerifyInputs
 *   DESCRIPTION:  Verifies common API inputs for validity
 *   INPUT(S):     eDeviceId - The specific device being requested
 *   OUTPUT(S):    None
 *   RETURN VALUE: T_MCP2301X_STATUS
 ******************************************************************************/
static T_MCP2301X_STATUS Mcp2301x_VerifyInputs(const T_MCP2301X_DEVICE_ID eDeviceId, const T_MCP2301X_PORT ePort, const T_MCP2301X_PIN ePin, const bool * const pbPinVal)
{
    T_MCP2301X_STATUS eStatus;

#if(C_MCP2301X_RUN_TIME_ERROR_CHECK == C_MCP2301X_RUN_TIME_ERROR_CHECK_ENABLED)
    if(eDeviceId < E_MCP2301X_DEVICE_ID_MAX)
    {
        if(abInitialized[eDeviceId] == true)
        {
            if(ePort < E_MCP2301X_PORT_MAX)
            {
                if(ePin < E_MCP2301X_PIN_MAX)
                {
                    if(pbPinVal != NULL)
                    {
                        eStatus = E_MCP2301X_STATUS_OK;
                    }
                    else
                    {
                        eStatus = E_MCP2301X_STATUS_NULL_PTR;
                        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_NULL_PTR);
                    }
                }
                else
                {
                    eStatus = E_MCP2301X_STATUS_INVALID_PIN_ID;
                    Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_PIN_ID);
                }
            }
            else
            {
                eStatus = E_MCP2301X_STATUS_INVALID_PORT_ID;
                Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_PORT_ID);
            }
        }
        else
        {
            eStatus = E_MCP2301X_STATUS_NOT_INITALIZED;
            Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_NOT_INITALIZED);
        }
    }
    else
    {
        eStatus = E_MCP2301X_STATUS_INVALID_DEVICE_ID;
        Mcp2301x_CfgIf_ReportStatus(E_MCP2301X_STATUS_INVALID_DEVICE_ID);
    }
#else
    eStatus = E_MCP2301X_STATUS_OK;
#endif
    
    return(eStatus);
}
