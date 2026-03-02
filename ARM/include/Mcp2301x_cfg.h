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
**     Mcp2301x_cfg.h
**
** MODULE FUNCTION:
**      Configuration file for the MCP2301X component to be included by 
**      the user. 
**      This file is required and should be modified by the user.  
**
************************************************************************************
*/

#ifndef MCP2301X_CFG_H_
#define MCP2301X_CFG_H_

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/
 
/* None */

/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

/* 
 * This macro allows for run-time error checking of the read and write pin API 
 * interfaces. This can be used to validate input parameters, etc. at run-time 
 * for invalid settings. If users don't want to validate input parameters at run-time
 * (for extremely critical timing), leave this macro disabled.
 * 
 * Valid options are below (Do NOT remove these!) 
 */
#define C_MCP2301X_RUN_TIME_ERROR_CHECK_DISABLED (0U)
#define C_MCP2301X_RUN_TIME_ERROR_CHECK_ENABLED  (1U)

#define C_MCP2301X_RUN_TIME_ERROR_CHECK          (C_MCP2301X_RUN_TIME_ERROR_CHECK_ENABLED)

/*
 * Defines the available devices for communication
 * with multiple devices. The names of the enumeration values are 
 * defined by the user/integrator of this module, where the
 * naming/format may align to:
 * <E_MCP2301X_DEVICE_ID_xxx>, where xxx is a descriptive 
 * name for the device.
 *
 */
typedef enum
{
   /* User defined enumerations */
   E_MCP2301X_DEVICE_SC594_SOM,      /* EV-SC594-SOM Device                          */
   E_MCP2301X_DEVICE_EZKIT_REV_A,    /* EV-SOMCRR-EZKIT Rev A Device                 */
   E_MCP2301X_DEVICE_EZKIT_REV_D,    /* EV-SOMCRR-EZKIT Rev D Device                 */
   E_MCP2301X_DEVICE_EZKIT_HW_PROBE, /* EV-SOMCRR-EZKIT Config for HW Identification */

   /* Must always be in the list and last */
   E_MCP2301X_DEVICE_ID_MAX  
}T_MCP2301X_DEVICE_ID;

#endif /* MCP2301X_CFG_H_ */
