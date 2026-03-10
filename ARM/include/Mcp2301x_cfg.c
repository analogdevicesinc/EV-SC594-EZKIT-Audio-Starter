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
**     Mcp2301x_cfg.c
**
** MODULE FUNCTION:
**      This is the project specific configurable portion of the Mcp2301x 
**      component. This file includes configurable interface functions which
**      should be filled in based on the users project specific needs as well
**      as the device specific registration table and pin mapping for each 
**      device. 
**      This file is required and should be modified by the user. 
**
************************************************************************************
*/

/***********************************************************************************
 *  INCLUDED HEADERS
 **********************************************************************************/

#include "Mcp2301x.h"

/* Project specific includes */
#include "context.h"
#include "twi_simple.h"
#include "ss.h"
      
/***********************************************************************************
 *  DEFINITIONS
 **********************************************************************************/

/***********************************************************************************
 *                          SC594 SOM Details
 **********************************************************************************/
  /*
    Port B                                |      Port A                                     
    7--------------- NOT USED             |      7--------------- ~OSPIFLASH_CS_EN        
    | 6------------- NOT USED             |      | 6------------- ~UART0_FLOW_EN          
    | | 5----------- NOT USED             |      | | 5----------- ~UART0_EN               
    | | | 4--------- NOT USED             |      | | | 4--------- ~SPID2_D3_EN            
    | | | | 3------- NOT USED             |      | | | | 3------- ~SPI2FLASH_CS_EN        
    | | | | | 2----- NOT USED             |      | | | | | 2----- LED2                    
    | | | | | | 1--- NOT USED             |      | | | | | | 1--- LED5                    
    | | | | | | | 0- NOT USED             |      | | | | | | | 0- LED4                    
    X X X X X X X X                       |      N N Y Y Y N N N            ( Active Y or N - After applying these settings )   
    1 1 1 1 1 1 1 1  (0xFF)               |      0 0 0 0 0 0 0 0 (0x00)     ( DIR_VALUE  = GPIO Direction 1 = Input, 0 = Ouput )
    0 0 0 0 0 0 0 0  (0x00)               |      1 1 0 0 0 0 0 0 (0xC0)     ( INIT_VALUE = Value being set )                   
  */

/* Initial values are U16 value or'd values from above 8-bit values - PORTB | PORTA */
#define C_SC594_DIR_VALUE       (0xFF00U)
#define C_SC594_INIT_VALUE      (0x00C0U)

/***********************************************************************************
 *                          EZKIT Rev A. Details
 **********************************************************************************/

/*
ADAU1962 info:
  ADAU_RESET (SS,OUT) -> /RST       (connected by ~ADAU1962_EN SS, pull-up)
  ADAU1962A_CLKIN     -> DAI1_P03   (connected by ~ADAU1962_EN SS, pull-up)
  DAI1_P05            -> BCLK       (connected by ~ADAU1962_EN SS)
  DAI1_P04            -> LRCLK      (connected by ~ADAU1962_EN SS)
  DAI1_P01            -> DSDATA1    (connected by ~ADAU1962_EN SS)
  DAI1_P02            -> DSDATA2    (connected by ~ADAU1962_EN SS)
  DAI1_P10            -> DSDATA3    (connected by ~ADAU1962_EN SS)
  Analog OUT 1-2      -> J17
  Analog OUT 3-4      -> J18
  Analog OUT 5-6      -> J19
  Analog OUT 7-8      -> J20
  Analog OUT 9-10     -> J12        (connected by AUDIO_JACK_SEL SS set low, pull-up)
  Analog OUT 11-12    -> J16        (connected by AUDIO_JACK_SEL SS set low, pull-up)
  TWI2_SCL            -> SCL
  TWI2_SDA            -> SDA

ADAU1979 info:
  ADAU_RESET (SS,OUT) -> /RST       (connected by ~ADAU1979_EN SS, pull-up)
  DAI1_P20            -> LRCLK      (connected by ~ADAU1979_EN SS)
  DAI1_P12            -> BCLK       (connected by ~ADAU1979_EN SS)
  DAI1_P06            -> SDATAOUT1  (connected by ~ADAU1979_EN SS)
  DAI1_P07            -> SDATAOUT2  (connected by ~ADAU1979_EN SS)
  TWI2_SCL            -> SCL
  TWI2_SDA            -> SDA
  Analog IN 1-2       -> J12        (connected by AUDIO_JACK_SEL SS set high, pull-up)
  Analog IN 3-4       -> J16        (connected by AUDIO_JACK_SEL SS set high, pull-up)
*/

 /*
    Port B                                        Port A                                      
    7--------------- ~GIGe_RESET          |       7--------------- ~ADAU1979_EN           
    | 6------------- ~ETH1_RESET          |       | 6------------- ~ADAU_1962_EN         
    | | 5----------- ~ETH1_EN             |       | | 5----------- ~ADAU_RESET           
    | | | 4--------- ~MLB_EN              |       | | | 4--------- ~CAN_EN               
    | | | | 3------- AUDIO_JACK_SEL       |       | | | | 3------- ~FTDI_USB_EN          
    | | | | | 2----- ~SPDIF_OPTICAL_EN    |       | | | | | 2----- ~MicroSD_SPI          
    | | | | | | 1--- ~SPDIF_DIGITAL_EN    |       | | | | | | 1--- PUSHBUTTON_EN         
    | | | | | | | 0- OCTAL_SPI_CS_EN      |       | | | | | | | 0- EEPROM_EN   
    | | | | | | | |                       |       | | | | | | | |                        
    Y Y N N Y Y N N                       |       Y Y N N N N Y Y           ( Active Y or N - After applying these settings )  
    0 0 0 0 0 0 0 0  (0x00)               |       0 0 0 0 0 0 0 0 (0x00)    ( DIR_VALUE  = GPIO Direction 1 = Input, 0 = Ouput )                
    0 0 1 1 1 0 1 0  (0x3A)               |       0 0 1 1 1 1 1 1 (0x3F)    ( INIT_VALUE = Value being set )   
*/

/* Initial values are U16 value or'd values from above 8-bit values - PORTB | PORTA */
#define C_EZKIT_REVA_DIR_VALUE       (0x0000U)
#define C_EZKIT_REVA_INIT_VALUE      (0x3A3FU)

/***********************************************************************************
 *                          EZKIT Rev D. Details
 **********************************************************************************/
/*
ADAU1962 info:
  ADAU_RESET (SS,OUT) -> /RST       (connected by ~ADAU1962_EN SS, pull-up)
  ADAU1962A_CLKIN     -> DAI1_P03   (connected by ~ADAU1962_EN SS, pull-up)
  DAI1_P05            -> BCLK       (connected by ~ADAU1962_EN SS)
  DAI1_P04            -> LRCLK      (connected by ~ADAU1962_EN SS)
  DAI1_P01            -> DSDATA1    (connected by ~ADAU1962_EN SS)
  DAI1_P02            -> DSDATA2    (connected by ~ADAU1962_EN SS)
  DAI1_P10            -> DSDATA3    (connected by ~ADAU1962_EN SS)
  Analog OUT 1-2      -> J17
  Analog OUT 3-4      -> J18
  Analog OUT 5-6      -> J19
  Analog OUT 7-8      -> J20
  Analog OUT 9-10     -> J22
  Analog OUT 11-12    -> J23
  TWI2_SCL            -> SCL
  TWI2_SDA            -> SDA

ADAU1979 info:
  ADAU_RESET (SS,OUT) -> /RST       (connected by ~ADAU1979_EN SS, pull-up)
  DAI1_P20            -> LRCLK      (connected by ~ADAU1979_EN SS)
  DAI1_P12            -> BCLK       (connected by ~ADAU1979_EN SS)
  DAI1_P06            -> SDATAOUT1  (connected by ~ADAU1979_EN SS)
  DAI1_P07            -> SDATAOUT2  (connected by ~ADAU1979_EN SS)
  TWI2_SCL            -> SCL
  TWI2_SDA            -> SDA
  Analog IN 1-2       -> J12
  Analog IN 3-4       -> J16
*/

 /*
    Port B                                        Port A                                      
    7--------------- ~GIGe_RESET          |       7--------------- ~ADAU1979_EN           
    | 6------------- ~ETH1_RESET          |       | 6------------- ~ADAU_1962_EN         
    | | 5----------- ~ETH1_EN             |       | | 5----------- ~ADAU_RESET           
    | | | 4--------- ~MLB_EN              |       | | | 4--------- ~CAN_EN               
    | | | | 3------- NOT USED             |       | | | | 3------- ~FTDI_USB_EN          
    | | | | | 2----- ~SPDIF_OPTICAL_EN    |       | | | | | 2----- ~MicroSD_SPI          
    | | | | | | 1--- ~SPDIF_DIGITAL_EN    |       | | | | | | 1--- ~PUSHBUTTON_EN         
    | | | | | | | 0- ~OCTAL_SPI_CS_EN     |       | | | | | | | 0- ~EEPROM_EN   
    | | | | | | | |                       |       | | | | | | | |                        
    Y Y N N X Y N N                       |       Y Y N N N N Y Y           ( Active Y or N - After applying these settings )  
    0 0 0 0 1 0 0 0  (0x08)               |       0 0 0 0 0 0 0 0 (0x00)    ( DIR_VALUE  = GPIO Direction 1 = Input, 0 = Ouput )                
    0 0 1 1 0 0 1 1  (0x33)               |       0 0 1 1 1 1 0 0 (0x3C)    ( INIT_VALUE = Value being set )   
*/

/* Initial values are U16 value or'd values from above 8-bit values - PORTB | PORTA */
#define C_EZKIT_REVD_DIR_VALUE       (0x0800U)
#define C_EZKIT_REVD_INIT_VALUE      (0x333CU)

/***********************************************************************************
 *                          EZKIT Hardware Probing Details
 **********************************************************************************/

 /*
    Port B                                        Port A                                      
    7--------------- NOT USED             |       7--------------- NOT USED           
    | 6------------- NOT USED             |       | 6------------- NOT USED         
    | | 5----------- NOT USED             |       | | 5----------- NOT USED           
    | | | 4--------- NOT USED             |       | | | 4--------- NOT USED               
    | | | | 3------- NOT USED             |       | | | | 3------- NOT USED          
    | | | | | 2----- NOT USED             |       | | | | | 2----- NOT USED          
    | | | | | | 1--- NOT USED             |       | | | | | | 1--- NOT USED         
    | | | | | | | 0- (~)OCTAL_SPI_CS_EN   |       | | | | | | | 0- NOT USED  
    | | | | | | | |                       |       | | | | | | | |                        
    X X X X X X X Y                       |       X X X X X X X X           ( Active Y or N - After applying these settings )  
    1 1 1 1 1 1 1 0  (0xFE)               |       1 1 1 1 1 1 1 1 (0xFF)    ( DIR_VALUE  = GPIO Direction 1 = Input, 0 = Ouput )                
    0 0 0 0 0 0 0 0  (0x00)               |       0 0 0 0 0 0 0 0 (0x00)    ( INIT_VALUE = Value being set )   
*/

/* 
 * Initial values are U16 value or'd values from above 8-bit values - PORTB | PORTA 
 * These are the minimal settings required for a hardware probe to determine the 
 * Carrier version. Supported carrier variants are EZKIT Rev A. and EZKIT Rev D. 
 * These carriers can be distinguished by the polarity of OCTAL_SPI_CS_EN which is
 * active/enabled when high (Rev A.) and active/enabled when low (Rev D.)
 */
#define C_EZKIT_PROBE_DIR_VALUE       (0xFEFFU)
#define C_EZKIT_PROBE_INIT_VALUE      (0x0000U)

/***********************************************************************************
 *  FUNCTION PROTOTYPES
 **********************************************************************************/

 /* None */

/***********************************************************************************
 *  MODULE VARIABLES
 **********************************************************************************/

/* This is the system configuration pin mapping for the supported SC594 SOMs */
static const T_MCP2301X_SYSTEM_MAP sPinMapSoMSC594[] = 
{
    { .s32PinId = (int32_t)SS_PIN_ID_nOSPIFLASH_CS_EN, .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_nUART0_FLOW_EN,   .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_6 },
    { .s32PinId = (int32_t)SS_PIN_ID_nUART0_EN,        .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_5 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPID2_D3_EN,     .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_4 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPI2FLASH_CS_EN, .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_3 },
    { .s32PinId = (int32_t)SS_PIN_ID_LED2,             .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_LED5,             .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_LED4,             .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_0 },
};

/* This is the system configuration pin mapping for the EV-SOMCRR-EZKIT Rev A. */
static const T_MCP2301X_SYSTEM_MAP sPinMapEzKitRevA[] = {
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU1979_EN,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU_1962_EN,     .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_6 },
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU_RESET,       .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_5 },
    { .s32PinId = (int32_t)SS_PIN_ID_nCAN_EN,           .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_4 },
    { .s32PinId = (int32_t)SS_PIN_ID_nFTDI_USB_EN,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_3 },
    { .s32PinId = (int32_t)SS_PIN_ID_nMicroSD_SPI,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_PUSHBUTTON_EN,     .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_EEPROM_EN,         .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_0 },
    { .s32PinId = (int32_t)SS_PIN_ID_nGIGe_RESET,       .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_nETH1_RESET,       .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_6 },
    { .s32PinId = (int32_t)SS_PIN_ID_nETH1_EN,          .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_5 },
    { .s32PinId = (int32_t)SS_PIN_ID_nMLB_EN,           .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_4 },
    { .s32PinId = (int32_t)SS_PIN_ID_AUDIO_JACK_SEL,    .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_3 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPDIF_OPTICAL_EN, .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPDIF_DIGITAL_EN, .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_OCTAL_SPI_CS_EN,   .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_0 },
};  

/* This is the system configuration pin mapping for the EV-SOMCRR-EZKIT Rev D. */
static const T_MCP2301X_SYSTEM_MAP sPinMapEzKitRevD[] = {
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU1979_EN,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU_1962_EN,     .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_6 },
    { .s32PinId = (int32_t)SS_PIN_ID_nADAU_RESET,       .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_5 },
    { .s32PinId = (int32_t)SS_PIN_ID_nCAN_EN,           .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_4 },
    { .s32PinId = (int32_t)SS_PIN_ID_nFTDI_USB_EN,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_3 },
    { .s32PinId = (int32_t)SS_PIN_ID_nMicroSD_SPI,      .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_PUSHBUTTON_EN,     .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_EEPROM_EN,         .ePort = E_MCP2301X_PORT_A, .ePin = E_MCP2301X_PIN_0 },
    { .s32PinId = (int32_t)SS_PIN_ID_nGIGe_RESET,       .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_7 },
    { .s32PinId = (int32_t)SS_PIN_ID_nETH1_RESET,       .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_6 },
    { .s32PinId = (int32_t)SS_PIN_ID_nETH1_EN,          .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_5 },
    { .s32PinId = (int32_t)SS_PIN_ID_nMLB_EN,           .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_4 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPDIF_OPTICAL_EN, .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_2 },
    { .s32PinId = (int32_t)SS_PIN_ID_nSPDIF_DIGITAL_EN, .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_1 },
    { .s32PinId = (int32_t)SS_PIN_ID_OCTAL_SPI_CS_EN,   .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_0 },
};  

/* This is the system configuration pin mapping for performing a hardware check on all EZKITs. */
static const T_MCP2301X_SYSTEM_MAP sPinMapEzKitRevHwProbe[] = {
    { .s32PinId = (int32_t)SS_PIN_ID_OCTAL_SPI_CS_EN,   .ePort = E_MCP2301X_PORT_B, .ePin = E_MCP2301X_PIN_0 },
};

/* 
 * App context for TWI Handles 
 */
static APP_CONTEXT *context = &mainAppContext;

/* 
 * Module TWI Addresses 
 */
static const uint8_t au8McpTwiAddresses[E_MCP2301X_DEVICE_ID_MAX] = {0x21U, 0x22U, 0x22U, 0x22U};

/* 
 * Device specific configurations
 */
const T_MCP2301X_DEVICE_CONFIG asMcp2301xConfigRegistry[E_MCP2301X_DEVICE_ID_MAX] = 
{
    /* E_MCP2301X_DEVICE_SC594_SOM */ 
    {
        .u16PinDirMask = C_SC594_DIR_VALUE,       .u16PullUpEnMask = 0x0000U, .u16PinInitValueMask = C_SC594_INIT_VALUE,       .psPinMap = sPinMapSoMSC594,        .u16PinMapLength = sizeof(sPinMapSoMSC594)/sizeof(T_MCP2301X_SYSTEM_MAP)
    },
    /* E_MCP2301X_DEVICE_EZKIT_REV_A */ 
    {
        .u16PinDirMask = C_EZKIT_REVA_DIR_VALUE,  .u16PullUpEnMask = 0x0000U, .u16PinInitValueMask = C_EZKIT_REVA_INIT_VALUE,  .psPinMap = sPinMapEzKitRevA,       .u16PinMapLength = sizeof(sPinMapEzKitRevA)/sizeof(T_MCP2301X_SYSTEM_MAP)
    },
    /* E_MCP2301X_DEVICE_EZKIT_REV_D */ 
    {
        .u16PinDirMask = C_EZKIT_REVD_DIR_VALUE,  .u16PullUpEnMask = 0x0000U, .u16PinInitValueMask = C_EZKIT_REVD_INIT_VALUE,  .psPinMap = sPinMapEzKitRevD,       .u16PinMapLength = sizeof(sPinMapEzKitRevD)/sizeof(T_MCP2301X_SYSTEM_MAP)
    },
    /* E_MCP2301X_DEVICE_EZKIT_HW_PROBE */ 
    {
        .u16PinDirMask = C_EZKIT_PROBE_DIR_VALUE, .u16PullUpEnMask = 0x0000U, .u16PinInitValueMask = C_EZKIT_PROBE_INIT_VALUE, .psPinMap = sPinMapEzKitRevHwProbe, .u16PinMapLength = sizeof(sPinMapEzKitRevHwProbe)/sizeof(T_MCP2301X_SYSTEM_MAP)
    },
};

/***********************************************************************************
 *  CONFIGURABLE INTERFACE FUNCTIONS
 **********************************************************************************/
 
/******************************************************************************
*   FUNCTION:     Mcp2301x_CfgIf_I2cWrite
*
*   DESCRIPTION:  This configurable interface function should be filled in 
*                 with the project specific I2C implementation for performing
*                 a synchronous write operation. User's should keep track 
*                 of their own device I2C address when calling this function
*                 as it is not provided as an input to this function.
*
*   INPUT(S):     eDeviceId - The hardware device to write to
*                 pu8Data   - A pointer to a buffer of I2C write data. The 
*                             data does NOT contain the I2C address or any
*                             read/write bits. That must be handled here or by
*                             the project specific I2C API. 
*                 u8Length  - The length of the I2C data buffer
*   OUTPUT(S):    None
*   RETURN VALUE: true if successful, false otherwise
******************************************************************************/
bool Mcp2301x_CfgIf_I2cWrite(const T_MCP2301X_DEVICE_ID eDeviceId, const uint8_t * const pu8Data, const uint8_t u8Length)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;
    
    if(twi_write(context->softSwitchHandle, au8McpTwiAddresses[eDeviceId], (uint8_t *)pu8Data, u8Length) == TWI_SIMPLE_SUCCESS)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

/******************************************************************************
*   FUNCTION:     Mcp2301x_CfgIf_I2cRead
*
*   DESCRIPTION:  This configurable interface function should be filled in 
*                 with the project specific I2C implementation for performing
*                 a synchronous read operation. User's should keep track 
*                 of their own device I2C address when calling this function
*                 as it is not provided as an input to this function.
*
*   INPUT(S):     eDeviceId - The hardware device to read from
*                 pu8WData  - A pointer to a buffer of write data which 
*                             contains the device register address(es)
*                 u8WLength - The length of the write data
*                 u8RLength - The length of read data pointer buffer to 
*                             read into
*   OUTPUT(S):    pu8RData  - A pointer to a buffer to put the read 
*                             data into
*   RETURN VALUE: true if successful, false otherwise
******************************************************************************/
bool Mcp2301x_CfgIf_I2cRead(const T_MCP2301X_DEVICE_ID eDeviceId, const uint8_t * const pu8WData, const uint8_t u8WLength, uint8_t * const pu8RData, const uint8_t u8RLength)
{
    bool bSuccess;

    /* Local Inits */
    bSuccess = false;
    
    if(twi_writeRead(context->softSwitchHandle, au8McpTwiAddresses[eDeviceId], (uint8_t *)pu8WData, u8WLength, (uint8_t *)pu8RData, u8RLength) == TWI_SIMPLE_SUCCESS)
    {
        bSuccess = true;
    }
    
    return(bSuccess);
}

/******************************************************************************
 *   FUNCTION:     Mcp2301x_CfgIf_ReportStatus
 *   DESCRIPTION:  This function callback reports any non-OK statuses from 
 *                 the module. Useful for project specific assertions or 
 *                 traps.
 *   INPUT(S):     eStatus - Error Status
 *   OUTPUT(S):    None
 *   RETURN VALUE: None
 ******************************************************************************/
void Mcp2301x_CfgIf_ReportStatus(const T_MCP2301X_STATUS eStatus)
{
    if((eStatus != E_MCP2301X_STATUS_ALREADY_INITIALIZED) && (eStatus != E_MCP2301X_STATUS_ALREADY_UNINITIALIZED))
    {
        asm("bkpt #0");
    }
}