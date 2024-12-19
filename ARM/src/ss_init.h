/**
 * Copyright (c) 2022 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _init_ss_h
#define _init_ss_h

#include "context.h"

typedef bool (*SS_GET)(APP_CONTEXT *context, int pinId, bool *value);
typedef bool (*SS_SET)(APP_CONTEXT *context, int pinId, bool value);

enum SS_PIN_ID {
    SS_PIN_ID_UNKNOWN = -1,
    /* Carrier */
    SS_PIN_ID_nADAU1979_EN,
    SS_PIN_ID_nADAU_1962_EN,
    SS_PIN_ID_nADAU_RESET,
    SS_PIN_ID_nCAN_EN,
    SS_PIN_ID_nFTDI_USB_EN,
    SS_PIN_ID_nMicroSD_SPI,
    SS_PIN_ID_PUSHBUTTON_EN,
    SS_PIN_ID_EEPROM_EN,
    SS_PIN_ID_nGIGe_RESET,
    SS_PIN_ID_nETH1_RESET,
    SS_PIN_ID_nETH1_EN,
    SS_PIN_ID_nMLB_EN,
    SS_PIN_ID_AUDIO_JACK_SEL,
    SS_PIN_ID_nSPDIF_OPTICAL_EN,
    SS_PIN_ID_nSPDIF_DIGITAL_EN,
    SS_PIN_ID_OCTAL_SPI_CS_EN,
    /* SOM */
    SS_PIN_ID_nOSPIFLASH_CS_EN,
    SS_PIN_ID_nUART0_FLOW_EN,
    SS_PIN_ID_nUART0_EN,
    SS_PIN_ID_nSPID2_D3_EN,
    SS_PIN_ID_nSPI2FLASH_CS_EN,
    SS_PIN_ID_LED4,
    SS_PIN_ID_LED2,
    SS_PIN_ID_LED5,
    SS_PIN_ID_MAX
};

void ss_init(APP_CONTEXT *context);
bool ss_get(APP_CONTEXT *context, int pinId, bool *value);
bool ss_set(APP_CONTEXT *context, int pinId, bool value);

#endif
