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

#include "ss_init.h"

/* Init prototypes */
void ss_init_somcrr_ezkit(APP_CONTEXT *context);
void ss_init_sc594_som(APP_CONTEXT *context);

/* Set prototypes */
bool ss_set_somcrr_ezkit(APP_CONTEXT *context, int pinId, bool value);
bool ss_set_sc594_som(APP_CONTEXT *context, int pinId, bool value);

/* Get prototypes */
bool ss_get_somcrr_ezkit(APP_CONTEXT *context, int pinId, bool *value);
bool ss_get_sc594_som(APP_CONTEXT *context, int pinId, bool *value);

/* Convenience macros */
#define CRR_GET ss_get_somcrr_ezkit
#define SOM_GET ss_get_sc594_som
#define CRR_SET ss_set_somcrr_ezkit
#define SOM_SET ss_set_sc594_som

typedef struct _SS_PIN {
    int pinId;
    SS_GET get;
    SS_SET set;
} SS_PIN;

static SS_PIN SS_PINS[] = {
    /* Carrier */
    { .pinId = SS_PIN_ID_nADAU1979_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nADAU_1962_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nADAU_RESET, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nCAN_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nFTDI_USB_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nMicroSD_SPI, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_PUSHBUTTON_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_EEPROM_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nGIGe_RESET, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nETH1_RESET, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nETH1_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nMLB_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_AUDIO_JACK_SEL, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nSPDIF_OPTICAL_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_nSPDIF_DIGITAL_EN, .get = CRR_GET, .set = CRR_SET},
    { .pinId = SS_PIN_ID_OCTAL_SPI_CS_EN, .get = CRR_GET, .set = CRR_SET},
    /* SOM */
    { .pinId = SS_PIN_ID_nUART0_FLOW_EN, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_nUART0_EN, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_nSPID2_D3_EN, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_nSPI2FLASH_CS_EN, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_LED4, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_LED2, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_LED5, .get = SOM_GET, .set = SOM_SET},
    { .pinId = SS_PIN_ID_MAX, .get = NULL, .set = NULL}
};

bool ss_get(APP_CONTEXT *context, int pinId, bool *value)
{
    SS_PIN *pin = SS_PINS;
    while (pin->pinId != SS_PIN_ID_MAX) {
        if (pin->pinId == pinId) {
            return(pin->get(context, pinId, value));
        }
        pin++;
    }
    return(false);
}

bool ss_set(APP_CONTEXT *context, int pinId, bool value)
{
    SS_PIN *pin = SS_PINS;
    while (pin->pinId != SS_PIN_ID_MAX) {
        if (pin->pinId == pinId) {
            return(pin->set(context, pinId, value));
        }
        pin++;
    }
    return(false);
}

void ss_init(APP_CONTEXT *context)
{
    ss_init_somcrr_ezkit(context);
    ss_init_sc594_som(context);
}
