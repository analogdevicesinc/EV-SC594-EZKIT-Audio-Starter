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

#include "context.h"
#include "twi_simple.h"
#include "ss_init.h"

#pragma pack(push)
#pragma pack(1)
typedef struct {
    uint8_t reg;
    uint8_t value;
} SWITCH_CONFIG;
#pragma pack(pop)

#define SOFT_SWITCH_KIT_U6_I2C_ADDR    0x22
#define PORTA 0x12u
#define PORTB 0x13u

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

/* switch 2 register settings */
static SWITCH_CONFIG kitU6Config[] =
{

/*
       U6 Port A                                   U6 Port B
    7--------------- ~ADAU1979_EN       |       7--------------- ~GIGe_RESET
    | 6------------- ~ADAU_1962_EN      |       | 6------------- ~ETH1_RESET
    | | 5----------- ~ADAU_RESET        |       | | 5----------- ~ETH1_EN
    | | | 4--------- ~CAN_EN            |       | | | 4--------- ~MLB_EN
    | | | | 3------- ~FTDI_USB_EN       |       | | | | 3------- AUDIO_JACK_SEL
    | | | | | 2----- ~MicroSD_SPI       |       | | | | | 2----- ~SPDIF_OPTICAL_EN
    | | | | | | 1--- PUSHBUTTON_EN      |       | | | | | | 1--- ~SPDIF_DIGITAL_EN
    | | | | | | | 0- EEPROM_EN          |       | | | | | | | 0- OCTAL_SPI_CS_EN
    | | | | | | | |                     |       | | | | | | | |
    Y Y N N N N Y Y                     |       Y Y N N Y Y N N  ( Active Y or N )
    0 0 1 1 1 1 1 1                     |       0 0 1 1 1 0 1 0  ( value being set )
*/
  { PORTA, 0x3Fu }, /* 0x3F */                  { PORTB, 0x3Au }, /* 0x3A */

  { 0x0u, 0x00u },    /* Set IODIRA direction (all output) */
  { 0x1u, 0x00u },    /* Set IODIRB direction (all output) */
};

void ss_init_somcrr_ezkit(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT twiResult;
    uint8_t configLen;
    uint8_t i;

    /* Program carrier U6 soft switches */
    configLen = sizeof(kitU6Config) / sizeof(kitU6Config[0]);
    for (i = 0; i < configLen; i++) {
        twiResult = twi_write(context->softSwitchHandle, SOFT_SWITCH_KIT_U6_I2C_ADDR,
            (uint8_t *)&kitU6Config[i], sizeof(kitU6Config[i]));
    }

}

typedef struct _SS_SOMCRR_PIN {
    int pinId;
    uint8_t port;
    uint8_t bitp;
} SS_SOMCRR_PIN;

static SS_SOMCRR_PIN somCrrPins[] = {
    { .pinId = SS_PIN_ID_nADAU1979_EN, .port = PORTA, .bitp = 7 },
    { .pinId = SS_PIN_ID_nADAU_1962_EN, .port = PORTA, .bitp = 6 },
    { .pinId = SS_PIN_ID_nADAU_RESET, .port = PORTA, .bitp = 5 },
    { .pinId = SS_PIN_ID_nCAN_EN, .port = PORTA, .bitp = 4 },
    { .pinId = SS_PIN_ID_nFTDI_USB_EN, .port = PORTA, .bitp = 3 },
    { .pinId = SS_PIN_ID_nMicroSD_SPI, .port = PORTA, .bitp = 2 },
    { .pinId = SS_PIN_ID_PUSHBUTTON_EN, .port = PORTA, .bitp = 1 },
    { .pinId = SS_PIN_ID_EEPROM_EN, .port = PORTA, .bitp = 0},
    { .pinId = SS_PIN_ID_nGIGe_RESET, .port = PORTB, .bitp = 7 },
    { .pinId = SS_PIN_ID_nETH1_RESET, .port = PORTB, .bitp = 6 },
    { .pinId = SS_PIN_ID_nETH1_EN, .port = PORTB, .bitp = 5 },
    { .pinId = SS_PIN_ID_nMLB_EN, .port = PORTB, .bitp = 4 },
    { .pinId = SS_PIN_ID_AUDIO_JACK_SEL, .port = PORTB, .bitp = 3 },
    { .pinId = SS_PIN_ID_nSPDIF_OPTICAL_EN, .port = PORTB, .bitp = 2},
    { .pinId = SS_PIN_ID_nSPDIF_DIGITAL_EN, .port = PORTB, .bitp = 1 },
    { .pinId = SS_PIN_ID_OCTAL_SPI_CS_EN, .port = PORTB, .bitp = 0 },
    { .pinId = SS_PIN_ID_MAX }
};

static SS_SOMCRR_PIN *findPin(int pinId)
{
    SS_SOMCRR_PIN *somCrrPin;

    somCrrPin = somCrrPins;
    while (somCrrPin->pinId != SS_PIN_ID_MAX) {
        if (somCrrPin->pinId == pinId) {
            break;
        }
        somCrrPin++;
    }
    if (somCrrPin->pinId == SS_PIN_ID_MAX) {
        somCrrPin = NULL;
    }
    return(somCrrPin);
}

bool ss_get_somcrr_ezkit(APP_CONTEXT *context, int pinId, bool *value)
{
    TWI_SIMPLE_RESULT twiResult;
    SS_SOMCRR_PIN *somCrrPin;
    uint8_t reg;
    uint8_t val;

    somCrrPin = findPin(pinId);
    if (somCrrPin == NULL) {
        return(false);
    }

    reg = somCrrPin->port;
    twiResult = twi_writeRead(context->softSwitchHandle,
        SOFT_SWITCH_KIT_U6_I2C_ADDR, &reg, 1, &val, 1);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    if (value) {
        *value = (val & (1 << somCrrPin->bitp));
    }

    return(true);
}

bool ss_set_somcrr_ezkit(APP_CONTEXT *context, int pinId, bool value)
{
    TWI_SIMPLE_RESULT twiResult;
    SS_SOMCRR_PIN *somCrrPin;
    SWITCH_CONFIG sw;

    somCrrPin = findPin(pinId);
    if (somCrrPin == NULL) {
        return(false);
    }

    sw.reg = somCrrPin->port;
    twiResult = twi_writeRead(context->softSwitchHandle,
        SOFT_SWITCH_KIT_U6_I2C_ADDR, &sw.reg, 1, &sw.value, 1);
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    if (value) {
        sw.value |= (1 << somCrrPin->bitp);
    } else {
        sw.value &= ~(1 << somCrrPin->bitp);
    }

    twiResult = twi_write(context->softSwitchHandle,
        SOFT_SWITCH_KIT_U6_I2C_ADDR, (uint8_t *)&sw, sizeof(sw));
    if (twiResult != TWI_SIMPLE_SUCCESS) {
        return(false);
    }

    return(true);
}
