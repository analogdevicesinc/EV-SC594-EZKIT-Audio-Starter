/**
 * Copyright (c) 2021 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _init_h
#define _init_h

#include <stdint.h>
#include <stdbool.h>

#include "context.h"
#include "gpio_pins.h"

bool gpio_get_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId);
void gpio_set_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId, bool state);
void gpio_toggle_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId);
void gpio_init_pins(GPIO_CONFIG *pinConfig, int numPins);

void gic_init(void);
void system_clk_init(void);
void cgu_ts_init(void);
void gpio_init(void);
void flash_init(APP_CONTEXT *context);
void umm_heap_init(void);
void eth_hardware_init(APP_CONTEXT *context);

void mclk_init(APP_CONTEXT *context);
void disable_mclk(APP_CONTEXT *context);
void enable_mclk(APP_CONTEXT *context);

void adau1962_init(APP_CONTEXT *context);
void adau1979_init(APP_CONTEXT *context);

void spdif_init(APP_CONTEXT *context);

bool a2b_master_init(APP_CONTEXT *context);
void a2b_reset(APP_CONTEXT *context);
bool a2b_restart(APP_CONTEXT *context);
bool a2b_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode);
bool a2b_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG);
bool a2b_sport_stop(APP_CONTEXT *context);
void a2b_pint_init(APP_CONTEXT *context);

void sae_buffer_init(APP_CONTEXT *context);
void audio_routing_init(APP_CONTEXT *context);

#endif
