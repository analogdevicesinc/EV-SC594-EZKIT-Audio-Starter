/**
 * Copyright (c) 2024 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

#ifndef _clocks_h
#define _clocks_h

/* TODO: Make more complete */
#define OSC_CLK      (25000000)
#define CCLK         (1000000000)
#define SYSCLK       (CCLK / 2)
#define SCLK0        (CCLK / 8)
#define OCLK         (CCLK / 4)

/* Insure CGU_TS_CLK = SYSCLK / (2^CGU_TS_DIV) below */
#define CGU_TS_DIV   (5)
#define CGU_TS_CLK   (SYSCLK / (1<<CGU_TS_DIV))

/*
 * Need to override and round up the TWI prescale for the TWI simple
 * driver since SCLK0 is not evenly divisible to create the 10MHz time
 * reference.
 */
#define TWI_SIMPLE_PRESCALE ((SCLK0 / 10000000u) + 1)

#endif
