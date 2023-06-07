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

#ifndef _util_h
#define _util_h

#include <stdint.h>

void delay(unsigned ms);
uint32_t getTimeStamp(void);
uint32_t elapsedTimeMs(uint32_t elapsed);
uint32_t getHiResTimeStamp(void);

uint32_t roundUpPow2(uint32_t x);

#endif
