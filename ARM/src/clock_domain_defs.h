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
#ifndef _clock_domain_defs_h
#define _clock_domain_defs_h

typedef enum CLOCK_DOMAIN {
    CLOCK_DOMAIN_SYSTEM = 0,
    CLOCK_DOMAIN_A2B,
    CLOCK_DOMAIN_MAX
} CLOCK_DOMAIN;

enum {
    CLOCK_DOMAIN_BITM_CODEC_IN   = 0x00000001u,
    CLOCK_DOMAIN_BITM_CODEC_OUT  = 0x00000002u,
    CLOCK_DOMAIN_BITM_A2B_IN     = 0x00000004u,
    CLOCK_DOMAIN_BITM_A2B_OUT    = 0x00000008u,
    CLOCK_DOMAIN_BITM_SPDIF_IN   = 0x00000400u,
    CLOCK_DOMAIN_BITM_SPDIF_OUT  = 0x00000800u,
};

#endif
