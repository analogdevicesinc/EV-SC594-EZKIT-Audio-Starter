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

#ifndef _route_h
#define _route_h

#include "clock_domain_defs.h"

/* Audio routing */
#define MAX_AUDIO_ROUTES  16

/*
 * Audio stream identifiers
 */
typedef enum _STREAM_ID {
    STREAM_ID_UNKNOWN = 0,
    STREAM_ID_CODEC_IN,
    STREAM_ID_CODEC_OUT,
    STREAM_ID_SPDIF_IN,
    STREAM_ID_SPDIF_OUT,
    STREAM_ID_A2B_IN,
    STREAM_ID_A2B_OUT,
    STREAM_ID_MAX
} STREAM_ID;

#endif
