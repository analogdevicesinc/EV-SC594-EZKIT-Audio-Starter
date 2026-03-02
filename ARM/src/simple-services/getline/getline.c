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

/*
 * Based on 'getline' function definition from:
 *    The Open Group Base Specifications Issue 8
 *    IEEE Std 1003.1-2024
 */

#include <stdlib.h>
#include <stdio.h>
#include <limits.h>

#include "getline_cfg.h"

#ifndef GETLINE_MALLOC
#define GETLINE_MALLOC  malloc
#endif

#ifndef GETLINE_REALLOC
#define GETLINE_REALLOC realloc
#endif

static ssize_t _getdelim(char **ppcBuf, size_t *psSize, int delim, FILE *pFile)
{
    char *pcBuf = NULL;
    char *pcEnd = NULL;
    size_t sSize = 0u;
    int c;
    size_t idx;
    bool err = false;

    /* Validate inputs. */
    if(pFile == NULL) {
        return -1;
    }

    if(psSize == NULL) {
        return -1;
    }

    if((delim < 0) || (delim > UCHAR_MAX)) {
        return -1;
    }

    if(ppcBuf == NULL) {
        return -1;
    }

    /* Validate buffer and size */
    pcBuf = *ppcBuf;
    if(pcBuf == NULL) {
        /* Buffer was not pre-allocated. Allocate default size. */
        sSize = *psSize > 0u ? *psSize : BUFSIZ;
        pcBuf = GETLINE_MALLOC(sSize);
        if(pcBuf == NULL) {
            return -1;
        }        
    } else {
        /* Buffer was pre-allocated. Only validate size. */
        sSize = *psSize;
    }

    idx = 0u;
    pcEnd = &pcBuf[sSize];
    while(1) {
        /* Check if there is enough space in the buffer. */
        if(&pcBuf[idx + 2u] >= pcEnd) {
            sSize += BUFSIZ;
            pcBuf = GETLINE_REALLOC(pcBuf, sSize);
            if(pcBuf == NULL) {
                return -1;
            }
            pcEnd = &pcBuf[sSize];
        }

        c = fgetc(pFile);
        if(c == -1) {
            break;
        }

        /* Copy character to buffer. */
        pcBuf[idx] = (char) c;
        idx++;

        /* Stop copying on new-line and append NUL terminator. */
        if( c == delim) {
            pcBuf[idx] = '\0';
            break;
        }
    }

    /* Check why fgetc failed. */
    if(c == -1) {
        if(feof(pFile) != 0) {
            if(idx > 0) {
                /* No new-line on last line. */
                pcBuf[idx] = '\0';
            } else {
                /* EOF after last line. */
                err = true;
            }
        } else {
            /* Failed before EOF. */
            err = true;
        }
    }

    /* Output buffer location and size. */
    *ppcBuf = &pcBuf[0];
    *psSize = sSize;    

    return err ? -1 : idx;
}

ssize_t getdelim(char ** lineptr, size_t * n, int delimiter, FILE * stream)
{
    return _getdelim(lineptr, n, delimiter, stream);
}

ssize_t getline(char ** lineptr, size_t * n, FILE * stream)
{
    return _getdelim(lineptr, n, '\n', stream);
}