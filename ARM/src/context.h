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
#ifndef _context_h
#define _context_h

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart_simple.h"
#include "twi_simple.h"
#include "spi_simple.h"
#include "sport_simple.h"
#include "flash.h"
#include "shell.h"
#include "clock_domain_defs.h"
#include "spiffs.h"
#include "route.h"
#include "sae.h"
#include "ipc.h"

#include "lwip_adi_ether_netif.h"
#include "lwip/netif.h"

/* Misc defines */
#define UNUSED(expr) do { (void)(expr); } while (0)

/*
 * WARNING: While reasonable effort has gone into uniformly using a
 *          consistent audio data type (i.e. int32, int16, etc) throughout
 *          the code, changing SYSTEM_AUDIO_TYPE from 'int32_t' to a
 *          different type may need to be manually configured elsewhere
 *          in the system.
 *
 * WARNING: If changing ADC or DAC audio channels, be sure to update
 *          and confirm the SPORT initialization in 'init.c' and
 *          audio processing functions in 'codec_audio.c'.
 *
 */
#define SYSTEM_MCLK_RATE            (24576000)
#define SYSTEM_SAMPLE_RATE          (48000)
#define SYSTEM_BLOCK_SIZE           (32)
#define SYSTEM_AUDIO_TYPE           int32_t
#define SYSTEM_MAX_CHANNELS         (32)

#define ADC_AUDIO_CHANNELS          (4)
#define ADC_DMA_CHANNELS            (8)
#define DAC_AUDIO_CHANNELS          (12)
#define DAC_DMA_CHANNELS            (16)
#define A2B_AUDIO_CHANNELS          (32)
#define A2B_DMA_CHANNELS            (32)
#define SPDIF_AUDIO_CHANNELS        (2)
#define SPDIF_DMA_CHANNELS          (2)

#define A2B_I2C_ADDR                (0x68)

#define WRITE_ONCE_VOL_NAME         "wo:"
#define SPIFFS_VOL_NAME             "sf:"

typedef enum A2B_BUS_MODE {
    A2B_BUS_MODE_UNKNOWN = 0,
    A2B_BUS_MODE_MAIN,
    A2B_BUS_MODE_SUB
} A2B_BUS_MODE;

/* 8 slot packed I2S, both RX and TX serializers enabled */
#define SYSTEM_I2SGCFG                 (0xE4)
#define SYSTEM_I2SCFG                  (0x7F)

/* Ethernet defines */
#define DEFAULT_ETH0_IP_ADDR       "0.0.0.0"
#define DEFAULT_ETH0_GW_ADDR       "0.0.0.0"
#define DEFAULT_ETH0_NETMASK       "255.255.255.0"
#define DEFAULT_ETH0_STATIC_IP     false
#define DEFAULT_ETH0_DEFAULT_IFACE true

#define DEFAULT_ETH1_IP_ADDR       "0.0.0.0"
#define DEFAULT_ETH1_GW_ADDR       "0.0.0.0"
#define DEFAULT_ETH1_NETMASK       "255.255.255.0"
#define DEFAULT_ETH1_STATIC_IP     false
#define DEFAULT_ETH1_DEFAULT_IFACE false

typedef struct ETH_CFG {
    ADI_ETHER_EMAC_PORT port;
    char *ip_addr;
    char *gateway_addr;
    char *netmask;
    bool static_ip;
    bool default_iface;
} ETH_CFG;

typedef struct ETH {
    struct netif netif;
    adi_ether_netif *adi_ether;
    uint8_t macaddr[6];
    char hostname[32];
} ETH;

/* System Configuration */
typedef struct APP_CFG {
    ETH_CFG eth0;
    ETH_CFG eth1;
} APP_CFG;

/*
 * The main application context.  Used as a container to carry a
 * variety of useful pointers, handles, etc., between various
 * modules and subsystems.
 */
typedef struct _APP_CONTEXT {

    /* Device handles */
    sUART *stdioHandle;
    sSPI *spi2Handle;
    sSPIPeriph *spiFlashHandle;
    FLASH_INFO *flashHandle;
    sTWI *twi0Handle;
    sTWI *twi1Handle;
    sTWI *twi2Handle;
    sTWI *si5356Handle;
    sTWI *softSwitchHandle;
    sTWI *adau1962TwiHandle;
    sTWI *adau1979TwiHandle;
    sTWI *a2bTwiHandle;
    sSPORT *dacSportOutHandle;
    sSPORT *adcSportInHandle;
    sSPORT *a2bSportOutHandle;
    sSPORT *a2bSportInHandle;
    sSPORT *spdifSportOutHandle;
    sSPORT *spdifSportInHandle;
    spiffs *spiffsHandle;

    /* SHARC status */
    volatile bool sharc0Ready;
    volatile bool sharc1Ready;

    /* Shell context */
    SHELL_CONTEXT shell;

    /* SHARC Audio Engine context */
    SAE_CONTEXT *saeContext;

    /* Task handles (used in 'stacks' command) */
    TaskHandle_t houseKeepingTaskHandle;
    TaskHandle_t startupTaskHandle;
    TaskHandle_t idleTaskHandle;
    TaskHandle_t a2bSlaveTaskHandle;
    TaskHandle_t a2bIrqTaskHandle;

    /* Audio ping/pong buffer pointers */
    void *codecAudioIn[2];
    void *codecAudioOut[2];
    void *spdifAudioIn[2];
    void *spdifAudioOut[2];
    void *a2bAudioIn[2];
    void *a2bAudioOut[2];

    /* Audio ping/pong buffer lengths */
    unsigned codecAudioInLen;
    unsigned codecAudioOutLen;
    unsigned spdifAudioInLen;
    unsigned spdifAudioOutLen;
    unsigned a2bAudioInLen;
    unsigned a2bAudioOutLen;

    /* SAE buffer pointers */
    SAE_MSG_BUFFER *codecMsgIn[2];
    SAE_MSG_BUFFER *codecMsgOut[2];
    SAE_MSG_BUFFER *spdifMsgIn[2];
    SAE_MSG_BUFFER *spdifMsgOut[2];
    SAE_MSG_BUFFER *a2bMsgIn[2];
    SAE_MSG_BUFFER *a2bMsgOut[2];

    /* Audio routing table */
    SAE_MSG_BUFFER *routingMsgBuffer;
    IPC_MSG *routingMsg;

    /* APP config */
    APP_CFG cfg;

    /* Current time in mS */
    uint64_t now;

    /* SHARC Cycles */
    uint32_t sharc0Cycles[CLOCK_DOMAIN_MAX];
    uint32_t sharc1Cycles[CLOCK_DOMAIN_MAX];

    /* A2B mode */
    A2B_BUS_MODE a2bmode;
    bool a2bSlaveActive;
    bool discoverCmdStatus;
    bool a2bPresent;
    bool a2bIrqDisable;

    /* Clock domain management */
    uint32_t clockDomainMask[CLOCK_DOMAIN_MAX];
    uint32_t clockDomainActive[CLOCK_DOMAIN_MAX];

    /* Ethernet Network interface */
    ETH eth[2];

} APP_CONTEXT;

/*
 * Make the application context global for convenience
 */
extern APP_CONTEXT mainAppContext;

#endif
