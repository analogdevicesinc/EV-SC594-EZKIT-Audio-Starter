/**
 * Copyright (c) 2023 - Analog Devices Inc. All Rights Reserved.
 * This software is proprietary and confidential to Analog Devices, Inc.
 * and its licensors.
 *
 * This software is subject to the terms and conditions of the license set
 * forth in the project LICENSE file. Downloading, reproducing, distributing or
 * otherwise using the software constitutes acceptance of the license. The
 * software may not be used except as expressly authorized under the license.
 */

/* Standard library includes */
#include <string.h>
#include <limits.h>
#include <assert.h>
#include <stdbool.h>

/* ADI service includes */
#include <services/gpio/adi_gpio.h>
#include <services/tmr/adi_tmr.h>
#include <services/pwr/adi_pwr.h>
#include <services/int/adi_gic.h>

/* ADI processor includes */
#include <sruSC594.h>

/* Kernel includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Simple driver includes */
#include "uart_simple.h"
#include "uart_stdio.h"
#include "twi_simple.h"
#include "spi_simple.h"
#include "sport_simple.h"
#include "pcg_simple.h"
#include "flash.h"
#include "is25lp512.h"

/* Simple service includes */
#include "adau1962.h"
#include "adau1979.h"
#include "adi_a2b_cmdlist.h"
#include "syslog.h"
#include "sae.h"

/* OSS service includes */
#include "umm_malloc.h"
#include "umm_malloc_heaps.h"

/* Application includes */
#include "context.h"
#include "clocks.h"
#include "init.h"
#include "util.h"
#include "codec_audio.h"
#include "a2b_audio.h"
#include "route.h"
#include "clock_domain.h"
#include "spdif_audio.h"
#include "ss_init.h"

/* This code can be placed in external memory */
#include "external_memory.h"

/***********************************************************************
 * System Clock Initialization
 **********************************************************************/
void system_clk_init(void)
{
    ADI_PWR_RESULT ePwrResult;
    uint32_t cclk,sclk,sclk0,sclk1,dclk,oclk;

    /* Initialize ADI power service */
    ePwrResult = adi_pwr_Init(0, OSC_CLK);

    /* Query primary clocks from CGU0 for confirmation */
    ePwrResult = adi_pwr_GetCoreClkFreq(0, &cclk);
    ePwrResult = adi_pwr_GetSystemFreq(0, &sclk, &sclk0, &sclk1);
    ePwrResult = adi_pwr_GetDDRClkFreq(0, &dclk);
    ePwrResult = adi_pwr_GetOutClkFreq(0, &oclk);

    /* Make sure they match clocks.h */
    assert(cclk == CCLK);
    assert(sclk == SYSCLK);
    assert(sclk0 == SCLK0);
}

/***********************************************************************
 * GPIO / Pin MUX / SRU Initialization
 **********************************************************************/

/* TWI2 GPIO FER bit positions */
#define TWI2_SCL_PORTA_FER   (1 << BITP_PORT_DATA_PX14)
#define TWI2_SDA_PORTA_FER   (1 << BITP_PORT_DATA_PX15)

/* TWI2 GPIO MUX bit positions (two bits per MUX entry */
#define TWI2_SCL_PORTA_MUX   (0 << (BITP_PORT_DATA_PX14 << 1))
#define TWI2_SDA_PORTA_MUX   (0 << (BITP_PORT_DATA_PX15 << 1))

/* SPI2 GPIO FER bit positions (one bit per FER entry) */
#define SPI2_CLK_PORTA_FER   (1 << BITP_PORT_DATA_PX4)
#define SPI2_MISO_PORTA_FER  (1 << BITP_PORT_DATA_PX0)
#define SPI2_MOSI_PORTA_FER  (1 << BITP_PORT_DATA_PX1)
#define SPI2_D2_PORTA_FER    (1 << BITP_PORT_DATA_PX2)
#define SPI2_D3_PORTA_FER    (1 << BITP_PORT_DATA_PX3)
#define SPI2_SEL_PORTA_FER   (1 << BITP_PORT_DATA_PX5)

/* SPI2 GPIO MUX bit positions (two bits per MUX entry) */
#define SPI2_CLK_PORTA_MUX   (0 << (BITP_PORT_DATA_PX4 << 1))
#define SPI2_MISO_PORTA_MUX  (0 << (BITP_PORT_DATA_PX0 << 1))
#define SPI2_MOSI_PORTA_MUX  (0 << (BITP_PORT_DATA_PX1 << 1))
#define SPI2_D2_PORTA_MUX    (0 << (BITP_PORT_DATA_PX2 << 1))
#define SPI2_D3_PORTA_MUX    (0 << (BITP_PORT_DATA_PX3 << 1))
#define SPI2_SEL_PORTA_MUX   (0 << (BITP_PORT_DATA_PX5 << 1))

/* UART0 GPIO FER bit positions */
#define UART0_TX_PORTA_FER   (1 << BITP_PORT_DATA_PX6)
#define UART0_RX_PORTA_FER   (1 << BITP_PORT_DATA_PX7)
#define UART0_RTS_PORTA_FER  (1 << BITP_PORT_DATA_PX8)
#define UART0_CTS_PORTA_FER  (1 << BITP_PORT_DATA_PX9)

/* UART0 GPIO MUX bit positions (two bits per MUX entry */
#define UART0_TX_PORTA_MUX   (1 << (BITP_PORT_DATA_PX6 << 1))
#define UART0_RX_PORTA_MUX   (1 << (BITP_PORT_DATA_PX7 << 1))
#define UART0_RTS_PORTA_MUX  (1 << (BITP_PORT_DATA_PX8 << 1))
#define UART0_CTS_PORTA_MUX  (1 << (BITP_PORT_DATA_PX9  << 1))

/* EMAC0 */
#define EMAC0_COL_PORTD_FER          (1 << BITP_PORT_DATA_PX7)
#define EMAC0_CRS_PORTD_FER          (1 << BITP_PORT_DATA_PX2)
#define EMAC0_MDC_PORTH_FER          (1 << BITP_PORT_DATA_PX3)
#define EMAC0_MDIO_PORTH_FER         (1 << BITP_PORT_DATA_PX4)
#define EMAC0_PTPAUXIN0_PORTI_FER    (1 << BITP_PORT_DATA_PX2)
#define EMAC0_PTPCLKIN0_PORTI_FER    (1 << BITP_PORT_DATA_PX1)
#define EMAC0_PTPPPS0_PORTI_FER      (1 << BITP_PORT_DATA_PX4)
#define EMAC0_PTPPPS1_PORTI_FER      (1 << BITP_PORT_DATA_PX3)
#define EMAC0_PTPPPS2_PORTI_FER      (1 << BITP_PORT_DATA_PX5)
#define EMAC0_PTPPPS3_PORTI_FER      (1 << BITP_PORT_DATA_PX6)
#define EMAC0_RXCLK_REFCLK_PORTH_FER (1 << BITP_PORT_DATA_PX7)
#define EMAC0_RXCTL_RXDV_PORTH_FER   (1 << BITP_PORT_DATA_PX8)
#define EMAC0_RXD0_PORTH_FER         (1 << BITP_PORT_DATA_PX5)
#define EMAC0_RXD1_PORTH_FER         (1 << BITP_PORT_DATA_PX6)
#define EMAC0_RXD2_PORTH_FER         (1 << BITP_PORT_DATA_PX11)
#define EMAC0_RXD3_PORTH_FER         (1 << BITP_PORT_DATA_PX12)
#define EMAC0_RX_ER_PORTD_FER        (1 << BITP_PORT_DATA_PX6)
#define EMAC0_TXCLK_PORTH_FER        (1 << BITP_PORT_DATA_PX14)
#define EMAC0_TXCTL_TXEN_PORTH_FER   (1 << BITP_PORT_DATA_PX13)
#define EMAC0_TXD0_PORTH_FER         (1 << BITP_PORT_DATA_PX9)
#define EMAC0_TXD1_PORTH_FER         (1 << BITP_PORT_DATA_PX10)
#define EMAC0_TXD2_PORTH_FER         (1 << BITP_PORT_DATA_PX15)
#define EMAC0_TXD3_PORTI_FER         (1 << BITP_PORT_DATA_PX0)

#define EMAC0_COL_PORTD_MUX          (1 << (BITP_PORT_DATA_PX7 << 1))
#define EMAC0_CRS_PORTD_MUX          (2 << (BITP_PORT_DATA_PX2 << 1))
#define EMAC0_MDC_PORTH_MUX          (0 << (BITP_PORT_DATA_PX3 << 1))
#define EMAC0_MDIO_PORTH_MUX         (0 << (BITP_PORT_DATA_PX4 << 1))
#define EMAC0_PTPAUXIN0_PORTI_MUX    (1 << (BITP_PORT_DATA_PX2 << 1))
#define EMAC0_PTPCLKIN0_PORTI_MUX    (1 << (BITP_PORT_DATA_PX1 << 1))
#define EMAC0_PTPPPS0_PORTI_MUX      (2 << (BITP_PORT_DATA_PX4 << 1))
#define EMAC0_PTPPPS1_PORTI_MUX      (2 << (BITP_PORT_DATA_PX3 << 1))
#define EMAC0_PTPPPS2_PORTI_MUX      (0 << (BITP_PORT_DATA_PX5 << 1))
#define EMAC0_PTPPPS3_PORTI_MUX      (0 << (BITP_PORT_DATA_PX6 << 1))
#define EMAC0_RXCLK_REFCLK_PORTH_MUX (0 << (BITP_PORT_DATA_PX7 << 1))
#define EMAC0_RXCTL_RXDV_PORTH_MUX   (0 << (BITP_PORT_DATA_PX8 << 1))
#define EMAC0_RXD0_PORTH_MUX         (0 << (BITP_PORT_DATA_PX5 << 1))
#define EMAC0_RXD1_PORTH_MUX         (0 << (BITP_PORT_DATA_PX6 << 1))
#define EMAC0_RXD2_PORTH_MUX         (0 << (BITP_PORT_DATA_PX11 << 1))
#define EMAC0_RXD3_PORTH_MUX         (0 << (BITP_PORT_DATA_PX12 << 1))
#define EMAC0_RX_ER_PORTD_MUX        (1 << (BITP_PORT_DATA_PX6 << 1))
#define EMAC0_TXCLK_PORTH_MUX        (0 << (BITP_PORT_DATA_PX14 << 1))
#define EMAC0_TXCTL_TXEN_PORTH_MUX   (0 << (BITP_PORT_DATA_PX13 << 1))
#define EMAC0_TXD0_PORTH_MUX         (0 << (BITP_PORT_DATA_PX9 << 1))
#define EMAC0_TXD1_PORTH_MUX         (0 << (BITP_PORT_DATA_PX10 << 1))
#define EMAC0_TXD2_PORTH_MUX         (0 << (BITP_PORT_DATA_PX15 << 1))
#define EMAC0_TXD3_PORTI_MUX         (0 << (BITP_PORT_DATA_PX0 << 1))

/* EMAC1 */
#define EMAC1_CRS_PORTF_FER     (1 << BITP_PORT_DATA_PX3)
#define EMAC1_MDC_PORTF_FER     (1 << BITP_PORT_DATA_PX2)
#define EMAC1_MDIO_PORTF_FER    (1 << BITP_PORT_DATA_PX1)
#define EMAC1_REFCLK_PORTE_FER  (1 << BITP_PORT_DATA_PX11)
#define EMAC1_RXD0_PORTE_FER    (1 << BITP_PORT_DATA_PX15)
#define EMAC1_RXD1_PORTF_FER    (1 << BITP_PORT_DATA_PX0)
#define EMAC1_TXD0_PORTE_FER    (1 << BITP_PORT_DATA_PX13)
#define EMAC1_TXD1_PORTE_FER    (1 << BITP_PORT_DATA_PX14)
#define EMAC1_TXEN_PORTE_FER    (1 << BITP_PORT_DATA_PX12)

#define EMAC1_CRS_PORTF_MUX     (0 << (BITP_PORT_DATA_PX3 << 1))
#define EMAC1_MDC_PORTF_MUX     (0 << (BITP_PORT_DATA_PX2 << 1))
#define EMAC1_MDIO_PORTF_MUX    (0 << (BITP_PORT_DATA_PX1 << 1))
#define EMAC1_REFCLK_PORTE_MUX  (0 << (BITP_PORT_DATA_PX11 << 1))
#define EMAC1_RXD0_PORTE_MUX    (0 << (BITP_PORT_DATA_PX15 << 1))
#define EMAC1_RXD1_PORTF_MUX    (0 << (BITP_PORT_DATA_PX0 << 1))
#define EMAC1_TXD0_PORTE_MUX    (0 << (BITP_PORT_DATA_PX13 << 1))
#define EMAC1_TXD1_PORTE_MUX    (0 << (BITP_PORT_DATA_PX14 << 1))
#define EMAC1_TXEN_PORTE_MUX    (0 << (BITP_PORT_DATA_PX12 << 1))

/* DAI IE Bit definitions (not in any ADI header files) */
#define BITP_PADS0_DAI0_IE_PB03   (2)
#define BITP_PADS0_DAI0_IE_PB04   (3)
#define BITP_PADS0_DAI0_IE_PB05   (4)

/*
 * WARNING: Order must match the GPIO_PIN_ID enum in gpio_pins.h!
 */
GPIO_CONFIG gpioPins[GPIO_PIN_MAX] = {
    { ADI_GPIO_PORT_D, ADI_GPIO_PIN_0,  ADI_GPIO_DIRECTION_INPUT,  0 }, // GPIO_PIN_SOMCRR_PB1,
    { ADI_GPIO_PORT_H, ADI_GPIO_PIN_0,  ADI_GPIO_DIRECTION_INPUT,  0 }, // GPIO_PIN_SOMCRR_PB2,
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_1,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // GPIO_PIN_SOMCRR_LED7,
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_2,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // GPIO_PIN_SOMCRR_LED10,
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_3,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // GPIO_PIN_SOMCRR_LED9,
    { ADI_GPIO_PORT_C, ADI_GPIO_PIN_5,  ADI_GPIO_DIRECTION_INPUT,  0 }, // GPIO_PIN_SOMCRR_A2B1_IRQ,
    { ADI_GPIO_PORT_I, ADI_GPIO_PIN_4,  ADI_GPIO_DIRECTION_OUTPUT, 0 }, // GPIO_PIN_SOMCRR_PTPPPS0,
};

bool gpio_get_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    uint32_t value;

    pin = &pinConfig[pinId];

    result = adi_gpio_GetData(pin->port, &value);

    pin->state = value & pin->pinNum;

    return(pin->state);
}

void gpio_set_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId, bool state)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    if (state) {
        result = adi_gpio_Set(pin->port, pin->pinNum);
    } else {
        result = adi_gpio_Clear(pin->port, pin->pinNum);
    }

    pin->state = state;
}

void gpio_toggle_pin(GPIO_CONFIG *pinConfig, GPIO_PIN_ID pinId)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;

    pin = &pinConfig[pinId];

    result = adi_gpio_Toggle(pin->port, pin->pinNum);

    gpio_get_pin(pinConfig, pinId);
}

void gpio_init_pins(GPIO_CONFIG *pinConfig, int numPins)
{
    ADI_GPIO_RESULT result;
    GPIO_CONFIG *pin;
    int i;

    for (i = 0; i < GPIO_PIN_MAX; i++) {

        /* Select the pin */
        pin = &pinConfig[i];

        /* Set the initial pin state */
        if (pin->state) {
            result = adi_gpio_Set(pin->port, pin->pinNum);
        } else {
            result = adi_gpio_Clear(pin->port, pin->pinNum);
        }

        /* Set the direction */
        result = adi_gpio_SetDirection(pin->port, pin->pinNum, pin->dir);
    }
}

void gpio_init(void)
{
    static uint8_t gpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE * 16];
    uint32_t numCallbacks;
    ADI_GPIO_RESULT result;

    /* Init the GPIO system service */
    result = adi_gpio_Init(gpioMemory, sizeof(gpioMemory), &numCallbacks);

    /* Configure TWI2 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        TWI2_SCL_PORTA_FER |
        TWI2_SDA_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        TWI2_SCL_PORTA_MUX |
        TWI2_SDA_PORTA_MUX
    );

    /* Configure SPI2 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        SPI2_CLK_PORTA_FER |
        SPI2_MISO_PORTA_FER |
        SPI2_MOSI_PORTA_FER |
        SPI2_D2_PORTA_FER |
        SPI2_D3_PORTA_FER |
        SPI2_SEL_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        SPI2_CLK_PORTA_MUX |
        SPI2_MISO_PORTA_MUX |
        SPI2_MOSI_PORTA_MUX |
        SPI2_D2_PORTA_MUX |
        SPI2_D3_PORTA_MUX |
        SPI2_SEL_PORTA_MUX
    );

    /* Configure EMAC0 Alternate Function GPIO */
    *pREG_PORTD_FER |= (
        EMAC0_COL_PORTD_FER |
        EMAC0_CRS_PORTD_FER |
        EMAC0_RX_ER_PORTD_FER
    );
    *pREG_PORTH_FER |= (
        EMAC0_MDC_PORTH_FER |
        EMAC0_MDIO_PORTH_FER |
        EMAC0_RXCLK_REFCLK_PORTH_FER |
        EMAC0_RXCTL_RXDV_PORTH_FER |
        EMAC0_RXD0_PORTH_FER |
        EMAC0_RXD1_PORTH_FER |
        EMAC0_RXD2_PORTH_FER |
        EMAC0_RXD3_PORTH_FER |
        EMAC0_TXCLK_PORTH_FER |
        EMAC0_TXCTL_TXEN_PORTH_FER |
        EMAC0_TXD0_PORTH_FER |
        EMAC0_TXD1_PORTH_FER |
        EMAC0_TXD2_PORTH_FER
    );
    *pREG_PORTI_FER |= (
        EMAC0_PTPAUXIN0_PORTI_FER |
        EMAC0_PTPCLKIN0_PORTI_FER |
#if 0
        EMAC0_PTPPPS0_PORTI_FER |
#endif
        EMAC0_PTPPPS1_PORTI_FER |
        EMAC0_PTPPPS2_PORTI_FER |
        EMAC0_PTPPPS3_PORTI_FER |
        EMAC0_TXD3_PORTI_FER
    );

    *pREG_PORTD_MUX |= (
        EMAC0_COL_PORTD_MUX |
        EMAC0_CRS_PORTD_MUX |
        EMAC0_RX_ER_PORTD_MUX
    );
    *pREG_PORTH_MUX |= (
        EMAC0_MDC_PORTH_MUX |
        EMAC0_MDIO_PORTH_MUX |
        EMAC0_RXCLK_REFCLK_PORTH_MUX |
        EMAC0_RXCTL_RXDV_PORTH_MUX |
        EMAC0_RXD0_PORTH_MUX |
        EMAC0_RXD1_PORTH_MUX |
        EMAC0_RXD2_PORTH_MUX |
        EMAC0_RXD3_PORTH_MUX |
        EMAC0_TXCLK_PORTH_MUX |
        EMAC0_TXCTL_TXEN_PORTH_MUX |
        EMAC0_TXD0_PORTH_MUX |
        EMAC0_TXD1_PORTH_MUX |
        EMAC0_TXD2_PORTH_MUX
    );
    *pREG_PORTI_MUX |= (
        EMAC0_PTPAUXIN0_PORTI_MUX |
        EMAC0_PTPCLKIN0_PORTI_MUX |
#if 0
        EMAC0_PTPPPS0_PORTI_MUX |
#endif
        EMAC0_PTPPPS1_PORTI_MUX |
        EMAC0_PTPPPS2_PORTI_MUX |
        EMAC0_PTPPPS3_PORTI_MUX |
        EMAC0_TXD3_PORTI_MUX
    );

    /* Configure EMAC1 Alternate Function GPIO */
    *pREG_PORTE_FER |= (
        EMAC1_REFCLK_PORTE_FER |
        EMAC1_RXD0_PORTE_FER |
        EMAC1_TXD0_PORTE_FER |
        EMAC1_TXD1_PORTE_FER |
        EMAC1_TXEN_PORTE_FER
    );
    *pREG_PORTE_MUX |= (
        EMAC1_REFCLK_PORTE_MUX |
        EMAC1_RXD0_PORTE_MUX |
        EMAC1_TXD0_PORTE_MUX |
        EMAC1_TXD1_PORTE_MUX |
        EMAC1_TXEN_PORTE_MUX
    );
    *pREG_PORTF_FER |= (
        EMAC1_CRS_PORTF_FER |
        EMAC1_MDC_PORTF_FER |
        EMAC1_MDIO_PORTF_FER |
        EMAC1_RXD1_PORTF_FER
    );
    *pREG_PORTF_MUX |= (
        EMAC1_CRS_PORTF_MUX |
        EMAC1_MDC_PORTF_MUX |
        EMAC1_MDIO_PORTF_MUX |
        EMAC1_RXD1_PORTF_MUX
    );


    /* Configure UART0 Alternate Function GPIO */
    *pREG_PORTA_FER |= (
        UART0_TX_PORTA_FER |
        UART0_RX_PORTA_FER |
        UART0_RTS_PORTA_FER |
        UART0_CTS_PORTA_FER
    );
    *pREG_PORTA_MUX |= (
        UART0_TX_PORTA_MUX |
        UART0_RX_PORTA_MUX |
        UART0_RTS_PORTA_MUX |
        UART0_CTS_PORTA_MUX
    );

    /* Configure straight GPIO */
    gpio_init_pins(gpioPins, GPIO_PIN_MAX);

    /* PADS0 DAI0/1 Port Input Enable Control Register */
    *pREG_PADS0_DAI0_IE = BITM_PADS_DAI0_IE_VALUE;
    *pREG_PADS0_DAI1_IE = BITM_PADS_DAI1_IE_VALUE;

}

/*
 * This macro is used to set the interrupt priority.  Interrupts of a
 * higher priority (lower number) will nest with interrupts of a lower
 * priority (higher number).
 *
 * Priority can range from 0 (highest) to 15 (lowest)
 *
 * Currently only USB interrupts are elevated, all others are lower.
 *
 */
#define INTERRUPT_PRIO(x) \
    ((configMAX_API_CALL_INTERRUPT_PRIORITY + x) << portPRIORITY_SHIFT)


/***********************************************************************
 * GIC Initialization
 **********************************************************************/
void gic_init(void)
{
    ADI_GIC_RESULT  result;

    result = adi_gic_Init();

#ifdef FREE_RTOS
    /*
     * Setup peripheral interrupt priorities:
     *   Details: FreeRTOSv9.0.0/portable/GCC/ARM_CA9/port.c (line 574)
     *
     * All registered system interrupts can be identified by setting a breakpoint in
     * adi_rtl_register_dispatched_handler().  Only interrupts that need to call FreeRTOS
     * functions must be registered with the required interrupt priority.
     */
    adi_gic_SetBinaryPoint(ADI_GIC_CORE_0, 0);
    adi_gic_SetIntPriority(INTR_SPI0_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPI1_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPI2_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI0_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI1_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_TWI2_DATA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART0_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART1_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_UART2_STAT, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT0_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT0_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT1_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT1_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT2_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT2_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT3_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT3_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT4_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT4_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT5_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT5_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT6_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT6_B_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT7_A_DMA, INTERRUPT_PRIO(1));
    adi_gic_SetIntPriority(INTR_SPORT7_B_DMA, INTERRUPT_PRIO(1));

    /* WARNING: The ADI FreeRTOS port uses the last timer as the tick timer
     *          which must be configured as the lowest priority interrupt.
     *          If you're using the stock ADI v9.0.0 or v10.0.1 port, be sure to
     *          enable the line below.  This countermeasure has been applied
     *          to the reusable module FreeRTOS v10.2.1+.
     *
     *    The SysTick handler needs to run at the lowest priority.  This is because the critical section
     *    within the handler itself assumes it is running at the lowest priority, so saves time by not
     *    saving the old priority mask and then restoring the previous priority mask.
     */
    //adi_gic_SetIntPriority(INTR_TIMER0_TMR15, 30 << portPRIORITY_SHIFT);
#endif
}

/***********************************************************************
 * libc heap initialization
 **********************************************************************/
#ifndef STD_C_HEAP_SIZE
#define STD_C_HEAP_SIZE (1024 * 1024)
#endif
uint8_t __adi_heap_object[STD_C_HEAP_SIZE] __attribute__ ((section (".heap")));

/***********************************************************************
 * UMM_MALLOC heap initialization
 **********************************************************************/
__attribute__ ((section(".heap")))
    static uint8_t umm_sdram_heap[UMM_SDRAM_HEAP_SIZE];

__attribute__ ((section(".l3_uncached_data")))
    static uint8_t umm_sdram_uncached_heap[UMM_SDRAM_UNCACHED_HEAP_SIZE];

__attribute__ ((section(".l2_uncached_data")))
    static uint8_t umm_l2_uncached_heap[UMM_L2_UNCACHED_HEAP_SIZE];

__attribute__ ((section(".l2_cached_data")))
    static uint8_t umm_l2_cached_heap[UMM_L2_CACHED_HEAP_SIZE];

void umm_heap_init(void)
{
    /* Initialize the cached L3 SDRAM heap (default heap). */
    umm_init(UMM_SDRAM_HEAP, umm_sdram_heap, UMM_SDRAM_HEAP_SIZE);

    /* Initialize the un-cached L3 SDRAM heap. */
    umm_init(UMM_SDRAM_UNCACHED_HEAP, umm_sdram_uncached_heap,
        UMM_SDRAM_UNCACHED_HEAP_SIZE);

    /* Initialize the L2 uncached heap. */
    umm_init(UMM_L2_UNCACHED_HEAP, umm_l2_uncached_heap,
        UMM_L2_UNCACHED_HEAP_SIZE);

    /* Initialize the L2 cached heap. */
    umm_init(UMM_L2_CACHED_HEAP, umm_l2_cached_heap, UMM_L2_CACHED_HEAP_SIZE);
}

/***********************************************************************
 * SPI Flash initialization
 **********************************************************************/
void flash_init(APP_CONTEXT *context)
{
    SPI_SIMPLE_RESULT spiResult;

    /* Open a SPI handle to SPI2 */
    spiResult = spi_open(SPI2, &context->spi2Handle);

    /* Open a SPI2 device handle for the flash */
    spiResult = spi_openDevice(context->spi2Handle, &context->spiFlashHandle);

    /* Configure the flash device handle */
    spiResult = spi_setClock(context->spiFlashHandle, 5);
    spiResult = spi_setMode(context->spiFlashHandle, SPI_MODE_3);
    spiResult = spi_setFastMode(context->spiFlashHandle, true);
    spiResult = spi_setLsbFirst(context->spiFlashHandle, false);
    spiResult = spi_setSlaveSelect(context->spiFlashHandle, SPI_SSEL_1);

    /* Open the flash driver with the configured SPI device handle */
    context->flashHandle = is25lp_open(context->spiFlashHandle);
}

/***********************************************************************
 * CGU Timestamp init
 **********************************************************************/
void cgu_ts_init(void)
{
    /* Configure the CGU timestamp counter.  See clocks.h for more detail. */
    *pREG_CGU0_TSCTL =
        ( 1 << BITP_CGU_TSCTL_EN ) |
        ( CGU_TS_DIV << BITP_CGU_TSCTL_TSDIV );
}

/***********************************************************************
 * This function allocates audio buffers in L2 cached memory and
 * initializes a single SPORT using the simple SPORT driver.
 **********************************************************************/
static sSPORT *single_sport_init(SPORT_SIMPLE_PORT sport,
    SPORT_SIMPLE_CONFIG *cfg, SPORT_SIMPLE_AUDIO_CALLBACK cb,
    void **pingPongPtrs, unsigned *pingPongLen, void *usrPtr,
    bool cached, SPORT_SIMPLE_RESULT *result)
{
    sSPORT *sportHandle;
    SPORT_SIMPLE_RESULT sportResult;
    uint32_t dataBufferSize;
    int i;

    /* Open a handle to the SPORT */
    sportResult = sport_open(sport, &sportHandle);
    if (sportResult != SPORT_SIMPLE_SUCCESS) {
        if (result) {
            *result = sportResult;
        }
        return(NULL);
    }

    /* Copy application callback info */
    cfg->callBack = cb;
    cfg->usrPtr = usrPtr;

    /* Allocate audio buffers if not already allocated */
    dataBufferSize = sport_buffer_size(cfg);
    for (i = 0; i < 2; i++) {
        if (!cfg->dataBuffers[i]) {
            cfg->dataBuffers[i] = umm_malloc_heap_aligned(
                UMM_L2_CACHED_HEAP, dataBufferSize, ADI_CACHE_LINE_LENGTH);
            memset(cfg->dataBuffers[i], 0, dataBufferSize);
        }
    }
    cfg->dataBuffersCached = cached;
    cfg->syncDMA = true;

    /* Configure the SPORT */
    sportResult = sport_configure(sportHandle, cfg);

    /* Save ping pong data pointers */
    if (pingPongPtrs) {
        pingPongPtrs[0] = cfg->dataBuffers[0];
        pingPongPtrs[1] = cfg->dataBuffers[1];
    }
    if (pingPongLen) {
        *pingPongLen = dataBufferSize;
    }
    if (result) {
        *result = sportResult;
    }

    return(sportHandle);
}

/***********************************************************************
 * Simple SPORT driver 8/16-ch packed I2S settings
 * Compatible A2B I2S Register settings:
 * 8 ch
 *    I2SGCFG: 0xE2
 *     I2SCFG: 0x7F
 * 16 ch
 *    I2SGCFG: 0xE4
 *     I2SCFG: 0x7F
 **********************************************************************/
SPORT_SIMPLE_CONFIG cfg8chPackedI2S = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_SLAVE,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_INV | SPORT_SIMPLE_FS_OPTION_EARLY |
                 SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_8,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_BOTH,
    .frames = SYSTEM_BLOCK_SIZE,
    .syncDMA = true
};

SPORT_SIMPLE_CONFIG cfg16chPackedI2S = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_SLAVE,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_INV | SPORT_SIMPLE_FS_OPTION_EARLY |
                 SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_16,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_BOTH,
    .frames = SYSTEM_BLOCK_SIZE,
    .syncDMA = true
};

void disable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE &= ~(
        (1 << BITP_PADS0_DAI0_IE_PB05)
    );
}

void enable_mclk(APP_CONTEXT *context)
{
    *pREG_PADS0_DAI1_IE |= (
        (1 << BITP_PADS0_DAI0_IE_PB05)
    );
}

static void sru_config_mclk(APP_CONTEXT *context)
{
    /* 24.576Mhz MCLK in from DAC 512Fs BCLK on DAI1.05 */
    SRU2(LOW, DAI1_PBEN05_I);

    /* Cross-route DAI1.05 MCLK to DAI0.11 as an output */
    SRU(HIGH, DAI0_PBEN11_I);
    SRU(DAI1_PB05_O, DAI0_PB11_I);
}

static void pcg_init_dai1_tdm8_bclk(void)
{
 /* Configure static PCG C parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_C,                   // PCG C
        .clk_src = PCG_SRC_DAI_PIN,     // Sourced from DAI
        .clk_in_dai_pin = 5,            // Sourced from DAI pin 5
        .lrclk_clocks_per_frame = 256,  // Not used
        .sync_to_fs = false
    };

    /* Configure a 12.288 MHz BCLK from 24.576 BCLK */
    pcg.bitclk_div = 2;
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

void mclk_init(APP_CONTEXT *context)
{
    sru_config_mclk(context);
    pcg_init_dai1_tdm8_bclk();
}

/***********************************************************************
 * ADAU1962 DAC / SPORT4 / SRU initialization (TDM16 clock slave)
 **********************************************************************/
#define ADAU1962_I2C_ADDR  (0x04)

static void sru_config_adau1962_master(void)
{
    /* Setup DAI Pin I/O */
    SRU2(HIGH, DAI1_PBEN01_I);      // ADAU1962 DAC data1 is an output
    SRU2(HIGH, DAI1_PBEN02_I);      // ADAU1962 DAC data2 is an output
    SRU2(LOW, DAI1_PBEN04_I);       // ADAU1962 FS is an input
    //SRU2(LOW, DAI1_PBEN05_I);       // ADAU1962 CLK is an input

    /* Route clocks */
    SRU2(DAI1_PB04_O, SPT4_AFS_I);   // route FS
    SRU2(DAI1_PB05_O, SPT4_ACLK_I);  // route BCLK

    /* Route to SPORT4A */
    SRU2(SPT4_AD0_O, DAI1_PB01_I);    // SPORT4A-D0 output to ADAU1962 data1 pin
    SRU2(SPT4_AD1_O, DAI1_PB02_I);    // SPORT4A-D1 output to ADAU1962 data2 pin
}

static void adau1962_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT4A: DAC 16-ch packed I2S data out */
    sportCfg = cfg16chPackedI2S;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
    memcpy(sportCfg.dataBuffers, context->codecAudioOut, sizeof(sportCfg.dataBuffers));
    context->dacSportOutHandle = single_sport_init(
        SPORT4A, &sportCfg, dacAudioOut,
        NULL, &len, context, false, NULL
    );
    assert(context->codecAudioOutLen == len);

    if (context->dacSportOutHandle) {
        sportResult = sport_start(context->dacSportOutHandle, true);
        assert(sportResult == SPORT_SIMPLE_SUCCESS);
    }
}

static void adau1962_sport_deinit(APP_CONTEXT *context)
{
    if (context->dacSportOutHandle) {
        sport_close(&context->dacSportOutHandle);
    }
}

void adau1962_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_adau1962_master();

    /* Configure the SPORT */
    adau1962_sport_init(context);

    /* Initialize the DAC */
    init_adau1962(context->adau1962TwiHandle, ADAU1962_I2C_ADDR);
}

/***********************************************************************
 * ADAU1979 ADC / SPORT6 / SRU initialization (TDM8 clock slave)
 *
 * WARNING: The ADAU1979 does not have the drive strength to reliably
 *          drive data out with a TDM16 bit clock.
 **********************************************************************/
#define ADAU1979_I2C_ADDR  (0x11)

static void sru_config_adau1979_slave(void)
{
    /* Setup DAI Pin I/O */
    SRU2(LOW, DAI1_PBEN06_I);      // ADAU1979 ADC data1 is an input
    SRU2(LOW, DAI1_PBEN07_I);      // ADAU1979 ADC data2 is an input
    SRU2(HIGH, DAI1_PBEN12_I);     // ADAU1979 CLK is an output
    SRU2(HIGH, DAI1_PBEN20_I);     // ADAU1979 FS is an output

    /* Route audio clocks to ADAU1979 */
    SRU2(SPT6_AFS_O, DAI1_PB20_I);   // route SPORT4A FS to ADAU1979 FS
    SRU2(PCG0_CLKC_O, DAI1_PB12_I);  // route TDM8 BCLK to ADAU1979 BCLK

    /* Route to SPORT6A */
    SRU2(DAI1_PB20_O, SPT6_AFS_I);   // route ADAU1979 FS to SPORT6A frame sync
    SRU2(DAI1_PB12_O, SPT6_ACLK_I);  // route ADAU1979 BCLK to SPORT6A clock input
    SRU2(DAI1_PB06_O, SPT6_AD0_I);   // ADAU1979 SDATAOUT1 pin to SPORT6A data0
    SRU2(DAI1_PB07_O, SPT6_AD1_I);   // ADAU1979 SDATAOUT2 pin to SPORT6A data1
}

static void adau1979_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT6A: ADC 8-ch packed I2S data in */
    sportCfg = cfg8chPackedI2S;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    memcpy(sportCfg.dataBuffers, context->codecAudioIn, sizeof(sportCfg.dataBuffers));
    context->adcSportInHandle = single_sport_init(
        SPORT6A, &sportCfg, adcAudioIn,
        NULL, &len, context, true, NULL
    );
    assert(context->codecAudioInLen == len);

    if (context->adcSportInHandle) {
        sportResult = sport_start(context->adcSportInHandle, true);
        assert(sportResult == SPORT_SIMPLE_SUCCESS);
    }
}

static void adau1979_sport_deinit(APP_CONTEXT *context)
{
    if (context->adcSportInHandle) {
        sport_close(&context->adcSportInHandle);
    }
}

void adau1979_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    sru_config_adau1979_slave();

    /* Configure the SPORT */
    adau1979_sport_init(context);

    /* Initialize the ADC */
    init_adau1979(context->adau1962TwiHandle, ADAU1979_I2C_ADDR);
}

/**************************************************************************
 * SPDIF Init
 *************************************************************************/
SPORT_SIMPLE_CONFIG cfgI2Sx1 = {
    .clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE,
    .fsDir = SPORT_SIMPLE_FS_DIR_MASTER,
    .dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN,
    .bitClkOptions = SPORT_SIMPLE_CLK_FALLING,
    .fsOptions = SPORT_SIMPLE_FS_OPTION_INV | SPORT_SIMPLE_FS_OPTION_EARLY |
                 SPORT_SIMPLE_FS_OPTION_50,
    .tdmSlots = SPORT_SIMPLE_TDM_2,
    .wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT,
    .dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY,
    .frames = SYSTEM_BLOCK_SIZE,
};

/* PCGA generates 12.288 MHz CLK from 24.576 MCLK/BCLK */
static void spdif_cfg_hfclk(void)
{
    /* Configure static PCG A parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_A,                    // PCG A
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = 11,            // Sourced from DAI pin 11
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgI2Sx1 SPORT config */
    pcg.bitclk_div = 2;

    /* This sets everything up */
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

/* PCGB generates 3.072 MHz I2S BCLK from 24.576 MCLK/BCLK */
static void spdif_cfg_bclk(void)
{
    /* Configure static PCG B parameters */
    PCG_SIMPLE_CONFIG pcg = {
        .pcg = PCG_B,                    // PCG B
        .clk_src = PCG_SRC_DAI_PIN,      // Sourced from DAI
        .clk_in_dai_pin = 11,            // Sourced from DAI pin 11
        .lrclk_clocks_per_frame = 256,   // Not used
        .sync_to_fs = false
    };

    /* Configure the PCG BCLK depending on the cfgI2Sx1 SPORT config */
    pcg.bitclk_div =
        SYSTEM_MCLK_RATE / (cfgI2Sx1.wordSize * cfgI2Sx1.tdmSlots * SYSTEM_SAMPLE_RATE);
    assert(pcg.bitclk_div > 0);

    /* This sets everything up */
    pcg_open(&pcg);
    pcg_enable(pcg.pcg, true);
}

/*
 * The SPDIF HFCLK is derived from PCG0_CLKA_O
 * The SPDIF BCLK is derived from PCG0_CLKB_O
 *
 */
static void spdif_sru_config(void)
{
    // Assign SPDIF I/O pins
    SRU(HIGH, DAI0_PBEN10_I);       // SPDIF TX is an output
    SRU(LOW,  DAI0_PBEN09_I);       // SPDIF RX is an input

    // Connect I/O pins to SPDIF module
    SRU(DAI0_PB09_O, SPDIF0_RX_I);  // route SPDIF RX
    SRU(SPDIF0_TX_O, DAI0_PB10_I);  // route SPDIF TX

    // Connect 64Fs BCLK to SPORT2A/B
    SRU(PCG0_CLKB_O, SPT2_ACLK_I);     // route PCG 64fs BCLK signal to SPORT2A BCLK
    SRU(PCG0_CLKB_O, SPT2_BCLK_I);     // route PCG 64fs BCLK signal to SPORT2B BCLK

    // Connect SPDIF RX to SRC 0 "IP" side
    SRU(SPDIF0_RX_CLK_O, SRC0_CLK_IP_I);     // route SPDIF RX BCLK to SRC IP BCLK
    SRU(SPDIF0_RX_FS_O,  SRC0_FS_IP_I);      // route SPDIF RX FS to SRC IP FS
    SRU(SPDIF0_RX_DAT_O, SRC0_DAT_IP_I);     // route SPDIF RX Data to SRC IP Data

    // Connect SPORT2B to SRC 0 "OP" side
    SRU(PCG0_CLKB_O,    SRC0_CLK_OP_I);     // route PCG 64fs BCLK signal to SRC OP BCLK
    SRU(SPT2_BFS_O,     SRC0_FS_OP_I);      // route SPORT FS signal to SRC OP FS
    SRU(SRC0_DAT_OP_O,  SPT2_BD0_I);        // route SRC0 OP Data output to SPORT 2B data

    // Connect 256Fs HFCLK to SPDIF TX
    SRU(PCG0_CLKA_O, SPDIF0_TX_HFCLK_I); // route PCGA_CLK to SPDIF TX HFCLK

    // Connect SPORT2A to SPDIF TX
    SRU(PCG0_CLKB_O, SPDIF0_TX_CLK_I);    // route 64fs BCLK signal to SPDIF TX BCLK
    SRU(SPT2_AFS_O,  SPDIF0_TX_FS_I);     // route SPORT2A FS signal to SPDIF TX FS
    SRU(SPT2_AD0_O,  SPDIF0_TX_DAT_I);    // SPT2A AD0 output to SPDIF TX data pin
}

static void spdif_sport_deinit(APP_CONTEXT *context)
{
    if (context->spdifSportOutHandle) {
        sport_close(&context->spdifSportOutHandle);
    }
    if (context->spdifSportInHandle) {
        sport_close(&context->spdifSportInHandle);
    }
}

static void spdif_sport_init(APP_CONTEXT *context)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    unsigned len;

    /* SPORT2A: SPDIF data out */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_TX;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->spdifAudioOut, sizeof(sportCfg.dataBuffers));
    context->spdifSportOutHandle = single_sport_init(
        SPORT2A, &sportCfg, spdifAudioOut,
        NULL, &len, context, false, NULL
    );
    assert(context->spdifAudioOutLen == len);

    /* SPORT2B: SPDIF data in */
    sportCfg = cfgI2Sx1;
    sportCfg.dataDir = SPORT_SIMPLE_DATA_DIR_RX;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->spdifAudioIn, sizeof(sportCfg.dataBuffers));
    context->spdifSportInHandle = single_sport_init(
        SPORT2B, &sportCfg, spdifAudioIn,
        NULL, &len, context, true, NULL
    );
    assert(context->spdifAudioInLen == len);

    /* Start SPORT2A/B */
    sportResult = sport_start(context->spdifSportOutHandle, true);
    sportResult = sport_start(context->spdifSportInHandle, true);
}

static void spdif_asrc_init(void)
{
    // Configure and enable SRC 0/1
    *pREG_ASRC0_CTL01 =
        BITM_ASRC_CTL01_EN0 |                // Enable SRC0
        (0x1 << BITP_ASRC_CTL01_SMODEIN0) |  // Input mode = I2S
        (0x1 << BITP_ASRC_CTL01_SMODEOUT0) | // Output mode = I2S
        0;

    // Configure and enable SPDIF RX
    *pREG_SPDIF0_RX_CTL =
        BITM_SPDIF_RX_CTL_EN |          // Enable the SPDIF RX
        BITM_SPDIF_RX_CTL_FASTLOCK |    // Enable SPDIF Fastlock (see HRM 32-15)
        BITM_SPDIF_RX_CTL_RSTRTAUDIO |
        0;

    // Configure SPDIF Transmitter in auto mode
    *pREG_SPDIF0_TX_CTL =
        (0x1 << BITP_SPDIF_TX_CTL_SMODEIN) |  // I2S Mode
        BITM_SPDIF_TX_CTL_AUTO |             // Standalone mode
        0;

    // Enable SPDIF transmitter
    *pREG_SPDIF0_TX_CTL |=
        BITM_SPDIF_TX_CTL_EN |         // Enable SPDIF TX
        0;
}

void spdif_init(APP_CONTEXT *context)
{
    /* Configure the DAI routing */
    spdif_sru_config();

    /* Initialize the SPDIF BCLK and HFCLK PCGs */
    spdif_cfg_bclk();
    spdif_cfg_hfclk();

    /* Initialize the SPDIF and ASRC modules */
    spdif_asrc_init();

    /* Initialize the SPORTs */
    spdif_sport_init(context);
}

/***********************************************************************
 * A2B / SPORT1 / SRU / IRQ initialization
 **********************************************************************/
#include "a2b_irq.h"

void a2b_pint_init(APP_CONTEXT *context)
{
    ADI_GPIO_RESULT result;
    uint32_t pint2pins = 0x00000000;

    /* Don't wire in interrupts of no A2B */
    if (!context->a2bPresent) {
        return;
    }

    /* This code must match the values in a2b_irq.h */
    assert(A2B1_PINT_IRQ == ADI_GPIO_PIN_INTERRUPT_2);
    assert(A2B1_PINT_PIN == ADI_GPIO_INT_PIN_5);

    /*
     * A1B1 (J10): Pin C5 (PINT2.5)
     */
    pint2pins |= ADI_GPIO_INT_PIN_5;

    /* Map Pins PC0-PC7 to PINT2 Byte 0 */
    result = adi_gpio_PinInterruptAssignment(
        ADI_GPIO_PIN_INTERRUPT_2,
        ADI_GPIO_PIN_ASSIGN_BYTE_0,
        ADI_GPIO_PIN_ASSIGN_PCL_PINT2
    );
    result = adi_gpio_SetPinIntEdgeSense(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        ADI_GPIO_SENSE_RISING_EDGE
    );
    result = adi_gpio_RegisterCallback(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        a2b_irq,
        context
    );
    result = adi_gpio_EnablePinInterruptMask(
        ADI_GPIO_PIN_INTERRUPT_2,
        pint2pins,
        true
    );
}

bool a2b_to_sport_cfg(bool master, bool rxtx,
    uint8_t I2SGCFG, uint8_t I2SCFG, SPORT_SIMPLE_CONFIG *sportCfg,
    bool verbose)
{
    SPORT_SIMPLE_CONFIG backup;
    bool ok = false;
    uint8_t bits;

    if (!sportCfg) {
        goto abort;
    }

    if (verbose) { syslog_print("A2B SPORT CFG"); }

    /* Save a backup in case of failure */
    memcpy(&backup, sportCfg, sizeof(backup));

    /* Reset elements that are configured */
    sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_UNKNOWN;
    sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_UNKNOWN;
    sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_UNKNOWN;
    sportCfg->tdmSlots = SPORT_SIMPLE_TDM_UNKNOWN;
    sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_UNKNOWN;
    sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_NONE;
    sportCfg->bitClkOptions = SPORT_SIMPLE_CLK_DEFAULT;
    sportCfg->fsOptions = SPORT_SIMPLE_FS_OPTION_DEFAULT;

    /*
     * Set .clkDir, .fsDir, .dataDir
     *
     * if master, set clk/fs to master, else slave
     * if rxtx, set to input, else output
     *
     */
    if (master) {
        sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_MASTER;
        sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg->clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
        sportCfg->fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    if (rxtx) {
        sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_RX;
        if (verbose) { syslog_print(" Direction: RX (AD24xx DTX pins)"); }
    } else {
        sportCfg->dataDir = SPORT_SIMPLE_DATA_DIR_TX;
        if (verbose) { syslog_print(" Direction: TX (AD24xx DRX pins)"); }
    }

    /*
     * Set .wordSize
     *
     */
    if (I2SGCFG & 0x10) {
        sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_16BIT;
        if (verbose) { syslog_print(" Size: 16-bit"); }
    } else {
        sportCfg->wordSize = SPORT_SIMPLE_WORD_SIZE_32BIT;
        if (verbose) { syslog_print(" Size: 32-bit"); }
    }

    /*
     * Set .tdmSlots
     */
    switch (I2SGCFG & 0x07) {
        case 0:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_2;
            if (verbose) { syslog_print(" TDM: 2 (I2S)"); }
            break;
        case 1:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_4;
            if (verbose) { syslog_print(" TDM: 4"); }
            break;
        case 2:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_8;
            if (verbose) { syslog_print(" TDM: 8"); }
            break;
        case 4:
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_16;
            if (verbose) { syslog_print(" TDM: 16"); }
            break;
        case 7:
            /*
             * TDM32 with 32-bit word size is not supported with a
             * 24.576MCLK
             */
            if (sportCfg->wordSize == SPORT_SIMPLE_WORD_SIZE_32BIT) {
                goto abort;
            }
            sportCfg->tdmSlots = SPORT_SIMPLE_TDM_32;
            if (verbose) { syslog_print(" TDM: 32"); }
            break;
        default:
            goto abort;
    }

    /*
     * Set .dataEnable
     *
     */
    if (rxtx) {
        bits = I2SCFG >> 0;
    } else {
        bits = I2SCFG >> 4;
    }
    switch (bits & 0x03) {
        case 0x01:
            sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_PRIMARY;
            if (verbose) { syslog_print(" Data Pins: Primary"); }
            break;
        case 0x02:
            sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_SECONDARY;
            if (verbose) { syslog_print(" Data Pins: Secondary"); }
            break;
        case 0x03:
            sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_BOTH;
            if (verbose) {
                syslog_print(" Data Pins: Both");
                syslog_printf(" Interleave: %s", (bits & 0x04) ? "Yes" : "No");
            }
            break;
        default:
            sportCfg->dataEnable = SPORT_SIMPLE_ENABLE_NONE;
            if (verbose) { syslog_print(" Data Pins: None"); }
            break;
    }

    /*
     * Set .bitClkOptions
     *
     * Default setting is assert on the rising edge, sample on falling (TDM)
     *
     */
    if (rxtx) {
        if ((I2SCFG & 0x80) == 0) {
            sportCfg->bitClkOptions |= SPORT_SIMPLE_CLK_FALLING;
            if (verbose) { syslog_print(" CLK: Assert falling, Sample rising (I2S)"); }
        } else {
            if (verbose) { syslog_print(" CLK: Assert rising, Sample falling"); }
        }
    } else {
        if (I2SCFG & 0x08) {
            sportCfg->bitClkOptions |= SPORT_SIMPLE_CLK_FALLING;
            if (verbose) { syslog_print(" CLK: Assert falling, Sample rising (I2S)"); }
        } else {
            if (verbose) { syslog_print(" CLK: Assert rising, Sample falling"); }
        }
    }

    /*
     * Set .fsOptions
     *
     * Default setting is pulse, rising edge frame sync where the
     * frame sync signal asserts in the same cycle as the MSB of the
     * first data slot (TDM)
     */
    if (I2SGCFG & 0x80) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_INV;
        if (verbose) { syslog_print(" FS: Falling edge (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Rising edge"); }
    }
    if (I2SGCFG & 0x40) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_EARLY;
        if (verbose) { syslog_print(" FS: Early (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Not Early"); }
    }
    if (I2SGCFG & 0x20) {
        sportCfg->fsOptions |= SPORT_SIMPLE_FS_OPTION_50;
        if (verbose) { syslog_print(" FS: 50% (I2S)"); }
    } else {
        if (verbose) { syslog_print(" FS: Pulse"); }
    }

    ok = true;

abort:
    if (!ok) {
        memcpy(sportCfg, &backup, sizeof(*sportCfg));
    }
    return(ok);
}

/**
 *
 * A2B Master Mode Configuration:
 *    - MCLK/BCLK to SPORT1B/A2B Transceiver
 *    - SPORT1A FS to SPORT1B/A2B Transceiver
 */
void sru_config_a2b_master(void)
{
    // Set up pins for J10 A2B
    SRU(HIGH,  DAI0_PBEN03_I);        // pin for A2B BCLK is an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN04_I);        // pin for A2B FS is an output (to A2B bus)
    SRU(LOW,   DAI0_PBEN01_I);        // DTX0 is always an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN02_I);        // DTX1 is always an input (from A2B bus)
    SRU(HIGH,  DAI0_PBEN05_I);        // DRX0 is always an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN06_I);        // DRX1 is always an output (to A2B bus)

    // BCLK/MCLK to A2B and SPORTA/B CLK */
    SRU(DAI0_PB11_O, DAI0_PB03_I);     // route MCLK/BCLK to A2B
    SRU(DAI0_PB11_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB11_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B

    // SPORT1A FS to A2B and SPORT1B FS */
    SRU(SPT1_AFS_O, DAI0_PB04_I);     // route SPORT1A FS to A2B
    SRU(SPT1_AFS_O, SPT1_BFS_I);      // route SPORT1A FS to SPORT1B

    // Connect A2B data signals to SPORT1
    SRU(SPT1_AD0_O, DAI0_PB05_I);     // route SPORT1A data TX primary to A2B DRX0
    SRU(SPT1_AD1_O, DAI0_PB06_I);     // route SPORT1A data TX secondary to A2B DRX0
    SRU(DAI0_PB01_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
    SRU(DAI0_PB02_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
}

/**
 *
 * A2B Slave Mode Configuration:
 *    - A2B BCLK to SPORT1B
 *    - A2B FS to SPORT1B
 *
 */
void sru_config_a2b_slave(void)
{
    // Set up pins for J10 A2B
    SRU(LOW,   DAI0_PBEN03_I);        // pin for A2B BCLK is an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN04_I);        // pin for A2B FS is an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN01_I);        // DTX0 is always an input (from A2B bus)
    SRU(LOW,   DAI0_PBEN02_I);        // DTX1 is always an input (from A2B bus)
    SRU(HIGH,  DAI0_PBEN05_I);        // DRX0 is always an output (to A2B bus)
    SRU(HIGH,  DAI0_PBEN06_I);        // DRX1 is always an output (to A2B bus)

    // BCLK/MCLK to SPORTA/B CLK */
    SRU(DAI0_PB03_O, SPT1_ACLK_I);     // route MCLK/BCLK to SPORT1A
    SRU(DAI0_PB03_O, SPT1_BCLK_I);     // route MCLK/BCLK to SPORT1B

    // SPORT1A FS to SPORT1B FS */
    SRU(DAI0_PB04_O, SPT1_AFS_I);      // route FS to SPORT1A
    SRU(DAI0_PB04_O, SPT1_BFS_I);      // route FS to SPORT1B

    // Connect A2B data signals to SPORT1
    SRU(SPT1_AD0_O, DAI0_PB05_I);     // route SPORT1A data TX primary to A2B DRX0
    SRU(SPT1_AD1_O, DAI0_PB06_I);     // route SPORT1A data TX secondary to A2B DRX0
    SRU(DAI0_PB01_O, SPT1_BD0_I);     // route A2B DTX0 to SPORT1B data RX primary
    SRU(DAI0_PB02_O, SPT1_BD1_I);     // route A2B DTX1 to SPORT1B data RX secondary
}

#define AD242X_CONTROL             0x12u
#define AD242X_CONTROL_SOFTRST     0x04u
#define AD242X_CONTROL_MSTR        0x80u

/* Soft reset a single transceiver */
bool a2b_restart(APP_CONTEXT *context)
{
    TWI_SIMPLE_RESULT result;
    uint8_t wBuf[2];

    wBuf[0] = AD242X_CONTROL;
    wBuf[1] = AD242X_CONTROL_SOFTRST;
    if (context->a2bmode == A2B_BUS_MODE_MAIN) {
        wBuf[1] |= AD242X_CONTROL_MSTR;
    }

    result = twi_write(context->a2bTwiHandle, A2B_I2C_ADDR,
        wBuf, sizeof(wBuf));

    delay(10);

    return(result == TWI_SIMPLE_SUCCESS);
}

void a2b_reset(APP_CONTEXT *context)
{
    a2b_restart(context);
}

void sportCfg2ipcMsg(SPORT_SIMPLE_CONFIG *sportCfg, unsigned dataLen, IPC_MSG *msg)
{
    msg->audio.wordSize = sportCfg->wordSize / 8;
    msg->audio.numChannels = dataLen / (sportCfg->frames * msg->audio.wordSize);
}

bool a2b_sport_init(APP_CONTEXT *context,
    bool master, CLOCK_DOMAIN clockDomain, uint8_t I2SGCFG, uint8_t I2SCFG,
    bool verbose)
{
    SPORT_SIMPLE_CONFIG sportCfg;
    SPORT_SIMPLE_RESULT sportResult;
    bool sportCfgOk;
    IPC_MSG *msg;
    bool rxtx;
    int i;

    /* Calculate the SPORT0A TX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = false;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    if (master) {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_MASTER;
    } else {
        sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    }
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->a2bAudioOut, sizeof(sportCfg.dataBuffers));
    context->a2bSportOutHandle = single_sport_init(
        SPORT1A, &sportCfg, a2bAudioOut,
        NULL, &context->a2bAudioOutLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        for (i = 0; i < 2; i++) {
            msg = (IPC_MSG *)sae_getMsgBufferPayload(context->a2bMsgOut[i]);
            sportCfg2ipcMsg(&sportCfg, context->a2bAudioOutLen, msg);
        }
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_OUT);
        sportResult = sport_start(context->a2bSportOutHandle, true);
    } else {
        if (context->a2bSportOutHandle) {
            sport_close(&context->a2bSportOutHandle);
        }
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);
    }

    /* Calculate the SPORT0B RX configuration */
    memset(&sportCfg, 0, sizeof(sportCfg));
    rxtx = true;
    sportCfgOk = a2b_to_sport_cfg(master, rxtx, I2SGCFG, I2SCFG, &sportCfg, verbose);
    if (!sportCfgOk) {
        goto abort;
    }
    sportCfg.clkDir = SPORT_SIMPLE_CLK_DIR_SLAVE;
    sportCfg.fsDir = SPORT_SIMPLE_FS_DIR_SLAVE;
    sportCfg.frames = SYSTEM_BLOCK_SIZE;
    sportCfg.fs = SYSTEM_SAMPLE_RATE;
    sportCfg.dataBuffersCached = false;
    memcpy(sportCfg.dataBuffers, context->a2bAudioIn, sizeof(sportCfg.dataBuffers));
    context->a2bSportInHandle = single_sport_init(
        SPORT1B, &sportCfg, a2bAudioIn,
        NULL, &context->a2bAudioInLen, context, false, &sportResult
    );
    if (sportResult == SPORT_SIMPLE_SUCCESS) {
        for (i = 0; i < 2; i++) {
            msg = (IPC_MSG *)sae_getMsgBufferPayload(context->a2bMsgIn[i]);
            sportCfg2ipcMsg(&sportCfg, context->a2bAudioInLen, msg);
        }
        clock_domain_set(context, clockDomain, CLOCK_DOMAIN_BITM_A2B_IN);
        sportResult = sport_start(context->a2bSportInHandle, true);
    } else {
        if (context->a2bSportInHandle) {
            sport_close(&context->a2bSportInHandle);
        }
        clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    }

abort:
    return(sportCfgOk);
}

bool a2b_sport_deinit(APP_CONTEXT *context)
{
    if (context->a2bSportOutHandle) {
        sport_close(&context->a2bSportOutHandle);
    }
    if (context->a2bSportInHandle) {
        sport_close(&context->a2bSportInHandle);
    }
    return(true);
}

bool a2b_master_init(APP_CONTEXT *context)
{
    bool ok;

    sru_config_a2b_master();

    ok = a2b_sport_init(context, true, CLOCK_DOMAIN_SYSTEM,
        SYSTEM_I2SGCFG, SYSTEM_I2SCFG, false);
    if (ok) {
        context->a2bmode = A2B_BUS_MODE_MAIN;
        context->a2bSlaveActive = false;
    }

    return(ok);
}

static void a2b_disconnect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE &= ~(
        BITP_PADS0_DAI0_IE_PB03 | BITP_PADS0_DAI0_IE_PB04
    );
}

static void a2b_connect_slave_clocks(void)
{
    *pREG_PADS0_DAI0_IE |= (
        BITP_PADS0_DAI0_IE_PB03 | BITP_PADS0_DAI0_IE_PB04
    );
}

bool a2b_init_slave(APP_CONTEXT *context)
{
    sru_config_a2b_slave();

    context->a2bmode = A2B_BUS_MODE_SUB;

    /*
     * Disconnect A2B from all clock domains.  IN and OUT will be re-attached
     * to the A2B domain during discovery when/if the TX and RX serializers
     * are enabled.
     */
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_IN);
    clock_domain_set(context, CLOCK_DOMAIN_MAX, CLOCK_DOMAIN_BITM_A2B_OUT);

    return(true);
}

bool a2b_set_mode(APP_CONTEXT *context, A2B_BUS_MODE mode)
{
    if (mode == context->a2bmode) {
        return(true);
    }

    a2b_sport_deinit(context);

    if (mode == A2B_BUS_MODE_SUB) {
        a2b_init_slave(context);
    } else {
        adau1962_sport_deinit(context);
        adau1979_sport_deinit(context);
        spdif_sport_deinit(context);
        disable_mclk(context);
        adau1962_sport_init(context);
        spdif_sport_init(context);
        a2b_master_init(context);
        enable_mclk(context);
    }

    a2b_restart(context);

    return(true);
}

bool a2b_sport_start(APP_CONTEXT *context, uint8_t I2SGCFG, uint8_t I2SCFG)
{
    bool ok;
    ok = a2b_sport_init(context, false, CLOCK_DOMAIN_A2B,
        I2SGCFG, I2SCFG, true);
    a2b_connect_slave_clocks();
    return(ok);
}

bool a2b_sport_stop(APP_CONTEXT *context)
{
    bool ok;
    ok = a2b_sport_deinit(context);
    a2b_disconnect_slave_clocks();
    return(ok);
}

/**********************************************************************
 * Ethernet initialization
 **********************************************************************/
static void eth_emac0_init(APP_CONTEXT *context)
{
    /* Reset PHY */
    ss_set(context, SS_PIN_ID_nGIGe_RESET, 0);
    delay(5);
    ss_set(context, SS_PIN_ID_nGIGe_RESET, 1);
    delay(5);

    /* Select RGMII for EMAC0 */
    *pREG_PADS0_PCFG0 &= ~(BITM_PADS_PCFG0_EMACPHYISEL);
    *pREG_PADS0_PCFG0 |= (1 << BITP_PADS_PCFG0_EMACPHYISEL);

    /* Bring EMAC0 out of reset */
    *pREG_PADS0_PCFG0 |= BITM_PADS_PCFG0_EMACRESET;

    /* SPU Settings for EMAC0 */
    if (*pREG_SPU0_SECURECHK == 0xFFFFFFFF)
    {
        *pREG_SPU0_SECUREP55 = 0x03;
    }
}

static void eth_emac1_init(APP_CONTEXT *context)
{
    /* Enable ETH1 */
    ss_set(context, SS_PIN_ID_nETH1_EN, 0);

    /* Reset PHY */
    ss_set(context, SS_PIN_ID_nETH1_RESET, 0);
    delay(5);
    ss_set(context, SS_PIN_ID_nETH1_RESET, 1);
    delay(5);

    /* SPU Settings for EMAC1 */
    if (*pREG_SPU0_SECURECHK == 0xFFFFFFFF)
    {
        *pREG_SPU0_SECUREP56 = 0x03;
    }
}

void eth_hardware_init(APP_CONTEXT *context)
{
    eth_emac0_init(context);
    eth_emac1_init(context);
}

/***********************************************************************
 * SHARC Audio Engine (SAE) Audio IPC buffer configuration
 **********************************************************************/
/*
 * allocateIpcAudioMsg()
 *
 * Allocates an IPC_MSG_AUDIO Audio message and saves the data payload
 * pointer.
 *
 */
static SAE_MSG_BUFFER *allocateIpcAudioMsg(APP_CONTEXT *context,
    uint16_t size, uint8_t streamID, uint8_t numChannels, uint8_t wordSize,
    void **audioPtr)
{
    SAE_CONTEXT *saeContext = context->saeContext;
    SAE_MSG_BUFFER *msgBuffer;
    IPC_MSG *msg;
    uint16_t msgSize;

    /* Create an IPC message large enough to hold an IPC_MSG_AUDIO struct
     * with the data payload.
     */
    msgSize = sizeof(*msg) + size;

    /* Allocate a message buffer and initialize both the USB_IPC_SRC_MSG's
     * 'msgBuffer' and 'msg' members.
     */
    msgBuffer = sae_createMsgBuffer(saeContext, msgSize, (void **)&msg);
    assert(msgBuffer);

    /* Set fixed 'IPC_MSG_AUDIO' parameters */
    msg->type = IPC_TYPE_AUDIO;
    msg->audio.streamID = streamID;
    msg->audio.numChannels = numChannels;
    msg->audio.wordSize = wordSize;
    msg->audio.numFrames = size / (numChannels * wordSize);
    if (audioPtr) {
        *audioPtr = msg->audio.data;
    }

    return(msgBuffer);
}

/*
 * sae_buffer_init()
 *
 * Allocates and configures all of the SAE message/audio ping/pong
 * buffers between the ARM and SHARC0 and SHARC1.  Audio DMA buffers
 * are sent by reference from the ARM to the SHARCs
 *
 * These buffers can be referenced and used locally through the
 * context->xxxAudioIn/Out[] ping/pong buffers and sent/received via
 * the IPC message buffers context->xxxMsgIn/Out[].
 *
 */
void sae_buffer_init(APP_CONTEXT *context)
{
    int i;

    /* Allocate and initialize audio IPC ping/pong message buffers */
    for (i = 0; i < 2; i++) {

        /* ADC Audio In */
        context->codecAudioInLen =
            ADC_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->codecMsgIn[i] = allocateIpcAudioMsg(
            context, context->codecAudioInLen,
            IPC_STREAMID_CODEC_IN, ADC_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->codecAudioIn[i]
        );
        assert(context->codecMsgIn[i]);
        memset(context->codecAudioIn[i], 0, context->codecAudioInLen);

        /* DAC Audio Out */
        context->codecAudioOutLen =
            DAC_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->codecMsgOut[i] = allocateIpcAudioMsg(
            context, context->codecAudioOutLen,
            IPC_STREAMID_CODEC_OUT, DAC_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->codecAudioOut[i]
        );
        assert(context->codecMsgOut[i]);
        memset(context->codecAudioOut[i], 0, context->codecAudioOutLen);

        /* SPDIF Audio In */
        context->spdifAudioInLen =
            SPDIF_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->spdifMsgIn[i] = allocateIpcAudioMsg(
            context, context->spdifAudioInLen,
            IPC_STREAMID_SPDIF_IN, SPDIF_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->spdifAudioIn[i]
        );
        assert(context->spdifMsgIn[i]);
        memset(context->spdifAudioIn[i], 0, context->spdifAudioInLen);

        /* SPDIF Audio Out */
        context->spdifAudioOutLen =
            SPDIF_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->spdifMsgOut[i] = allocateIpcAudioMsg(
            context, context->spdifAudioOutLen,
            IPC_STREAMID_SPDIF_OUT, SPDIF_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->spdifAudioOut[i]
        );
        assert(context->spdifMsgOut[i]);
        memset(context->spdifAudioOut[i], 0, context->spdifAudioOutLen);

        /* A2B Audio In */
        context->a2bAudioInLen =
            A2B_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->a2bMsgIn[i] = allocateIpcAudioMsg(
            context, context->a2bAudioInLen,
            IPC_STREAMID_A2B_IN, A2B_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->a2bAudioIn[i]
        );
        assert(context->a2bMsgIn[i]);
        memset(context->a2bAudioIn[i], 0, context->a2bAudioInLen);

        /* A2B Audio Out */
        context->a2bAudioOutLen =
            A2B_DMA_CHANNELS * sizeof(SYSTEM_AUDIO_TYPE) * SYSTEM_BLOCK_SIZE;
        context->a2bMsgOut[i] = allocateIpcAudioMsg(
            context, context->a2bAudioOutLen,
            IPC_STREAMID_A2B_OUT, A2B_DMA_CHANNELS, sizeof(SYSTEM_AUDIO_TYPE),
            &context->a2bAudioOut[i]
        );
        assert(context->a2bMsgOut[i]);
        memset(context->a2bAudioOut[i], 0, context->a2bAudioOutLen);
    }
}

/*
 * audio_routing_init()
 *
 * Allocates and configures the audio routing array message for use
 * with the SAE.
 *
 */
void audio_routing_init(APP_CONTEXT *context)
{
    SAE_CONTEXT *saeContext = context->saeContext;
    IPC_MSG *msg;
    unsigned msgSize;

    /* Create an IPC message large enough to hold the routing table */
    msgSize = sizeof(*msg) +
        (MAX_AUDIO_ROUTES - 1) * sizeof(ROUTE_INFO);

    /* Allocate a message buffer */
    context->routingMsgBuffer = sae_createMsgBuffer(
        saeContext, msgSize, (void **)&context->routingMsg
    );
    assert(context->routingMsgBuffer);

    /* Initialize the message and routing table */
    memset(context->routingMsg, 0 , msgSize);
    context->routingMsg->type = IPC_TYPE_AUDIO_ROUTING;
    context->routingMsg->routes.numRoutes = MAX_AUDIO_ROUTES;
}
