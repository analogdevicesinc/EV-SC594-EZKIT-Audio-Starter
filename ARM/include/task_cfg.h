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

#ifndef _task_cfg_h
#define _task_cfg_h

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* The priorities assigned to the tasks (higher number == higher prio). */
#define HOUSEKEEPING_PRIORITY       (tskIDLE_PRIORITY + 1)
#define STARTUP_TASK_LOW_PRIORITY   (tskIDLE_PRIORITY + 2)
#define ETHERNET_PRIORITY           (tskIDLE_PRIORITY + 4)
#define ETHER_WORKER_PRIO           (tskIDLE_PRIORITY + 4)
#define A2B_IRQ_TASK_PRIORITY       (tskIDLE_PRIORITY + 4)
#define STARTUP_TASK_HIGH_PRIORITY  (tskIDLE_PRIORITY + 5)

/* The check task uses the sprintf function so requires a little more stack. */
#define HOUSEKEEPING_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
#define STARTUP_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE + 512)
#define ETHERNET_TASK_STACK_SIZE     (configMINIMAL_STACK_SIZE + 256)
#define TCPIP_THREAD_STACKSIZE       (configMINIMAL_STACK_SIZE + 256)
#define GENERIC_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE)

#endif
