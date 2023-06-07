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

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <assert.h>

/* CCES includes */
#include <sys/adi_core.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* Simple driver includes */
#include "spi_simple.h"
#include "twi_simple.h"
#include "sport_simple.h"
#include "uart_simple.h"
#include "uart_stdio.h"

/* Simple service includes */
#include "cpu_load.h"
#include "syslog.h"
#include "sae.h"
#include "fs_devman.h"
#include "fs_devio.h"
#include "fs_dev_romfs.h"
#include "fs_dev_fatfs.h"
#include "fs_dev_spiffs.h"

/* oss-services includes */
#include "shell.h"
#include "umm_malloc.h"
#include "romfs.h"

/* Project includes */
#include "context.h"
#include "init.h"
#include "clocks.h"
#include "util.h"
#include "a2b_slave.h"
#include "clock_domain.h"
#include "spiffs_fs.h"
#include "cpu_load.h"
#include "task_cfg.h"
#include "ss_init.h"
#include "a2b_irq.h"
#include "ethernet_init.h"
#include "ipc.h"

/* Application context */
APP_CONTEXT mainAppContext;

/***********************************************************************
 * Shell console I/O functions
 **********************************************************************/
static void term_out( char data, void *usr )
{
    putc(data, stdout); fflush(stdout);
}

static int term_in( int mode, void *usr )
{
    int c;
    int timeout;

    if (mode == TERM_INPUT_DONT_WAIT) {
        timeout = STDIO_TIMEOUT_NONE;
    } else if (mode == TERM_INPUT_WAIT) {
        timeout = STDIO_TIMEOUT_INF;
    } else {
        timeout = mode / 1000;
    }

    uart_stdio_set_read_timeout(timeout);

    if ((c = getc(stdin)) == EOF) {
        return(-1);
    }

    return(c);
}

/***********************************************************************
 * CPU idle time / High precision timestamp functions
 **********************************************************************/
uint32_t getTimeStamp(void)
{
    uint32_t timeStamp;
    timeStamp = *pREG_CGU0_TSCOUNT0;
    return timeStamp;
}

void taskSwitchHook(void *taskHandle)
{
    cpuLoadtaskSwitchHook(taskHandle);
}

uint32_t elapsedTimeMs(uint32_t elapsed)
{
    return(((1000ULL) * (uint64_t)elapsed) / CGU_TS_CLK);
}

/***********************************************************************
 * Misc application utility functions (util.h)
 **********************************************************************/
void delay(unsigned ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

time_t util_time(time_t *tloc)
{
    APP_CONTEXT *context = &mainAppContext;
    time_t t;

    /*
     * Our time starts at zero so add 10 years + 2 days of milliseconds
     * to adjust UNIX epoch of 1970 to FAT epoch of 1980 to keep
     * FatFS happy.  See get_fattime() in diskio.c.
     *
     * https://www.timeanddate.com/date/timeduration.html
     *
     */
    vTaskSuspendAll();
    t = (context->now + (uint64_t)315532800000) / configTICK_RATE_HZ;
    xTaskResumeAll();

    if (tloc) {
        memcpy(tloc, &t, sizeof(*tloc));
    }

    return(t);
}

/***********************************************************************
 * Application IPC functions
 **********************************************************************/
SAE_RESULT ipcToCore(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *ipcBuffer, SAE_CORE_IDX core)
{
    SAE_RESULT result;

    result = sae_sendMsgBuffer(saeContext, ipcBuffer, core, true);
    if (result != SAE_RESULT_OK) {
        sae_unRefMsgBuffer(saeContext, ipcBuffer);
    }

    return(result);
}

SAE_RESULT quickIpcToCore(APP_CONTEXT *context, enum IPC_TYPE type, SAE_CORE_IDX core)
{
    SAE_CONTEXT *saeContext = context->saeContext;
    SAE_MSG_BUFFER *ipcBuffer;
    SAE_RESULT result;
    IPC_MSG *msg;

    ipcBuffer = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
    if (ipcBuffer) {
        msg->type = type;
        result = ipcToCore(saeContext, ipcBuffer, core);
    } else {
        result = SAE_RESULT_ERROR;
    }

    return(result);
}

static void ipcMsgHandler(SAE_CONTEXT *saeContext, SAE_MSG_BUFFER *buffer,
    void *payload, void *usrPtr)
{
    APP_CONTEXT *context = (APP_CONTEXT *)usrPtr;
    IPC_MSG *msg = (IPC_MSG *)payload;
    IPC_MSG_AUDIO *audio;
    IPC_MSG_CYCLES *cycles;
    SAE_RESULT result;
    uint32_t max, i;

    UNUSED(context);

    /* Process the message */
    switch (msg->type) {
        case IPC_TYPE_PING:
            /* Do nothing */
            break;
        case IPC_TYPE_SHARC0_READY:
            context->sharc0Ready = true;
            break;
        case IPC_TYPE_SHARC1_READY:
            context->sharc1Ready = true;
            break;
        case IPC_TYPE_AUDIO:
            audio = (IPC_MSG_AUDIO *)&msg->audio;
            break;
        case IPC_TYPE_CYCLES:
            cycles = (IPC_MSG_CYCLES *)&msg->cycles;
            max = CLOCK_DOMAIN_MAX < IPC_CYCLE_DOMAIN_MAX ?
                CLOCK_DOMAIN_MAX : IPC_CYCLE_DOMAIN_MAX;
            for (i = 0; i < max; i++) {
                if (cycles->core == IPC_CORE_SHARC0) {
                    context->sharc0Cycles[i] = cycles->cycles[i];
                } else if (cycles->core == IPC_CORE_SHARC1) {
                    context->sharc1Cycles[i] = cycles->cycles[i];
                }
            }
            break;
        default:
            break;
    }

    /* Done with the message so decrement the ref count */
    result = sae_unRefMsgBuffer(saeContext, buffer);
}

/***********************************************************************
 * Tasks
 **********************************************************************/
void ledFlashOff(TimerHandle_t xTimer)
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvTimerGetTimerID(xTimer);
    ss_set(context, SS_PIN_ID_LED4, 0);
}

/* Background housekeeping task */
static portTASK_FUNCTION( houseKeepingTask, pvParameters )
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    SAE_CONTEXT *saeContext = context->saeContext;
    SAE_MSG_BUFFER *msgBuffer;
    TickType_t flashRate, lastFlashTime, clk, lastClk;
    bool calcLoad;
    TimerHandle_t ledFlashTimer;
    IPC_MSG *msg;

    /* Configure the LED to flash at a 1Hz rate */
    flashRate = pdMS_TO_TICKS(1000);
    lastFlashTime = xTaskGetTickCount();
    lastClk = xTaskGetTickCount();

    /* Calculate the system load every other cycle */
    calcLoad = false;

    /* Initialize the LED flash off timer */
    ledFlashTimer = xTimerCreate(
        "ledFlashTimer", pdMS_TO_TICKS(25), pdFALSE, context, ledFlashOff
    );

    /* Spin forever doing houseKeeping tasks */
    while (1) {

        /* Calculate the system load */
        if (calcLoad) {
            cpuLoadCalculateLoad(NULL);
            calcLoad = false;
        } else {
            calcLoad = true;
        }

        /* Flash the LED */
        ss_set(context, SS_PIN_ID_LED4, 1);
        xTimerStart(ledFlashTimer, portMAX_DELAY);

        /* Ping both SHARCs with the same message */
        msgBuffer = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
        if (msgBuffer) {
            msg->type = IPC_TYPE_PING;
            sae_refMsgBuffer(saeContext, msgBuffer);
            ipcToCore(saeContext, msgBuffer, IPC_CORE_SHARC0);
            ipcToCore(saeContext, msgBuffer, IPC_CORE_SHARC1);
        }

        /* Get cycles from both SHARCs */
        msgBuffer = sae_createMsgBuffer(saeContext, sizeof(*msg), (void **)&msg);
        if (msgBuffer) {
            msg->type = IPC_TYPE_CYCLES;
            sae_refMsgBuffer(saeContext, msgBuffer);
            ipcToCore(saeContext, msgBuffer, IPC_CORE_SHARC0);
            ipcToCore(saeContext, msgBuffer, IPC_CORE_SHARC1);
        }

        clk = xTaskGetTickCount();
        context->now += (uint64_t)(clk - lastClk);
        lastClk = clk;

        /* Sleep for a while */
        vTaskDelayUntil( &lastFlashTime, flashRate );
    }
}

/***********************************************************************
 * Application defaults
 **********************************************************************/
static char *cfgStrDup(const char *value)
{
    char *str = umm_malloc(strlen(value)+1);
    strcpy(str, value);
    return(str);
}

static void setAppDefaults(APP_CFG *cfg)
{
    cfg->eth0.port = EMAC0;
    cfg->eth0.ip_addr = cfgStrDup(DEFAULT_ETH0_IP_ADDR);
    cfg->eth0.gateway_addr = cfgStrDup(DEFAULT_ETH0_GW_ADDR);
    cfg->eth0.netmask = cfgStrDup(DEFAULT_ETH0_NETMASK);
    cfg->eth0.static_ip = DEFAULT_ETH0_STATIC_IP;
    cfg->eth0.default_iface = DEFAULT_ETH0_DEFAULT_IFACE;

    cfg->eth1.port = EMAC1;
    cfg->eth1.ip_addr = cfgStrDup(DEFAULT_ETH1_IP_ADDR);
    cfg->eth1.gateway_addr = cfgStrDup(DEFAULT_ETH1_GW_ADDR);
    cfg->eth1.netmask = cfgStrDup(DEFAULT_ETH1_NETMASK);
    cfg->eth1.static_ip = DEFAULT_ETH1_STATIC_IP;
    cfg->eth1.default_iface = DEFAULT_ETH1_DEFAULT_IFACE;
}

/***********************************************************************
 * Startup
 **********************************************************************/
static void execShellCmdFile(SHELL_CONTEXT *shell)
{
    FILE *f = NULL;
    char *name = NULL;
    char cmd[32];

    name = SPIFFS_VOL_NAME "shell.cmd";
    f = fopen(name, "r");
    if (f == NULL) {
        name = WRITE_ONCE_VOL_NAME "shell.cmd";
        f = fopen(name, "r");
    }
    if (f) {
        fclose(f);
        cmd[0] = '\0';
        strcat(cmd, "run "); strcat(cmd, name);
        shell_exec(shell, cmd);
    }
}

/* System startup task -> background shell task */
static portTASK_FUNCTION( startupTask, pvParameters )
{
    APP_CONTEXT *context = (APP_CONTEXT *)pvParameters;
    SPI_SIMPLE_RESULT spiResult;
    TWI_SIMPLE_RESULT twiResult;
    SPORT_SIMPLE_RESULT sportResult;
    int fsckRepaired;
    int fsckOk;
    FS_DEVMAN_DEVICE *device;
    FS_DEVMAN_RESULT fsdResult;
    s32_t spiffsResult;

    /* Initialize the CPU load module. */
    cpuLoadInit(getTimeStamp, CGU_TS_CLK);

    /* Init the SHARC Audio Engine.  This core is configured to be the
     * IPC master so this function must run to completion before any
     * other core calls sae_initialize().
     */
    sae_initialize(&context->saeContext, SAE_CORE_IDX_0, true);

    /* Register an IPC message Rx callback */
    sae_registerMsgReceivedCallback(context->saeContext, ipcMsgHandler, context);

    /* Initialize the simple SPI driver */
    spiResult = spi_init();

    /* Initialize the simple TWI driver */
    twiResult = twi_init();

    /* Initialize the simple SPORT driver */
    sportResult = sport_init();

    /* Intialize the filesystem device manager */
    fs_devman_init();

    /* Intialize the filesystem device I/O layer */
    fs_devio_init();

    /* Open up a global device handle for TWI2 @ 400KHz */
    twiResult = twi_open(TWI2, &context->twi2Handle);
    twi_setSpeed(context->twi2Handle, TWI_SIMPLE_SPEED_400);

    /* Set the adau1962, adau1979, and soft switch device handles to TWI2 */
    context->adau1962TwiHandle = context->twi2Handle;
    context->adau1979TwiHandle = context->twi2Handle;
    context->softSwitchHandle = context->twi2Handle;
    context->a2bTwiHandle = context->twi2Handle;
    context->si5356Handle = context->twi2Handle;

    /* Initialize the soft switches */
    ss_init(context);

    /* Initialize the flash */
    flash_init(context);

    /* Initialize the wo filesystem */
    romfs_init(context->flashHandle);

    /* Check the wo filesystem */
    fsckOk = romfs_fsck(1, &fsckRepaired);
    if (fsckOk != FS_OK) {
        syslog_printf("Filesystem corrupt: %s Repaired\n", fsckRepaired ? "" : "Not");
    }

    /* Hook the wo filesystem into the stdio libraries */
    device = fs_dev_romfs_device();
    fsdResult = fs_devman_register(WRITE_ONCE_VOL_NAME, device, NULL);

    /* Initialize the SPIFFS filesystem */
    context->spiffsHandle = umm_calloc(1, sizeof(*context->spiffsHandle));
    spiffsResult = spiffs_mount(context->spiffsHandle, context->flashHandle);
    if (spiffsResult == SPIFFS_OK) {
        device = fs_dev_spiffs_device();
        fsdResult = fs_devman_register(SPIFFS_VOL_NAME, device, context->spiffsHandle);
    } else {
        syslog_print("SPIFFS mount error, reformat via command line\n");
    }

    /* Set the SPIFFS as the default device */
    if (spiffsResult == SPIFFS_OK) {
        device = fs_dev_spiffs_device();
        fsdResult = fs_devman_set_default(SPIFFS_VOL_NAME);
    }

    /* Load default configuration */
    setAppDefaults(&context->cfg);

    /* Initialize the IPC audio buffers in shared L2 SAE memory */
    sae_buffer_init(context);

    /* Initialize A2B in master mode */
    context->a2bPresent = a2b_restart(context);

    /* Configure A2B interrupts */
    a2b_irq_init(context);
    a2b_pint_init(context);

    /* Disable main MCLK/BCLK */
    disable_mclk(context);

    /* Initialize main MCLK/BCLK */
    mclk_init(context);

    /* Initialize the ADAU1979 ADC */
    adau1979_init(context);

    /* Initialize the ADAU1962 DAC */
    adau1962_init(context);

    /* Initialize the SPDIF I/O */
    spdif_init(context);

    /* Initialize A2B audio in master mode */
    a2b_master_init(context);

    /* Initialize the various audio clock domains */
    clock_domain_init(context);

    /* Enable main MCLK/BCLK for a synchronous start */
    enable_mclk(context);

    /* Initialize the Ethernet related HW */
    eth_hardware_init(context);

    /* Initialize lwIP and the Ethernet interfaces */
    ethernet_init(context, &context->eth[0], &context->cfg.eth0);
    ethernet_init(context, &context->eth[1], &context->cfg.eth1);

    /* Start the SHARC cores */
    adi_core_enable(ADI_CORE_SHARC0);
    adi_core_enable(ADI_CORE_SHARC1);

    /* Wait for SHARC cores to become ready */
    while (!context->sharc0Ready || !context->sharc1Ready) {
        delay(1);
    }

    /* Initialize the audio routing table */
    audio_routing_init(context);

    /* Tell SHARC0 where to find the routing table.  Add a reference to
     * so it doesn't get destroyed upon receipt.
     */
    sae_refMsgBuffer(context->saeContext, context->routingMsgBuffer);
    ipcToCore(context->saeContext, context->routingMsgBuffer, IPC_CORE_SHARC0);

    /* Get the idle task handle */
    context->idleTaskHandle = xTaskGetIdleTaskHandle();

    /* Start A2B slave mode handling task */
    a2b_slave_init(context);

    /* Start the housekeeping tasks */
    xTaskCreate( houseKeepingTask, "HouseKeepingTask", GENERIC_TASK_STACK_SIZE,
        context, HOUSEKEEPING_PRIORITY, &context->houseKeepingTaskHandle );

    /* Lower the startup task priority for the shell */
    vTaskPrioritySet( NULL, STARTUP_TASK_LOW_PRIORITY);

    /* Initialize the shell */
    shell_init(&context->shell, term_out, term_in, SHELL_MODE_BLOCKING, NULL);

    /* Execute shell initialization command file */
    execShellCmdFile(&context->shell);

    /* Drop into the shell */
    while (1) {
        shell_start(&context->shell);
    }
}

int main(int argc, char *argv[])
{
    APP_CONTEXT *context = &mainAppContext;
    UART_SIMPLE_RESULT uartResult;

    /* Initialize system clocks */
    system_clk_init();

    /* Enable the CGU timestamp */
    cgu_ts_init();

    /* Initialize the GIC */
    gic_init();

    /* Initialize the application context */
    memset(context, 0, sizeof(*context));

    /* Initialize GPIO */
    gpio_init();

    /* Init the system heaps */
    umm_heap_init();

    /* Init the system logger */
    syslog_init();

    /* Initialize the simple UART driver */
    uartResult = uart_init();

    /* Open UART0 as the console device (115200,N,8,1) */
    uartResult = uart_open(UART0, &context->stdioHandle);
    uart_setProtocol(context->stdioHandle,
        UART_SIMPLE_BAUD_115200, UART_SIMPLE_8BIT,
        UART_SIMPLE_PARITY_DISABLE, UART_SIMPLE_STOP_BITS1
    );

    /* Initialize the UART stdio driver with the console device */
    uart_stdio_init(context->stdioHandle);

    /* Init the rest of the system and launch the remaining tasks */
    xTaskCreate( startupTask, "StartupTask", STARTUP_TASK_STACK_SIZE,
        context, STARTUP_TASK_HIGH_PRIORITY, &context->startupTaskHandle );

    /* Start the scheduler. */
    vTaskStartScheduler();

    return(0);
}

/*-----------------------------------------------------------
 * FreeRTOS critical error and debugging hooks
 *-----------------------------------------------------------*/
void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
    ( void ) pcFile;
    ( void ) ulLine;

    /* Disable interrupts so the tick interrupt stops executing, then sit in a loop
    so execution does not move past the line that failed the assertion. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    /* Run time allocation failure checking is performed if
    configUSE_MALLOC_FAILED_HOOK is defined.  This hook
    function is called if an allocation failure is detected. */
    taskDISABLE_INTERRUPTS();
    while (1);
}

/*-----------------------------------------------------------*/
