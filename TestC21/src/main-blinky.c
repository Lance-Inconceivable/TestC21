/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 */


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "cli.h"
#include <asf.h>
#include <string.h>
#include "UARTCommandConsole.h"
#include "can_utils.h"
#include "adctest.h"

/*-----------------------------------------------------------*/

extern uint32_t can_rx_count;
static int verbose_rx_dump = 1;
TaskHandle_t txTaskHandle;
TaskHandle_t rxTaskHandle = (TaskHandle_t) 0;
extern struct can_rx_element_fifo_0 *pRxFIFO;
static void can_rx_task(void *dummy);
extern int gHaveReader;
unsigned char can_data[8];
int dispatch_cmd(char *cmd);

/* Forward declare some functions to quell compiler warnings */
static int do_detect(void);
static void do_start_reader(void);
static int do_send_loop(uint32_t n);
static int do_help(uint32_t cmd);
static int do_baud(int baud);

static gCanInit = 0;

static uint8_t gGenSelect = 0;

/*
 * Called by main() to create the simply blinky style application if
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 */
void main_blinky( void );

/*-----------------------------------------------------------*/

static struct can_module can_instance;
struct can_module *pCAN = &can_instance;

/*-----------------------------------------------------------*/

void main_blinky( void )
{
    vUARTCommandConsoleStart(configMINIMAL_STACK_SIZE * 4, tskIDLE_PRIORITY);
    /* Initialize "stdio" */
    debug_msg_init();
    port_pin_set_output_level(RED_LED_PIN, false);
    port_pin_set_output_level(GREEN_LED_PIN, false);

    microtimer_init();

    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks	to be created.  See the memory management section on the
    FreeRTOS web site for more details. */

    for(;;)
        ;
}

static void can_transceiver_enable(void)
{
    port_pin_set_output_level(CAN_STANDBY_PIN, false);
    port_pin_set_output_level(CAN_SILENTMODE_PIN, true);
}

static void do_can_pinmux(void)
{
    struct system_pinmux_config pin_config;
    /* Set up I/O pins */
    system_pinmux_get_config_defaults(&pin_config);
    pin_config.mux_position = CAN_TX_MUX_SETTING;
    system_pinmux_pin_set_config(CAN_TX_PIN, &pin_config);
    pin_config.mux_position = CAN_RX_MUX_SETTING;
    system_pinmux_pin_set_config(CAN_RX_PIN, &pin_config);
}

static void do_can_init(void)
{
    struct can_config config_can;

    do_can_pinmux();
    can_transceiver_enable();

    /* Initialize the CAN driver */
    /* Note:  the default CAN config uses GCLK 8, so be sure to turn it on in conf_clocks.h */
    can_get_config_defaults(&config_can);

#ifdef PROMISCUOUS_MODE
    config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
    config_can.nonmatching_frames_action_extended = CAN_NONMATCHING_FRAMES_FIFO_0;
#endif

    can_init(pCAN, CAN0, &config_can);

    can_utils_init();

#ifdef CAN_LOOPBACK
    can_enable_test_mode(pCAN);
#endif

    /* Enable CAN0 interrupt in the NVIC */
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);

    /* Enable the particular interrupts we want */
#if 0
    can_enable_interrupt(pCAN, CAN_TX_FIFO_EMPTY | CAN_RX_FIFO_0_NEW_MESSAGE);
#else
    /* Only activate RX interrupt for now.  We'll do TX asynchronously */
    can_enable_interrupt(pCAN, CAN_RX_FIFO_0_NEW_MESSAGE);
#endif
    can_start(pCAN);
    debug_msg("CAN started!\r\n");
    gCanInit = 1;
}



/* Send <count> 'ping' packets.
 * If we get a ping replay, immediately send the next packet, otherwise wait 1 sec.
 * A "ping" is SID 0x422.
 * A reply is SID 0x423.
 * Note: This is not a standard ping, just a testing construct.
 */
static void do_can_ping(int count)
{
    int rval;
    union _data {
        unsigned char data[8];
        uint32_t counter;
    } x;
    CAN_HW_FILTER rx_filter;
    int index;

    if (gCanInit == 0)
        do_can_init();

    /* Add a 423 filter for "ping reply" testing */
    rx_filter.filter = 0x423;
    rx_filter.mask = MATCH_ALL;
    rx_filter.ext = 0;
    can_filter_remove(-1, 0);  /* All, standard */
    can_filter_add(&rx_filter);
    gHaveReader = 1;   

    memset(x.data, 0, 8);
    /* Put counter in payload, little endian.  Use union trick */
    x.counter = 1;
top:
    while (count--) {
        debug_msg("ping, counter = ");
        printhex(x.counter - 1, CRLF);
        rval = can_send(0x422, x.data);
        if (rval)
            debug_msg("Can TX error\r\n");
        /* Read reply.  If nothing after 1 second, continue */
        index = can_msg_get(100);
        x.counter++;
        if (index < 0) 
            continue;
        else
            break;
    }
    if (index >= 0) {
        struct can_rx_element_fifo_0 *pMsg = pRxFIFO + index;
        uint32_t *pData = (uint32_t *) pMsg->data;
        debug_msg("ping reply, val = ");
        printhex(*pData, CRLF);
        can_msg_free(index);
        if (count)    /* delete this block if you want ping to stop upon RX */
           goto top;
    }
    else
        debug_msg("No reply\r\n");
}

/* This is a CAN RX task
 * If it receives 0x422, it responds with 0x423.
 * Other packets are echoed.
 * Works for STD and EXT. Assumes payload is 8 bytes.
 */
static void can_rx_task(void *dummy)
{
    int index;
    int i;
    unsigned int id;
    struct can_rx_element_fifo_0 *pMsg;
    CAN_HW_FILTER rx_filter;

    if (gCanInit == 0)
        do_can_init();

    /* Add a 422 filter for testing */
    rx_filter.filter = 0x422;
    rx_filter.mask = MATCH_ALL;
    rx_filter.ext = 0;
    can_filter_add(&rx_filter);
    gHaveReader = 1;
    debug_msg("Reader task started\r\n");
    while (1) {
        /* Get a packet */
        index = can_msg_get(portMAX_DELAY);
        if (index == -1) {
            debug_msg("timeout or error in RX FIFO\r\n");
            continue;
        }

        pMsg = pRxFIFO + index;
        id = pMsg->R0.bit.ID;
        /* Right justify if not an extended ID */
        if (!pMsg->R0.bit.XTD)
            id >>= 18;

        /* Handle the message */
        switch (id) {
            case 0x422:
                debug_msg("RX 0x422\r\n");
                /* Send 0x423 in response */
                /* Could re-use received payload, but doing memcpy */
                memcpy(can_data, &pMsg->data, 8);
                can_send(0x423, can_data);
                break;
            default:
                debug_msg("RX! ID = ");
                printhex(id, 0);
                if (verbose_rx_dump) {
                    debug_msg(" : ");
                    for (i = 0; i < 7; i++) {
                        printhex(pMsg->data[i], 0);
                        debug_msg(" ");
                    }
                    printhex(pMsg->data[7], CRLF);
                }
        }
        can_msg_free(index);
    }
}

/*
 * Detect CAN bus baud rate. 
 * Only trying 250K and 500K.  
 */
int do_detect(void)
{
    int rval = 0;
    int baud;
    int index;
    int i;
    int ntries = 1;
    struct can_rx_element_fifo_0 *pMsg;
    uint32_t regval;
    int trybaud[] = {250, 500, 0};

    if (gCanInit == 0)
        do_can_init();

    /* Save the global filter register */
    regval = pCAN->hw->GFC.reg;

    /* Stop CAN and put it in monitor mode */
    can_enable_bus_monitor_mode(pCAN);
    gHaveReader = 1;
    pCAN->hw->GFC.reg = 0x3; /* promiscuous mode for std and ext */
    while (ntries--) {
        i = 0;
        baud = trybaud[i++];
        while(baud) {
            can_set_baudrate(pCAN->hw, baud * 1000);
            can_start(pCAN);
            index = can_msg_get(500);   /* Wait 5 seconds for a msg */

            /* Stop the CAN controller and put it in config mode */
            can_stop(pCAN);
            pCAN->hw->CCCR.reg |= CAN_CCCR_CCE; 

            if (index >= 0) 
                goto found;        /* Got a msg! */

            baud = trybaud[i++];
        }
    }

    /* If we get here, no bus was found */
    rval = -1;
    debug_msg("No CAN messages detected\r\n");

found:
    /* Read the packet in msg memory */
    if (rval == 0) {
        pMsg = pRxFIFO + index;
        if (pMsg->R0.bit.XTD) {
            debug_msg("Got ext CAN msg, XID = ");
            printhex(pMsg->R0.bit.ID, CRLF);
        }
        else {
            debug_msg("Got std CAN msg, SID = ");
            printhex((pMsg->R0.bit.ID >> 18), CRLF);
        }
        if (baud == 250)
            debug_msg("Baud = 250\r\n");
        else if (baud == 500)
           debug_msg("Baud = 500\r\n");
    }

    can_disable_bus_monitor_mode(pCAN);
    pCAN->hw->GFC.reg = regval; /* restore original global filter */
    gHaveReader = 0;
    can_start(pCAN);

    return (rval);
}


/*
 *  Send 'n' CAN messages
 */
int do_send_loop(uint32_t n)
{
    int rval;
    uint32_t index;
    uint32_t count = 0;
#ifdef CAN_LOOPBACK
    CAN_HW_FILTER rx_filter;
    static char add_filter = 1;
    if (gCanInit == 0)
        do_can_init();
    if (add_filter) {
        add_filter--;
        rx_filter.filter = 0x422;
        rx_filter.mask = MATCH_ALL;
        rx_filter.ext = 0;
        rval = can_filter_add(&rx_filter);
    }
#else
    if (gCanInit == 0)
        do_can_init();
#endif
    while (n--) {
        memset(can_data, 0, 8);
        index = n >> 8;
        if (index > 7)
            index = 7;
        can_data[index] = n & 0xff;
        rval = can_send(0x422, can_data);
        if (rval == 0)
            count++;
        else
            break;

#ifdef CAN_LOOPBACK
        debug_msg("rx count = ");
        printhex(can_rx_count, CRLF);
#endif
#if 0
        vTaskDelay(1);    /* Delay 10 msec */
#endif
    }
    debug_msg("Packets sent = ");
    printhex(count, CRLF);

    /* If there was an error, dump some registers of interest */
    if (rval) {
        debug_msg("\tError = ");
        printhex(-rval, CRLF);
        debug_msg("\tTX FIFO status = ");
        printhex(can_tx_get_fifo_queue_status(pCAN), 1);
        debug_msg("\tController status = ");
        printhex(pCAN->hw->CCCR.reg, CRLF);
        debug_msg("\tError counter = ");
        printhex(pCAN->hw->ECR.reg, CRLF);
        debug_msg("\tInterrupt flags = ");
        printhex(pCAN->hw->IR.reg, CRLF);
    }
    return (rval);
}

static
void dump_fuses(void)
{
    volatile uint32_t *p = (uint32_t *) 0x00804000;
    debug_msg("fuses : ");
    printhex(*p++, 0);
    printhex(*p++, CRLF);
}

static
int do_nvm_init(int manual)
{
    struct nvm_config nvm;
    struct nvm_parameters params;

    /* Initialize the NVM controller */
    nvm_get_config_defaults(&nvm);
    if (manual)
        nvm.manual_page_write = true;
    else
        nvm.manual_page_write = false;
    nvm_set_config(&nvm);

    /* Dump some NVM params */
    nvm_get_parameters(&params);
    debug_msg("Number of pages = ");
    printhex(params.rww_eeprom_number_of_pages, CRLF);
    debug_msg("Page size = ");
    printhex(params.page_size, CRLF);

    dump_fuses();

    debug_msg("RWWEE first word : ");
    printhex(* (uint32_t *) NVMCTRL_RWW_EEPROM_ADDR, CRLF);
    return (0);
}

static 
int do_led(int num, int on)
{
    int pin;
    if (num > LED_COUNT || num < 1) {
        debug_msg("LED number out of range\r\n");
        return (-1);
    }
    if (num == 1)
        pin = GREEN_LED_PIN;
    else
        pin = RED_LED_PIN;

    port_pin_set_output_level(pin, on);
    return (0);
}

static
void do_eep(void)
{
     char buffer[65];
     uint16_t *p = (uint16_t *) (NVMCTRL_RWW_EEPROM_ADDR + 256); 
     int i;
     nvm_erase_row((uint32_t) p);
     strcpy(buffer, "deadbeefbabebabedeadbeefbabebabedeadbeefbabebabedeadbeefbabebabe");
     nvm_write_buffer((uint32_t) p, buffer, 64);
     nvm_write_buffer(((uint32_t) p) + 64, buffer, 64);
     for (i = 0; i < 40; i++)
        printhex(*p++, 1);
}

int do_help(uint32_t cmd)
{
    int i;
    if (cmd) {
        debug_msg(get_help(cmd));
        debug_msg(get_help_verbose(cmd));
    }
    else {
        for (i = 0; i < NUM_COMMANDS; i++)
            debug_msg(get_help(i + 1));
    }
    return (0);
}

static void sr_strobe()
{
    port_pin_set_output_level(SR_CLK_PIN, 1);
    port_pin_set_output_level(SR_CLK_PIN, 0);
}

int do_shift(unsigned int val)
{
    uint8_t mask = 0x80;
    uint8_t x = val;
    uint8_t i;

    if (val > 0xff) {
        debug_msg("Error: not an 8-bit value\r\n");
        return (-1);
    }
    
    /* Clear the register */
    port_pin_set_output_level(SR_CLEAR_PIN, 0);
    sr_strobe();
    port_pin_set_output_level(SR_CLEAR_PIN, 1);

    for (i = 0; i < 8; i++) {
        port_pin_set_output_level(SR_DATA_PIN, (x & mask) ? 1 : 0);
        sr_strobe();
        mask >>= 1;
    }

    port_pin_set_output_level(SR_LATCH_PIN, 1);
    sr_strobe();
    port_pin_set_output_level(SR_LATCH_PIN, 0);
}

do_gen_select(int ain, int r)
{
    uint8_t bit;
    uint8_t leftshift;
    uint8_t mask;
    unsigned int arg;

    if (ain < 0 || ain > 3) {
        debug_msg("Invalid analog input number\r\n");
        return (-1);
    }
    
    if (r != 2 && r != 10) {
        debug_msg("Invalid resistance value\r\n");
        return (-1);
    }

    if (r == 10)
       bit = 1;
    else
       bit = 2;

    leftshift = 2 * ain;
    mask = 3 << leftshift;
    gGenSelect &= ~mask;
    gGenSelect |= (bit << leftshift);
    arg = ~gGenSelect;
    arg &= 0xff;
    do_shift(arg);
}

do_xgen(int ain)
{
    uint8_t bit;
    uint8_t leftshift;
    uint8_t mask;
    unsigned int arg;

    if (ain < 0 || ain > 3) {
        debug_msg("Invalid analog input number\r\n");
        return (-1);
    }

    leftshift = 2 * ain;
    mask = 3 << leftshift;
    gGenSelect &= ~mask;
    arg = ~gGenSelect;
    arg &= 0xff;
    do_shift(arg);
}

static
void do_adctest(void)
{
   int i;
   uint32_t timer;
   configure_adc();
   configure_adc_callbacks();
microtimer_start();
for (i = 0; i < 100; i++) {
   adc_run();
   adc_wait();
}

timer = microtimer_stop();
debug_msg("ADC microseconds =");
printhex(microtimer_convert(timer), CRLF);

   /* When we get here, the result_buffer should contain
    * 128 16-bit samples
    * Print a few of them.
    */
   debug_msg("ADC results\r\n");
   for (i = 0; i < 10; i++)
       printhex(adc_result_buffer[i], CRLF);
}

static
void do_sdtest(void)
{
   int i;
   uint32_t timer;
   configure_sdadc();
   configure_sdadc_callbacks();
microtimer_start();
#if 0
for (i = 0; i < 100; i++) {
#endif
   sdadc_run();
   sdadc_wait();
#if 0
}
#endif

timer = microtimer_stop();
debug_msg("ADC microseconds (hex) =");
printhex(microtimer_convert(timer), CRLF);
#if 0
microtimer_start();
vTaskDelay(100);
timer = microtimer_stop();
debug_msg("check timer 100 ticks in microseconds = ");
printhex(microtimer_convert(timer), CRLF);
#endif

   /* When we get here, the result_buffer should contain
    * 128 16-bit samples
    * Print a few of them.
    */
   debug_msg("SDADC results\r\n");
   for (i = 0; i < 10; i++)
       printhex(sdadc_result_buffer[i], CRLF);
}

int do_baud(int baud)
{
    if (baud != 250 && baud != 500 && baud != 1000) {
        debug_msg("Error: Invalid baud rate\r\n");
        return (-1);
    }
    can_stop(pCAN);
    pCAN->hw->CCCR.reg |= CAN_CCCR_CCE;    /* Enable config register write */
    can_set_baudrate(pCAN->hw, baud);
    can_start(pCAN);
    return (0);
}

/* 
 * Starts 'can_rx_task'.  
 */
void do_start_reader(void)
{
    xTaskCreate(can_rx_task,  
        "CAN_Rx", 
        configMINIMAL_STACK_SIZE *3, 
        (void *) 0,                           /* The parameter passed to the task */
        tskIDLE_PRIORITY + 1,                 /* The priority assigned to the task. */
        &rxTaskHandle	                      /* Not used.  Just illustrates creating sync handle */
    );	
}

int dispatch_cmd(char *cmd)
{
    int rval;
    uint32_t param;
    uint32_t param2;

    /* "cmd" is the full command string.  Parse the string, returning the
     * command code in 'rval', and the numeric args in 'param' and 'param2'
     */
    rval = get_command(cmd, &param, &param2);
    switch (rval) {
        case CMD_HELP:
            do_help(param); 
            break;
        case CMD_CAN:
            do_can_init();
            break;
        case CMD_READER:
            do_start_reader();
            break;
        case CMD_SEND:
            do_send_loop(1);
            break;
        case CMD_LOOP:
            do_send_loop(param);
            break;
        case CMD_PING:
            do_can_ping(param);
            break;
        case CMD_DETECT:
            do_detect();
            break;
        case CMD_BAUD:
            do_baud(param);
            break;
        case CMD_NVM:
            do_nvm_init(1);
            break;
        case CMD_EEP:
            do_nvm_init(1);
            do_eep();
            break;
        case CMD_LED:
            do_led(param, 1);
            break;
        case CMD_XLED:
            do_led(param, 0);
            break;
        case CMD_SHIFT:
            do_shift(param);
            break;
        case CMD_GEN:
            do_gen_select(param, param2);
            break;
        case CMD_XGEN:
            do_xgen(param);
            break;
        case CMD_ADCTEST:
            do_adctest();
            break;
        case CMD_SDTEST:
            do_sdtest();
            break;
        default:
            rval = -1;
            do_help(0);
            break;
    }
    if (rval > 0)
        return (0);
    else
        return (rval);
}
