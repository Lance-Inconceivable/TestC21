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
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * NOTE 1:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky style version.
 *
 * NOTE 2:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, and functions
 * required to configure the hardware, are defined in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, and two tasks.  It then starts the
 * scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 milliseconds before sending the value 100 to the queue that
 * was created within main_blinky().  Once the value is sent, the task loops
 * back around to block for another 200 milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * main_blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, toggles the LED.  The 'block
 * time' parameter passed to the queue receive function specifies that the
 * task should be held in the Blocked state indefinitely to wait for data to
 * be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 200 milliseconds, the queue receive
 * task leaves the Blocked state every 200 milliseconds, and therefore toggles
 * the LED every 200 milliseconds.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cli.h"
#include <asf.h>
#include <string.h>

#include "can_utils.h"

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 200 / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )

/* Values passed to the two tasks just to check the task parameter
functionality. */
#define mainQUEUE_SEND_PARAMETER			( 0x1111UL )
#define mainQUEUE_RECEIVE_PARAMETER			( 0x22UL )

#define MEM_BASE 0x20000000
#define CAN_LOOPBACKx

/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );
static void can_tx_task(void *dummy);
int can_send(uint32_t id, unsigned char *data);
void debug_msg(const char *string);
void debug_msg_init(void);
void printhex(uint32_t val, int crlf);
uint16_t ulong_to_string(uint32_t number, char *ascii);
static uint32_t can_rx_count = 0;
static uint32_t can_tx_count = 0;
static int verbose_rx_dump = 1;
TaskHandle_t txTaskHandle;
TaskHandle_t rxTaskHandle = (TaskHandle_t) 0;
struct can_tx_element *pTxFIFO;
struct can_rx_element_fifo_0 *pRxFIFO;
static void can_rx_task(void *dummy);
int have_reader = 0;
SemaphoreHandle_t canRxSemaphore = (SemaphoreHandle_t) NULL;

/*
 * Called by main() to create the simply blinky style application if
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 */
void main_blinky( void );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

static struct can_module can_instance;
struct can_module *pCAN = &can_instance;

/*-----------------------------------------------------------*/

#define JIMMY_CANx
void main_blinky( void )
{
    /* Initialize "stdio" */
    debug_msg_init();

#ifdef JIMMY_CAN   /* Some old test code */

    xTaskCreate(can_tx_task,  
        "CAN_Tx", 
        configMINIMAL_STACK_SIZE *3, 
        (void *) 0,                           /* The parameter passed to the task */
        mainQUEUE_RECEIVE_TASK_PRIORITY,      /* The priority assigned to the task. */
        &txTaskHandle	                              
    );	
    
#endif
#if 0
    xTaskCreate(can_rx_task,  
        "CAN_Rx", 
        configMINIMAL_STACK_SIZE *3, 
        (void *) 0,                           /* The parameter passed to the task */
        mainQUEUE_RECEIVE_TASK_PRIORITY,      /* The priority assigned to the task. */
        &rxTaskHandle	                              
    );	
#endif

#if 0  /* JIMMY Test!!! */
    /* Create the queue. */
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

    if (xQueue != NULL) {
        /* Start the two tasks as described in the comments at the top of this
         file. */
        xTaskCreate(prvQueueReceiveTask,  /* The function that implements the task. */
           "Rx", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
            configMINIMAL_STACK_SIZE,  /* The size of the stack to allocate to the task. */
            (void *) mainQUEUE_RECEIVE_PARAMETER, /* The parameter passed to the task - just to check the functionality. */
            mainQUEUE_RECEIVE_TASK_PRIORITY,      /* The priority assigned to the task. */
            NULL	/* The task handle is not required, so NULL is passed. */
         );	/* The task handle is not required, so NULL is passed. */

         xTaskCreate(prvQueueSendTask, 
             "TX", 
              configMINIMAL_STACK_SIZE, 
              (void *) mainQUEUE_SEND_PARAMETER, 
              mainQUEUE_SEND_TASK_PRIORITY,
              NULL
         );

         /* Start the tasks and timer running. */
	 vTaskStartScheduler();
    }
#else /* JIMMY Test!!! */
	 vTaskStartScheduler();
#endif

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the idle and/or
    timer tasks	to be created.  See the memory management section on the
    FreeRTOS web site for more details. */

    for(;;)
        ;
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
    uint32_t temp;

    do_can_pinmux();

    /* Initialize the CAN driver */
    /* Note:  the default CAN config uses GCLK 8, so be sure to turn it on in conf_clocks.h */
    can_get_config_defaults(&config_can);

#define PROMISCUOUS_MODEx
#ifdef PROMISCUOUS_MODE
config_can.nonmatching_frames_action_standard = CAN_NONMATCHING_FRAMES_FIFO_0;
#endif

    can_init(pCAN, CAN0, &config_can);

    /* Save the address of the TX FIFO */
    temp = pCAN->hw->TXBC.bit.TBSA;
    pTxFIFO = (struct can_tx_element *) (temp | MEM_BASE);

    /* Save the address of the RX FIFO */
    temp = pCAN->hw->RXF0C.bit.F0SA;
    pRxFIFO = (struct can_rx_element_fifo_0 *) (temp | MEM_BASE);

    can_filters_init();
#ifdef CAN_LOOPBACK
    can_enable_test_mode(pCAN);
#endif

    if (canRxSemaphore == NULL)
        canRxSemaphore = xSemaphoreCreateBinary();

    /* Enable CAN0 interrupt in the NVIC */
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);

    /* Enable the particular interrupts we want */
    can_enable_interrupt(pCAN, CAN_TX_FIFO_EMPTY | CAN_RX_FIFO_0_NEW_MESSAGE);
    can_start(pCAN);
    debug_msg("CAN started!\r\n");
}

struct can_tx_element tx_msg;
unsigned char can_data[8];
 
static void can_tx_task(void *dummy)
{
    struct can_tx_element canmsg;
    struct can_tx_event_element txEvent;
    struct can_standard_message_filter_element filter;
    struct can_rx_element_fifo_0 rx_msg;
    CAN_HW_FILTER rx_filter;
    uint32_t i;
    static int init_once = 1;
volatile    int16_t getindex;       /* Volatiles here to stop optimizer */
volatile    int16_t putindex;   
volatile    int32_t temp;
volatile    int32_t temp1;
volatile    int32_t temp2;
    debug_msg("\r\nWelcome to the CAN task!\r\n");
    if (init_once != 1)
        goto skip_init;
    init_once--;
    do_can_pinmux();
    do_can_init();
    can_filters_init();
#ifdef CAN_LOOPBACK
    can_enable_test_mode(pCAN);
#endif

#if 0 /* BIG SKIP */
    /* Initialize a CAN msg, 11 bit ID, 8 byte data field */
    /* Also has T1:CAN_TX_ELEMENT_T1_EFC set. This is "event fifo control",
     * meaning "store an event upon TX.  Turn this off....
     */
    can_get_tx_buffer_element_defaults(&tx_msg);

#if 0
/*     tx_msg.T0.reg = CAN_TX_ELEMENT_T0_STANDARD_ID(0x422); */
/*    tx_msg.T0.reg = 0x422 << 18;  */
    tx_msg.T0.reg = CAN_TX_ELEMENT_T0_STANDARD_ID(0x422); 

    for (i = 0; i < 8; i++) {
        tx_msg.data[i] = i;
    }
    can_set_tx_buffer_element(pCAN, &tx_msg, 0);
#endif

#if 0
    /* Construct a filter matching the TX packet (for loopback test) */
    can_get_standard_message_filter_element_default(&filter);
    filter.S0.bit.SFID1 = 0x422;
    can_set_rx_standard_filter(pCAN, &filter, 0);   /* Index in filter table */
#else
    rx_filter.filter = 0x422;
    rx_filter.mask = MATCH_ALL;
    rx_filter.ext = 0;
    temp = can_filter_add(&rx_filter);
#endif

#endif /* BIG_SKIP */

    /* Enable CAN0 interrupt in the NVIC */
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_CAN0);

    /* Enable the particular interrupts we want */
    can_enable_interrupt(pCAN, CAN_TX_FIFO_EMPTY | CAN_RX_FIFO_0_NEW_MESSAGE);
    can_start(pCAN);

skip_init:

for (i = 0; i < 8; i++)
    can_data[i] = 'a' + i;
can_send(0x422, can_data);

    /* Initiate a Tx */
#if 0
    can_tx_transfer_request(pCAN, 1 << putindex);
#else
    /* can_tx_transfer_request(pCAN, 1); */
#endif

    /* Wake up when it's gone from the TX fifo */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    temp = can_tx_get_fifo_queue_status(pCAN);
    getindex = (temp >> 8) & 0x1f;
    putindex = (temp >> 16) & 0x1f;

    /* Take a look at the Tx Event Fifo */
    /* Read the TX Event Fifo Status register */
#if 1
retry:
    temp = pCAN->hw->TXEFS.reg;
    if ((temp & 0x1f) == 0)       /* Should be a fifo fill level of 1 */
       goto retry;
#else
#endif

temp1 = can_tx_count;
    /* Get the read index pointer */
    getindex = (temp >> 8) & 0x1f;
    can_get_tx_event_fifo_element(pCAN, &txEvent, getindex);
    temp1 = txEvent.E0.reg;

debug_msg("tx fifo event, msgid = ");
printhex(temp1, 1);

    temp2 = txEvent.E1.reg;
debug_msg("tx fifo event, result = ");
printhex(temp2, 1);
    can_tx_event_fifo_acknowledge(pCAN, getindex);

    /* Look at receive fifo */
    can_get_rx_fifo_0_element(pCAN, &rx_msg, 0);
debug_msg("rx msg id = ");
printhex(rx_msg.R0.reg, 1);
debug_msg("rx msg meta = ");
printhex(rx_msg.R1.reg, 1);
debug_msg("rx msg data1 = ");
printhex(*(uint32_t *) &rx_msg.data[0], 1);
debug_msg("rx msg data2 = ");
printhex(*(uint32_t *) &rx_msg.data[4], 1);
    
#if 0
for(;;);
#else
    /* Jimmy, leave CAN running for now! */
    /* can_stop(pCAN); */
#endif
}

/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
    TickType_t xNextWakeTime;
    const unsigned long ulValueToSend = 100UL;

    /* Check the task parameter is as expected. */
    configASSERT( ( ( unsigned long ) pvParameters ) == mainQUEUE_SEND_PARAMETER );

    /* Initialise xNextWakeTime - this only needs to be done once. */
    xNextWakeTime = xTaskGetTickCount();

    for( ;; ) {
        /* Place this task in the blocked state until it is time to run again.
        The block time is specified in ticks, the constant used converts ticks
        to ms.  While in the Blocked state this task will not consume any CPU
        time. */

        vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

        /* Send to the queue - causing the queue receive task to unblock and
        toggle the LED.  0 is used as the block time so the sending operation
        will not block - it shouldn't need to block as the queue should always
        be empty at this point in the code. */
        xQueueSend( xQueue, &ulValueToSend, 0U );
    }
}

/* Send a packet every second for every <count> seconds
 * or until we get a ping reply.
 * A "ping" is SID 0x422.
 * A reply is SID 0x423.
 * Note: This is a Jimmy creation.  
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

    /* Add a 423 filter for "ping reply" testing */
    rx_filter.filter = 0x423;
    rx_filter.mask = MATCH_ALL;
    rx_filter.ext = 0;
    can_filter_remove(-1, 0);  /* All, standard */
    can_filter_add(&rx_filter);
    have_reader = 1;   

    memset(x.data, 0, 8);
    x.counter = 0;
top:
    while (count--) {
        x.counter++;
        debug_msg("ping, counter = ");
        printhex(x.counter - 1, 1);
        rval = can_send(0x422, x.data);
        /* Read reply.  If nothing after 1 second, continue */
        index = can_msg_get(100);
        if (index < 0) 
            continue;
        else
            break;
    }
    if (index >= 0) {
        struct can_rx_element_fifo_0 *pMsg;
        int *pData;
        pMsg = &pRxFIFO[index];
        pData = (int *) &pMsg->data[0];
        debug_msg("ping reply, val = ");
        printhex(*pData, 1);
        can_msg_free(index);
        if (count)
           goto top;
    }
    else
        debug_msg("No reply\r\n");
}

/* This is a CAN RX task
 * For now it simply reads CAN messages and throws them away.
 * In a real application, it should perform some action based
 * on the ID of the CAN message.  
 */
static void can_rx_task(void *dummy)
{
    int index;
    int i;
    unsigned int id;
    struct can_rx_element_fifo_0 *pMsg;
    CAN_HW_FILTER rx_filter;
    /* Add a 422 filter for testing */
    rx_filter.filter = 0x422;
    rx_filter.mask = MATCH_ALL;
    rx_filter.ext = 0;
    can_filter_add(&rx_filter);
    have_reader = 1;
    while (1) {
        /* Get a packet */
        index = can_msg_get(portMAX_DELAY);
        if (index == -1) {
            debug_msg("timeout or error in RX FIFO\r\n");
            continue;
        }

        pMsg = &pRxFIFO[index];
        id = pMsg->R0.bit.ID;
        /* Right justify if not an extended ID */
        if (!pMsg->R0.bit.XTD)
            id >>= 18;

        /* Handle the message */
        switch (id) {
            case 0x422:
                debug_msg("My favorite packet!\r\n");
                /* Send 0x423 in response */
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
                    printhex(pMsg->data[7], 1);
                }
        }
        can_msg_free(index);
    }
}

/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
    unsigned long ulReceivedValue;

    /* Check the task parameter is as expected. */
    configASSERT( ( ( unsigned long ) pvParameters ) == mainQUEUE_RECEIVE_PARAMETER );

    for(;;) {
        /* Wait until something arrives in the queue - this task will block
        indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
        */

        xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

        /*  To get here something must have been received from the queue, but
        is it the expected value?  If it is, toggle the LED. */
        if(ulReceivedValue == 100UL) {
            /* Toggle the LED. */
            port_pin_toggle_output_level( LED_0_PIN );
            ulReceivedValue = 0U;
	}
    }
}
/*-----------------------------------------------------------*/

/* CAN Interrupts */

void CAN0_Handler(void)
{
    uint32_t status = can_read_interrupt_status(pCAN);
    uint32_t rx_fifo_status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   
    if (status & CAN_RX_FIFO_0_NEW_MESSAGE) {
        can_clear_interrupt_status(pCAN, CAN_RX_FIFO_0_NEW_MESSAGE);
        can_rx_count++;
        rx_fifo_status = can_rx_get_fifo_status(pCAN, 0) & 0x7f;
       
        /* Set the RX semaphore if this if the first msg in the FIFO */
        if (rx_fifo_status == 1 && have_reader)
            xSemaphoreGiveFromISR(canRxSemaphore, &xHigherPriorityTaskWoken);
    }
    if (status & CAN_TX_FIFO_EMPTY) {
        can_clear_interrupt_status(pCAN, CAN_TX_FIFO_EMPTY);
        can_tx_count++;
        /* This may be used when there's a can Tx task.  */
        vTaskNotifyGiveFromISR(txTaskHandle, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Add a message to the TX FIFO.  */
int can_send(uint32_t id, unsigned char *data)
{
    uint32_t val;
    struct can_tx_element *msg;
    uint16_t putindex;
    uint16_t i;
    int status;
    uint16_t count = 5;   /* Was 1 */

    /* Make sure we're initialized */
    if (pCAN->hw->CCCR.reg & CAN_CCCR_CCE) 
        return (-2);

    /* If the number of available FIFO entries is 0, 
     * wait for 1 tick and try again.
     */
retry:
    val = can_tx_get_fifo_queue_status(pCAN);
    if ((val & 0x1f) == 0) {   /* FIFO full */
        if (count--) {
#if 0
debug_msg("Wait for FIFO!\r\n");
#endif
            vTaskDelay(1);    /* Delay 10 msec */
            goto retry;
        }
        else
            return (-1);
    }

    putindex = (val >> 16) & 0x1f;
    msg = &pTxFIFO[putindex];
    can_get_tx_buffer_element_defaults(msg);
    msg->T0.reg = CAN_TX_ELEMENT_T0_STANDARD_ID(id); 
    for (i = 0; i < 8; i++) {
        msg->data[i] = data[i];
    }
#if 0
debug_msg("putindex = ");
printhex(putindex, 1);
#endif
    can_set_tx_buffer_element(pCAN, msg, putindex);
    status = can_tx_transfer_request(pCAN, 1 << putindex); 
    return (-status);   /* Status codes are positive in ASF */
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
    if (add_filter) {
        add_filter--;
        rx_filter.filter = 0x422;
        rx_filter.mask = MATCH_ALL;
        rx_filter.ext = 0;
        rval = can_filter_add(&rx_filter);
    }
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
        printhex(can_rx_count, 1);
#endif
#if 0
        vTaskDelay(1);    /* Delay 10 msec */
#endif
    }
    debug_msg("Packets sent = ");
    printhex(count, 1);
    if (rval) {
        debug_msg("\tError = ");
        printhex(-rval, 1);
        debug_msg("\tTX FIFO status = ");
        printhex(can_tx_get_fifo_queue_status(pCAN), 1);
        debug_msg("\tController status = ");
        printhex(pCAN->hw->CCCR.reg, 1);
        debug_msg("\tError counter = ");
        printhex(pCAN->hw->ECR.reg, 1);
        debug_msg("\tInterrupt flags = ");
        printhex(pCAN->hw->IR.reg, 1);
    }
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
}

void do_baud(int baud)
{
    if (baud != 250 && baud != 500 && baud != 1000) {
        debug_msg("Error: Invalid baud rate\r\n");
        return;
    }
    can_stop(pCAN);
    can_set_baudrate(pCAN->hw, baud);
    can_start(pCAN);
}

void do_start_reader(void)
{
    if ((uint32_t) rxTaskHandle == 0)
        xTaskCreate(can_rx_task,  
            "CAN_Rx", 
            configMINIMAL_STACK_SIZE *3, 
            (void *) 0,                           /* The parameter passed to the task */
            mainQUEUE_RECEIVE_TASK_PRIORITY,      /* The priority assigned to the task. */
            &rxTaskHandle	                              
        );	
    else {
        /* Turn off the standard ID rejection filter */
        can_filter_switch(1, 0, 0);
        have_reader = 1;   /* Enable ISRs wake-up of the reader task */
    }
}

void do_xreader(void)
{
    have_reader = 0;
    can_filter_switch(0, 0, 0);   /* Disable rejection filter for SIDs */
}

int dispatch_cmd(char *cmd)
{
    int rval;
    uint16_t param;
    uint16_t param2;

#if 0
    if (strcmp(cmd, "go") == 0)
       can_tx_task ((void *) 0);
    return (1);
#endif

    /* "cmd" is the full command string.  Parse the string, returning the
     * command code in 'rval', and the numeric args in 'param' and 'param2'
     */
    rval = get_command(cmd, &param, &param2);
    switch (rval) {
        case CMD_HELP:
            do_help(param); 
            break;
        case CMD_CAN:
#if 0
            can_tx_task ((void *) 0);
#else
            do_can_init();
#endif
            break;
        case CMD_READER:
            do_start_reader();
            break;
        case CMD_XREADER:
            do_xreader();
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
        case CMD_BAUD:
            do_baud(param);
            break;
        case CMD_LED:
            break;
        case CMD_XLED:
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
