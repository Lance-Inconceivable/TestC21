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

/* Standard includes. */
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Library includes. */
#include "asf.h"

/* Example includes. */
#include "UARTCommandConsole.h"

/* Dimensions the buffer into which input characters are placed. */
#define cmdMAX_INPUT_SIZE	50

/* The maximum time in ticks to wait for the UART access mutex. */
#define cmdMAX_MUTEX_WAIT    ( 200 / portTICK_PERIOD_MS )

/* Characters are only ever received slowly on the CLI so it is ok to pass
received characters from the UART interrupt to the task on a queue.  This sets
the length of the queue used for that purpose. */
#define cmdRXED_CHARS_QUEUE_LENGTH	( 10 )

/* DEL acts as a backspace. */
#define cmdASCII_DEL		( 0x7F )

extern int dispatch_cmd(char *cmd);

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void prvUARTCommandConsoleTask( void *pvParameters );

/*
 * Ensure a previous interrupt driven Tx has completed before sending the next
 * data block to the UART.
 */
static void prvSendBuffer(struct usart_module *pxCDCUsart, const char * pcBuffer, size_t xBufferLength);

/*
 * Configure the UART used for IO.and register prvUARTRxNotificationHandler()
 * to handle UART Rx events.
 */
static void prvConfigureUART( struct usart_module *pxCDCUsart );

/*
 * Callback functions registered with the Atmel UART driver.  Both functions
 * just 'give' a semaphore to unblock a task that may be waiting for a
 * character to be received, or a transmission to complete.
 */
static void prvUARTTxNotificationHandler( const struct usart_module *const pxUSART );
static void prvUARTRxNotificationHandler( const struct usart_module *const pxUSART );

/*-----------------------------------------------------------*/

/* Const messages output by the command console. */
static char * const pcWelcomeMessage = "\r\n\r\nFreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n\r\n>";
static const char * const pcEndOfOutputMessage = "\r\n>";
static const char * const pcNewLine = "\r\n";

/* This semaphore is used to allow the task to wait for a Tx to complete
without wasting any CPU time. */
static SemaphoreHandle_t xTxCompleteSemaphore = NULL;

/* This semaphore is sued to allow the task to wait for an Rx to complete
without wasting any CPU time. */
static SemaphoreHandle_t xRxCompleteSemaphore = NULL;

/*-----------------------------------------------------------*/

void vUARTCommandConsoleStart( uint16_t usStackSize, unsigned portBASE_TYPE uxPriority )
{
    /* Create that task that handles the console itself. */
    xTaskCreate(prvUARTCommandConsoleTask, /* Task entry point. */
        "CLI",	                           /* Task name */
        usStackSize,                       /* Stack size */
        NULL,	                           /* Task parameter, unused. */
        uxPriority,	                   /* Task priority. */
        NULL                               /* Task Handle unused */
    );
}
/*-----------------------------------------------------------*/

static struct usart_module xCDCUsart; /* Static so it doesn't take up too much stack. */

static char outbuf[20];

uint16_t ulong_to_string(uint32_t number, char *ascii)
{
    uint32_t remainder = 0;
    uint8_t i = 0;
    char *asciiptr = ascii;
    char strptr[12];
    int16_t rval;

    if (number == 0) {
        *ascii++ = '0';
        *ascii = '\0';
        return (1);    /* string length 1 */
    }

    while (number != 0) {
        remainder = number & 0x0000000f;                /* mod 16 */
        if ( remainder <= 9 )
            strptr[i++] = (char)(remainder + 0x30);	// 0-9
        else
            strptr[i++] = (char)(remainder + 0x37);	// A-F
        number >>= 4;                                   /* divide by 16 */
    }

    rval = i;
    while (i > 0) {
        *asciiptr++ = strptr[--i];
    }
    *asciiptr = 0;	// terminate string
    return (rval);
}

void debug_msg_init(void)
{
    prvConfigureUART(&xCDCUsart);
}

void debug_msg(const char *string)
{
    prvSendBuffer(&xCDCUsart, string, strlen(string));
}

void printhex(uint32_t val, int crlf)
{
    int len = ulong_to_string(val, outbuf);
    if (crlf) {
        outbuf[len++] = '\r';
        outbuf[len++] = '\n';
    }
    prvSendBuffer(&xCDCUsart, outbuf, len);
}

static void prvUARTCommandConsoleTask( void *pvParameters )
{
    char cRxedChar;
    uint8_t ucInputIndex = 0;
    static char cInputString[cmdMAX_INPUT_SIZE], cLastInputString[cmdMAX_INPUT_SIZE];
    int rval = 1;

    ( void ) pvParameters;

    /* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */

    /* Send the welcome message. */
    prvSendBuffer(&xCDCUsart, pcWelcomeMessage, strlen(pcWelcomeMessage));

    for ( ;; ) {
        /* Wait for the next character to arrive.  A semaphore is used to
        ensure no CPU time is used until data has arrived. */

        usart_read_buffer_job(&xCDCUsart, (uint8_t *) &cRxedChar, 
            sizeof(cRxedChar));

        if (xSemaphoreTake(xRxCompleteSemaphore, portMAX_DELAY) == pdPASS) {
            /* Echo the character back. */

            prvSendBuffer( &xCDCUsart, &cRxedChar, sizeof( cRxedChar ) );

            /* Was it the end of the line? */
            if (cRxedChar == '\n' || cRxedChar == '\r') {
                /* Just to space the output from the input. */
                prvSendBuffer(&xCDCUsart, pcNewLine, strlen(pcNewLine));

                /* See if the command is empty, indicating that the last 
                    command is to be executed again. */

#if 0 /* Jimmy: disable command repeat */
                if (ucInputIndex == 0) {
                   /* Copy the last command back into the input string. */
                   if (rval == 0)
                       strcpy(cInputString, cLastInputString);
                }
#endif

                /* Pass the received command to the command interpreter.  The
                    command interpreter is called repeatedly until it returns pdFALSE
                    (indicating there is no more output) as it might generate more than
                    one string. */

                /* Jimmy's dispatcher */

                rval = dispatch_cmd(cInputString);

                /* All the strings generated by the input command have been sent.
                    Clear the input string ready to receive the next command.  Remember
                    the command that was just processed first in case it is to be
                    processed again. */

                if (rval == 0)
                    strcpy(cLastInputString, cInputString);
                ucInputIndex = 0;
                memset(cInputString, 0x00, cmdMAX_INPUT_SIZE);

                prvSendBuffer(&xCDCUsart, pcEndOfOutputMessage, 
                    strlen(pcEndOfOutputMessage));
             }
             else {
                 if (cRxedChar == '\r') {
                     /* Ignore the character. */
                 }
                 else if ((cRxedChar == '\b') || 
                     (cRxedChar == cmdASCII_DEL)) {
                     /* Backspace was pressed.  Erase the last character in the
                         string - if any. */

                     if (ucInputIndex > 0) {
                         ucInputIndex--;
                         cInputString[ucInputIndex] = '\0';
                     }
                 }
                 else {
                     /* A character was entered.  Add it to the string
                         entered so far.  When a \n is entered the complete
                         string will be passed to the command interpreter. */

                     if ((cRxedChar >= ' ') && (cRxedChar <= '~')) {
                         if (ucInputIndex < cmdMAX_INPUT_SIZE) {
                             cInputString[ucInputIndex] = cRxedChar;
                             ucInputIndex++;
                         }
                     }
                }
            }
        }
    }
}
/*-----------------------------------------------------------*/

static void prvSendBuffer( struct usart_module *pxCDCUsart, const char * pcBuffer, size_t xBufferLength )
{
    const TickType_t xBlockMax100ms = 100UL / portTICK_PERIOD_MS;

    if (xBufferLength > 0) {
        usart_write_buffer_job( pxCDCUsart, ( uint8_t * ) pcBuffer, xBufferLength );

        /* Wait for the Tx to complete so the buffer can be reused without
            corrupting the data that is being sent. */

        xSemaphoreTake( xTxCompleteSemaphore, xBlockMax100ms );
    }
}
/*-----------------------------------------------------------*/

static void prvConfigureUART( struct usart_module *pxCDCUsart )
{
struct usart_config xUARTConfig;

	/* This semaphore is used to allow the task to wait for the Tx to complete
	without wasting any CPU time. */
	vSemaphoreCreateBinary( xTxCompleteSemaphore );
	configASSERT( xTxCompleteSemaphore );

	/* This semaphore is used to allow the task to block for an Rx to complete
	without wasting any CPU time. */
	vSemaphoreCreateBinary( xRxCompleteSemaphore );
	configASSERT( xRxCompleteSemaphore );

	/* Take the semaphores so they start in the wanted state.  A block time is
	not necessary, and is therefore set to 0, as it is known that the semaphores
	exists - they have just been created. */
	xSemaphoreTake( xTxCompleteSemaphore, 0 );
	xSemaphoreTake( xRxCompleteSemaphore, 0 );

	/* Configure the hardware. */
	usart_get_config_defaults( &xUARTConfig );
	xUARTConfig.baudrate    = 115200;
	xUARTConfig.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	xUARTConfig.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	xUARTConfig.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	xUARTConfig.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	xUARTConfig.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	while(usart_init(pxCDCUsart, EDBG_CDC_MODULE, &xUARTConfig) != STATUS_OK) {
            /* Nothing to do here.  Should include a timeout really but this is
	    init code only. */
	}
	usart_enable( pxCDCUsart );

	/* Register the driver callbacks. */

	usart_register_callback( pxCDCUsart, prvUARTTxNotificationHandler, USART_CALLBACK_BUFFER_TRANSMITTED );
	usart_register_callback( pxCDCUsart, prvUARTRxNotificationHandler, USART_CALLBACK_BUFFER_RECEIVED );
	usart_enable_callback( pxCDCUsart, USART_CALLBACK_BUFFER_TRANSMITTED );
	usart_enable_callback( pxCDCUsart, USART_CALLBACK_BUFFER_RECEIVED );
}
/*-----------------------------------------------------------*/

static void prvUARTRxNotificationHandler( const struct usart_module *const pxUSART )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Remove compiler warnings. */
    (void) pxUSART;

    /* Give the semaphore  to unblock any tasks that might be waiting for an Rx
        to complete.  If a task is unblocked, and the unblocked task has a priority
        above the currently running task, then xHigherPriorityTaskWoken will be set
        to pdTRUE inside the xSemaphoreGiveFromISR() function. */

    xSemaphoreGiveFromISR(xRxCompleteSemaphore, &xHigherPriorityTaskWoken);

    /* portEND_SWITCHING_ISR() or portYIELD_FROM_ISR() can be used here. */

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/*-----------------------------------------------------------*/

static void prvUARTTxNotificationHandler( const struct usart_module *const pxUSART )
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Remove compiler warnings. */
    (void) pxUSART;

    /* Give the semaphore  to unblock any tasks that might be waiting for a Tx
	to complete.  If a task is unblocked, and the unblocked task has a priority
	above the currently running task, then xHigherPriorityTaskWoken will be set
	to pdTRUE inside the xSemaphoreGiveFromISR() function. */

    xSemaphoreGiveFromISR( xTxCompleteSemaphore, &xHigherPriorityTaskWoken );

    /* portEND_SWITCHING_ISR() or portYIELD_FROM_ISR() can be used here. */

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
