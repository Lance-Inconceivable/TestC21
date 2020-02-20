
#include <ASF.h>
#include "can_utils.h"

#define MEM_BASE 0x20000000

/* Only using 1 can controller, so make it a global, rather than
 * passing in fuction arglist.
 */

extern struct can_module *pCAN;

int gHaveReader = 0;  /* Flag for ISR */

struct can_tx_element *pTxFIFO;
struct can_rx_element_fifo_0 *pRxFIFO;

static SemaphoreHandle_t canRxSemaphore = (SemaphoreHandle_t) NULL;

struct can_standard_message_filter_element *pHwFilters;
struct can_extended_message_filter_element *pHwFiltersX;

/*
 * Look for the first free slot in the filter table and insert filter 
 */
int can_filter_add(CAN_HW_FILTER *x)
{
    int i;
    if (x->ext) {
        uint32_t *p = (uint32_t *) pHwFiltersX;
        struct can_extended_message_filter_element filter;

        /* Scan filter table for empty slot */
        for (i = 0; i < CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM; i++) {
            if (*p == 0)
                break;
            p += 2;
        }

        if (i == CONF_CAN0_RX_STANDARD_ID_FILTER_NUM)
            return -1;

        can_get_extended_message_filter_element_default(&filter);
        filter.F0.bit.EFID1 = x->filter;
        filter.F1.bit.EFID2 = x->mask;
        can_set_rx_extended_filter(pCAN, &filter, i); 
    }
    else {
        uint32_t *p = (uint32_t *) pHwFilters;
        struct can_standard_message_filter_element filter;

        /* Scan filter table for empty slot */
        for (i = 0; i < CONF_CAN0_RX_STANDARD_ID_FILTER_NUM; i++) {
            if (*p++ == 0)
                break;
        }

        if (i == CONF_CAN0_RX_STANDARD_ID_FILTER_NUM)
            return -1;
        
        can_get_standard_message_filter_element_default(&filter);
        filter.S0.bit.SFID1 = x->filter;
        filter.S0.bit.SFID2 = x->mask;
        can_set_rx_standard_filter(pCAN, &filter, i); 
    }
    return i;
}

/* 
 * Remove an entry from the CAN hardware filter table.
 * If index is -1, remove them all, excluding index = 0.
 */
int can_filter_remove(int index, uint8_t ext)
{
    if (ext) {
        struct can_extended_message_filter_element *filter = pHwFiltersX;

        if (index != -1) {
            if (index >= CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM) 
                return 1;
            filter[index].F0.reg = 0;
            filter[index].F1.reg = 0;
        }
        else {
            for (index = 1; index < CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM; index++) {
                filter[index].F0.reg = 0;
                filter[index].F1.reg = 0;
            }
        }
    }
    else {
        struct can_standard_message_filter_element *filter = pHwFilters;

        if (index != -1)  {
            if (index >= CONF_CAN0_RX_STANDARD_ID_FILTER_NUM) 
                return 1;
            filter[index].S0.reg = 0; 
        }
        else {
            for (index = 1; index < CONF_CAN0_RX_STANDARD_ID_FILTER_NUM; index++) 
                filter[index].S0.reg = 0;
        }
    }
    return (0);
}

/* Turn a filter on or off. 
 * Don't do anything if index out of range or filter unused.
 */
int can_filter_switch(uint8_t on, uint8_t index, uint8_t ext)
{
    if (ext) {
        struct can_extended_message_filter_element *filter = pHwFiltersX;

        if (index >= CONF_CAN0_RX_STANDARD_ID_FILTER_NUM) 
            return 1;

        if (filter[index].F0.reg == 0)    /* Don't touch a NULL filter */
            return (1);

        if (on) {
            /* Handle special case, filter 0, is the 'reject' filter */
            if (index == 0)
                filter[0].F0.bit.EFEC = 0x3;  /* Reject */
            else
                filter[index].F0.bit.EFEC = 0x1;  /* Store in FIFO 0 */
        }
        else {
            filter[index].F0.bit.EFEC = 0x0;  /* Disable */
        }
    }
    else {
        struct can_standard_message_filter_element *filter = pHwFilters;

        if (index >= CONF_CAN0_RX_STANDARD_ID_FILTER_NUM) 
            return 1;

        if (filter[index].S0.reg == 0)    /* Don't touch a NULL filter */
            return (1);

        if (on) {
            /* Handle special case, filter 0, is the 'reject' filter */
            if (index == 0)
                filter[index].S0.bit.SFEC = 0x3;  /* Reject */
            else
                filter[index].S0.bit.SFEC = 0x1;  /* Store in FIFO 0 */
        }
        else 
            filter[index].S0.bit.SFEC = 0x0;  /* Disable */
    }
    return (0);
}

void can_filters_init(void)
{
    int i;
    uint32_t *p;
    struct can_standard_message_filter_element filter;
    struct can_extended_message_filter_element xfilter;

    /* Save the base addresses of the filter tables */
    pHwFiltersX = (struct can_extended_message_filter_element *) 
        (pCAN->hw->XIDFC.bit.FLESA | MEM_BASE);
    pHwFilters  = (struct can_standard_message_filter_element *) 
        (pCAN->hw->SIDFC.bit.FLSSA | MEM_BASE);

    /* Clear all the standard filters */
    p = (uint32_t *) pHwFilters;
    for (i = 0; i < CONF_CAN0_RX_STANDARD_ID_FILTER_NUM; i++) 
        *p++ = 0x0;

    p = (uint32_t *) pHwFiltersX;
    /* Clear all the extended filters */
    for (i = 0; i < CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM; i++) {
        *p++ = 0x0;
        *p++ = 0x0;
    }

    /* Install a "freeze filter* at the beginning of each filter table.
     * A freeze filter is:
     *   ID: all bits on
     *   Mask: all bits off   (everything matches)
     *   Action: reject (0x3).  Initialized to 0 for "disable"
     */
    if (CONF_CAN0_RX_EXTENDED_ID_FILTER_NUM) {
        can_get_extended_message_filter_element_default(&xfilter);
        xfilter.F0.bit.EFID1 = 0x1ffffffful;
        xfilter.F1.bit.EFID2 = 0;              /* Mask is 0 */
        xfilter.F0.bit.EFEC = 0;               /* Disable */
        can_set_rx_extended_filter(pCAN, &xfilter, 0); 
    }

    if (CONF_CAN0_RX_STANDARD_ID_FILTER_NUM) {
        can_get_standard_message_filter_element_default(&filter);
        filter.S0.bit.SFID1 = 0x7fful;
        filter.S0.bit.SFID2 = 0;              /* Mask is 0 */
        filter.S0.bit.SFEC = 0;               /* Disable */
        can_set_rx_standard_filter(pCAN, &filter, 0); 
    }
}

/* Returns the FIFO index of a CAN message, or -1 if timeout expires 
 * Note: Assumes module CAN0 and FIFO0.  
 *
 * If a packet is available when this function is called, only a single read
 * of the RX FIFO status register is necessary.
 * If the FIFO is empty, the call will try to take 'canRxSemaphore'.  The semaphore
 * can be available with the FIFO empty because non-blocking reads don't take
 * the semaphore.  So, if the semaphore is taken, the FIFO status register is
 * checked to confirm the presence of data.  If the FIFO is really empty, we'll try to
 * take the semaphore again and this time we'll
 * block until timeout expires or the ISR detects that the FIFO level has gone
 * from 0 to 1.  In that case, we'll get the semaphore and read the status register
 * to get the packet index.
 */
int can_msg_get(int timeout)
{
    uint32_t index;
    uint32_t status;

retry:
    status = can_rx_get_fifo_status(pCAN, 0);
    if ((status & 0x7f) == 0) {                /* Nothing in FIFO, wait... */
        if (xSemaphoreTake(canRxSemaphore, timeout) == pdTRUE) {
            goto retry;    /* re-read status to confirm that this isn't an old semaphore */
        }
        else
            return (-1);         /* timeout */
    }

    index = (status >> 8) & 0x3f;

    return ((int) index);
}

/* For CAN0, FIFO 0, return FIFO buffer back to controller */
int can_msg_free(int index)
{
    if (index < CONF_CAN0_RX_FIFO_0_NUM) {
        can_rx_fifo_acknowledge(pCAN, 0, index);
        return (0);
    }
    else
        return (-1);
}

void can_reset_fifo(void)
{
#if 0  /* This is the slow way that only drains RX FIFO */
    int index;
    do {
        index = can_msg_get(0);   /* Get msg without waiting */
        if (index == -1)          /* FIFO empty */
            break;
        can_msg_free(index);      /* Got a msg, free it */
    } while(1);
#else               /* Hard reset of RX and TX FIFOs */
    can_stop(pCAN);
    can_start(pCAN);
#endif
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
	if(id <= 0x7FF) //standard ID message
		msg->T0.reg = CAN_TX_ELEMENT_T0_STANDARD_ID(id); 
	else //I don't think we will ever have a Extended message ID less than 0x800(0x7FF)
		msg->T0.reg = CAN_TX_ELEMENT_T0_EXTENDED_ID(id); 
    for (i = 0; i < 8; i++) {
        msg->data[i] = data[i];
    }
    can_set_tx_buffer_element(pCAN, msg, putindex);
    status = can_tx_transfer_request(pCAN, 1 << putindex); 
    return (-status);   /* Status codes are positive in ASF */
}

/* CAN Interrupts */

uint32_t can_rx_count = 0;

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
        if (rx_fifo_status == 1 && gHaveReader)
            xSemaphoreGiveFromISR(canRxSemaphore, &xHigherPriorityTaskWoken);
    }
#if 0
    if (status & CAN_TX_FIFO_EMPTY) {
        can_clear_interrupt_status(pCAN, CAN_TX_FIFO_EMPTY);
        /* This may be used when there's a can Tx task.  */
        vTaskNotifyGiveFromISR(txTaskHandle, &xHigherPriorityTaskWoken);
    }
#endif
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int can_utils_init(void)
{
    /* Save the address of the TX FIFO */
    uint32_t temp = pCAN->hw->TXBC.bit.TBSA;
    pTxFIFO = (struct can_tx_element *) (temp | MEM_BASE);

    /* Save the address of the RX FIFO */
    temp = pCAN->hw->RXF0C.bit.F0SA;
    pRxFIFO = (struct can_rx_element_fifo_0 *) (temp | MEM_BASE);

    can_filters_init();

    if (canRxSemaphore == NULL)
        canRxSemaphore = xSemaphoreCreateBinary();

    if (canRxSemaphore == NULL)
        return (-1);
    else
        return (0);
}

struct rtc_module rtc_mod;

/* Use the 32K clock source as a counter.
 * The counter increments every clock cycle and wraps around
 * when it reaches 0xffffffff.
 * Each increment is 30.5 microseconds.
 * The primary use for this timer is to measure the duration of an
 * operation.  
 *
 * microtimer_start() will clear the counter to 0.
 * microtimer_read() will return the value of the timer.
 * microtimer_convert(microtimer_read()) will return the value
 * in microseconds.
 */
void
microtimer_init(void)
{
    struct rtc_count_config config;
    rtc_count_get_config_defaults(&config);
    config.prescaler = RTC_COUNT_PRESCALER_OFF;
    rtc_count_init(&rtc_mod, RTC, &config);
}

void
microtimer_start(void)
{
    rtc_count_set_count(&rtc_mod, 0);
    rtc_count_enable(&rtc_mod);
}

uint32_t
microtimer_read(void)
{
    return(rtc_count_get_count(&rtc_mod));
}

uint32_t
microtimer_stop(void)
{
    rtc_count_disable(&rtc_mod);
    return(rtc_count_get_count(&rtc_mod));
}

/* Multiply incoming value by 30.5 to get microseconds */
uint32_t
microtimer_convert(uint32_t x)
{
    uint32_t val;
    val = 30 * x;
    val += (x >> 1);
    return (val);
}
