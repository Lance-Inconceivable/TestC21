
#include <ASF.h>
#include "can_utils.h"

#define MEM_BASE 0x20000000

/* Only using 1 can controller, so make it a global, rather than
 * passing in fuction arglist.
 */

extern struct can_module *pCAN;
extern SemaphoreHandle_t canRxSemaphore;
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

/* Returns the FIFO index of a CAN message, or -1 if timeout expires */
/* Note: Assumes module CAN0 and FIFO0.  */
/* Note: This function assumes that there is only one reader of CAN
 *       packets and that it is 'can_rx_task'.  In that case, we
 *       can use a FreeRTOS task variable instead of a semaphore.
 */
int can_msg_get(int timeout)
{
    int index;
    uint32_t status;

retry:
    status = can_rx_get_fifo_status(pCAN, 0);
    if ((status & 0x7f) == 0) {                /* Nothing in FIFO, wait... */
#if 0
        if (ulTaskNotifyTake(pdTRUE, timeout) == 0) {
#else
        if (xSemaphoreTake(canRxSemaphore, timeout) == pdTRUE) {
#endif
            goto retry;    /* re-read status to confirm that this isn't an old semaphore */
        }
        else
            return (-1);         /* timeout */
    }

    index = (status >> 8) & 0x3f;

    return (index);
}

/* For CAN0, FIFO 0, return FIFO buffer back to controller */
int can_msg_free(int index)
{
    can_rx_fifo_acknowledge(pCAN, 0, index);
}

void can_msg_drain_fifo(void)
{
    int index;
    do {
        index = can_msg_get(0);   /* Get msg without waiting */
        if (index == -1)          /* FIFO empty */
            break;
        can_msg_free(index);      /* Got a msg, free it */
    } while(1);
}

