#ifndef _CAN_UTILS_H_
#define _CAN_UTILS_H_

#define MATCH_ALL 0xfffffffful

typedef struct _can_hw_filter {
    uint32_t filter;
    uint32_t mask;
    uint8_t ext;
} CAN_HW_FILTER;

int can_filter_add(CAN_HW_FILTER *pfilter);
int can_filter_switch(uint8_t on, uint8_t index, uint8_t ext);
int can_filter_remove(int index, uint8_t ext);
void can_filters_init(void);
int can_msg_get(int timeout);
int can_msg_free(int index);
int can_send(uint32_t id, unsigned char *data);
int can_fifo_read(struct can_rx_element_fifo_0* packet, uint mstimeout);
int can_receive(unsigned int ArbID, int mstimeOut);
int can_utils_init(void);
void can_reset_fifo(void);
void microtimer_init(void);
uint32_t microtimer_read(void);
uint32_t microtimer_stop(void);
uint32_t microtimer_convert(uint32_t x);
void microtimer_start(void);
void microtimer_delayus(uint32_t usdelay);
void CAN_Copy_TxBuf(unsigned char *src, unsigned char src_count, unsigned char *dest, unsigned char dest_index, unsigned char padding);
void CAN_Copy_RxBuf(unsigned char *src, unsigned char *dest, unsigned char count);

#endif

