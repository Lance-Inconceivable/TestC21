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
int can_utils_init(void);
void can_reset_fifo(void);

#endif

