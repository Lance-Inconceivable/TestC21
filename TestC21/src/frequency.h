
#ifndef _FREQUENCY_H_
#define _FREQUENCY_H_

void configure_freqm(void);
void configure_freqm_callbacks(void);
void freqm_run(void);
int freqm_wait(uint32_t *result);

extern uint32_t gFreq;
extern void freq_gpio_init(int pin);
extern void printtc(void);

#endif
