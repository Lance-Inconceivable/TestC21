#ifndef _ADCTEST_H_
#define _ADCTEST_H_

extern uint16_t adc_result_buffer[];
extern uint32_t sdadc_result_buffer[];
void configure_adc(void);
void configure_adc_callbacks(void);
void adc_run(void);
void adc_wait(void);
void configure_sdadc(void);
void configure_sdadc_callbacks(void);
void sdadc_run(void);
void sdadc_wait(void);

extern uint32_t freq_window;
extern uint32_t freq_ref;

void configure_freqm(void);
void configure_freqm_callbacks(void);
void freqm_run(void);
int freqm_wait(uint32_t *result);

#endif
