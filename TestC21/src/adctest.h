#ifndef _ADCTEST_H_
#define _ADCTEST_H_

extern uint16_t adc_result_buffer[];
extern uint32_t sdadc_result_buffer[];
void configure_adc(uint32_t adcselect);
void configure_adc_callbacks(void);
void adc_run(void);
void adc_wait(void);
void configure_sdadc(void);
void configure_sdadc_callbacks(void);
void sdadc_run(void);
void sdadc_wait(void);

#endif
