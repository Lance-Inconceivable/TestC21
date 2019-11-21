
#include <asf.h>

struct adc_module adc_instance;

#define ADC_SAMPLES 128
uint16_t adc_result_buffer[ADC_SAMPLES];

volatile bool adc_read_done = false;

/* After 128 samples have been taken, the ISR will set this flag to 'true'.
 * We can poll on this flag for now.  The right way to do it is to give a
 * semaphore indicating that the result_buffer is full.
 */
void adc_complete_callback(struct adc_module *module)
{
    adc_read_done = true;
    return;
}

void configure_adc(void)
{
    struct adc_config config_adc;
    adc_get_config_defaults(&config_adc);

    config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
    config_adc.reference = ADC_REFERENCE_INTVCC2;
    config_adc.positive_input = ADC_POSITIVE_INPUT_PIN8;
    config_adc.resolution = ADC_RESOLUTION_12BIT;
    adc_init(&adc_instance, ADC0, &config_adc);
}

void configure_adc_callbacks(void)
{
    adc_register_callback(&adc_instance, adc_complete_callback, 
        ADC_CALLBACK_READ_BUFFER);
    adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

void adc_run(void)
{
    adc_enable(&adc_instance);
    adc_read_buffer_job(&adc_instance, adc_result_buffer, ADC_SAMPLES);
}

void adc_wait(void)
{
   while (adc_read_done == false)
       {}
}



   
