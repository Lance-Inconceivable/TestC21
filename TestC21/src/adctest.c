
#include <asf.h>

struct adc_module adc_instance;
struct sdadc_module sdadc_instance;
struct sdadc_module freqm_instance;

#define ADC_SAMPLES 128
uint16_t adc_result_buffer[ADC_SAMPLES];
uint32_t sdadc_result_buffer[ADC_SAMPLES];

uint32_t freq_window;
uint32_t freq_ref;

volatile bool adc_read_done = false;
volatile bool sdadc_read_done = false;
volatile bool freqm_read_done = false;

/* After 128 samples have been taken, the ISR will set this flag to 'true'.
 * We can poll on this flag for now.  The right way to do it is to give a
 * semaphore indicating that the result_buffer is full.
 */
void adc_complete_callback(struct adc_module *module)
{
    adc_read_done = true;
    return;
}

void sdadc_complete_callback(struct sdadc_module *module)
{
    sdadc_read_done = true;
    return;
}

void freqm_complete_callback(struct freqm_module *module)
{
    freqm_read_done = true;
    return;
}

void configure_adc(void)
{
    struct adc_config config_adc;
    adc_get_config_defaults(&config_adc);

    config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
    config_adc.reference = ADC_REFERENCE_INTVCC2;
#if 1
    config_adc.positive_input = ADC_POSITIVE_INPUT_PIN8;
#else
    config_adc.positive_input = ADC_POSITIVE_INPUT_PIN0;
#endif
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
   adc_read_done = false;
}

/*******************  SDADC ************************************************/



void configure_sdadc(void)
{
    struct sdadc_config config;
    sdadc_get_config_defaults(&config);
    config.mux_input = SDADC_MUX_INPUT_AIN0;
    config.reference.ref_sel = SDADC_REFERENCE_INTVCC;    /* 5V VDDANA */
    config.reference.ref_range = SDADC_REFRANGE_3;
    config.skip_count = 3;
    config.clock_prescaler = 8;
    config.correction.shift_correction = 8;
   
    sdadc_init(&sdadc_instance, SDADC, &config);
}

void configure_sdadc_callbacks(void)
{
    sdadc_register_callback(&sdadc_instance, sdadc_complete_callback, 
        SDADC_CALLBACK_READ_BUFFER);
    sdadc_enable_callback(&sdadc_instance, SDADC_CALLBACK_READ_BUFFER);
}

void sdadc_run(void)
{
    sdadc_enable(&sdadc_instance);
    sdadc_read_buffer_job(&sdadc_instance, sdadc_result_buffer, 2);
}

void sdadc_wait(void)
{
   while (sdadc_read_done == false)
       {}
   sdadc_read_done = false;
}

/************************    FREQM **********************/

void configure_freqm(void)
{
    struct freqm_config config;
    freqm_get_config_defaults(&config);
    config.ref_clock_circles = 48;
    /* Using all the defaults for this case means that we will measure
     * GCLK0 using GCLK1 as a reference.
     * The measurement duration is config.ref_clock_circles: 127.
     * GCLK1 is 6 MHz.
     */
#define EXTERNAL_CLOCK_TESTx
#ifdef EXTERNAL_CLOCK_TEST
    config.msr_clock_source = GCLK_GENERATOR_4;
#endif
   
    freqm_init(&freqm_instance, FREQM, &config);
}

void configure_freqm_callbacks(void)
{
    freqm_register_callback(&freqm_instance, freqm_complete_callback, 
        FREQM_CALLBACK_MEASURE_DONE);
    freqm_enable_callback(&freqm_instance, FREQM_CALLBACK_MEASURE_DONE);
}

void freqm_run(void)
{
    freqm_enable(&freqm_instance);
    freqm_start_measure(&freqm_instance);
}

int freqm_wait(uint32_t *result)
{
   int rval;
   while (freqm_read_done == false)
       {}
   freqm_read_done = false;
   return (freqm_get_result_value(&freqm_instance, result));
}
