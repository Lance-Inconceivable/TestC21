
#include <asf.h>
#include <adctest.h>

struct adc_module adc_instance;
struct sdadc_module sdadc_instance;

#define ADC_SAMPLES 128
uint16_t adc_result_buffer[ADC_SAMPLES];
int32_t sdadc_result_buffer[ADC_SAMPLES];

volatile bool adc_read_done = false;
volatile bool sdadc_read_done = false;

/* After 128 samples have been taken, the ISR will set this flag to 'true'.
 * We can poll on this flag for now.  The right way to do it is to give a
 * semaphore indicating that the result_buffer is full.
 */
static
void adc_complete_callback(struct adc_module *module)
{
    adc_read_done = true;
    return;
}

static
void sdadc_complete_callback(struct sdadc_module *module)
{
    sdadc_read_done = true;
    return;
}

void configure_adc_callbacks(void)
{
    adc_register_callback(&adc_instance, adc_complete_callback, 
        ADC_CALLBACK_READ_BUFFER);
    adc_enable_callback(&adc_instance, ADC_CALLBACK_READ_BUFFER);
}

void configure_adc(uint32_t adc_select)
{
    struct adc_config config_adc;
    adc_get_config_defaults(&config_adc);

    if (ADC0->CTRLA.reg & ADC_CTRLA_ENABLE) {
        /* ADC is already configured, so just update the
         * register that selects which ADC line to sample.
         */
        ADC0->INPUTCTRL.reg = adc_select;
        return;
    }

    /* Derive 8 MHz sampling clock from 48 MHz GCLK_0 */
    config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV8;
    config_adc.reference = ADC_REFERENCE_INTVCC2;
    config_adc.positive_input = adc_select;
    config_adc.resolution = ADC_RESOLUTION_12BIT;
    adc_init(&adc_instance, ADC0, &config_adc);
    configure_adc_callbacks();
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
