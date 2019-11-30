
#include <asf.h>
#include <UARTCommandConsole.h>

struct freqm_module freqm_instance;

volatile bool freqm_read_done = false;

void freqm_complete_callback(void)
{
    freqm_read_done = true;
    return;
}

/************************    FREQM **********************/

/*
 * Use the built-in FREQM module to measure a clock frequency
 */

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
#ifdef EXTERNAL_CLOCK_TEST
    /* The input pin at SIG_GEN2 will be used as GCLK_IO4 */
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

   /* clear the ready flag */
   freqm_read_done = false;

   return (freqm_get_result_value(&freqm_instance, result));
}

/************************  FREQCOUNT **********************/

struct tc_module tc_instance;
uint32_t gFreq = 0;

/*
 * Use a GPIO line to measure frequency by counting clock pulses.
 */
static int counter = -1;
static void my_callback(void)
{
    static uint32_t start;
    uint32_t stop;
    uint32_t interval;
  
#if 0
    if (!measuring)
        return;
#endif

    counter++;

    if (counter == 0) {
        start = tc_get_count_value(&tc_instance);
    }
    else if (counter == 10) {
        stop = tc_get_count_value(&tc_instance);
        if (stop > start)
            interval = stop - start;
        else
            interval = stop + (0xffffffff - start);

#if 0
        gFreq = 100000 / interval;
#else
        interval /= 10;
        gFreq = 1000000 / interval;
#endif
        counter = -1;

        /* Stop timer and read the value */
    }
    return;
}

/* This method of frequency capture seems to work, but if the
 * input frequence is > 15K, the processor gets swamped by interrupts.
 *
 * Need to try the TC in capture or timestamp mode.  This eliminates the
 * GPIO interrupt and I think will just give 1 or 2 interrupts for a
 * measurement.
 */
freq_gpio_init(int pin)
{
    struct extint_chan_conf config;
    struct tc_config tc_config;
    uint32_t temp;

    tc_get_config_defaults(&tc_config);
    tc_config.counter_size = TC_COUNTER_SIZE_32BIT;
    tc_config.clock_source = GCLK_GENERATOR_2;
    tc_config.clock_prescaler = TC_CLOCK_PRESCALER_DIV2;

    tc_init(&tc_instance, TC0, &tc_config);
    tc_enable(&tc_instance);
    tc_start_counter(&tc_instance);

    /* 1 MHz clock running, hook up the FREQ_COUNT pin */

#if 1
    extint_chan_get_config_defaults(&config);
    config.gpio_pin = pin;
    config.gpio_pin_pull = EXTINT_PULL_NONE;
    config.detection_criteria = EXTINT_DETECT_RISING;
    extint_chan_set_config(2, &config);
    extint_register_callback(my_callback, 2, EXTINT_CALLBACK_TYPE_DETECT);
    extint_chan_enable_callback(2, EXTINT_CALLBACK_TYPE_DETECT);
#endif
}
