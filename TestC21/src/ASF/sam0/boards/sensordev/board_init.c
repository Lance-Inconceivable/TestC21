/**
 * \file
 *
 * \brief SAM C21 Xplained Pro board initialization
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <compiler.h>
#include <board.h>
#include <conf_board.h>
#include <port.h>

#if defined(__GNUC__)
void board_init(void) WEAK __attribute__((alias("system_board_init")));
#elif defined(__ICCARM__)
void board_init(void);
#  pragma weak board_init=system_board_init
#endif

void system_board_init(void)
{
        struct system_pinmux_config adc_pin_config;
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	/* Configure LEDs as outputs, turn them off */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_conf);
	port_pin_set_output_level(LED_0_PIN, true);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(GREEN_LED_PIN, &pin_conf);
	port_pin_set_output_level(GREEN_LED_PIN, true);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(RED_LED_PIN, &pin_conf);
	port_pin_set_output_level(RED_LED_PIN, true);

        /* CAN */

        /* Pins are active low.
         * The code below puts transceiver in standby mode
         * and disables silent mode.
         */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(CAN_STANDBY_PIN, &pin_conf);
	port_pin_set_output_level(CAN_STANDBY_PIN, true);

        /* Microchip recommends silent mode set high when in standby */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(CAN_SILENTMODE_PIN, &pin_conf);
	port_pin_set_output_level(CAN_SILENTMODE_PIN, true);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SR_CLK_PIN, &pin_conf);
	port_pin_set_output_level(SR_CLK_PIN, false);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(SR_LATCH_PIN, &pin_conf);
	port_pin_set_output_level(SR_LATCH_PIN, false);

        /* Configure the ADC inputs */

	system_pinmux_get_config_defaults(&adc_pin_config);

	/* Analog functions are all on MUX setting B */
	adc_pin_config.input_pull   = SYSTEM_PINMUX_PIN_PULL_NONE;
	adc_pin_config.mux_position = 1;

	system_pinmux_pin_set_config(ADC_1_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_2_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_3_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_4_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_5_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_6_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_7_PIN, &adc_pin_config);
	system_pinmux_pin_set_config(ADC_8_PIN, &adc_pin_config);
}
