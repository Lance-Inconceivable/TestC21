/**
 * \file
 *
 * \brief EZLYNK Sensor Dev board
 *
 */

#ifndef SENSORDEV_H_INCLUDED
#define SENSORDEV_H_INCLUDED

#include <conf_board.h>
#include <compiler.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \ingroup group_common_boards
 * \defgroup samc21_xplained_pro_group SAM C21 Xplained Pro board
 *
 * @{
 */

void system_board_init(void);

/**
 * \defgroup samc21_xplained_pro_features_group Features
 *
 * Symbols that describe features and capabilities of the board.
 *
 * @{
 */

/** Name string macro */
#define BOARD_NAME                "SENSORDEV"

/** \name Resonator definitions
 *  @{ */
#define BOARD_FREQ_SLCK_XTAL      (32768U)
#define BOARD_FREQ_SLCK_BYPASS    (32768U)
#define BOARD_FREQ_MAINCK_XTAL    0 /* Not Mounted */
#define BOARD_FREQ_MAINCK_BYPASS  0 /* Not Mounted */
#define BOARD_MCK                 CHIP_FREQ_CPU_MAX
#define BOARD_OSC_STARTUP_US      15625
/** @} */

/* Use LED Green1 as LED0/Heartbeart */
/** \name LED0 definitions
 *  @{ */
#define LED0_PIN                  PIN_PA16
#define LED0_ACTIVE               false
#define LED0_INACTIVE             !LED0_ACTIVE
/** @} */

#define GREEN_LED_PIN             PIN_PA17
#define RED_LED_PIN               PIN_PA19

#define CAN_SILENTMODE_PIN        PIN_PA22    
#define CAN_STANDBY_PIN           PIN_PA23

#define SR_CLEAR_PIN              RED_LED_PIN
#define SR_DATA_PIN               GREEN_LED_PIN
#define SR_CLK_PIN                PIN_PA27
#define SR_LATCH_PIN              PIN_PA28

/**
 * \name LED #0 definitions
 *
 * Wrapper macros for LED0, to ensure common naming across all Xplained Pro
 * boards.
 *
 *  @{ */
#define LED_0_NAME                "LED0 (yellow)"
#define LED_0_PIN                 LED0_PIN
#define LED_0_ACTIVE              LED0_ACTIVE
#define LED_0_INACTIVE            LED0_INACTIVE
#define LED0_GPIO                 LED0_PIN
#define LED0                      LED0_PIN

/** @} */

/** Number of on-board LEDs */
/* Jimmy: 2 user LEDs.  Heartbeat led LED0, not included in count */
#define LED_COUNT                 2

/** \name Embedded debugger CDC Gateway USART interface definitions
 * @{
 */
#define EDBG_CDC_MODULE              SERCOM2
#define EDBG_CDC_SERCOM_MUX_SETTING  USART_RX_3_TX_2_XCK_3
#define EDBG_CDC_SERCOM_PINMUX_PAD0  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD1  PINMUX_UNUSED
#define EDBG_CDC_SERCOM_PINMUX_PAD2  PINMUX_PA14C_SERCOM2_PAD2
#define EDBG_CDC_SERCOM_PINMUX_PAD3  PINMUX_PA15C_SERCOM2_PAD3
/** @} */


/** \name CAN interface definitions
 * @{
 */
#define CAN_MODULE              CAN0
#define CAN_TX_PIN              PIN_PA24G_CAN0_TX
#define CAN_TX_MUX_SETTING      MUX_PA24G_CAN0_TX
#define CAN_RX_PIN              PIN_PA25G_CAN0_RX
#define CAN_RX_MUX_SETTING      MUX_PA25G_CAN0_RX
/** @} */

/* ADC_1 at top of board, ADC_8 at bottom, near EZLynk logo */
/* This parameter is the mux position setting in the ADC controller */
#define ADC_1 ADC_POSITIVE_INPUT_PIN0
#define ADC_2 ADC_POSITIVE_INPUT_PIN1
#define ADC_3 ADC_POSITIVE_INPUT_PIN4
#define ADC_4 ADC_POSITIVE_INPUT_PIN5
#define ADC_5 ADC_POSITIVE_INPUT_PIN8
#define ADC_6 ADC_POSITIVE_INPUT_PIN9
#define ADC_7 ADC_POSITIVE_INPUT_PIN10
#define ADC_8 ADC_POSITIVE_INPUT_PIN11

/* This is the pin number in the port multiplexer  */
#define ADC_1_PIN PIN_PA02B_ADC0_AIN0
#define ADC_2_PIN PIN_PA03B_ADC0_AIN1
#define ADC_3_PIN PIN_PA04B_ADC0_AIN4
#define ADC_4_PIN PIN_PA05B_ADC0_AIN5
#define ADC_5_PIN PIN_PA08B_ADC0_AIN8
#define ADC_6_PIN PIN_PA09B_ADC0_AIN9
#define ADC_7_PIN PIN_PA10B_ADC0_AIN10
#define ADC_8_PIN PIN_PA11B_ADC0_AIN11

/**
 * \brief Turns off the specified LEDs.
 *
 * \param led_gpio LED to turn off (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Off(led_gpio)     port_pin_set_output_level(led_gpio,true)

/**
 * \brief Turns on the specified LEDs.
 *
 * \param led_gpio LED to turn on (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_On(led_gpio)      port_pin_set_output_level(led_gpio,false)

/**
 * \brief Toggles the specified LEDs.
 *
 * \param led_gpio LED to toggle (LEDx_GPIO).
 *
 * \note The pins of the specified LEDs are set to GPIO output mode.
 */
#define LED_Toggle(led_gpio)  port_pin_toggle_output_level(led_gpio)

#ifdef __cplusplus
}
#endif

#endif  /* SENSORDEV_H_INCLUDED */
