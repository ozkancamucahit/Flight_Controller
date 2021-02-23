/*
 * DUE.h
 *
 * Created: 22/01/2021 16:55:09
 *  Author: mmuca
 */ 


#ifndef DUE_H_
#define DUE_H_

#include "sam3x8e.h"

#define PINS_UART        (PIO_PA8A_URXD | PIO_PA9A_UTXD)
#define PINS_UART_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT | PIO_PULLUP)

#define PINS_UART_MASK (PIO_PA8A_URXD | PIO_PA9A_UTXD)
#define PINS_UART_PIO  PIOA
#define PINS_UART_ID   ID_PIOA
#define PINS_UART_TYPE PIO_PERIPH_A
#define PINS_UART_ATTR PIO_DEFAULT

#define CONSOLE_UART               UART
#define CONSOLE_UART_ID            ID_UART

/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define CONSOLE_PINS        {PINS_UART}

/** Usart Hw ID used by the console (UART0). */
#define CONSOLE_UART_ID          ID_UART

#define CONF_UART            CONSOLE_UART
/** Baudrate setting */
#define CONF_UART_BAUDRATE   115200
/** Parity setting */
#define CONF_UART_PARITY     UART_MR_PAR_NO

#define IOPORT_MODE_MUX_A               (  0 << 0) /*!< MUX function A */
#define IOPORT_MODE_MUX_B               (  1 << 0) /*!< MUX function B */

#define PIN_TC0_TIOA1           (PIO_PA2_IDX)
#define PIN_TC0_TIOA1_MUX       (IOPORT_MODE_MUX_A)
#define PIN_TC0_TIOA1_FLAGS     (PIO_PERIPH_A | PIO_DEFAULT)

#define PIN_TC0_TIOA1_PIO     PIOA
#define PIN_TC0_TIOA1_MASK    PIO_PA2
#define PIN_TC0_TIOA1_ID      ID_PIOA
#define PIN_TC0_TIOA1_TYPE    PIO_PERIPH_A
#define PIN_TC0_TIOA1_ATTR    PIO_DEFAULT


#define PIN_TC0_TIOA0         (PIO_PB25_IDX)
#define PIN_TC0_TIOA0_MUX     (IOPORT_MODE_MUX_B)
#define PIN_TC0_TIOA0_FLAGS   (PIO_INPUT | PIO_DEFAULT)

#define PIN_TC0_TIOA0_PIO     PIOB
#define PIN_TC0_TIOA0_MASK    PIO_PB25
#define PIN_TC0_TIOA0_ID      ID_PIOB
#define PIN_TC0_TIOA0_TYPE    PIO_INPUT
#define PIN_TC0_TIOA0_ATTR    PIO_DEFAULT


/** Use TC Peripheral 0 **/
#define TC  TC0
#define TC_PERIPHERAL  0

/** Configure TC0 channel 1 as waveform output. **/

#define ID_TC_WAVEFORM      ID_TC1
#define PIN_TC_WAVEFORM     PIN_TC0_TIOA1
#define PIN_TC_WAVEFORM_MUX PIN_TC0_TIOA1_MUX

/** Configure TC0 channel 0 as capture input. **/

#define ID_TC_CAPTURE      ID_TC0
#define PIN_TC_CAPTURE     PIN_TC0_TIOA0
#define PIN_TC_CAPTURE_MUX PIN_TC0_TIOA0_MUX

/** Use TC2_Handler for TC capture interrupt**/
#define TC_Handler  TC0_Handler
#define TC_IRQn     TC0_IRQn


#endif /* DUE_H_ */