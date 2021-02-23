/*
 * Helper.h
 *
 * Created: 23/12/2020 22:00:41
 *  Author: mmuca
 */ 


#ifndef HELPER_H_
#define HELPER_H_

#include "gpio_driver.h"
#include "pio_handler.h"
#include "PMC_Driver.h"
#include "delay.h"
#include "TWI_driver.h"
#include "UART_Driver.h"
#include "USART_Driver.h"
#include <stdio.h>
#include <string.h>
#include "UART_Serial.h"
#include "stdio_serial.h"
#include "MPU6050.h"
#include "PWM_Driver.h"
#include "TimerDriver.h"
#include "DUE.h"

#define LOW			0
#define HIGH		1
#define PULL_UP		1
#define NO_PULLUP	0
#define DISABLE		0
#define ENABLE		1

#define TWI0_DATA_GPIO   PIO_PA17_IDX
#define TWI0_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define TWI0_CLK_GPIO    PIO_PA18_IDX
#define TWI0_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

/*! TWI1 pins definition */
#define TWI1_DATA_GPIO   PIO_PB12_IDX
#define TWI1_DATA_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT)
#define TWI1_CLK_GPIO    PIO_PB13_IDX
#define TWI1_CLK_FLAGS   (PIO_PERIPH_A | PIO_DEFAULT)

#define PINS_UART        (PIO_PA8A_URXD | PIO_PA9A_UTXD)
#define PINS_UART_FLAGS  (PIO_PERIPH_A | PIO_DEFAULT | PIO_PULLUP)

#define PINS_UART_MASK (PIO_PA8A_URXD | PIO_PA9A_UTXD)
#define PINS_UART_PIO  PIOA
#define PINS_UART_ID   ID_PIOA
#define PINS_UART_TYPE PIO_PERIPH_A
#define PINS_UART_ATTR PIO_DEFAULT
#endif /* HELPER_H_ */