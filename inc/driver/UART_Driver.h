/*
 * UART_Driver.h
 *
 * Created: 31/12/2020 12:01:22
 *  Author: mmuca
 */ 


#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include "sam3x8e.h"

/* UART internal div factor for sampling */
#define UART_MCK_DIV             16
/* Div factor to get the maximum baud rate */
#define UART_MCK_DIV_MIN_FACTOR  1
/* Div factor to get the minimum baud rate */
#define UART_MCK_DIV_MAX_FACTOR  65535


typedef struct UART_opt {
    /* MASTERCLOCK */
    uint32_t ul_mck;
    uint32_t ul_baudrate;
    uint32_t ul_mode;
}UART_opt_t;

uint32_t uart_init(Uart *p_uart, const UART_opt_t *p_uart_opt);

void uart_enable_tx(Uart *p_uart);
void uart_disable_tx(Uart *p_uart);

void uart_reset_tx(Uart *p_uart);

void uart_enable_rx(Uart *p_uart);
void uart_disable_rx(Uart *p_uart);

void uart_reset_rx(Uart *p_uart);
void uart_enable(Uart *p_uart);
void uart_disable(Uart *p_uart);
void uart_reset(Uart *p_uart);

void uart_enable_interrupt(Uart *p_uart, uint32_t ul_sources);
void uart_disable_interrupt(Uart *p_uart, uint32_t ul_sources);
uint32_t uart_get_interrupt_mask(Uart *p_uart);

uint32_t uart_get_status(Uart *p_uart);
void uart_reset_status(Uart *p_uart);

uint32_t uart_is_tx_ready(Uart *p_uart);
uint32_t uart_is_tx_empty(Uart *p_uart);
uint32_t uart_is_rx_ready(Uart *p_uart);
uint32_t uart_is_tx_buf_empty(Uart *p_uart);

void uart_set_clock_divisor(Uart *p_uart, uint16_t us_divisor);

uint32_t uart_write(Uart *p_uart, const uint8_t uc_data);
uint32_t uart_read(Uart *p_uart, uint8_t *puc_data);







#endif /* UART_DRIVER_H_ */