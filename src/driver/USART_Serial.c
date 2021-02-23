/*
 * USART_Serial.c
 *
 * Created: 01/01/2021 18:01:38
 *  Author: mmuca
 */ 


#include "UART_Serial.h"

/**
 * \brief Send a sequence of bytes to USART device
 *
 * \param usart  Base address of the USART instance.
 * \param data   Data buffer to read
 * \param len    Length of data
 *
 */
uint32_t usart_serial_write_packet(Usart* usart, const uint8_t *data,
		uint8_t len)
{
	while (len) {
		usart_serial_putchar(usart, *data);
		len--;
		data++;
	}
	return 0;
}

/**
 * \brief Receive a sequence of bytes from USART device
 *
 * \param usart  Base address of the USART instance.
 * \param data   Data buffer to write
 * \param len    Length of data
 *
 */
uint32_t usart_serial_read_packet(Usart* usart, uint8_t *data,
		uint8_t len)
{
	while (len) {
		usart_serial_getchar(usart, data);
		len--;
		data++;
	}
	return 0;
}








