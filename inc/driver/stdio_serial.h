/*
 * stdio_serial.h
 *
 * Created: 01/01/2021 20:26:39
 *  Author: mmuca
 */ 


#ifndef STDIO_SERIAL_H_
#define STDIO_SERIAL_H_

#include "sam3x8e.h"
#include "UART_Serial.h"
#include <stdio.h>

//! Pointer to the base of the USART module instance to use for stdio.
extern volatile void *volatile stdio_base;
//! Pointer to the external low level write function.
extern int (*ptr_put)(void volatile*, char);

//! Pointer to the external low level read function.
extern void (*ptr_get)(void volatile*, char*);

/*! \brief Initializes the stdio in Serial Mode.
 *
 * \param usart       Base address of the USART instance.
 * \param opt         Options needed to set up RS232 communication (see \ref usart_options_t).
 *
 */
static inline void stdio_serial_init(volatile void *usart, const usart_rs232_options_t *opt)
{
	stdio_base = (void *)usart;
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;
	usart_serial_init((Usart *)usart,(usart_rs232_options_t *)opt);


	// Specify that stdout and stdin should not be buffered.
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	// - printf() emits one character at a time.
	// - getchar() requests only 1 byte to exit.

}

#endif /* STDIO_SERIAL_H_ */