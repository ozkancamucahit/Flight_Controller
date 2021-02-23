/*
 * UART_Serial.h
 *
 * Created: 01/01/2021 17:58:22
 *  Author: mmuca
 */ 


#ifndef UART_SERIAL_H_
#define UART_SERIAL_H_

#include "UART_Driver.h"
#include "USART_Driver.h"
#include "PMC_Driver.h"

#define F_CPU 84000000UL

/** Input parameters for RS232. */
typedef struct uart_rs232_options {
	/** Set baud rate of the USART (unused in slave modes). */
	uint32_t baudrate;

	/** Number of bits to transmit as a character (5-bit to 9-bit). */
	uint32_t charlength;

	/**
	 * Parity type: USART_PMODE_DISABLED_gc, USART_PMODE_EVEN_gc,
	 * USART_PMODE_ODD_gc.
	 */
	uint32_t paritytype;

	/** 1, 1.5 or 2 stop bits. */
	uint32_t stopbits;

} usart_rs232_options_t;


/** 
 * \brief Initializes the Usart in master mode.
 *
 * \param p_usart  Base address of the USART instance.
 * \param opt      Options needed to set up RS232 communication (see
 * \ref usart_options_t).
 */
static inline void usart_serial_init(Usart* p_usart,
		usart_rs232_options_t *opt)
{

	UART_opt_t uart_settings;
	uart_settings.ul_mck = F_CPU;
	uart_settings.ul_baudrate = opt->baudrate;
	uart_settings.ul_mode = opt->paritytype;


	usart_opt_t usart_settings;
	usart_settings.baudrate = opt->baudrate;
	usart_settings.char_length = opt->charlength;
	usart_settings.parity_type = opt->paritytype;
	usart_settings.stop_bits= opt->stopbits;
	usart_settings.channel_mode= US_MR_CHMODE_NORMAL;
	
    // TODO: USART ta olabilir switch case yapabilirsin
	if (UART == (Uart*)p_usart) {
		pmc_enable_periph_clk(ID_UART);
		/* Configure UART */
		uart_init((Uart*)p_usart, &uart_settings);
	}
}


/**
 * \brief Sends a character with the USART.
 *
 * \param p_usart   Base address of the USART instance.
 * \param c       Character to write.
 *
 * \return Status.
 *   \retval 1  The character was written.
 */
static inline int usart_serial_putchar(Usart* p_usart, const uint8_t c)
{
	if (UART == (Uart*)p_usart) {
		while (uart_write((Uart*)p_usart, c)!=0);
		return 1;
	}
}

/**
 * \brief Wait until a character is received
 *
 * \param p_usart   Base address of the USART instance.
 * \param data   Data to read
 *
 */
static inline void usart_serial_getchar(Usart* p_usart, uint8_t *data)
{
    if (UART == (Uart*)p_usart) {
		while (uart_read((Uart*)p_usart, data));
	}

}

/**
 * \brief Send a sequence of bytes to a USART device
 *
 * \param usart Base address of the USART instance.
 * \param data   data buffer to write
 * \param len    Length of data
 *
 */
uint32_t usart_serial_write_packet(Usart* usart, const uint8_t *data,
		uint8_t len);

/**
 * \brief Receive a sequence of bytes to a USART device
 *
 * \param usart Base address of the USART instance.
 * \param data   data buffer to write
 * \param len    Length of data
 *
 */
uint32_t usart_serial_read_packet(Usart* usart, uint8_t *data,
		uint8_t len);






#endif /* UART_SERIAL_H_ */