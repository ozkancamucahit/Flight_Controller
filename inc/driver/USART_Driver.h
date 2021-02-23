/*
 * USART_Driver.h
 *
 * Created: 31/12/2020 12:01:42
 *  Author: mmuca
 */ 


#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_

#include "sam3x8e.h"


#define US_WPMR_WPKEY(value) ((US_WPMR_WPKEY_Msk & ((value) << US_WPMR_WPKEY_Pos)))

/** Clock phase. */
#define SPI_CPHA    (1 << 0)

/** Clock polarity. */
#define SPI_CPOL    (1 << 1)

/** SPI mode definition. */
#define SPI_MODE_0  0
#define SPI_MODE_1  (SPI_CPHA)
#define SPI_MODE_2  (SPI_CPOL)
#define SPI_MODE_3  (SPI_CPOL | SPI_CPHA)

/* Input parameters for RS232 */
typedef struct {
	/* Set baud rate of the USART (unused in slave modes). */
	uint32_t baudrate;

	/*
	 * Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	 * US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or
	 * US_MR_MODE9.
	 */
	uint32_t char_length;

	/*
	 * Parity type, which should be one of the following: US_MR_PAR_EVEN,
	 * US_MR_PAR_ODD, US_MR_PAR_SPACE, US_MR_PAR_MARK, US_MR_PAR_NO
	 * or US_MR_PAR_MULTIDROP.
	 */
	uint32_t parity_type;

	/*
	 * Number of stop bits between two characters: US_MR_NBSTOP_1_BIT,
	 * US_MR_NBSTOP_1_5_BIT, US_MR_NBSTOP_2_BIT.
	 * \note US_MR_NBSTOP_1_5_BIT is supported in asynchronous modes only.
	 */
	uint32_t stop_bits;

	/*
	 * Run the channel in test mode, which should be one of following:
	 * US_MR_CHMODE_NORMAL, US_MR_CHMODE_AUTOMATIC,
	 * US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK.
	 */
	uint32_t channel_mode;

	/* Filter of IrDA mode, useless in other modes. */
	uint32_t irda_filter;
} usart_opt_t;

/* Input parameters when initializing SPI mode. */
typedef struct {
	/* Set the frequency of the SPI clock (unused in slave mode). */
	uint32_t baudrate;

	/*
	 * Number of bits, which should be one of the following: US_MR_CHRL_5_BIT,
	 * US_MR_CHRL_6_BIT, US_MR_CHRL_7_BIT, US_MR_CHRL_8_BIT or
	 * US_MR_MODE9.
	 */
	uint32_t char_length;

	/*
	 * Which SPI mode to use, which should be one of the following:
	 * SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3.
	 */
	uint32_t spi_mode;

	/*
	 * Run the channel in test mode, which should be one of following:
	 * US_MR_CHMODE_NORMAL, US_MR_CHMODE_AUTOMATIC,
	 * US_MR_CHMODE_LOCAL_LOOPBACK, US_MR_CHMODE_REMOTE_LOOPBACK.
	 */
	uint32_t channel_mode;
} usart_spi_opt_t;

void usart_reset(Usart *p_usart);

uint32_t usart_set_async_baudrate(Usart *p_usart,
		uint32_t baudrate, uint32_t ul_mck);
uint32_t usart_init_rs232(Usart *p_usart,
		const usart_opt_t *p_usart_opt, uint32_t ul_mck);


void usart_enable_tx(Usart *p_usart);
void usart_disable_tx(Usart *p_usart);
void usart_reset_tx(Usart *p_usart);

void usart_enable_rx(Usart *p_usart);
void usart_disable_rx(Usart *p_usart);
void usart_reset_rx(Usart *p_usart);

uint32_t usart_write(Usart *p_usart, uint32_t c);
uint32_t usart_putchar(Usart *p_usart, uint32_t c);
void usart_write_line(Usart *p_usart, const char *string);
uint32_t usart_read(Usart *p_usart, uint32_t *c);
uint32_t usart_getchar(Usart *p_usart, uint32_t *c);







#endif /* USART_DRIVER_H_ */