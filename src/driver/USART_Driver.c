/*
 * USART_Driver.c
 *
 * Created: 31/12/2020 12:02:06
 *  Author: mmuca
 */ 


#include "USART_Driver.h"


  


/* The CD value scope programmed in MR register. */
#define MIN_CD_VALUE                  0x01
#define MIN_CD_VALUE_SPI              0x04
#define MAX_CD_VALUE                  US_BRGR_CD_Msk

/* The receiver sampling divide of baudrate clock. */
#define HIGH_FRQ_SAMPLE_DIV           16
#define LOW_FRQ_SAMPLE_DIV            8

/**
 * \brief Calculate a clock divider(CD) and a fractional part (FP) for the
 * USART asynchronous modes to generate a baudrate as close as possible to
 * the baudrate set point.
 *
 * \note Baud rate calculation: Baudrate = ul_mck/(Over * (CD + FP/8))
 * (Over being 16 or 8). The maximal oversampling is selected if it allows to
 * generate a baudrate close to the set point.
 *
 * \param p_usart Pointer to a USART instance.
 * \param baudrate Baud rate set point.
 * \param ul_mck USART module input clock frequency.
 *
 * \retval 0 Baud rate is successfully initialized.
 * \retval 1 Baud rate set point is out of range for the given input clock
 * frequency.
 */
uint32_t usart_set_async_baudrate(Usart *p_usart,
		uint32_t baudrate, uint32_t ul_mck)
{
	uint32_t over;
	uint32_t cd_fp;
	uint32_t cd;
	uint32_t fp;

	/* Calculate the receiver sampling divide of baudrate clock. */
	if (ul_mck >= HIGH_FRQ_SAMPLE_DIV * baudrate) {
		over = HIGH_FRQ_SAMPLE_DIV;
	} else {
		over = LOW_FRQ_SAMPLE_DIV;
	}

	/* Calculate clock divider according to the fraction calculated formula. */
	cd_fp = (8 * ul_mck + (over * baudrate) / 2) / (over * baudrate);
	cd = cd_fp >> 3;
	fp = cd_fp & 0x07;
	if (cd < MIN_CD_VALUE || cd > MAX_CD_VALUE) {
		return 1;
	}

	/* Configure the OVER bit in MR register. */
	if (over == 8) {
		p_usart->US_MR |= US_MR_OVER;
	}

	/* Configure the baudrate generate register. */
	p_usart->US_BRGR = (cd << US_BRGR_CD_Pos) | (fp << US_BRGR_FP_Pos);

	return 0;
}

/**
 * \brief Reset the USART and disable TX and RX.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset(Usart *p_usart)
{
	/* Disable the Write Protect. */
	//usart_disable_writeprotect(p_usart);

	/* Reset registers that could cause unpredictable behavior after reset. */
	p_usart->US_MR = 0;
	p_usart->US_RTOR = 0;
	p_usart->US_TTGR = 0;

	/* Disable TX and RX. */
	usart_reset_tx(p_usart);
	usart_reset_rx(p_usart);
	/* Reset status bits. */
	//usart_reset_status(p_usart);
	/* Turn off RTS and DTR if exist. */
	//usart_drive_RTS_pin_high(p_usart);
}

/**
 * \brief Configure USART to work in RS232 mode.
 *
 * \note By default, the transmitter and receiver aren't enabled.
 *
 * \param p_usart Pointer to a USART instance.
 * \param p_usart_opt Pointer to usart_opt_t instance.
 * \param ul_mck USART module input clock frequency.
 *
 * \retval 0 on success.
 * \retval 1 on failure.
 */
uint32_t usart_init_rs232(Usart *p_usart,
		const usart_opt_t *p_usart_opt, uint32_t ul_mck)
{
	static uint32_t ul_reg_val;

	/* Reset the USART and shut down TX and RX. */
	usart_reset(p_usart);

	ul_reg_val = 0;
	/* Check whether the input values are legal. */
	if (!p_usart_opt || usart_set_async_baudrate(p_usart,
			p_usart_opt->baudrate, ul_mck)) {
		return 1;
	}

	/* Configure the USART option. */
	ul_reg_val |= p_usart_opt->char_length | p_usart_opt->parity_type |
			p_usart_opt->channel_mode | p_usart_opt->stop_bits;

	/* Configure the USART mode as normal mode. */
	ul_reg_val |= US_MR_USART_MODE_NORMAL;

	p_usart->US_MR |= ul_reg_val;

	return 0;
}

/**
 * \brief Enable USART transmitter.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_enable_tx(Usart *p_usart)
{
	p_usart->US_CR = US_CR_TXEN;
}

/**
 * \brief Disable USART transmitter.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_disable_tx(Usart *p_usart)
{
	p_usart->US_CR = US_CR_TXDIS;
}

/**
 * \brief Immediately stop and disable USART transmitter.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset_tx(Usart *p_usart)
{
	/* Reset transmitter */
	p_usart->US_CR = US_CR_RSTTX | US_CR_TXDIS;
}

/**
 * \brief Enable USART receiver.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_enable_rx(Usart *p_usart)
{
	p_usart->US_CR = US_CR_RXEN;
}

/**
 * \brief Disable USART receiver.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_disable_rx(Usart *p_usart)
{
	p_usart->US_CR = US_CR_RXDIS;
}

/**
 * \brief Immediately stop and disable USART receiver.
 *
 * \param p_usart Pointer to a USART instance.
 */
void usart_reset_rx(Usart *p_usart)
{
	/* Reset Receiver */
	p_usart->US_CR = US_CR_RSTRX | US_CR_RXDIS;
}

/**
 * \brief Write to USART Transmit Holding Register.
 *
 * \note Before writing user should check if tx is ready (or empty).
 *
 * \param p_usart Pointer to a USART instance.
 * \param c Data to be sent.
 *
 * \retval 0 on success.
 * \retval 1 on failure.
 */
uint32_t usart_write(Usart *p_usart, uint32_t c)
{
	if (!(p_usart->US_CSR & US_CSR_TXRDY)) {
		return 1;
	}

	p_usart->US_THR = US_THR_TXCHR(c);
	return 0;
}

/**
 * \brief Write to USART Transmit Holding Register.
 *
 * \note Before writing user should check if tx is ready (or empty).
 *
 * \param p_usart Pointer to a USART instance.
 * \param c Data to be sent.
 *
 * \retval 0 on success.
 * \retval 1 on failure.
 */
uint32_t usart_putchar(Usart *p_usart, uint32_t c)
{
	while (!(p_usart->US_CSR & US_CSR_TXRDY)) {
	}

	p_usart->US_THR = US_THR_TXCHR(c);

	return 0;
}

/**
 * \brief Write one-line string through USART.
 *
 * \param p_usart Pointer to a USART instance.
 * \param string Pointer to one-line string to be sent.
 */
void usart_write_line(Usart *p_usart, const char *string)
{
	while (*string != '\0') {
		usart_putchar(p_usart, *string++);
	}
}

/**
 * \brief Read from USART Receive Holding Register.
 *
 * \note Before reading user should check if rx is ready.
 *
 * \param p_usart Pointer to a USART instance.
 * \param c Pointer where the one-byte received data will be stored.
 *
 * \retval 0 on success.
 * \retval 1 if no data is available or errors.
 */
uint32_t usart_read(Usart *p_usart, uint32_t *c)
{
	if (!(p_usart->US_CSR & US_CSR_RXRDY)) {
		return 1;
	}

	/* Read character */
	*c = p_usart->US_RHR & US_RHR_RXCHR_Msk;

	return 0;
}

/**
 * \brief Read from USART Receive Holding Register.
 * Before reading user should check if rx is ready.
 *
 * \param p_usart Pointer to a USART instance.
 * \param c Pointer where the one-byte received data will be stored.
 *
 * \retval 0 Data has been received.
 * \retval 1 on failure.
 */
uint32_t usart_getchar(Usart *p_usart, uint32_t *c)
{
	/* Wait until it's not empty or timeout has reached. */
	while (!(p_usart->US_CSR & US_CSR_RXRDY)) {
	}

	/* Read character */
	*c = p_usart->US_RHR & US_RHR_RXCHR_Msk;

	return 0;
}






















