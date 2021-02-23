/*
 * UART_Driver.c
 *
 * Created: 31/12/2020 12:01:55
 *  Author: mmuca
 */ 

#include "UART_Driver.h"


/**
 * \brief Configure UART.
 *
 * \param p_uart Pointer to a UART instance.
 * \param p_uart_opt Pointer to UART_opt_t instance.
 *
 * \retval 0 Success.
 * \retval 1 Bad baud rate generator value.
 */
uint32_t uart_init(Uart *p_uart, const UART_opt_t *p_uart_opt)
{
	uint32_t cd = 0;

    /* Reset and disable receiver & transmitter */
	p_uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;

    /* Check and configure baudrate */
	/* Asynchronous, no oversampling */
	cd = (p_uart_opt->ul_mck / p_uart_opt->ul_baudrate) / UART_MCK_DIV;
	if (cd < UART_MCK_DIV_MIN_FACTOR || cd > UART_MCK_DIV_MAX_FACTOR)
		return 1;

    p_uart->UART_BRGR = cd;
	/* Configure mode */
	p_uart->UART_MR = p_uart_opt->ul_mode;
	/* Disable PDC channel */
	p_uart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	/* Enable receiver and transmitter */
	p_uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

	return 0;
}

/**
 * \brief Enable UART transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_enable_tx(Uart *p_uart)
{
	/* Enable transmitter */
	p_uart->UART_CR = UART_CR_TXEN;
}

/**
 * \brief Disable UART transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_disable_tx(Uart *p_uart)
{
	/* Disable transmitter */
	p_uart->UART_CR = UART_CR_TXDIS;
}

/**
 * \brief Reset UART transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_reset_tx(Uart *p_uart)
{
	/* Reset transmitter */
	p_uart->UART_CR = UART_CR_RSTTX | UART_CR_TXDIS;
}

/**
 * \brief Enable UART receiver.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_enable_rx(Uart *p_uart)
{
	/* Enable receiver */
	p_uart->UART_CR = UART_CR_RXEN;
}

/**
 * \brief Disable UART receiver.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_disable_rx(Uart *p_uart)
{
	/* Disable receiver */
	p_uart->UART_CR = UART_CR_RXDIS;
}

/**
 * \brief Reset UART receiver.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_reset_rx(Uart *p_uart)
{
	/* Reset receiver */
	p_uart->UART_CR = UART_CR_RSTRX | UART_CR_RXDIS;
}

/**
 * \brief Enable UART receiver and transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_enable(Uart *p_uart)
{
	/* Enable receiver and transmitter */
	p_uart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}

/**
 * \brief Disable UART receiver and transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_disable(Uart *p_uart)
{
	/* Disable receiver and transmitter */
	p_uart->UART_CR = UART_CR_RXDIS | UART_CR_TXDIS;
}

/**
 * \brief Reset UART receiver and transmitter.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_reset(Uart *p_uart)
{
	/* Reset and disable receiver & transmitter */
	p_uart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
			| UART_CR_RXDIS | UART_CR_TXDIS;
}

/** \brief Enable UART interrupts.
 *
 * \param p_uart Pointer to a UART instance.
 *  \param ul_sources Interrupts to be enabled.
 */
void uart_enable_interrupt(Uart *p_uart, uint32_t ul_sources)
{
	p_uart->UART_IER = ul_sources;
}

/** \brief Disable UART interrupts.
 *
 * \param p_uart Pointer to a UART instance.
 *  \param ul_sources Interrupts to be disabled.
 */
void uart_disable_interrupt(Uart *p_uart, uint32_t ul_sources)
{
	p_uart->UART_IDR = ul_sources;
}

/** \brief Read UART interrupt mask.
 *
 * \param p_uart Pointer to a UART instance.
 *
 *  \return The interrupt mask value.
 */
uint32_t uart_get_interrupt_mask(Uart *p_uart)
{
	return p_uart->UART_IMR;
}

/**
 * \brief Get current status.
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \return The current UART status.
 */
uint32_t uart_get_status(Uart *p_uart)
{
	return p_uart->UART_SR;
}

/**
 * \brief Reset status bits.
 *
 * \param p_uart Pointer to a UART instance.
 */
void uart_reset_status(Uart *p_uart)
{
	p_uart->UART_CR = UART_CR_RSTSTA;
}

/**
 * \brief Check if Transmit is Ready.
 * Check if data has been loaded in UART_THR and is waiting to be loaded in the
 * Transmit Shift Register (TSR).
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \retval 1 Data has been transmitted.
 * \retval 0 Transmit is not ready, data pending.
 */
uint32_t uart_is_tx_ready(Uart *p_uart)
{
	return (p_uart->UART_SR & UART_SR_TXRDY) > 0;
}

/**
 * \brief Check if Transmit Hold Register is empty.
 * Check if the last data written in UART_THR has been loaded in TSR and the
 * last data loaded in TSR has been transmitted.
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \retval 1 Transmitter is empty.
 * \retval 0 Transmitter is not empty.
 */
uint32_t uart_is_tx_empty(Uart *p_uart)
{
	return (p_uart->UART_SR & UART_SR_TXEMPTY) > 0;
}

/**
 * \brief Check if Received data is ready.
 * Check if data has been received and loaded in UART_RHR.
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \retval 1 One data has been received.
 * \retval 0 No data has been received.
 */
uint32_t uart_is_rx_ready(Uart *p_uart)
{
	return (p_uart->UART_SR & UART_SR_RXRDY) > 0;
}

/**
 * \brief Check if both transmit buffers are sent out.
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \retval 1 Transmit buffer is empty.
 * \retval 0 Transmit buffer is not empty.
 */
uint32_t uart_is_tx_buf_empty(Uart *p_uart)
{
	return (p_uart->UART_SR & UART_SR_TXEMPTY) > 0;
}

/**
 * \brief Set UART clock divisor value
 *
 * \param p_uart Pointer to a UART instance.
 * \param us_divisor Value to be set.
 *
 */
void uart_set_clock_divisor(Uart *p_uart, uint16_t us_divisor)
{
	p_uart->UART_BRGR = us_divisor;
}

/**
 * \brief Write to UART Transmit Holding Register
 * Before writing user should check if tx is ready (or empty).
 *
 * \param p_uart Pointer to a UART instance.
 * \param data Data to be sent.
 *
 * \retval 0 Success.
 * \retval 1 I/O Failure, UART is not ready.
 */
uint32_t uart_write(Uart *p_uart, const uint8_t uc_data)
{
	/* Check if the transmitter is ready */
	if (!(p_uart->UART_SR & UART_SR_TXRDY))
		return 1;

	/* Send character */
	p_uart->UART_THR = uc_data;
	return 0;
}

/**
 * \brief Read from UART Receive Holding Register.
 * Before reading user should check if rx is ready.
 *
 * \param p_uart Pointer to a UART instance.
 *
 * \retval 0 Success.
 * \retval 1 I/O Failure, UART is not ready.
 */
uint32_t uart_read(Uart *p_uart, uint8_t *puc_data)
{
	/* Check if the receiver is ready */
	if ((p_uart->UART_SR & UART_SR_RXRDY) == 0)
		return 1;

	/* Read character */
	*puc_data = (uint8_t) p_uart->UART_RHR;
	return 0;
}










