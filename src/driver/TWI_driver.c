/*
 * TWI_driver.c
 *
 * Created: 27/12/2020 18:31:42
 *  Author: mmuca
 */ 
#include "TWI_driver.h"



/**
 * \brief Enable TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_enable_master_mode(Twi *p_twi)
{
	/* Set Master Disable bit and Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
	p_twi->TWI_CR = TWI_CR_SVDIS;

	/* Set Master Enable bit */
	p_twi->TWI_CR = TWI_CR_MSEN;
}

/**
 * \brief Disable TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_disable_master_mode(Twi *p_twi)
{
	/* Set Master Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
}

/**
 * \brief Initialize TWI master mode.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_opt init params.
 *
 * \return TWI_SUCCESS if initialization is complete, error code otherwise.
 */
uint32_t twi_master_init(Twi *p_twi, const twi_options_t *p_opt)
{
	uint32_t status = TWI_SUCCESS;

	/* Disable TWI interrupts */
	p_twi->TWI_IDR = ~0UL;

	/* Dummy read in status register */
	p_twi->TWI_SR;

	/* Reset TWI peripheral */
	twi_reset(p_twi);

	twi_enable_master_mode(p_twi);

	/* Select the speed */
	if (twi_set_speed(p_twi, p_opt->speed, p_opt->master_clk) == FAIL) {
		/* The desired speed setting is rejected */
		status = TWI_INVALID_ARGUMENT;
	}

	if (p_opt->smbus == 1) {
		p_twi->TWI_CR = TWI_CR_QUICK;
	}

	return status;
}

/**
 * \brief Set the I2C bus speed in conjunction with the clock frequency.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_speed The desired I2C bus speed (in Hz).
 * \param ul_mck Main clock of the device (in Hz).
 *
 * \retval PASS New speed setting is accepted.
 * \retval FAIL New speed setting is rejected.
 */
uint32_t twi_set_speed(Twi *p_twi, uint32_t ul_speed, uint32_t ul_mck)
{
	uint32_t ckdiv = 0;
	uint32_t c_lh_div;
	uint32_t cldiv, chdiv;

	if (ul_speed > I2C_FAST_MODE_SPEED) {
		return FAIL;
	}

	// Low level time not less than 1.3us of I2C Fast Mode. 
	if (ul_speed > LOW_LEVEL_TIME_LIMIT) {
		// Low level of time fixed for 1.3us.
		cldiv = ul_mck / (LOW_LEVEL_TIME_LIMIT * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;
		chdiv = ul_mck / ((ul_speed + (ul_speed - LOW_LEVEL_TIME_LIMIT)) * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;
		
		// cldiv must fit in 8 bits, ckdiv must fit in 3 bits 
		while ((cldiv > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
			// Increase clock divider 
			ckdiv++;
			// Divide cldiv value 
			cldiv /= TWI_CLK_DIVIDER;
		}
		// chdiv must fit in 8 bits, ckdiv must fit in 3 bits 
		while ((chdiv > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
			// Increase clock divider 
			ckdiv++;
			// Divide cldiv value 
			chdiv /= TWI_CLK_DIVIDER;
		}

		// set clock waveform generator register 
		p_twi->TWI_CWGR =
				TWI_CWGR_CLDIV(cldiv) | TWI_CWGR_CHDIV(chdiv) |
				TWI_CWGR_CKDIV(ckdiv);		
	} else {
		c_lh_div = ul_mck / (ul_speed * TWI_CLK_DIVIDER) - TWI_CLK_CALC_ARGU;

		// cldiv must fit in 8 bits, ckdiv must fit in 3 bits 
		while ((c_lh_div > TWI_CLK_DIV_MAX) && (ckdiv < TWI_CLK_DIV_MIN)) {
			//Increase clock divider
			ckdiv++;
			// Divide cldiv value
			c_lh_div /= TWI_CLK_DIVIDER;
		}

		// set clock waveform generator register
		p_twi->TWI_CWGR =
				TWI_CWGR_CLDIV(c_lh_div) | TWI_CWGR_CHDIV(c_lh_div) |
				TWI_CWGR_CKDIV(ckdiv);
	}

	return PASS;
}


/*
uint32_t twi_set_speed(Twi *pTwi, uint32_t dwTwCk, uint32_t dwMCk)
{
	uint32_t dwCkDiv = 0 ;
    uint32_t dwClDiv ;
    uint32_t dwOk = 0 ;

    // Configure clock 
    while ( !dwOk )
    {
        dwClDiv = ((dwMCk / (2 * dwTwCk)) - 4) / (1<<dwCkDiv) ;

        if ( dwClDiv <= 255 )
        {
            dwOk = 1 ;
        }
        else
        {
            dwCkDiv++ ;
        }
    }

    pTwi->TWI_CWGR = 0 ;
    pTwi->TWI_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv ;
}
*/

/**
 * \brief Test if a chip answers a given I2C address.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param uc_slave_addr Address of the remote chip to search for.
 *
 * \return TWI_SUCCESS if a chip was found, error code otherwise.
 */
uint32_t twi_probe(Twi *p_twi, uint8_t uc_slave_addr)
{
	twi_packet_t packet;
	uint8_t data = 0;

	/* Data to send */
	packet.buffer = &data;
	/* Data length */
	packet.length = 1;
	/* Slave chip address */
	packet.chip_addr = uc_slave_addr;
	/* Internal chip address */
	packet.iaddr[0] = 0;
	/* Address length */
	packet.iaddr_length = 0;

	/* Perform a master write access */
	return (twi_master_write(p_twi, &packet));
}

/**
 * \brief Construct the TWI module address register field
 *
 * The TWI module address register is sent out MSB first. And the size controls
 * which byte is the MSB to start with.
 *
 * Please see the device datasheet for details on this.
 */
uint32_t twi_mk_addr(const uint8_t *addr, int len)
{
	uint32_t val;

	if (len == 0)
		return 0;

	val = addr[0];
	if (len > 1) {
		val <<= 8;
		val |= addr[1];
	}
	if (len > 2) {
		val <<= 8;
		val |= addr[2];
	}
	return val;
}


/**
 * \brief Read multiple bytes from a TWI compatible slave device.
 *
 * \note DONT RETURN untill all data is read
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_packet Packet information and data
 *
 * \return TWI_SUCCESS if all bytes were read, error code otherwise.
 */
uint32_t twi_master_read(Twi *p_twi, twi_packet_t *p_packet)
{
    uint32_t status;
    uint32_t count = p_packet->length;
    uint8_t* buffer = p_packet->buffer;
    uint8_t stop_sent = 0;
    uint32_t timeout = TWI_TIMEOUT;

    if ( count == 0 )
        return TWI_INVALID_ARGUMENT;

    // Set read mode, slave address and internal reg address
    p_twi->TWI_MMR = 0;
    p_twi->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(p_packet->chip_addr) |
        (( p_packet->iaddr_length << TWI_MMR_IADRSZ_Pos ) & 
        TWI_MMR_DADR_Msk);

    p_twi->TWI_IADR = 0;
    p_twi->TWI_IADR = twi_mk_addr(p_packet->iaddr, 
                                p_packet->iaddr_length);
    
    // SEND START

    if ( count == 1 )
    {
        p_twi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        stop_sent = 1;
    }else
        {
            p_twi->TWI_CR = TWI_CR_START;
            stop_sent = 0;
        }
    
    while (count > 0)
    {
        status = p_twi->TWI_SR;
        if ( status & TWI_SR_NACK )
            return TWI_RECEIVE_NACK;
        if (!timeout--)
            return TWI_ERROR_TIMEOUT;
        // LAST BYTE ?
        if ( count == 1 && !stop_sent )
        {
            p_twi->TWI_CR = TWI_CR_STOP;
            stop_sent = 1;
        }

        if ( !(status & TWI_SR_RXRDY) )
            continue;

        *buffer++ = p_twi->TWI_RHR;

        count--;
        timeout = TWI_TIMEOUT;
    }
    
    while ( !(p_twi->TWI_SR & TWI_SR_TXCOMP) ) {} // WAIT

    p_twi->TWI_SR;
    return TWI_SUCCESS;
}

/**
 * \brief Write multiple bytes to a TWI compatible slave device.
 *
 * \note This function will NOT return until all data has been written or error occurred.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param p_packet Packet information and data (see \ref twi_packet_t).
 *
 * \return TWI_SUCCESS if all bytes were written, error code otherwise.
 */
uint32_t twi_master_write(Twi *p_twi, twi_packet_t *p_packet)
{
	uint32_t status;
	uint32_t cnt = p_packet->length;
	uint8_t *buffer = p_packet->buffer;

	/* Check argument */
	if (cnt == 0) {
		return TWI_INVALID_ARGUMENT;
	}

	/* Set write mode, slave address and 3 internal address byte lengths */
	p_twi->TWI_MMR = 0;
	p_twi->TWI_MMR = TWI_MMR_DADR(p_packet->chip_addr) |
			((p_packet->iaddr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

	/* Set internal address for remote chip */
	p_twi->TWI_IADR = 0;
	p_twi->TWI_IADR = twi_mk_addr(p_packet->iaddr, p_packet->iaddr_length);

	/* Send all bytes */
	while (cnt > 0) {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (!(status & TWI_SR_TXRDY)) {
			continue;
		}
		p_twi->TWI_THR = *buffer++;

		cnt--;
	}

	while (1) {
		status = p_twi->TWI_SR;
		if (status & TWI_SR_NACK) {
			return TWI_RECEIVE_NACK;
		}

		if (status & TWI_SR_TXRDY) {
			break;
		}
	}

	p_twi->TWI_CR = TWI_CR_STOP; /*STOP*/

	while (!(p_twi->TWI_SR & TWI_SR_TXCOMP));

	return TWI_SUCCESS;
}

/**
 * \brief Enable TWI interrupts.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_sources Interrupts to be enabled.
 */
void twi_enable_interrupt(Twi *p_twi, uint32_t ul_sources)
{
	/* Enable the specified interrupts */
	p_twi->TWI_IER = ul_sources;
}

/**
 * \brief Disable TWI interrupts.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param ul_sources Interrupts to be disabled.
 */
void twi_disable_interrupt(Twi *p_twi, uint32_t ul_sources)
{
	/* Disable the specified interrupts */
	p_twi->TWI_IDR = ul_sources;
	/* Dummy read */
	p_twi->TWI_SR;
}

/**
 * \brief Get TWI interrupt status.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \retval TWI interrupt status.
 */
uint32_t twi_get_interrupt_status(Twi *p_twi)
{
	return p_twi->TWI_SR;
}

/**
 * \brief Read TWI interrupt mask.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \return The interrupt mask value.
 */
uint32_t twi_get_interrupt_mask(Twi *p_twi)
{
	return p_twi->TWI_IMR;
}

/**
 * \brief Reads a byte from the TWI bus.
 *
 * \param p_twi Pointer to a TWI instance.
 *
 * \return The byte read.
 */
uint8_t twi_read_byte(Twi *p_twi)
{
	return p_twi->TWI_RHR;
}

/**
 * \brief Sends a byte of data to one of the TWI slaves on the bus.
 *
 * \param p_twi Pointer to a TWI instance.
 * \param byte The byte to send.
 */
void twi_write_byte(Twi *p_twi, uint8_t uc_byte)
{
	p_twi->TWI_THR = uc_byte;
}

/**
 * \brief Enable TWI slave mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_enable_slave_mode(Twi *p_twi)
{
	/* Set Master Disable bit and Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_MSDIS;
	p_twi->TWI_CR = TWI_CR_SVDIS;

	/* Set Slave Enable bit */
	p_twi->TWI_CR = TWI_CR_SVEN;
}   

/**
 * \brief Disable TWI slave mode.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_disable_slave_mode(Twi *p_twi)
{
	/* Set Slave Disable bit */
	p_twi->TWI_CR = TWI_CR_SVDIS;
}

/**
 * \brief Reset TWI.
 *
 * \param p_twi Pointer to a TWI instance.
 */
void twi_reset(Twi *p_twi)
{
	/* Set SWRST bit to reset TWI peripheral */
	p_twi->TWI_CR = TWI_CR_SWRST;
	p_twi->TWI_RHR;

}

/**
 * \brief Sends Stop (p) signal
 * 
 */
void Send_Stop(Twi* p_twi)
{
    p_twi->TWI_CR = TWI_CR_STOP;
}

uint32_t twi_master_config(Twi *p_twi, const twi_options_t *p_opt)
{
	/* SVEN: TWI Slave Mode Enabled */
    p_twi->TWI_CR = TWI_CR_SVEN ;

    twi_reset(p_twi);

    /* Set master mode */
    p_twi->TWI_CR = TWI_CR_MSEN ;

    /* Configure clock */
    twi_set_speed(p_twi, p_opt->speed, p_opt->master_clk);
}

/**
 * \brief Start read operation as master
 * 
 */
void twi_start_read( Twi *p_twi,  const twi_packet_t *p_packet)
{
	p_twi->TWI_MMR = 0;
    p_twi->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(p_packet->chip_addr) |
        (( p_packet->iaddr_length << TWI_MMR_IADRSZ_Pos ) & 
        TWI_MMR_DADR_Msk);

    p_twi->TWI_IADR = 0;
    p_twi->TWI_IADR = twi_mk_addr(p_packet->iaddr, 
                                p_packet->iaddr_length);

	p_twi->TWI_CR = TWI_CR_START; // send start
}

/**
 * \brief start write operation 
 */
void twi_start_write( Twi *p_twi,  const twi_packet_t *p_packet, uint8_t byte)
{
	p_twi->TWI_MMR = 0;
	p_twi->TWI_MMR = TWI_MMR_DADR(p_packet->chip_addr) |
			((p_packet->iaddr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

	/* Set internal address for remote chip */
	p_twi->TWI_IADR = 0;
	p_twi->TWI_IADR = twi_mk_addr(p_packet->iaddr, p_packet->iaddr_length);

	twi_write_byte(p_twi, byte);

}

uint8_t twi_ByteReceived(Twi *pTwi)
{
	return ((pTwi->TWI_SR & TWI_SR_RXRDY) == TWI_SR_RXRDY);
}


uint8_t twi_ByteSent(Twi *pTwi)
{
	return ((pTwi->TWI_SR & TWI_SR_TXRDY) == TWI_SR_TXRDY);
}

uint8_t twi_TransferComplete(Twi *pTwi)
{
    return ((pTwi->TWI_SR & TWI_SR_TXCOMP) == TWI_SR_TXCOMP);
}

uint32_t twi_GetStatus(Twi *pTwi)
{
    return pTwi->TWI_SR;
}


uint8_t twi_FailedAcknowledge(Twi *pTwi)
{
	return pTwi->TWI_SR & TWI_SR_NACK;
}

uint8_t twi_WaitTransferComplete(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = twi_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return 0;

		if (--_timeout == 0)
			return 0;
	}
	return 1;
}

uint8_t twi_WaitByteSent(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = twi_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return 0;

		if (--_timeout == 0)
			return 0;
	}

	return 1;
}

uint8_t twi_WaitByteReceived(Twi *_twi, uint32_t _timeout)
{
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = twi_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return 0;

		if (--_timeout == 0)
			return 0;
	}

	return 1;
}

uint8_t twi_STATUS_SVREAD(uint32_t status)
{
	return ((status & TWI_SR_SVREAD) == TWI_SR_SVREAD);

}

uint8_t twi_STATUS_SVACC(uint32_t status)
{
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
}

uint8_t twi_STATUS_GACC(uint32_t status)
{
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
}

uint8_t twi_STATUS_EOSACC(uint32_t status)
{
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
}

uint8_t twi_STATUS_NACK(uint32_t status)
{
	return (status & TWI_SR_NACK) == TWI_SR_NACK;

}






