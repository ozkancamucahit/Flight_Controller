/*
 * TWI_driver.h
 *
 * Created: 27/12/2020 18:31:28
 *  Author: mmuca
 */ 


#ifndef TWI_DRIVER_H_
#define TWI_DRIVER_H_

#include "sam3x8e.h"

#define TWI_TIMEOUT     30000


/** 
* \brief Return Values
*/
#define TWI_SUCCESS              0
#define TWI_INVALID_ARGUMENT     1
#define TWI_ARBITRATION_LOST     2
#define TWI_NO_CHIP_FOUND        3
#define TWI_RECEIVE_OVERRUN      4
#define TWI_RECEIVE_NACK         5
#define TWI_SEND_OVERRUN         6
#define TWI_SEND_NACK            7
#define TWI_BUSY                 8
#define TWI_ERROR_TIMEOUT        9
/*----------------------------------------------------------------------------
 *        Macros
 *----------------------------------------------------------------------------*/
/* Returns 1 if the TXRDY bit (ready to transmit data) is set in the given status register value.*/
#define TWI_STATUS_TXRDY(status) (((status) & TWI_SR_TXRDY) == TWI_SR_TXRDY)

/* Returns 1 if the RXRDY bit (ready to receive data) is set in the given status register value.*/
#define TWI_STATUS_RXRDY(status) (((status) & TWI_SR_RXRDY) == TWI_SR_RXRDY)

/* Returns 1 if the TXCOMP bit (transfer complete) is set in the given status register value.*/
#define TWI_STATUS_TXCOMP(status) (((status) & TWI_SR_TXCOMP) == TWI_SR_TXCOMP)

#define FAIL 0
#define PASS 1



/* Low level time limit of I2C Fast Mode. */
#define LOW_LEVEL_TIME_LIMIT 384000
#define I2C_FAST_MODE_SPEED  400000
#define TWI_CLK_DIVIDER      2
#define TWI_CLK_CALC_ARGU    4
#define TWI_CLK_DIV_MAX      0xFF
#define TWI_CLK_DIV_MIN      7

/** 
* \brief INIT params
*/
typedef struct twi_options {
	//! MCK for TWI.
	uint32_t master_clk;
	//! The baud rate of the TWI bus.
	uint32_t speed;
	//! The desired address.
	uint8_t chip;
	//! SMBUS mode (set 1 to use SMBUS quick command, otherwise don't).
	uint8_t smbus;
} twi_options_t;

/**
 * \brief Transmission Params.
 */
typedef struct twi_packet {
	//! TWI address/ to issue to the other chip (node).
	uint8_t iaddr[3];
	//! Length of the TWI data address segment (1-3 bytes).
	uint32_t iaddr_length;
	//! Where to find the data to be transferred.
	uint8_t *buffer;
	//! How many bytes do we want to transfer.
	uint32_t length;
	//! TWI chip address to communicate with.
	uint8_t chip_addr;
} twi_packet_t;

// TWI state
typedef	enum TwoWireStatus {
		UNINITIALIZED,
		MASTER_IDLE,
		MASTER_SEND,
		MASTER_RECV,
		SLAVE_IDLE,
		SLAVE_RECV,
		SLAVE_SEND
	}TwoWireStatus_t;
	
TwoWireStatus_t status;

void twi_enable_master_mode(Twi *p_twi);
void twi_disable_master_mode(Twi *p_twi);

uint32_t twi_set_speed(Twi *p_twi, uint32_t ul_speed, uint32_t ul_mck);

uint32_t twi_probe(Twi *p_twi, uint8_t uc_slave_addr);
uint32_t twi_mk_addr(const uint8_t *addr, int len);

uint32_t twi_master_init(Twi *p_twi, const twi_options_t *p_opt);
uint32_t twi_master_read(Twi *p_twi, twi_packet_t *p_packet);
uint32_t twi_master_write(Twi *p_twi, twi_packet_t *p_packet);
void	 twi_start_read( Twi *p_twi,  const twi_packet_t *p_packet);
void	 twi_start_write( Twi *p_twi,  const twi_packet_t *p_packet, uint8_t byte);


uint32_t twi_master_config(Twi *p_twi, const twi_options_t *p_opt);

void twi_enable_interrupt(Twi *p_twi, uint32_t ul_sources);
void twi_disable_interrupt(Twi *p_twi, uint32_t ul_sources);

uint32_t twi_get_interrupt_status(Twi *p_twi);
uint32_t twi_get_interrupt_mask(Twi *p_twi);

uint8_t twi_read_byte(Twi *p_twi);
void twi_write_byte(Twi *p_twi, uint8_t uc_byte);

void twi_enable_slave_mode(Twi *p_twi);
void twi_disable_slave_mode(Twi *p_twi);

void twi_reset(Twi *p_twi);

void Send_Stop(Twi* p_twi);

uint8_t twi_ByteReceived(Twi *pTwi);
uint8_t twi_ByteSent(Twi *pTwi);
uint8_t TWI_TransferComplete(Twi *pTwi);

uint32_t twi_GetStatus(Twi *pTwi);

uint8_t twi_FailedAcknowledge(Twi *pTwi);
uint8_t twi_WaitTransferComplete(Twi *_twi, uint32_t _timeout);
uint8_t twi_WaitByteSent(Twi *_twi, uint32_t _timeout);
uint8_t twi_WaitByteReceived(Twi *_twi, uint32_t _timeout);

uint8_t twi_STATUS_SVREAD(uint32_t status);
uint8_t twi_STATUS_SVACC(uint32_t status);
uint8_t twi_STATUS_GACC(uint32_t status);
uint8_t twi_STATUS_EOSACC(uint32_t status);
uint8_t twi_STATUS_NACK(uint32_t status);

#endif /* TWI_DRIVER_H_ */