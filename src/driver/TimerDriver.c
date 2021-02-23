/*
 * TimerDriver.c
 *
 * Created: 20/01/2021 13:02:48
 *  Author: mmuca
 */ 

#include "TimerDriver.h"



void tc_init(Tc *p_tc, uint32_t ul_Channel, uint32_t ul_Mode)
{
	TcChannel *tc_channel;

    tc_channel = p_tc->TC_CHANNEL + ul_Channel;

	/*  Disable TC clock. */
	tc_channel->TC_CCR = TC_CCR_CLKDIS;

	/*  Disable interrupts. */
	tc_channel->TC_IDR = 0xFFFFFFFF;

	/*  Clear status register. */
	tc_channel->TC_SR;

	/*  Set mode. */
	tc_channel->TC_CMR = ul_Mode;

}

/**
 * \brief Start the TC clock on the channel.
 *
 * \param p_tc      base address pointer
 * \param ul_channel Channel to configure
 */
void tc_start(Tc *p_tc, uint32_t ul_channel)
{
 p_tc->TC_CHANNEL[ul_channel].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}


/**
 * \brief Stop the TC clock on the channel.
 *
 * \param p_tc      base address pointer
 * \param ul_channel Channel to configure
 */
void tc_stop(Tc *p_tc, uint32_t ul_channel)
{
	p_tc->TC_CHANNEL[ul_channel].TC_CCR = TC_CCR_CLKDIS;
}

/**
 * \brief Read the counter value on the channel.
 *
 * \param p_tc       base address pointer
 * \param ul_channel Channel to read
 *
 * \return The counter value.
 */
uint32_t tc_read_cv(Tc *p_tc, uint32_t ul_channel)
{
	return p_tc->TC_CHANNEL[ul_channel].TC_CV;
}

/**
 * \brief Read TC Register A (RA) on the channel.
 *
 * \param[in] p_tc       base address pointer
 * \param[in] ul_channel Channel to read
 *
 * \return The TC Register A (RA) value.
 */
uint32_t tc_read_ra(Tc *p_tc, uint32_t ul_channel)
{
	return p_tc->TC_CHANNEL[ul_channel].TC_RA;
}

/**
 * \brief Read TC Register B (RB) on the channel.
 *
 * \param[in] p_tc       base address pointer
 * \param[in] ul_channel Channel to read
 *
 * \return The TC Register B (RB) value.
 */
uint32_t tc_read_rb(Tc *p_tc, uint32_t ul_channel)
{
	return p_tc->TC_CHANNEL[ul_channel].TC_RB;

}

/**
 * \brief Read TC Register C (RC) on the channel.
 *
 * \param[in] p_tc       base address pointer
 * \param[in] ul_channel Channel to read
 *
 * \return The Register C (RC) value.
 */
uint32_t tc_read_rc(Tc *p_tc, uint32_t ul_channel)
{
	return p_tc->TC_CHANNEL[ul_channel].TC_RC;
}

/**
 * \brief Write to TC Register A (RA) on the channel.
 *
 * \param[out] p_tc      base address pointer
 * \param[in] ul_channel Channel to write
 * \param[in] ul_value   Value to write
 */
void tc_write_ra(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value)
{
	p_tc->TC_CHANNEL[ul_channel].TC_RA = ul_value;
}


/**
 * \brief Write to TC Register B (RB) on the channel.
 *
 * \param[out] p_tc      base address pointer
 * \param[in] ul_channel Channel to write
 * \param[in] ul_value   Value to write
 */
void tc_write_rb(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value)
{
	p_tc->TC_CHANNEL[ul_channel].TC_RB = ul_value;
}


/**
 * \brief Write to TC Register C (RC) on the channel.
 *
 * \param[out] p_tc      base address pointer
 * \param[in] ul_channel Channel to write
 * \param[in] ul_value   Value to write
 */
void tc_write_rc(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value)

{
	p_tc->TC_CHANNEL[ul_channel].TC_RC = ul_value;
}


/**
 * \brief Enable the TC interrupts on the channel.
 *
 * \param p_tc   base address pointer
 * \param ul_channel Channel to configure
 * \param ul_sources Bitmask of interrupt sources
 * TC_IER_COVFS Enables the Counter Overflow Interrupt
 * TC_IER_LOVRS Enables the Load Overrun Interrupt
 * TC_IER_CPAS Enables the RA Compare Interrupt
 * TC_IER_CPBS Enables the RB Compare Interrupt
 * TC_IER_CPCS Enables the RC Compare Interrupt
 * TC_IER_LDRAS Enables the RA Load Interrupt
 * TC_IER_LDRBS Enables the RB Load Interrupt
 * TC_IER_ETRGS Enables the External Trigger Interrupt
 * 
 */
void tc_enable_interrupt(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_sources)
{
	TcChannel *tc_channel;

    tc_channel = p_tc->TC_CHANNEL + ul_channel;
	tc_channel->TC_IER = ul_sources;
}

/**
 * \brief Disable TC interrupts on the channel.
 *
 * \param[in,out] p_tc   base address pointer
 * \param[in] ul_channel Channel to configure
 * \param[in] ul_sources A bitmask of Interrupt sources
 *     TC_IDR_COVFS Disables the Counter Overflow Interrupt
 *     TC_IDR_LOVRS Disables the Load Overrun Interrupt
 *     TC_IDR_CPAS Disables the RA Compare Interrupt
 *     TC_IDR_CPBS Disables the RB Compare Interrupt
 *     TC_IDR_CPCS Disables the RC Compare Interrupt
 *     TC_IDR_LDRAS Disables the RA Load Interrupt
 *     TC_IDR_LDRBS Disables the RB Load Interrupt
 *     TC_IDR_ETRGS Disables the External Trigger Interrupt
 * 
 */
void tc_disable_interrupt(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_sources)
{
	TcChannel *tc_channel;

    tc_channel = p_tc->TC_CHANNEL + ul_channel;
	tc_channel->TC_IDR = ul_sources;
}


/**
 * \brief Read the TC interrupt mask for the channel.
 *
 * \param p_tc       base address pointer
 * \param ul_channel Channel to read
 *
 * \return The TC interrupt mask value.
 */
uint32_t tc_get_interrupt_mask(Tc *p_tc, uint32_t ul_channel)
{
	TcChannel *tc_channel;

    tc_channel = p_tc->TC_CHANNEL + ul_channel;
	return tc_channel->TC_IMR;
}


/**
 * \brief Get the current status for the TC channel.
 *
 * \param p_tc       base address pointer
 * \param ul_channel Channel number
 *
 * \return The current TC status.
 */
uint32_t tc_get_status(Tc *p_tc, uint32_t ul_channel)
{
	TcChannel *tc_channel;

    tc_channel = p_tc->TC_CHANNEL + ul_channel;
	return tc_channel->TC_SR;
}

/**
 * \brief Init channel to capture mode
 * 
 * \param us_id_TC TC peripheral ID
 */
void tc_capture_initialize( uint16_t us_id_TC )
{
	
	switch (us_id_TC)
    {
    case ID_TC0:
    pmc_enable_periph_clk(ID_TC0);
        /* Init TC to capture mode. */
	tc_init(TC0, TC_CHANNEL_CAPTURE0,
			TC_CAPTURE_TIMER_SELECTION /* Clock Selection */
			| TC_CMR_LDRA_RISING /* RA Loading: rising edge of TIOA */
			| TC_CMR_LDRB_FALLING /* RB Loading: falling edge of TIOA */
			| TC_CMR_ABETRG /* External Trigger: TIOA */
			| TC_CMR_ETRGEDG_FALLING /* External Trigger Edge: Falling edge */
	);
        break;
    case ID_TC1:
    pmc_enable_periph_clk(ID_TC1);
        /* Init TC to capture mode. */
	tc_init(TC1, TC_CHANNEL_CAPTURE0,
			TC_CAPTURE_TIMER_SELECTION /* Clock Selection */
			| TC_CMR_LDRA_RISING /* RA Loading: rising edge of TIOA */
			| TC_CMR_LDRB_FALLING /* RB Loading: falling edge of TIOA */
			| TC_CMR_ABETRG /* External Trigger: TIOA */
			| TC_CMR_ETRGEDG_FALLING /* External Trigger Edge: Falling edge */
	);
        break;
    case ID_TC2:
    pmc_enable_periph_clk(ID_TC2);
        /* Init TC to capture mode. */
	tc_init(TC2, TC_CHANNEL_CAPTURE0,
			TC_CAPTURE_TIMER_SELECTION /* Clock Selection */
			| TC_CMR_LDRA_RISING /* RA Loading: rising edge of TIOA */
			| TC_CMR_LDRB_FALLING /* RB Loading: falling edge of TIOA */
			| TC_CMR_ABETRG /* External Trigger: TIOA */
			| TC_CMR_ETRGEDG_FALLING /* External Trigger Edge: Falling edge */
	);
        break;
    default:
        break;
    }

}



/**
 * \brief Init channel to wave out mode
 * 
 * \param us_id_TC TC peripheral ID
 */
void tc_waveform_initialize(uint16_t us_id_TC, const waveconfig_t* wave,
 uint8_t config_idx)
{
	uint32_t ra, rc;

	switch (us_id_TC)
    {
    case ID_TC0:
    pmc_enable_periph_clk(ID_TC1);

	printf("\r\nIniting with %lu\r\n", wave[config_idx].ul_intclock);

    /* Init TC to waveform mode. */
	tc_init(TC0, TC_CHANNEL_WAVEFORM1,
			/* Waveform Clock Selection */
			wave[config_idx].ul_intclock // MCK / 128
			| TC_CMR_WAVE /* Waveform mode is enabled */
			| TC_CMR_ACPA_SET /* RA Compare Effect: set */
			| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
			| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);

    /* Configure waveform frequency and duty cycle. */
	rc = (CHIP_FREQ_CPU_MAX /
			divisors[wave[config_idx].ul_intclock]) /
			wave[config_idx].us_frequency;
	tc_write_rc(TC0, TC_CHANNEL_WAVEFORM1, rc);
	ra = (100 - wave[config_idx].us_dutycycle) * rc / 100;
	tc_write_ra(TC0, TC_CHANNEL_WAVEFORM1, ra);
    /* Enable TC TC_CHANNEL_WAVEFORM1. */
	tc_start(TC0, TC_CHANNEL_WAVEFORM1);
    break;

    case ID_TC1:
    pmc_enable_periph_clk(ID_TC1);

    /* Init TC to waveform mode. */
	tc_init(TC1, TC_CHANNEL_WAVEFORM1,
			/* Waveform Clock Selection */
			wave[config_idx].ul_intclock // MCK / 128
			| TC_CMR_WAVE /* Waveform mode is enabled */
			| TC_CMR_ACPA_SET /* RA Compare Effect: set */
			| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
			| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);
    /* Configure waveform frequency and duty cycle. */
	rc = (CHIP_FREQ_CPU_MAX /
			divisors[wave[config_idx].ul_intclock]) /
			wave[config_idx].us_frequency;
	tc_write_rc(TC1, TC_CHANNEL_WAVEFORM1, rc);
	ra = (100 - wave[config_idx].us_dutycycle) * rc / 100;
	tc_write_ra(TC1, TC_CHANNEL_WAVEFORM1, ra);
    /* Enable TC TC_CHANNEL_WAVEFORM1. */
	tc_start(TC1, TC_CHANNEL_WAVEFORM1);
        break;

    case ID_TC2:
    pmc_enable_periph_clk(ID_TC2);
    /* Init TC to waveform mode. */
	tc_init(TC2, TC_CHANNEL_WAVEFORM1,
			/* Waveform Clock Selection */
			wave[config_idx].ul_intclock // MCK / 128
			| TC_CMR_WAVE /* Waveform mode is enabled */
			| TC_CMR_ACPA_SET /* RA Compare Effect: set */
			| TC_CMR_ACPC_CLEAR /* RC Compare Effect: clear */
			| TC_CMR_CPCTRG /* UP mode with automatic trigger on RC Compare */
	);
    /* Configure waveform frequency and duty cycle. */
	rc = (CHIP_FREQ_CPU_MAX /
			divisors[wave[config_idx].ul_intclock]) /
			wave[config_idx].us_frequency;
	tc_write_rc(TC2, TC_CHANNEL_WAVEFORM1, rc);
	ra = (100 - wave[config_idx].us_dutycycle) * rc / 100;
	tc_write_ra(TC2, TC_CHANNEL_WAVEFORM1, ra);
    /* Enable TC TC_CHANNEL_WAVEFORM1. */
	tc_start(TC2, TC_CHANNEL_WAVEFORM1);
        break;

    default:
        break;
    }	

	printf("\r\nRc : %lu \nRa: %lu\r\n", rc, ra);
	
	printf("Start waveform: Frequency = %d Hz,Duty Cycle = %2d%%\n\r",
			wave[config_idx].us_frequency,
			wave[config_idx].us_dutycycle);
}






