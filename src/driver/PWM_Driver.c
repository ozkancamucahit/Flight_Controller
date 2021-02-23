/*
 * PWM_Driver.c
 *
 * Created: 25/12/2020 13:32:38
 *  Author: mmuca
 */ 

#include "PWM_Driver.h"

#ifndef PWM_WPCR_WPKEY_PASSWD
#  define PWM_WPCR_WPKEY_PASSWD 0x50574D00
#endif

#ifndef PWM_WPCR_WPCMD_DISABLE_SW_PROT
#  define PWM_WPCR_WPCMD_DISABLE_SW_PROT (PWM_WPCR_WPCMD(0))
#endif

#ifndef PWM_WPCR_WPCMD_ENABLE_SW_PROT
#  define PWM_WPCR_WPCMD_ENABLE_SW_PROT (PWM_WPCR_WPCMD(1))
#endif

#ifndef PWM_WPCR_WPCMD_ENABLE_HW_PROT
#  define PWM_WPCR_WPCMD_ENABLE_HW_PROT (PWM_WPCR_WPCMD(2))
#endif

#define PWM_CLOCK_DIV_MAX  256
#define PWM_CLOCK_PRE_MAX  11

/**
 * \brief Find a prescaler/divisor couple to generate the desired ul_frequency
 * from ul_mck.
 *
 * \param ul_frequency Desired frequency in Hz.
 * \param ul_mck Master clock frequency in Hz.
 *
 * \retval Return the value to be set in the PWM Clock Register (PWM Mode Register for
 * SAM3N/SAM4N/SAM4C/SAM4CP/SAM4CM) or PWM_INVALID_ARGUMENT if the configuration cannot be met.
 */
static uint32_t pwm_clocks_generate(uint32_t ul_frequency, uint32_t ul_mck)
{
	uint32_t ul_divisors[PWM_CLOCK_PRE_MAX] =
			{1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024 };
	uint32_t ul_pre = 0;
	uint32_t ul_div;

	/* Find prescaler and divisor values */
	do {
		ul_div = (ul_mck / ul_divisors[ul_pre]) / ul_frequency;
		if (ul_div <= PWM_CLOCK_DIV_MAX) {
			break;
		}
		ul_pre++;
	} while (ul_pre < PWM_CLOCK_PRE_MAX);

	/* Return result */
	if (ul_pre < PWM_CLOCK_PRE_MAX) {
		return ul_div | (ul_pre << 8);
	} else {
		return PWM_INVALID_ARGUMENT;
	}
}

/**
 * \brief Initialize the PWM source clock (clock A and clock B).
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param clock_config PWM clock configuration.
 *
 * \retval 0 if initialization succeeds, otherwise fails.
 */
uint32_t pwm_init(Pwm *p_pwm, pwm_clock_t *clock_config)
{
    uint32_t clock = 0;
	uint32_t result;

	/* Clock A */
	if (clock_config->ul_clka != 0) {
		result = pwm_clocks_generate(clock_config->ul_clka, clock_config->ul_mck);
		if (result == PWM_INVALID_ARGUMENT) 
			return result; /* INVALID */
		

		clock = result;
	}
    	/* Clock B */
	if (clock_config->ul_clkb != 0) {
		result = pwm_clocks_generate(clock_config->ul_clkb, clock_config->ul_mck);

		if (result == PWM_INVALID_ARGUMENT) 
			return result;  /* INVALID */
		

		clock |= (result << 16);
	}

    p_pwm->PWM_CLK = clock;
	return 0;

}

/**
 * \brief Initialize one PWM channel.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 *
 * \retval 0 if initialization succeeds, otherwise fails.
 */
uint32_t pwm_channel_init(Pwm *p_pwm, pwm_channel_t *p_channel)
{
	uint32_t tmp_reg = 0;
	uint32_t ch_num = p_channel->channel;

	/* Channel Mode/Clock Register */
	tmp_reg = (p_channel->ul_prescaler & 0xF) |
			(p_channel->polarity << 9) |
			(p_channel->counter_event) |
			(p_channel->b_deadtime_generator << 16) |
			(p_channel->b_pwmh_output_inverted << 17) |
			(p_channel->b_pwml_output_inverted << 18) |
			(p_channel->alignment);
	p_pwm->PWM_CH_NUM[ch_num].PWM_CMR = tmp_reg;

	/* Channel Duty Cycle Register */
	p_pwm->PWM_CH_NUM[ch_num].PWM_CDTY = p_channel->ul_duty;

	/* Channel Period Register */
	p_pwm->PWM_CH_NUM[ch_num].PWM_CPRD = p_channel->ul_period;
	
	/* Channel Dead Time Register */
	if (p_channel->b_deadtime_generator) {
		p_pwm->PWM_CH_NUM[ch_num].PWM_DT =
				PWM_DT_DTL(p_channel->
				us_deadtime_pwml) | PWM_DT_DTH(p_channel->
				us_deadtime_pwmh);
	}

	/* Output Selection Register */
	tmp_reg  = p_pwm->PWM_OS & (~((PWM_OS_OSH0 | PWM_OS_OSL0) << ch_num));
	tmp_reg |= ((p_channel->output_selection.b_override_pwmh) << ch_num) |
			(((p_channel->output_selection.b_override_pwml) << ch_num)
					<< 16);
	p_pwm->PWM_OS = tmp_reg;

	/* Output Override Value Register */
	tmp_reg  = p_pwm->PWM_OOV & (~((PWM_OOV_OOVH0 | PWM_OOV_OOVL0) << ch_num));
	tmp_reg |= ((p_channel->output_selection.override_level_pwmh) << ch_num) |
			(((p_channel->output_selection.override_level_pwml) << ch_num)
					<< 16);
	p_pwm->PWM_OOV = tmp_reg;

	/* Sync Channels Mode Register */
	uint32_t channel = (1 << ch_num);
	if (p_channel->b_sync_ch) {
		p_pwm->PWM_SCM |= channel;
	} else {
		p_pwm->PWM_SCM &= ~((uint32_t) channel);
	}

	if (p_channel->ul_fault_output_pwmh == PWM_HIGH) {
		p_pwm->PWM_FPV |= (0x01 << ch_num);
	} else {
		p_pwm->PWM_FPV &= (~(0x01 << ch_num));
	}
	if (p_channel->ul_fault_output_pwml == PWM_HIGH) {
		p_pwm->PWM_FPV |= ((0x01 << ch_num) << 16);
	} else {
		p_pwm->PWM_FPV &= (~((0x01 << ch_num) << 16));
	}
	/* Fault Protection Enable Register */
	uint32_t fault_enable_reg = 0;
	if (ch_num < 4) {
		ch_num *= 8;
		fault_enable_reg = p_pwm->PWM_FPE1;
		fault_enable_reg &= ~(0xFF << ch_num);
		fault_enable_reg |= ((p_channel->fault_id) << ch_num);
		p_pwm->PWM_FPE1 = fault_enable_reg;
	} else {
		ch_num -= 4;
		ch_num *= 8;
		fault_enable_reg = p_pwm->PWM_FPE2;
		fault_enable_reg &= ~(0xFF << ch_num);
		fault_enable_reg |= ((p_channel->fault_id) << ch_num);
		p_pwm->PWM_FPE2 = fault_enable_reg;
	}

	ch_num = p_channel->channel;

	return 0;
}

/**
 * \brief Change the period of the PWM channel.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 * \param ul_period New period value.
 *
 * \retval 0 if changing succeeds, otherwise fails.
 */
uint32_t pwm_channel_update_period(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t ul_period)
{
	uint32_t ch_num = p_channel->channel;

	/** Check parameter */
	if (p_channel->ul_duty > ul_period) {
		return PWM_INVALID_ARGUMENT;
	} else {
		/* Save new period value */
		p_channel->ul_period = ul_period;
		p_pwm->PWM_CH_NUM[ch_num].PWM_CPRDUPD = ul_period;
	}

	return 0;
}

/**
 * \brief Change the duty cycle of the PWM channel.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 * \param ul_duty New duty cycle value.
 *
 * \retval 0 if changing succeeds, otherwise fails.
 */
uint32_t pwm_channel_update_duty(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t ul_duty)
{
	uint32_t ch_num = p_channel->channel;

		/** Check parameter */
	if (p_channel->ul_period < ul_duty) {
		return PWM_INVALID_ARGUMENT;
	} else {
		/* Save new duty cycle value */
		p_channel->ul_duty = ul_duty;
		p_pwm->PWM_CH_NUM[ch_num].PWM_CDTYUPD = ul_duty;
	}

	return 0;
}

/**
 * \brief Return channel counter value.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 *
 * \return channel Counter value.
 */
uint32_t pwm_channel_get_counter(Pwm *p_pwm, pwm_channel_t *p_channel)
{
	return p_pwm->PWM_CH_NUM[p_channel->channel].PWM_CCNT;
}

/**
 * \brief Enable the PWM channel.
 *
 * \note The PWM channel should be initialized by pwm_channel_init() before it is enabled.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param ul_channel PWM channel number to enable.
 */
void pwm_channel_enable(Pwm *p_pwm, uint32_t ul_channel)
{
	p_pwm->PWM_ENA = (1 << ul_channel);
}

/**
 * \brief Disable the PWM channel.
 *
 * \note A disabled PWM channel can be re-initialized using pwm_channel_init().
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param ul_channel PWM channel number to disable.
 */
void pwm_channel_disable(Pwm *p_pwm, uint32_t ul_channel)
{
	p_pwm->PWM_DIS = (1 << ul_channel);
}

/**
 * \brief Check which PWM channel is enabled.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 *
 * \return Bitmask of enabled PWM channel(s).
 */
uint32_t pwm_channel_get_status(Pwm *p_pwm)
{
	return p_pwm->PWM_SR;
}

/**
 * \brief Get channel counter event and fault protection trigger interrupt status.
 *
 * \param p_pwm Pointer to a PWM instance.
 *
 * \return Channel counter event and fault protection trigger interrupt status.
 */
uint32_t pwm_channel_get_interrupt_status(Pwm *p_pwm)
{
	return p_pwm->PWM_ISR1;
}

/**
 * \brief Get channel counter event and fault protection trigger interrupt mask.
 *
 * \param p_pwm Pointer to a PWM instance.
 *
 * \return Channel counter event and fault protection trigger interrupt mask.
 */
uint32_t pwm_channel_get_interrupt_mask(Pwm *p_pwm)
{
	return p_pwm->PWM_IMR1;
}

/**
 * \brief Enable the interrupt of a channel counter event and fault protection.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param ul_event Channel number to enable counter event interrupt.
 * \param ul_fault Channel number to enable fault protection interrupt
 */
void pwm_channel_enable_interrupt(Pwm *p_pwm, uint32_t ul_event,
		uint32_t ul_fault)
{
	p_pwm->PWM_IER1 = (1 << ul_event) | (1 << (ul_fault + 16));
}

/**
 * \brief Disable the interrupt of a channel counter event and fault protection.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param ul_event Bitmask of channel number to disable counter event interrupt.
 * \param ul_fault Bitmask of channel number to disable fault protection
 * interrupt(ignored by SAM3N/SAM4N/SAM4C/SAM4CP/SAM4CM).
 */
void pwm_channel_disable_interrupt(Pwm *p_pwm, uint32_t ul_event,
		uint32_t ul_fault)
{
	p_pwm->PWM_IDR1 = (1 << ul_event) | (1 << (ul_fault + 16));
}

/**
 * \brief Change output selection of the PWM channel.
 *
 * \param p_pwm Pointer to a PWM instance.
 * \param p_channel Configurations of the specified PWM channel.
 * \param p_output New PWM channel output selection.
 * \param b_sync Boolean of changing of output selection. Set true to change the output synchronously (at the beginning of the next PWM period). Set false to change the output asynchronously (at the end of the execution of the function).
 */
void pwm_channel_update_output(Pwm *p_pwm, pwm_channel_t *p_channel,
		pwm_output_t *p_output, BYTE b_sync)
{
	uint32_t ch_num = p_channel->channel;

	BYTE override_pwmh = p_output->b_override_pwmh;
	BYTE override_pwml = p_output->b_override_pwml;
	uint32_t pwmh = p_output->override_level_pwmh;
	uint32_t pwml = p_output->override_level_pwml;

	/* Save new output configuration */
	p_channel->output_selection.b_override_pwmh = override_pwmh;
	p_channel->output_selection.b_override_pwml = override_pwml;
	p_channel->output_selection.override_level_pwmh = (pwm_level_t) pwmh;
	p_channel->output_selection.override_level_pwml = (pwm_level_t) pwml;

	/* Change override output level */
	uint32_t override_value = p_pwm->PWM_OOV;
	override_value &= ~((PWM_OOV_OOVH0 | PWM_OOV_OOVL0) << ch_num);
	override_value |= (((pwml << 16) | pwmh) << ch_num);
	p_pwm->PWM_OOV = override_value;

	/* Apply new output selection */
	if (b_sync) {
		p_pwm->PWM_OSSUPD = ((override_pwml << ch_num) << 16) |
			(override_pwmh << ch_num);
		p_pwm->PWM_OSCUPD = ((!override_pwml << ch_num) << 16) |
			(!override_pwmh << ch_num);
	} else {
		p_pwm->PWM_OSS = ((override_pwml << ch_num) << 16) |
			(override_pwmh << ch_num);
		p_pwm->PWM_OSC = ((!override_pwml << ch_num) << 16) |
			(!override_pwmh << ch_num);
	}
}

























