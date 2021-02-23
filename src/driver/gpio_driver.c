/*
 * gpio_driver.c
 *
 * Created: 21/12/2020 13:29:18
 *  Author: mmuca
 */ 

#include "gpio_driver.h"

/**
 * \brief Enable or disable write protect of PIO registers.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_enable 1 to enable, 0 to disable.
 */
void pio_set_writeprotect(Pio *p_pio, const uint32_t ul_enable)
{
	p_pio->PIO_WPMR = PIO_WPMR_WPKEY_PASSWD | (ul_enable & PIO_WPMR_WPEN);
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO.
 * \param ul_mask Bitmask
 * \param ul_pull_up_enable "0" Disable "1" Enable
 */
void pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
		                    const uint32_t ul_pull_up_enable)
    {
        
        if (ul_pull_up_enable) {
		    p_pio->PIO_PUER = ul_mask;
	    } else {
		    p_pio->PIO_PUDR = ul_mask;
	    }
    }

/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void pio_set(Pio *p_pio, const uint32_t ul_mask)
{
    p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}


uint32_t pio_get_output_data_status(const Pio *p_pio,
		const uint32_t ul_mask)
{

}
/**
 * \brief Return the value of a pin.
 *
 * \param ul_pin The pin number.
 *
 * \return The pin value.
 *
 * \note If pin is output: a pull-up or pull-down could hide the actual value.
 *       The function \ref pio_get can be called to get the actual pin output
 *       level.
 */
uint32_t pio_get_pin_value(uint32_t ul_pin)
{
	Pio *p_pio = pio_get_pin_group(ul_pin);

	return (p_pio->PIO_PDSR >> (ul_pin & 0x1F)) & 1;
}

// TODO : get pio BASE ADDR according to pin_idx
Pio *pio_get_pin_group(uint32_t ul_pin)
{
    //PIO_PB27_IDX -> 59
    //PIO_PC9_IDX -> 73
	Pio *p_pio;

	p_pio = (Pio *)((uint32_t)PIOA + (PIO_DELTA * (ul_pin >> 5)));

	return p_pio;

}

/**
 * disable pin
 * \param pin pin index
 * 
 */
void disable_pin(uint32_t pin)
{
	pio_get_pin_group(pin)->PIO_PDR = pio_get_pin_group_mask(pin);
}



/**
 * \brief Configure one or more pin(s) of a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) can be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void pio_set_input(Pio *p_pio, const uint32_t ul_mask,
		            const uint32_t ul_attribute)
{
    pio_disable_interrupt(p_pio, ul_mask);
	pio_pull_up(p_pio, ul_mask, ul_attribute & PIO_PULLUP);

	/* Enable Input Filter if necessary */
	if (ul_attribute & (PIO_DEGLITCH | PIO_DEBOUNCE)) {
		p_pio->PIO_IFER = ul_mask;
	} else {
		p_pio->PIO_IFDR = ul_mask;
	}

    /* Configure pin as input */
	p_pio->PIO_ODR = ul_mask;
	p_pio->PIO_PER = ul_mask;
}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s). "0" OR "1"
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain. "0" OR "1"
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated. "0" OR "1"
 */
void pio_set_output(Pio *p_pio, const uint32_t ul_mask,
		            const uint32_t ul_default_level,
		            const uint32_t ul_multidrive_enable,
		            const uint32_t ul_pull_up_enable)
{
    pio_disable_interrupt(p_pio, ul_mask);
	pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);

	/* Enable multi-drive if necessary */
	if (ul_multidrive_enable) {
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}

	/* Set default value */
	if (ul_default_level) {
		p_pio->PIO_SODR = ul_mask;
	} else {
		p_pio->PIO_CODR = ul_mask;
	}

	/* Configure pin(s) as output(s) */
	p_pio->PIO_OER = ul_mask;
	p_pio->PIO_PER = ul_mask;
}

/**
 * \brief Return GPIO port peripheral ID for a GPIO pin.
 *
 * \param ul_pin The pin index.
 *
 * \return GPIO port peripheral ID.
 */
uint32_t pio_get_pin_group_id(uint32_t ul_pin)
{
	uint32_t ul_id;
	ul_id = ID_PIOA + (ul_pin >> 5);
	return ul_id;
}

/**
 * \brief Return GPIO port pin mask for a GPIO pin.
 *
 * \param ul_pin The pin index.
 *
 * \return GPIO port pin mask.
 */
uint32_t pio_get_pin_group_mask(uint32_t ul_pin)
{
	uint32_t ul_mask = 1 << (ul_pin & 0x1F);
	return ul_mask;
}

/**
 * \brief Configure PIO pin multi-driver.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_multi_driver_enable Indicates if the pin(s) multi-driver shall be
 * configured.
 */
void pio_set_multi_driver(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_multi_driver_enable)
{
	/* Enable the multi-driver if necessary */
	if (ul_multi_driver_enable) {
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}
}

/**
 * \brief Enable the given interrupt source.
 * The PIO must be configured as an NVIC interrupt source as well.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Interrupt sources bit map.
 */
void pio_enable_interrupt(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_IER = ul_mask;
}

/**
 * \brief Disable a given interrupt source
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Interrupt sources bit map.
 */
void pio_disable_interrupt(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_IDR = ul_mask;

}

/**
 * \brief Drive a GPIO port to 1.
 *
 * \param p_pio Base address of the PIO port.
 * \param ul_mask Bitmask of one or more pin(s) to toggle.
 */
void pio_set_pin_group_high(Pio *p_pio, uint32_t ul_mask)
{
    p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Drive a GPIO port to 0.
 *
 * \param p_pio Base address of the PIO port.
 * \param ul_mask Bitmask of one or more pin(s) to toggle.
 */
void pio_set_pin_group_low(Pio *p_pio, uint32_t ul_mask)
{
	/* Value to be driven on the I/O line: 0. */
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Perform complete pin(s) configuration; general attributes and PIO init
 * if necessary.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_flags Pin(s) attributes.
 *
 * \return Whether the pin(s) have been configured properly.
 */
uint32_t pio_configure_pin_group(Pio *p_pio,
		uint32_t ul_mask, const uint32_t ul_flags)
{
	/* Configure pins */
	switch (ul_flags & PIO_TYPE_Msk) {
	case PIO_TYPE_PIO_PERIPH_A:
		pio_set_peripheral(p_pio, PIO_PERIPH_A, ul_mask);
		pio_pull_up(p_pio, ul_mask, (ul_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_B:
		pio_set_peripheral(p_pio, PIO_PERIPH_B, ul_mask);
		pio_pull_up(p_pio, ul_mask, (ul_flags & PIO_PULLUP));
		break;

	case PIO_TYPE_PIO_INPUT:
		pio_set_input(p_pio, ul_mask, ul_flags);
		break;

	case PIO_TYPE_PIO_OUTPUT_0:
	case PIO_TYPE_PIO_OUTPUT_1:
		pio_set_output(p_pio, ul_mask,
				((ul_flags & PIO_TYPE_PIO_OUTPUT_1)
				== PIO_TYPE_PIO_OUTPUT_1) ? 1 : 0,
				(ul_flags & PIO_OPENDRAIN) ? 1 : 0,
				(ul_flags & PIO_PULLUP) ? 1 : 0);
		break;

	default:
		return 0;
	}

	return 1;
}

/**
 * \brief Perform complete pin(s) configuration; general attributes and PIO init
 * if necessary.
 *
 * \param ul_pin The pin index.
 * \param ul_flags Pins attributes.
 *
 * \return Whether the pin(s) have been configured properly.
 */
uint32_t pio_configure_pin(uint32_t ul_pin, const uint32_t ul_flags)
{
	Pio *p_pio = pio_get_pin_group(ul_pin);

	/* Configure pins */
	switch (ul_flags & PIO_TYPE_Msk) {
	case PIO_TYPE_PIO_PERIPH_A:
		pio_set_peripheral(p_pio, PIO_PERIPH_A, (1 << (ul_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (ul_pin & 0x1F)),
				(ul_flags & PIO_PULLUP));
		break;
	case PIO_TYPE_PIO_PERIPH_B:
		pio_set_peripheral(p_pio, PIO_PERIPH_B, (1 << (ul_pin & 0x1F)));
		pio_pull_up(p_pio, (1 << (ul_pin & 0x1F)),
				(ul_flags & PIO_PULLUP));
		break;

	case PIO_TYPE_PIO_INPUT:
		pio_set_input(p_pio, (1 << (ul_pin & 0x1F)), ul_flags);
		break;

	case PIO_TYPE_PIO_OUTPUT_0:
	case PIO_TYPE_PIO_OUTPUT_1:
		pio_set_output(p_pio, (1 << (ul_pin & 0x1F)),
				((ul_flags & PIO_TYPE_PIO_OUTPUT_1)
				== PIO_TYPE_PIO_OUTPUT_1) ? 1 : 0,
				(ul_flags & PIO_OPENDRAIN) ? 1 : 0,
				(ul_flags & PIO_PULLUP) ? 1 : 0);
		break;

	default:
		return 0;
	}

	return 1;
}

/**
 * \brief Configure IO of a PIO controller as being controlled by a specific
 * peripheral.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_type PIO type.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void pio_set_peripheral(Pio *p_pio, const pio_type_t ul_type,
		const uint32_t ul_mask)
{
	uint32_t ul_sr;

	/* Disable interrupts on the pin(s) */
	p_pio->PIO_IDR = ul_mask;
	switch (ul_type) {
	case PIO_PERIPH_A:
		ul_sr = p_pio->PIO_ABSR;
		p_pio->PIO_ABSR &= (~ul_mask & ul_sr);
		break;

	case PIO_PERIPH_B:
		ul_sr = p_pio->PIO_ABSR;
		p_pio->PIO_ABSR = (ul_mask | ul_sr);
		break;

		// other types are invalid in this function
	case PIO_INPUT:
	case PIO_OUTPUT_0:
	case PIO_OUTPUT_1:
	case PIO_NOT_A_PIN:
		return;
	}

	/* Remove the pins from under the control of PIO */
	p_pio->PIO_PDR = ul_mask;
}


/**
 * \brief Set additional interrupt mode.
 *
 * \param p_pio Pointer to a PIO.
 * \param ul_mask Interrupt sources bit map.
 * \param ul_attribute Pin(s) attributes.
 */
void pio_set_additional_interrupt_mode(Pio *p_pio,
		const uint32_t ul_mask, const uint32_t ul_attribute)
{
	/* Enables additional interrupt mode if needed */
	if (ul_attribute & PIO_IT_AIME) {
		/* Enables additional interrupt mode */
		p_pio->PIO_AIMER = ul_mask;

		/* Configures the Polarity of the event detection */
		/* (Rising/Falling Edge or High/Low Level) */
		if (ul_attribute & PIO_IT_RE_OR_HL) {
			/* Rising Edge or High Level */
			p_pio->PIO_REHLSR = ul_mask;
		} else {
			/* Falling Edge or Low Level */
			p_pio->PIO_FELLSR = ul_mask;
		}

		/* Configures the type of event detection (Edge or Level) */
		if (ul_attribute & PIO_IT_EDGE) {
			/* Edge select */
			p_pio->PIO_ESR = ul_mask;
		} else {
			/* Level select */
			p_pio->PIO_LSR = ul_mask;
		}
	} else {
		/* Disable additional interrupt mode */
		p_pio->PIO_AIMDR = ul_mask;
	    }
}
/**
 * \brief Drive a GPIO pin to 1.
 *
 * \param ul_pin The pin index.
 *
 * \note pio_configure_pin must be called beforehand.
 */
void pio_set_pin_high(uint32_t pin)
{
    //PIO_PB27 -> 1<<27
    //PIO_PB27_IDX -> 59
    Pio* p_pio = pio_get_pin_group(pin);
    p_pio->PIO_SODR = 1 << (pin & 0x1F);
}

/**
 * \brief Drive a GPIO pin to 0.
 *
 * \param ul_pin The pin index.
 *
 * \note pio_configure_pin must be called before.
 */
void pio_set_pin_low(uint32_t ul_pin)
{
	Pio *p_pio = pio_get_pin_group(ul_pin);

	/* Value to be driven on the I/O line: 0. */
	p_pio->PIO_CODR = 1 << (ul_pin & 0x1F);
}

/**
 * \brief Enable interrupt for a GPIO pin.
 *
 * \param ul_pin The pin index.
 *
 * \note gpio_configure_pin must be called before.
 */
void pio_enable_pin_interrupt(uint32_t ul_pin)
{
	Pio *p_pio = pio_get_pin_group(ul_pin);

	p_pio->PIO_IER = 1 << (ul_pin & 0x1F);
}

/**
 * \brief Disable interrupt for a GPIO pin.
 *
 * \param ul_pin The pin index.
 *
 * \note gpio_configure_pin must be called before.
 */
void pio_disable_pin_interrupt(uint32_t ul_pin)
{
	Pio *p_pio = pio_get_pin_group(ul_pin);

	p_pio->PIO_IDR = 1 << (ul_pin & 0x1F);
}

/**
 * \brief Read and clear PIO interrupt status.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt status value.
 */
uint32_t pio_get_interrupt_status(const Pio *p_pio)
{
	return p_pio->PIO_ISR;
}
/**
 * \brief Read PIO interrupt mask.
 *
 * \param p_pio Pointer to a PIO instance.
 *
 * \return The interrupt mask value.
 */
uint32_t pio_get_interrupt_mask(const Pio *p_pio)
{
	return p_pio->PIO_IMR;
}

/**
 * \brief Configure the given interrupt source.
 * Interrupt can be configured to trigger on rising edge, falling edge,
 * high level, low level or simply on level change.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Interrupt source bit map.
 * \param ul_attr Interrupt source attributes.
 */
void pio_configure_interrupt(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_attr)
{
	/* Configure additional interrupt mode registers. */
	if (ul_attr & PIO_IT_AIME) {
		/* Enable additional interrupt mode. */
		p_pio->PIO_AIMER = ul_mask;

		/* If bit field of the selected pin is 1, set as
		   Rising Edge/High level detection event. */
		if (ul_attr & PIO_IT_RE_OR_HL) {
			/* Rising Edge or High Level */
			p_pio->PIO_REHLSR = ul_mask;
		} else {
			/* Falling Edge or Low Level */
			p_pio->PIO_FELLSR = ul_mask;
		}

		/* If bit field of the selected pin is 1, set as
		   edge detection source. */
		if (ul_attr & PIO_IT_EDGE) {
			/* Edge select */
			p_pio->PIO_ESR = ul_mask;
		} else {
			/* Level select */
			p_pio->PIO_LSR = ul_mask;
		}
	} else {
		/* Disable additional interrupt mode. */
		p_pio->PIO_AIMDR = ul_mask;
	}
}

