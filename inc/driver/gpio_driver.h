/*
 * gpio_driver.h
 *
 * Created: 21/12/2020 02:16:56
 *  Author: mmuca
 */ 


#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

//#include "stdint.h"
#include "sam3x8e.h"

//#include "component/pio.h"

#ifndef PIO_WPMR_WPKEY_PASSWD
#define PIO_WPMR_WPKEY_PASSWD    PIO_WPMR_WPKEY(0x50494FU)
#endif

//512
#define PIO_DELTA   ((uint32_t) PIOB - (uint32_t) PIOA)
#define PIO_TYPE_Pos                  27
/* PIO Type Mask */
#define PIO_TYPE_Msk                  (0xFu << PIO_TYPE_Pos)
/* The pin is not a function pin. */
#define PIO_TYPE_NOT_A_PIN            (0x0u << PIO_TYPE_Pos)
/* The pin is controlled by the peripheral A. */
#define PIO_TYPE_PIO_PERIPH_A         (0x1u << PIO_TYPE_Pos)
/* The pin is controlled by the peripheral B. */
#define PIO_TYPE_PIO_PERIPH_B         (0x2u << PIO_TYPE_Pos)
/* The pin is an input. */
#define PIO_TYPE_PIO_INPUT            (0x5u << PIO_TYPE_Pos)
/* The pin is an output and has a default level of 0. */
#define PIO_TYPE_PIO_OUTPUT_0         (0x6u << PIO_TYPE_Pos)
/* The pin is an output and has a default level of 1. */
#define PIO_TYPE_PIO_OUTPUT_1         (0x7u << PIO_TYPE_Pos)
/*  Default pin configuration (no attribute). */
#define PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define PIO_DEGLITCH            (1u << 1)
/*  The pin is open-drain. */
#define PIO_OPENDRAIN           (1u << 2)

/*  The internal debouncing filter is active. */
#define PIO_DEBOUNCE            (1u << 3)

/*  Enable additional interrupt modes. */
#define PIO_IT_AIME             (1u << 4)

/*  Interrupt High Level/Rising Edge detection is active. */
#define PIO_IT_RE_OR_HL         (1u << 5)
/*  Interrupt Edge detection is active. */
#define PIO_IT_EDGE             (1u << 6)

/*  Low level interrupt is active */
#define PIO_IT_LOW_LEVEL        (0               | 0 | PIO_IT_AIME)
/*  High level interrupt is active */
#define PIO_IT_HIGH_LEVEL       (PIO_IT_RE_OR_HL | 0 | PIO_IT_AIME)
/*  Falling edge interrupt is active */
#define PIO_IT_FALL_EDGE        (0               | PIO_IT_EDGE | PIO_IT_AIME)
/*  Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE        (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)

typedef enum _pio_type {
	PIO_NOT_A_PIN   = PIO_TYPE_NOT_A_PIN,
	PIO_PERIPH_A    = PIO_TYPE_PIO_PERIPH_A,
	PIO_PERIPH_B    = PIO_TYPE_PIO_PERIPH_B,
	PIO_INPUT       = PIO_TYPE_PIO_INPUT,
	PIO_OUTPUT_0    = PIO_TYPE_PIO_OUTPUT_0,
	PIO_OUTPUT_1    = PIO_TYPE_PIO_OUTPUT_1
} pio_type_t;

/**
 * \brief Enable or disable write protect of PIO registers.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_enable 1 to enable, 0 to disable.
 */
void pio_set_writeprotect(Pio *p_pio, const uint32_t ul_enable);

void pio_pull_up(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_pull_up_enable);

void pio_set(Pio *p_pio, const uint32_t ul_mask);
void pio_clear(Pio *p_pio, const uint32_t ul_mask);
void disable_pin(uint32_t pin);

Pio *pio_get_pin_group(uint32_t ul_pin);

void pio_set_input(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_attribute);
void pio_set_output(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_default_level,
		const uint32_t ul_multidrive_enable,
		const uint32_t ul_pull_up_enable);
uint32_t pio_get(Pio *p_pio, const pio_type_t ul_type,
		const uint32_t ul_mask);
uint32_t pio_get_pin_group_id(uint32_t ul_pin);
uint32_t pio_get_pin_group_mask(uint32_t ul_pin);

void pio_set_peripheral(Pio *p_pio, const pio_type_t ul_type,
		const uint32_t ul_mask);
uint32_t pio_get_output_data_status(const Pio *p_pio,
		const uint32_t ul_mask);

uint32_t pio_get_pin_value(uint32_t ul_pin);

void pio_set_multi_driver(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_multi_driver_enable);

void pio_enable_interrupt(Pio *p_pio, const uint32_t ul_mask);

void pio_disable_interrupt(Pio *p_pio, const uint32_t ul_mask);
uint32_t pio_get_interrupt_status(const Pio *p_pio);
uint32_t pio_get_interrupt_mask(const Pio *p_pio);

void pio_set_additional_interrupt_mode(Pio *p_pio,
		const uint32_t ul_mask, const uint32_t ul_attribute);

void pio_set_pin_group_high(Pio *p_pio, uint32_t ul_mask);
void pio_set_pin_group_low(Pio *p_pio, uint32_t ul_mask);
uint32_t pio_configure_pin_group(Pio *p_pio, uint32_t ul_mask,
		const uint32_t ul_flags);

uint32_t pio_configure_pin(uint32_t ul_pin, const uint32_t ul_flags);

void pio_set_pin_high(uint32_t pin);
void pio_set_pin_low(uint32_t pin);

void pio_enable_pin_interrupt(uint32_t pin);
void pio_disable_pin_interrupt(uint32_t pin);
void pio_configure_interrupt(Pio *p_pio, const uint32_t ul_mask,
		const uint32_t ul_attr);

#endif /* GPIO_DRIVER_H_ */