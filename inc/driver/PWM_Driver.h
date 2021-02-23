/*
 * PWM_Driver.h
 *
 * Created: 25/12/2020 13:32:23
 *  Author: mmuca
 */ 


#ifndef PWM_DRIVER_H_
#define PWM_DRIVER_H_

#define LOW 0
#define HIGH 1

#include "sam3x8e.h"

#define PWM_INVALID_ARGUMENT  0xFFFF

typedef uint8_t BYTE; //!< Boolean.


/** Definitions for PWM channel number */
typedef enum _pwm_ch_t {
	PWM_CHANNEL_0 = 0,
	PWM_CHANNEL_1 = 1,
	PWM_CHANNEL_2 = 2,
	PWM_CHANNEL_3 = 3,
	PWM_CHANNEL_4 = 4,
	PWM_CHANNEL_5 = 5,
	PWM_CHANNEL_6 = 6,
	PWM_CHANNEL_7 = 7
} pwm_ch_t;

/** Definitions for PWM channel alignment */
typedef enum {
	PWM_ALIGN_LEFT = (0 << 8),   /* The period is left aligned. */
	PWM_ALIGN_CENTER = (1 << 8)  /* The period is center aligned. */
} pwm_align_t;

/** Definitions for PWM level */
typedef enum {
	PWM_LOW = LOW,     /* Low level */
	PWM_HIGH = HIGH,  /* High level */
} pwm_level_t;

/** Input parameters when initializing PWM */
typedef struct {
	/** Frequency of clock A in Hz (set 0 to turn it off) */
	uint32_t ul_clka;
	/** Frequency of clock B in Hz (set 0 to turn it off) */
	uint32_t ul_clkb;
	/** Frequency of master clock in Hz */
	uint32_t ul_mck;
} pwm_clock_t;

/** Definitions for PWM event */
typedef enum {
	PWM_EVENT_PERIOD_END = (0 << 10),      /* The channel counter event occurs at the end of the PWM period. */
	PWM_EVENT_PERIOD_HALF_END = (1 << 10)  /* The channel counter event occurs at the half of the PWM period. */
} pwm_counter_event_t;

/** Definitions of PWM register group */
typedef enum {
	PWM_GROUP_CLOCK = (1 << 0),
	PWM_GROUP_DISABLE = (1 << 1),
	PWM_GROUP_MODE = (1 << 2),
	PWM_GROUP_PERIOD = (1 << 3),
	PWM_GROUP_DEAD_TIME = (1 << 4),
	PWM_GROUP_FAULT = (1 << 5)
} pwm_protect_reg_group_t;

/** Definitions for PWM comparison unit */
typedef enum {
	PWM_CMP_UNIT_0 = 0,
	PWM_CMP_UNIT_1 = 1,
	PWM_CMP_UNIT_2 = 2,
	PWM_CMP_UNIT_3 = 3,
	PWM_CMP_UNIT_4 = 4,
	PWM_CMP_UNIT_5 = 5,
	PWM_CMP_UNIT_6 = 6,
	PWM_CMP_UNIT_7 = 7
} pmc_cmp_unit_t;

/** Configurations of a PWM channel output */
typedef struct {
	/** Boolean of using override output as PWMH */
	BYTE b_override_pwmh;
	/** Boolean of using override output as PWML */
	BYTE b_override_pwml;
	/** Level of override output for PWMH */
	pwm_level_t override_level_pwmh;
	/** Level of override output for PWML */
	pwm_level_t override_level_pwml;
} pwm_output_t;

/** Configurations of PWM comparison */
typedef struct {
	/** Comparison unit number */
	uint32_t unit;
	/** Boolean of comparison enable */
	BYTE b_enable;
	/** Comparison value */
	uint32_t ul_value;
	/** Comparison mode */
	BYTE b_is_decrementing;
	/** Comparison trigger value */
	uint32_t ul_trigger;
	/** Comparison period value */
	uint32_t ul_period;
	/** Comparison update period value */
	uint32_t ul_update_period;
	/** Boolean of generating a match pulse on PWM event line 0 */
	BYTE b_pulse_on_line_0;
	/** Boolean of generating a match pulse on PWM event line 1 */
	BYTE b_pulse_on_line_1;
} pwm_cmp_t;

/** Definitions for PWM fault input ID */
typedef enum {
	PWM_FAULT_PWMFI0 = (1 << 0),
	PWM_FAULT_PWMFI1 = (1 << 1),
	PWM_FAULT_PWMFI2 = (1 << 2),
	PWM_FAULT_MAINOSC = (1 << 3),
	PWM_FAULT_ADC = (1 << 4),
	PWM_FAULT_TIMER_0 = (1 << 5),
} pwm_fault_id_t;

/** Input parameters when configuring a PWM channel mode */
typedef struct {
	/** Channel number */
	uint32_t channel;
	/** Channel prescaler */
	uint32_t ul_prescaler;
    /** Channel alignment */
	pwm_align_t alignment;
    /** Channel initial polarity */
	pwm_level_t polarity;
	/** Duty Cycle Value */
	uint32_t ul_duty;
	/** Period Cycle Value */
	uint32_t ul_period;
    /** Channel counter event */
	pwm_counter_event_t counter_event;
    /** Boolean of channel dead-time generator */
	BYTE b_deadtime_generator;
    /** Boolean of channel dead-time PWMH output inverted */
	BYTE b_pwmh_output_inverted;
    /** Boolean of channel dead-time PWML output inverted */
	BYTE b_pwml_output_inverted;
	/** Dead-time Value for PWMH Output */
	uint16_t us_deadtime_pwmh;
	/** Dead-time Value for PWML Output */
	uint16_t us_deadtime_pwml;
	/** Channel output */
	pwm_output_t output_selection;
	/** Boolean of Synchronous Channel */
	BYTE b_sync_ch;
	/** Fault ID of the channel */
	pwm_fault_id_t fault_id;
	/** Channel PWMH output level in fault protection */
	pwm_level_t ul_fault_output_pwmh;
	/** Channel PWML output level in fault protection */
	pwm_level_t ul_fault_output_pwml;
 /* (SAM3U || SAM3S || SAM3XA || SAM4S || SAM4E || SAMV70 || SAMV71 || SAME70 || SAMS70) */
} pwm_channel_t;


uint32_t pwm_init(Pwm *p_pwm, pwm_clock_t *clock_config);
uint32_t pwm_channel_init(Pwm *p_pwm, pwm_channel_t *p_channel);
uint32_t pwm_channel_update_period(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t ul_period);
uint32_t pwm_channel_update_duty(Pwm *p_pwm, pwm_channel_t *p_channel,
		uint32_t ul_duty);
uint32_t pwm_channel_get_counter(Pwm *p_pwm, pwm_channel_t *p_channel);

void pwm_channel_enable(Pwm *p_pwm, uint32_t ul_channel);
void pwm_channel_disable(Pwm *p_pwm, uint32_t ul_channel);
uint32_t pwm_channel_get_status(Pwm *p_pwm);

uint32_t pwm_channel_get_interrupt_status(Pwm *p_pwm);
uint32_t pwm_channel_get_interrupt_mask(Pwm *p_pwm);
void pwm_channel_enable_interrupt(Pwm *p_pwm, uint32_t ul_event,
		uint32_t ul_fault);
void pwm_channel_disable_interrupt(Pwm *p_pwm, uint32_t ul_event,
		uint32_t ul_fault);

void pwm_channel_update_output(Pwm *p_pwm, pwm_channel_t *p_channel,
		pwm_output_t *p_output, BYTE b_sync);



#endif /* PWM_DRIVER_H_ */