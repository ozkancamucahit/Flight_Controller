/*
 * TimerDriver.h
 *
 * Created: 20/01/2021 13:02:24
 *  Author: mmuca
 */ 


#ifndef TIMERDRIVER_H_
#define TIMERDRIVER_H_

#include "sam3x8e.h"
#include "PMC_Driver.h"
#include <stdio.h>

#ifndef TC_WPMR_WPKEY_PASSWD
#define TC_WPMR_WPKEY_PASSWD TC_WPMR_WPKEY((uint32_t)0x54494D)
#endif

#define TC_CHANNEL_CAPTURE0 0 /* Channel 0 capture */
#define TC_CHANNEL_CAPTURE1 1 /* Channel 1 capture */
#define TC_CHANNEL_CAPTURE2 2 /* Channel 2 capture */

#define TC_CHANNEL_WAVEFORM0 0 /* Channel 0 Wave */
#define TC_CHANNEL_WAVEFORM1 1 /* Channel 1 Wave */
#define TC_CHANNEL_WAVEFORM2 2 /* Channel 2 Wave */

typedef struct waveconfig {
	/** Internal clock signals selection. */
	uint32_t ul_intclock;
	/** Waveform frequency (in Hz). */
	uint16_t us_frequency;
	/** Duty cycle in percent (positive).*/
	uint16_t us_dutycycle;
}waveconfig_t;

static const uint32_t divisors[5] = { 2, 8, 32, 128, 0};
/* (MCK / 32) */
#define TC_CAPTURE_TIMER_SELECTION TC_CMR_TCCLKS_TIMER_CLOCK3


void tc_init(Tc *p_tc, uint32_t ul_Channel, uint32_t ul_Mode);

void tc_start(Tc *p_tc, uint32_t ul_channel);
void tc_stop(Tc *p_tc, uint32_t ul_channel);

uint32_t tc_read_cv(Tc *p_tc, uint32_t ul_channel);
uint32_t tc_read_ra(Tc *p_tc, uint32_t ul_channel);
uint32_t tc_read_rb(Tc *p_tc, uint32_t ul_channel);
uint32_t tc_read_rc(Tc *p_tc, uint32_t ul_channel);

void tc_write_ra(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value);
void tc_write_rb(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value);
void tc_write_rc(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_value);

void tc_enable_interrupt(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_sources);
void tc_disable_interrupt(Tc *p_tc, uint32_t ul_channel,
		uint32_t ul_sources);

uint32_t tc_get_interrupt_mask(Tc *p_tc, uint32_t ul_channel);
uint32_t tc_get_status(Tc *p_tc, uint32_t ul_channel);

void tc_capture_initialize( uint16_t us_id_TC );
void tc_waveform_initialize(uint16_t id, const waveconfig_t* wave, 
uint8_t config_idx);



#endif /* TIMERDRIVER_H_ */