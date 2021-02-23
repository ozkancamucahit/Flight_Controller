/*
 * PMC_Driver.h
 *
 * Created: 21/12/2020 13:28:30
 *  Author: mmuca
 */ 


#ifndef PMC_DRIVER_H_
#define PMC_DRIVER_H_


#include "sam3x8e.h"

//#define PMC_WPMR_WPKEY_PASSWD;

/** Bit mask for all peripherals enabled (PCER0) */
#define PMC_MASK_STATUS0        (0xFFFFFFFC)
#define PMC_MASK_STATUS1        (0xFFFFFFFF)


void pmc_set_writeprotect(uint32_t ul_enable);



/**
 * \name Peripherals clock configuration
 *
 */
//@{

uint32_t pmc_enable_periph_clk(uint32_t ul_id);
uint32_t pmc_disable_periph_clk(uint32_t ul_id);
void pmc_enable_all_periph_clk(void);
void pmc_disable_all_periph_clk(void);

/**
 * \name Interrupt and status management
 *
 */
//@{

void pmc_enable_interrupt(uint32_t ul_sources);
void pmc_disable_interrupt(uint32_t ul_sources);
uint32_t pmc_get_interrupt_mask(void);
uint32_t pmc_get_status(void);

/**
 * \name Power management
 *
 * The following functions are used to configure sleep mode and additional
 * wake up inputs.
 */
//@{

void pmc_set_fast_startup_input(uint32_t ul_inputs);
void pmc_clr_fast_startup_input(uint32_t ul_inputs);
void pmc_enable_sleepmode(uint8_t uc_type);
void pmc_enable_waitmode(void);
//@}



#endif /* PMC_DRIVER_H_ */