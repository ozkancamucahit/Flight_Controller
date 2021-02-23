/*
 * PMC_Driver.c
 *
 * Created: 21/12/2020 13:29:38
 *  Author: mmuca
 */ 


#include "PMC_Driver.h"

# define MAX_PERIPH_ID    44


/**
 * \brief Enable or disable write protect of PMC registers.
 *
 * \param ul_enable "1" to enable, "0" to disable.
 */
void pmc_set_writeprotect(uint32_t ul_enable)
{
    if (ul_enable) {
		PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD | PMC_WPMR_WPEN;
	} else {
		PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD;
	}
}


/**
 * \brief Enable the specified peripheral clock.
 *
 * \note Dont shift the ID (id << 1)
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Fail
 */
uint32_t pmc_enable_periph_clk(uint32_t ul_id)
{
    if ( ul_id > MAX_PERIPH_ID )
        return 1; /* #Error */

    pmc_set_writeprotect(0);
    
    if ( ul_id < 32 )
    {
		if ((PMC->PMC_PCSR0 & (1u << ul_id)) != (1u << ul_id))
			PMC->PMC_PCER0 = 1 << ul_id;
    }   else
        {
            ul_id -= 32;
		    if ((PMC->PMC_PCSR1 & (1u << ul_id)) != (1u << ul_id)) 
			    PMC->PMC_PCER1 = 1 << ul_id;
		    
        }
    return 0;
}


/**
 * \brief Disable the specified peripheral clock.
 *
 * \note Dont shift the ID (id << 1)
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Fail
 */
uint32_t pmc_disable_periph_clk(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) 
		return 1;

    if (ul_id < 32) {
		if ((PMC->PMC_PCSR0 & (1u << ul_id)) == (1u << ul_id)) {
			PMC->PMC_PCDR0 = 1 << ul_id;
		}
    }   else {
        ul_id -= 32;
		if ((PMC->PMC_PCSR1 & (1u << ul_id)) == (1u << ul_id)) {
			PMC->PMC_PCDR1 = 1 << ul_id;
		}
    }
	
    return 0;
}

void pmc_enable_all_periph_clk(void)
{
	PMC->PMC_PCER0 = PMC_MASK_STATUS0;
	while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != PMC_MASK_STATUS0);
    
    PMC->PMC_PCER1 = PMC_MASK_STATUS1;
	while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != PMC_MASK_STATUS1);
}

void pmc_disable_all_periph_clk(void)
{
    PMC->PMC_PCDR0 = PMC_MASK_STATUS0;
	while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != 0);

    PMC->PMC_PCDR1 = PMC_MASK_STATUS1;
	while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != 0);
}

