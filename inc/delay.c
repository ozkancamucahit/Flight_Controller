/*
 * delay.c
 *
 * Created: 24/12/2020 13:28:53
 *  Author: mmuca
 */ 

#include "delay.h"

__attribute__((optimize("s")))
__attribute__ ((section(".ramfunc")))
void portable_delay_cycles(unsigned long n)
{
	(void)(n);
	
	__asm (
	"loop: DMB	\n"
	"SUBS R0, R0, #1  \n"
	"BNE.N loop         "
	);
}

