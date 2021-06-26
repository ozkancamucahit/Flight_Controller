/*
 * write.c
 *
 * Created: 01/01/2021 20:57:07
 *  Author: mmuca
 */ 

#include "sam3x8e.h"
#include "UART_Driver.h"
#include "UART_Serial.h"

volatile void *volatile stdio_base;
int (*ptr_put)(void volatile*, char);


int 
_write (int file, const char *ptr, int len);

int 
_write (int file, const char *ptr, int len)
{
	int nChars = 0;

	if ((file != 1) && (file != 2) && (file!=3)) {
		return -1;
	}

	for (; len != 0; --len) {
		if (ptr_put(stdio_base, *ptr++) < 0) {
			return -1;
		}
	++nChars;
	}
return nChars;
}