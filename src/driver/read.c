/*
 * read.c
 *
 * Created: 01/01/2021 20:57:19
 *  Author: mmuca
 */ 

#include "sam3x8e.h"
#include "UART_Driver.h"
#include "UART_Serial.h"

extern volatile void *volatile stdio_base;
void (*ptr_get)(void volatile*, char*);



int _read (int file, char * ptr, int len); // Remove GCC compiler warning

int _read (int file, char * ptr, int len)
{
	int nChars = 0;

	if (file != 0) {
		return -1;
	}
	for (; len > 0; --len) {
		ptr_get(stdio_base, ptr);
		ptr++;
		nChars++;
	}

	return nChars;
}







