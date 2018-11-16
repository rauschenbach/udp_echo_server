/******************************************************************************
 * Функции перевода дат, проверка контрольной суммы т.ж. здесь 
 * Все функции считают время от начала Эпохи (01-01-1970)
 * Все функции с маленькой буквы
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "userfunc.h"





/**
 * Печать в гексоидном виде
 */
void print_data_hex(void *data, int len)
{
    int i;
    uint8_t byte;
    

    for (i = 0; i < len; i++) {
	byte = *((uint8_t*)data + i);
	if (i % 8 == 0)
	    printf(" ");
	if (i % /*16*/32 == 0) {
	    printf("\r\n");
	}

	printf("%02X ", byte);
    }
    printf("\r\n");
}
