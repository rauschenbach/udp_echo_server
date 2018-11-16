#ifndef _USERFUNC_H
#define _USERFUNC_H


#include <stdio.h>
#include "globdefs.h"


int log_printf(const char *, ...);
void print_data_hex(void*, int);
void print_mac_addr(c8*, uint8_t*);
void print_ip_addr(c8*, uint8_t*);



#endif			/* userfunc.h */


