#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define SDSUPPORT
#define PSTR(x) x
#define PROGMEM
#define PGM_P const char*
static inline char* strchr_P(const char*s,int c){return (char*)strchr(s,c);}
