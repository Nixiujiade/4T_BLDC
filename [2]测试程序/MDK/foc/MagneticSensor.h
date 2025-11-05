
#ifndef MAGNETICSENSOR_LIB_H
#define MAGNETICSENSOR_LIB_H

#include "stm32f4xx.h"

/******************************************************************************/
uint16_t ReadAS5600(void);
uint16_t ReadAS5047P(void);
uint16_t ReadTLE5012B(void);
uint16_t ReadMA730(void);
uint16_t ReadMT6701(void);
/******************************************************************************/
#endif
