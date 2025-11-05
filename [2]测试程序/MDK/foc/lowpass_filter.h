#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include "main.h"

typedef struct 
{
	float Tf; 
	float y_prev;
	unsigned long timestamp_prev;  
} LowPassFilter;

extern LowPassFilter  LPF_current_q,LPF_current_d,LPF_velocity;

void LPF_init(void);
float LPFoperator(LowPassFilter* LPF,float x);


#endif

