#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct 
{
    float P;  
    float I;  
    float D;  
    float output_ramp;
    float limit;  
    float error_prev; 
    float output_prev;   
    float integral_prev; 
    unsigned long timestamp_prev;  
} PIDController;

extern PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;


void PID_init(void);
float PIDoperator(PIDController* PID,float error);


#endif

