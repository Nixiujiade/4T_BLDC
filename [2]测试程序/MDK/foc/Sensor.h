
#ifndef SENSOR_LIB_H
#define SENSOR_LIB_H

#include "main.h"

extern  long  cpr;
extern  float angle_prev;

void MagneticSensor_Init(void);
float getAngle(void);
float getVelocity(void);


#endif


