

#ifndef MYPROJECT_H
#define MYPROJECT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx.h"

#include <stdlib.h>
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "i2c.h"

#include "foc_utils.h"
#include "as5600.h"
#include "Sensor.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "lowpass_filter.h"
#include "pid.h"

#define M0_Enable    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
#define M0_Disable   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);      



#endif

