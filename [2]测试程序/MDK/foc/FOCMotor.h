#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "main.h" 

typedef enum
{
	Type_torque,
	Type_velocity,
	Type_angle,
	Type_velocity_openloop,
	Type_angle_openloop,
} MotionControlType;


typedef enum
{
	Type_voltage, 
	Type_dc_current, 
	Type_foc_current 
} TorqueControlType;

extern TorqueControlType torque_controller;
extern MotionControlType controller;

extern float shaft_angle;
extern float electrical_angle;
extern float shaft_velocity;
extern float current_sp;
extern float shaft_velocity_sp;
extern float shaft_angle_sp;
extern DQVoltage_s voltage;
extern DQCurrent_s current;

extern float sensor_offset;
extern float zero_electric_angle;

float shaftAngle(void);
float shaftVelocity(void);
float electricalAngle(void);

#endif

