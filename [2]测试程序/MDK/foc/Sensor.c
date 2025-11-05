#include "sensor.h"

long  cpr;
long  velocity_calc_timestamp;  
long  angle_data_prev;         
float angle_prev;               
float full_rotation_offset;  

uint16_t getRawCount(void)  
{
	return bsp_as5600GetRawAngle()&0x0FFF;
} 


void MagneticSensor_Init(void)
{
	cpr = 4096;
	
	angle_data_prev = getRawCount();
	full_rotation_offset = 0;
	velocity_calc_timestamp=_micros();
	angle_prev = getAngle();
}

float getAngle(void)
{
	long angle_data,d_angle;
	
	angle_data = getRawCount();
	
	d_angle = angle_data - angle_data_prev; 
	
	if(abs(d_angle) > (0.8f*cpr) ) full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI; 
	
	angle_data_prev = angle_data;
	
	
	if(full_rotation_offset >= ( _2PI*2000)) 
	{                                        
		full_rotation_offset = 0;              
		angle_prev = angle_prev - _2PI*2000;
	}
	if(full_rotation_offset <= (-_2PI*2000))
	{
		full_rotation_offset =  0;
		angle_prev = angle_prev + _2PI*2000;
	}
	
	return (full_rotation_offset + ( (float)angle_data / cpr * _2PI));
}

float getVelocity(void)
{
	long now_us;
	float Ts, angle_now, vel;

	
	now_us = _micros();
	Ts = (now_us - velocity_calc_timestamp) * 1e-6f;
	if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

	angle_now = getAngle();
	vel = (angle_now - angle_prev)/Ts;

	angle_prev = angle_now;
	velocity_calc_timestamp = now_us;
	return vel;
}



