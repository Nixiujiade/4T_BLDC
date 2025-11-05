
#include "BLDCMotor.h"


extern float target;

long sensor_direction;
float voltage_power_supply;
float voltage_limit;
float voltage_sensor_align;
int  pole_pairs;
unsigned long open_loop_timestamp;
float velocity_limit;
float current_limit;

int alignSensor(void);
float velocityOpenloop(float target_velocity);
float angleOpenloop(float target_angle);

void Motor_init(void)
{
	printf("MOT: Init\r\n");
	
	if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;
	
	PID_current_q.limit = voltage_limit;
	PID_current_d.limit = voltage_limit;
	
	if(torque_controller == Type_voltage)PID_velocity.limit = voltage_limit;  
	else  PID_velocity.limit = current_limit;
	P_angle.limit = velocity_limit;      
	
}
void Motor_initFOC(float zero_electric_offset, Direction _sensor_direction)
{	
	if(zero_electric_offset!=0)
	{
    zero_electric_angle = zero_electric_offset;
    sensor_direction = _sensor_direction;
  }
	alignSensor();    
	
	angle_prev = getAngle();  
	HAL_Delay(50);
	shaft_velocity = shaftVelocity();  
	HAL_Delay(5);
	shaft_angle = shaftAngle();
	if(controller==Type_angle)target=shaft_angle;
	
	HAL_Delay(200);
}
/******************************************************************************/
int alignSensor(void)
{
	long i;
	float angle;
	float mid_angle,end_angle;
	float moved;
	
	printf("MOT: Align sensor.\r\n");
	
	if(sensor_direction == UNKNOWN) 
	{
		for(i=0; i<=500; i++)
		{
			angle = _3PI_2 + _2PI * i / 500.0f;
			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			HAL_Delay(2);
		}
		mid_angle=getAngle();
		
		for(i=500; i>=0; i--) 
		{
			angle = _3PI_2 + _2PI * i / 500.0f;
			setPhaseVoltage(voltage_sensor_align, 0,  angle);
			HAL_Delay(2);
		}
		end_angle=getAngle();
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(200);
		
		printf("mid_angle=%.4f\r\n",mid_angle);
		printf("end_angle=%.4f\r\n",end_angle);
		
		moved = fabs(mid_angle - end_angle);
		if((mid_angle == end_angle)||(moved < 0.01f)) 
		{
			printf("MOT: Failed to notice movement loop222.\r\n");
			M0_Disable;  
			return 0;
		}
		else if(mid_angle < end_angle)
		{
			printf("MOT: sensor_direction==CCW\r\n");
			sensor_direction=CCW;
		}
		else
		{
			printf("MOT: sensor_direction==CW\r\n");
			sensor_direction=CW;
		}
		
		printf("MOT: PP check: ");  
		if( fabs(moved*pole_pairs - _2PI) > 0.5f) 
		{
			printf("fail - estimated pp:");
			pole_pairs=_2PI/moved+0.5f;    
			printf("%d\r\n",pole_pairs);
		}
		else printf("OK!\r\n");
	}
	else
		printf("MOT: Skip dir calib.\r\n");
	
	if(zero_electric_angle == 0)  
	{
		setPhaseVoltage(voltage_sensor_align, 0,  _3PI_2); 
		HAL_Delay(700);
		zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction*getAngle(), pole_pairs));
		HAL_Delay(20);
		printf("MOT: Zero elec. angle:");
		printf("%.4f\r\n",zero_electric_angle);
		setPhaseVoltage(0, 0, 0);
		HAL_Delay(200);
	}
	else
		printf("MOT: Skip offset calib.\r\n");
	
	return 1;
}

void loopFOC(void)
{
	if( controller==Type_angle_openloop || controller==Type_velocity_openloop ) return;
	
	shaft_angle = shaftAngle();
	electrical_angle = electricalAngle();
	
	switch(torque_controller)
	{
		case Type_voltage: 
			break;
		case Type_dc_current:
			break;
		case Type_foc_current:
			break;
		default:
			printf("MOT: no torque control selected!\r\n");
			break;
	}
	
  setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}
/******************************************************************************/
void move(float new_target)
{
	shaft_velocity = shaftVelocity();
	
	switch(controller)
	{
		case Type_torque:
			if(torque_controller==Type_voltage)voltage.q = new_target;
		  else
				current_sp = new_target; 
			break;
		case Type_angle:
      shaft_angle_sp = new_target;
		
      shaft_velocity_sp = PIDoperator(&P_angle,(shaft_angle_sp - shaft_angle));
		
      current_sp = PIDoperator(&PID_velocity,(shaft_velocity_sp - shaft_velocity)); 
		
      if(torque_controller == Type_voltage)
			{
				voltage.q = current_sp;
        voltage.d = 0;
      }
			break;
		case Type_velocity:
			
      shaft_velocity_sp = new_target;
		
      current_sp = PIDoperator(&PID_velocity,(shaft_velocity_sp - shaft_velocity)); 
		
      if(torque_controller == Type_voltage)
			{
        voltage.q = current_sp; 
        voltage.d = 0;
      }
			break;
		case Type_velocity_openloop:
			
      shaft_velocity_sp = new_target;
      voltage.q = velocityOpenloop(shaft_velocity_sp); 
      voltage.d = 0;
			break;
		case Type_angle_openloop:
	
      shaft_angle_sp = new_target;
      voltage.q = angleOpenloop(shaft_angle_sp); 
      voltage.d = 0;
			break;
	}
}
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	
	if(Ud) 
	{
		Uout = sqrtf(Ud*Ud + Uq*Uq) / voltage_power_supply;
		
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{
		Uout = Uq / voltage_power_supply;
	
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	
	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*sinf(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*sinf(angle_el - (sector-1.0f)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Ta*1000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Tb*1000);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Tc*1000);
}
/******************************************************************************/
float velocityOpenloop(float target_velocity)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = _micros();
	Ts = (now_us - open_loop_timestamp) * 1e-6f;
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f; 
	open_loop_timestamp=now_us;  
	
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts); 

  shaft_velocity = target_velocity;
	
	Uq = voltage_sensor_align;
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
	return Uq;
}
/******************************************************************************/
float angleOpenloop(float target_angle)
{
	unsigned long now_us;
	float Ts,Uq;
	
	now_us = _micros();
	Ts = (now_us - open_loop_timestamp) * 1e-6f;
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f; 
	open_loop_timestamp=now_us;  
	
  if(fabs( target_angle - shaft_angle ) > velocity_limit*Ts)
	{
    shaft_angle += _sign(target_angle - shaft_angle) * velocity_limit * Ts;
    shaft_velocity = velocity_limit;
  }
	else
	{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }
	
	Uq = voltage_sensor_align;
	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));
	
  return Uq;
}
/******************************************************************************/


