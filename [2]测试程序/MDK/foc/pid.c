
#include "pid.h"

PIDController  PID_current_q,PID_current_d,PID_velocity,P_angle;

void PID_init(void)
{
	PID_velocity.P=0.5f;  //0.5
	PID_velocity.I=0;     //0.5
	PID_velocity.D=0;
	PID_velocity.output_ramp=0;    //限制转速变化速率，
	//PID_velocity.limit=0;        //Motor_init()函数已经对limit初始化，此处无需处理
	PID_velocity.error_prev=0;
	PID_velocity.output_prev=0;
	PID_velocity.integral_prev=0;
	PID_velocity.timestamp_prev=_micros();
	
	P_angle.P=3;
	P_angle.I=0;
	P_angle.D=0;
	P_angle.output_ramp=0;
	//P_angle.limit=0;
	P_angle.error_prev=0;
	P_angle.output_prev=0;
	P_angle.integral_prev=0;
	P_angle.timestamp_prev=_micros();
	
	PID_current_q.P=0.5f;  //航模电机，速度闭环，不能大于1，否则容易失控
	PID_current_q.I=0;    //电流环I参数不太好调试，只用P参数也可以
	PID_current_q.D=0;
	PID_current_q.output_ramp=0;
	//PID_current_q.limit=0;
	PID_current_q.error_prev=0;
	PID_current_q.output_prev=0;
	PID_current_q.integral_prev=0;
	PID_current_q.timestamp_prev=_micros();
	
	PID_current_d.P=0.5f;  //0.5
	PID_current_d.I=0;
	PID_current_d.D=0;
	PID_current_d.output_ramp=0;
	//PID_current_d.limit=0;
	PID_current_d.error_prev=0;
	PID_current_d.output_prev=0;
	PID_current_d.integral_prev=0;
	PID_current_d.timestamp_prev=_micros();
}


float PIDoperator(PIDController* PID,float error)
{
	unsigned long timestamp_now;
	float Ts;
	float proportional,integral,derivative,output;
	float output_rate;
	
	timestamp_now = _micros();
	Ts = (timestamp_now - PID->timestamp_prev) * 1e-6f;
	PID->timestamp_prev = timestamp_now;
	if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
	
	proportional = PID->P * error;
	
	integral = PID->integral_prev + PID->I*Ts*0.5f*(error + PID->error_prev);
	
	integral = _constrain(integral, -PID->limit, PID->limit);
	
	derivative = PID->D*(error - PID->error_prev)/Ts;
	
	output = proportional + integral + derivative;
	
	output = _constrain(output, -PID->limit, PID->limit);
	
	if(PID->output_ramp > 0)
	{
		
		output_rate = (output - PID->output_prev)/Ts;
		if(output_rate > PID->output_ramp)output = PID->output_prev + PID->output_ramp*Ts;
		else if(output_rate < -PID->output_ramp)output = PID->output_prev - PID->output_ramp*Ts;
	}
	
	PID->integral_prev = integral;
	PID->output_prev = output;
	PID->error_prev = error;
	
	return output;
}

