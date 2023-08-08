/*
 * Parameters.h
 *
 * Created: 5/20/2023 10:21:51 AM
 *  Author: Dell
 */ 


#ifndef PARAMETERS_H_
#define PARAMETERS_H_


#define TS 0.01
/************************************************************************/
/* PID velocity control                                                                     */
/************************************************************************/
f32 VELOCITY_KP = 0.27;
f32 VELOCITY_KD  = 0.01;
f32 VELOCITY_KI  = 0.5;


/************************************************************************/
/* PID Position Control                                                                     */
/************************************************************************/
#define POSITION_KP 0.12
#define POSITION_KI 3.5
#define POSITION_KD 0.2


s16 Vmax = 12;
s16 Vmin = -12;
f32 v = 0.0;

s16 carPosition  = 0;
s16 carVelocity  = 0;
u8  carVelocityIn_M_per_Sec = 0;
u8 carDir = 'b';
s16 velovityDerivative = 0.0;
s16 velocityIntegrator = 0.0;
s16 velocityError = 0.0;
s16 velocityPrevError = 0.0;

s16 velocityControlSignal=0;
s16 velocityPrevControlSignal = 0;
u16 f_control_signal=0;
s16 previous_control_signal=0;
f32 previous_filter_control_signal;
f32 prev_target_speed = 0;
s16 targetSpeed = 200;     //target
s16 targetPosition = -10;  //target
u8  steeringDir = 'R'; //target

//f32 readings_[numAverage]={0};
f32 avgFilterReadings[15] = {0} ;

u8 index = 0;
////////////////

f32 positionDerivative = 0.0;
f32 positionIntegrator = 0.0;
f32 positionError = 0.0;
f32 positionPrevError = 0.0;
f32 orevInte = 0.0;
f32 positonControlSignal=0.0;

f32 positonPrevControlSignal=0.0;
f32 filter_position_control_signal;
f32 previous_filter_position_control_signal;
f32 kp_angle=1.3;
u32 max_steering_angle=30;

f32 Velocity_LPF()
{
	return (s16)(0.120198*velocityPrevControlSignal+0.4399*velocityControlSignal+0.4399*velocityPrevControlSignal);
}

f32 Position_LPF()
{
	return (f32)(-0.517093*previous_filter_position_control_signal+0.7585469*positonControlSignal+0.7585469*positonPrevControlSignal);
}

f64 AverageFilter()
{
	s32 Average=0;
	s32 total=0;
	total = total-avgFilterReadings[index];
	avgFilterReadings[index]=carVelocity;
	total = total + avgFilterReadings[index];
	index++;
	if (index>=15)
	index=0;
	Average=total/15;
	return Average;
}


void velocity_PID()
{
	
	velocityError = targetSpeed - carVelocity;
	velocityIntegrator= velocityIntegrator + velocityError*TS;
	velovityDerivative = (velocityError-velocityPrevError)/TS;
	velocityControlSignal=velocityError*VELOCITY_KP+velocityIntegrator*VELOCITY_KI+velovityDerivative*VELOCITY_KD;
	f_control_signal = velocityControlSignal * 4;
	
	velocityPrevError = velocityError;
	previous_control_signal=f_control_signal;
	previous_filter_control_signal=f_control_signal;
	prev_target_speed= targetSpeed;
	 

}

f32 max(f32 num1,f32 num2)
{
	if (num1>num2)
	return num1;
	else
	return num2;
}
f32 min(f32 num1,f32 num2)
{
	if (num1>num2)
	return num2;
	else
	return num1;
}
f32 computeSteering(f32 error)
{
	return (max(min(- kp_angle*error,max_steering_angle),- max_steering_angle));
}

/*void position_PID()
{
	f32 npos = (targetPosition* POSITION_PPR) /360.0;
	positionError=  -(HQUADEn_s32getPositionCount() - npos);
	positionIntegrator+=positionError*TS;
	positionDerivative=(positionError-prevPositionError)/TS;
	positon_control_signal=positionError*POSITION_KP+positionIntegrator*POSITION_KD+positionDerivative*POSITION_KI;
	filter_position_control_signal=Position_LPF();
	prevPositionError=positionError;
	previous_filter_position_control_signal=filter_position_control_signal;
	previous_positon_control_signal=positon_control_signal;
	
}*/

#endif /* PARAMETERS_H_ */