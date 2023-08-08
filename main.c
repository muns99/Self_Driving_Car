/*
 * CAR.c
 *
 * Created: 5/19/2023 6:41:48 AM
 * Author : MUNSOUR
 */ 
#define F_CPU 8000000UL
#include <util/delay.h>
#include "include/LIB/STD_TYPES.h"
#include "Include/LIB/BIT_MATH.h"

#include "Include/MCAL/DIO/DIO_Interface.h"
#include "Include/MCAL/EXTI/EXTI_Interface.h"
#include "Include/MCAL/GI/GI_Interface.h"
#include "Include/MCAL/TIMERS/TIMERS_Interface.h"
#include "Include/MCAL/TWI/TWI_Interface.h"
#include "Include/HAL/QUAD_ENCODER/QUAD_ENCODER_Interface.h"
#include "Include/HAL/LCD_Driver/LCD_Interface.h"
#include "Include/CAR/CAR.h"
#include "Parameters.h"


void setCar()
{

	s32 PPS = HQUADEn_s32getVelocityCount() / 0.01; //pulses per second
	carVelocity = (PPS / 385) * 60; // velocity in rpm
	carVelocity = carVelocity < 0 ? carVelocity * -1 : carVelocity;
	carPosition = HQUADEn_s32getPositionCount()  ;
	
	
	u32 circumference = 2 * pi * WHEEL_DIAMETER;
	carVelocityIn_M_per_Sec = (circumference * carVelocity)/60; //velocity in m/s
	HQUADEn_voidResetVelocityCount();


	velocityError =( targetSpeed - carVelocity);
	positionError = targetPosition - carPosition;
	
	f32 kp = VELOCITY_KP * velocityError;
	positionDerivative = ((positionError - positionPrevError)/TS) * POSITION_KD;
	velovityDerivative = ((velocityError - velocityPrevError)/TS) * VELOCITY_KD;
	
	velocityIntegrator += (VELOCITY_KI * velocityError * TS);
	positionIntegrator += (POSITION_KI * positionError* TS);
	
	positionPrevError = positionError;  
	velocityPrevError = velocityError;
	
	velocityControlSignal = kp + velocityIntegrator + velovityDerivative;
	positonControlSignal = POSITION_KP * positionError + positionIntegrator + positionDerivative; 
	//control_signal = Velocity_LPF();
	u16 tmp = velocityControlSignal + velocityPrevControlSignal;
	if (tmp > 0)
	{
		setBackMotor(tmp, carDir);
	}
	else if(tmp < 0)
	{
		tmp *= -1;
		setBackMotor(tmp , carDir);
	}
	else
	{
		setBackMotor(0 ,carDir);
	}
	velocityPrevControlSignal += velocityControlSignal;

	/*if(abs(positionError) < 5)
		{
			setSteeringMotor(0,Vmax);
			positonControlSignal = 0;*/		
	if (positonControlSignal > Vmax)  
	{
		positonControlSignal = Vmax;

	}
	else if (positonControlSignal < Vmin)
	{
		positonControlSignal = Vmin;
	}
	if(abs(positionError <= 5))
	positonControlSignal = 0.0;
	
	setSteeringMotor(positonControlSignal,Vmax);
	positionPrevError = positionError;
	
	
//	HLCD_voidSendData("CAR speed : ");
	
	//HLCD_voidDisplayNumber(HQUADEn_s32getPositionCount());;
	/*HLCD_voidSendData('n');
	HLCD_voidDisplayNumber(carPosition);
	HLCD_voidSendData('t');
	HLCD_voidDisplayNumber(targetPosition);*/
	/*HLCD_voidSendData("control signal : ");
	if (control_signal < 0)
	{
		HLCD_voidSendData('-');
	}
	HLCD_voidDisplayNumber(control_signal);*/
}


void Routine()
{
	targetSpeed = targetSpeed < 400 ? targetSpeed + 50 : targetSpeed;
	targetPosition = targetPosition > -360? targetPosition - 10 : 0;
}
int main(void)
{
	
	//f32 avgFilterReadings[15] = {0} ;
	MDIO_voidInit();
	MTIMER0_voidinit();
	MTIMER1_voidinit();
	
	HQUADEN_voidinitPositionEncoder();
	HQUADEN_voidinitVelocityEncoder();
	
	MTIMER0_voidSetCtcCallback(setCar);
	//MTWI_voidInitSlave(0x55);
	MEXTI_voidSetCfg(EXTI2,FALLING_EDGE);
	MEXTI_voidSetCallback(EXTI2,Routine);
	
	//
	
	MEXTI_voidEnableINT(EXTI0);
	MEXTI_voidEnableINT(EXTI1);
	MEXTI_voidEnableINT(EXTI2);
	MGI_voidEnable();
	//HQUADEN_voidinitPositionEncoder();
	
	//HLCD_voidInit();
	//MTIMER1_voidSetOcr1aValue(550);
	//HLCD_voidSendString("HHELLO");
	
	//HLCD_voidClearDisplay();
    /* Replace with your application code */
    while (1) 
    {
		

		//MDIO_voidSetPortValue(DIO_PORTC,(u8)HQUADEn_s32getVelocityCount());
	
		//_delay_ms(500);
		/*if ((targetSpeed/4 == carVelocity) && (targetSpeed <1000))
		{
			targetSpeed += 100;
		}*/
		/*if(!MDIO_u8GetPinValue(DIO_PORTB,PIN7))
		
			{
				targetSpeed += 100;
				if (targetSpeed > 1000)
				{
					targetSpeed = 1000;
				}
				
				MDIO_voidTogglePin(DIO_PORTA,PIN1);
			}*/
		//setCar();
		
		/*_delay_ms(1000);
		MTIMER1_voidSetOcr1aValue(400);
		_delay_ms(1000);
		MTIMER1_voidSetOcr1aValue(600);
		_delay_ms(1000);
		MTIMER1_voidSetOcr1aValue(800);
		_delay_ms(1000);
		MTIMER1_voidSetOcr1aValue(1000);
		_delay_ms(1000);*/
		/*targetSpeed = 0;
		_delay_ms(3000);
		targetSpeed= 100;
		_delay_ms(3000);
		targetSpeed = 150 ;
		_delay_ms(3000);
		targetSpeed = 200;
		_delay_ms(3000);
		targetSpeed = 250;
		_delay_ms(3000);*/
    }
	
}

