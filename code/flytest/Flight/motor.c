#include "motor.h"

/**
	@brief init motor
	@param void
	@return void
*/
void motor_init(void)
{
	motor_tim_init();
	motor1_set_pwm(0);
	motor2_set_pwm(0);
	motor3_set_pwm(0);
	motor4_set_pwm(0);
}

/**
	@brief init motor 1 PWM
	@param void
	@return void
*/
void motor1_set_pwm(uint32_t _pwm)
{
	motor1_tim_set_pwm(_pwm);
}
/**
	@brief init motor 2 PWM
	@param void
	@return void
*/
void motor2_set_pwm(uint32_t _pwm)
{
	motor2_tim_set_pwm(_pwm);
}
/**
	@brief init motor 3 PWM
	@param void
	@return void
*/
void motor3_set_pwm(uint32_t _pwm)
{
	motor3_tim_set_pwm(_pwm);
}
/**
	@brief init motor 4 PWM
	@param void
	@return void
*/
void motor4_set_pwm(uint32_t _pwm)
{
	motor4_tim_set_pwm(_pwm);
}