#ifndef __MOTOR_H
#define __MOTOR_H

#include "port.h"

void motor_init(void);
void motor1_set_pwm(uint32_t);
void motor2_set_pwm(uint32_t);
void motor3_set_pwm(uint32_t);
void motor4_set_pwm(uint32_t);


#endif