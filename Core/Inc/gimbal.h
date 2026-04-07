#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>

// 角度变量
extern double upper_angle;
extern double lower_angle;

// 舵机控制
void servo_set_upper_angle(double angle);
void servo_set_lower_angle(double angle);

// 步进电机控制
void stepper1_enable(uint8_t enable);
void stepper1_set_dir(uint8_t dir);
void stepper1_step_once(void);

#endif /* __GIMBAL_H */
