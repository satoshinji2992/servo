#include "gimbal.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>

// 角度变量
double upper_angle = 200.0;
double lower_angle = 0.0;

// 步进电机速度
int pulse_delay_us = 700;

// 上舵机控制：TIM2_CH2 (PA1) - 270度舵机
void servo_set_upper_angle(double angle)
{
    if (angle > 270)
        angle = 270;
    if (angle < 0)
        angle = 0;
    int pulse = 50 + (int)(angle * 200 / 270);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

// 下舵机控制：步进电机
void servo_set_lower_angle(double angle)
{
    double delta = angle - lower_angle;
    double abs_delta = fabs(delta);

    if (abs_delta < 0.1125)
        return;

    pulse_delay_us = (abs_delta > 5.0) ? 200 : 400;

    stepper1_set_dir(delta >= 0 ? 1 : 0);

    int steps = (int)(abs_delta / 0.1125);
    for (int i = 0; i < steps; i++)
    {
        stepper1_step_once();
        lower_angle += (delta < 0) ? -0.1125 : 0.1125;
    }
}

// 步进电机使能
void stepper1_enable(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOA, En1_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 步进电机方向
void stepper1_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOA, Dir1_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 步进电机单步
void stepper1_step_once(void)
{
    HAL_GPIO_WritePin(GPIOA, Stp1_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < pulse_delay_us * 10; d++)
        ;
    HAL_GPIO_WritePin(GPIOA, Stp1_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < pulse_delay_us * 10; d++)
        ;
}
