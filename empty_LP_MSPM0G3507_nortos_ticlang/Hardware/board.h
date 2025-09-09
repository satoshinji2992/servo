#ifndef _BOARD_H_
#define _BOARD_H_
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h"
#include "oled.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "encoder.h"
#include "show.h"
#include "control.h"
#include "DataScope_DP.h"
#include "uart_callback.h"
#include "IR_Module.h"
#include "adc.h"
#define ABS(a)      (a>0 ? a:(-a))
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

// Enumeration of car types
//小车型号的枚举定义
typedef enum 
{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car
} CarMode;

extern u8 Way_Angle;                                                     //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern u16 determine;                              //雷达跟随模式的一个标志位
extern int Motor_Left,Motor_Right;                                 //电机PWM变量 应是motor的 向moto致敬
extern u8 Flag_Stop,Flag_Show;                                           //停止标志位和 显示标志位 默认停止 显示打开
extern int Middle_angle;                                                                             //电池电压采样相关的变量
extern float Voltage,Angle_Balance,Gyro_Balance,Gyro_Turn;                           //平衡倾角 平衡陀螺仪 转向陀螺仪
extern u8 LD_Successful_Receive_flag;              //雷达成功接收数据标志位
extern int Temperature;
extern u32 Distance;                                                //超声波测距
extern u8 Flag_follow,Flag_avoid,delay_50,delay_flag,PID_Send;
extern float Acceleration_Z;                       //Z轴加速度计
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
extern float RC_Velocity,RC_Turn_Velocity,Move_X,Move_Y,Move_Z,PS2_ON_Flag;                //遥控控制的速度
extern u8 one_frame_data_success_flag,one_lap_data_success_flag;
extern int lap_count,PointDataProcess_count,test_once_flag,Dividing_point;
extern float Velocity_Left,Velocity_Right;  //车轮速度(mm/s)
extern uint8_t  recv0_flag;
extern u16 test_num,show_cnt;
extern u8 Car_Mode;
extern volatile unsigned long tick_ms;



//Systick最大计数值,24位
#define SysTickMAX_COUNT 0xFFFFFF

//Systick计数频率
#define SysTickFre 80000000

//将systick的计数值转换为具体的时间单位
#define SysTick_MS(x)  ((SysTickFre/1000U)*(uint32_t)(x))
#define SysTick_US(x)  ((SysTickFre/1000000U)*(uint32_t)(x))

uint32_t Systick_getTick(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

void delay_1us(unsigned long __us);
void delay_1ms(unsigned long ms);
#endif  /* #ifndef _BOARD_H_ */