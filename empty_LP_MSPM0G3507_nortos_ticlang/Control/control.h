/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "board.h"

extern int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;
#define Frequency	200.0f			//每5ms读取一次编码器的值
#define Perimeter	0.2104867	    //轮子周长(单位:m)=0.67*3.1415926
#define Wheelspacing 0.1610f		//主动轮轴距(单位:m)
#define PI 3.1415926
//电机速度控制相关参数结构体
typedef struct  
{
	float Current_Encoder;     	//编码器数值，读取电机实时速度
	float Motor_Pwm;     		//电机PWM数值，控制电机实时速度
	float Target_Encoder;  		//电机目标编码器速度值，控制电机目标速度
	float Velocity; 	 		//电机速度值
}Motor_parameter;

//编码器结构体
typedef struct  
{
  int A;      
  int B;  
}Encoder;
extern float Move_X,Move_Z;						//目标速度和目标转向速度
extern Encoder OriginalEncoder; 					//编码器原始数据   
extern Motor_parameter MotorA,MotorB;				//左右电机相关变量
extern float Voltage_Count,Voltage_All,Voltage;
extern float Velocity_KP,Velocity_KI;	
extern int Run_Mode;//小车运行模式
void TIM6_Init(void); 
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
void Get_Target_Encoder(float Vx,float Vz);
int Incremental_PI_Left (float Encoder,float Target);
int Incremental_PI_Right (float Encoder,float Target);
void Get_Motor_PWM(void);
void Set_Pwm(int motor_a,int motor_b);
int Turn_Off(void);
int myabs(int a);
void Get_RC(void);
void Key(void);
#endif
