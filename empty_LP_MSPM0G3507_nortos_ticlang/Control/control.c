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
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=300;	
int Run_Mode=1;//小车运行模式
u8 Flag_Stop=1;//小车启动标志位
void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerA_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {
			
			Key();
			LED_Flash(100);
			Get_Velocity_From_Encoder(Get_Encoder_countA,Get_Encoder_countB);
			Get_Encoder_countA=Get_Encoder_countB=0;
			if(Run_Mode==0)
			{
				Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			}else if(Run_Mode==1){
				IRDM_line_inspection();
				Get_Target_Encoder(Move_X,Move_Z);
			}
//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			{
				Set_PWM(MotorA.Motor_Pwm,MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
		}
    }
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	
	//Retrieves the original data of the encoder
	//获取编码器的原始数据
	float Encoder_A_pr,Encoder_B_pr; 
	OriginalEncoder.A=-Encoder1;	
	OriginalEncoder.B=-Encoder2;	
	Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B;
	//编码器原始数据转换为车轮速度，单位m/s
	MotorA.Current_Encoder= Encoder_A_pr*Frequency*Perimeter/780.0f;  
	MotorB.Current_Encoder= Encoder_B_pr*Frequency*Perimeter/780.0f;   //1560=2*13*30=2（两路脉冲）*1（上升沿计数）*霍尔编码器13线*电机的减速比
	
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	float amplitude=3.5f; //Wheel target speed limit //车轮目标速度限幅
	if(Vx<0) Vz=-Vz;
	else     Vz=Vz;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - Vz * Wheelspacing / 2.0f; //计算出左轮的目标速度
	 MotorB.Target_Encoder = Vx + Vz * Wheelspacing / 2.0f; //计算出右轮的目标速度
	//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
//	 MotorA.Target_Encoder=target_limit_float( MotorA.Target_Encoder,-amplitude,amplitude); 
//	 MotorB.Target_Encoder=target_limit_float( MotorB.Target_Encoder,-amplitude,amplitude); 
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	 if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	 switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
//	}
	
//	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	//   if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/200;
	}		
	
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	Get_Target_Encoder(Move_X,Move_Z);
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=key_scan(200);//click_N_Double(50);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//单击控制小车的启停
	else if(tmp==2)
	{
		Run_Mode++;
		Run_Mode%=2;
	}
}
