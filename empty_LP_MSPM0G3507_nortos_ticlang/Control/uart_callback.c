#include "ti_msp_dl_config.h"
#include <string.h>
#include "board.h"
#include "uart_callback.h"

// 蓝牙遥控相关的标志位
int Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; // 方向控制标志位

extern uint8_t PID_Send; // 外部声明的PID发送标志

#define BT_PACKET_SIZE (200) // 定义蓝牙数据包缓冲区大小
volatile uint8_t gBTPacket[BT_PACKET_SIZE]; // 蓝牙数据接收缓冲区
volatile uint8_t gBTCounts = 0; // 当前接收数据计数
volatile uint8_t lastBTCounts = 0; // 上次接收数据计数
volatile bool gCheckBT; // 蓝牙数据接收完成标志

uint8_t RecvOverFlag = 0; // 接收完成标志

void bt_control(uint8_t recv); // 蓝牙数据处理函数声明

/**
 * @brief 配置DMA用于蓝牙数据接收
 * 作用：设置DMA通道参数，实现UART接收数据的自动搬运
 */
void BT_DAMConfig(void)
{
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID); // 先禁用DMA通道
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));// 设置源地址为UART接收寄存器
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gBTPacket[0]); // 设置目标地址为接收缓冲区
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, BT_PACKET_SIZE); // 设置传输大小
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID); // 启用DMA通道
}

/**
 * @brief 蓝牙数据缓冲区处理函数
 * 作用：管理接收到的蓝牙数据，处理数据超时和半满情况
 */
void BTBufferHandler(void)
{
    uint32_t tick = 0;
    static uint8_t handleflag = 0; // 数据处理标志
    static uint8_t handleSize = 0; // 已处理数据大小
    static uint8_t lastSize = 0; // 上次接收数据大小

    // 计算已接收数据大小（总大小减去剩余传输大小）
    uint8_t recvsize = BT_PACKET_SIZE - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);

    if( recvsize != lastSize) // 有新数据到达
    {
        handleflag=1; // 设置需要处理标志
        tick = Systick_getTick(); // 刷新最后接收时间戳
    }
    else // 无新数据
    {
        // 检查是否超时（1ms）且有待处理数据
        if( ((tick-Systick_getTick())&SysTickMAX_COUNT) >= SysTick_MS(1) && handleflag == 1)
        {
            handleflag = 0; // 清除处理标志
            
            // 处理新接收的数据
            for(uint8_t i=handleSize;i<recvsize;i++)
                bt_control(gBTPacket[i]);

            handleSize = recvsize; // 更新已处理位置

            // 缓冲区过半时重置DMA，防止溢出
            if( recvsize >= BT_PACKET_SIZE/2 )
            {
                recvsize=0;
                handleSize=0;
                lastSize = 0;
                BT_DAMConfig(); // 重新配置DMA
            }
        }
    }

    lastSize = recvsize; // 保存当前接收大小
}

/**
 * @brief UART1中断服务函数
 * 作用：处理UART接收中断和DMA完成中断
 */
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_IIDX_RX: // 接收中断
            gBTPacket[gBTCounts++] = DL_UART_Main_receiveData(UART_1_INST); // 直接读取数据
            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_RX: // DMA接收完成
            BT_DAMConfig(); // 重新配置DMA
        default:
            break;
    }
}
/**
 * @brief 蓝牙数据处理函数
 * 作用：处理接收到的蓝牙数据
 */

void bt_control(uint8_t recv)
{
    static  int Usart_Receive=0;//蓝牙接收相关变量
    static uint8_t Flag_PID,i,j,Receive[50];
    static float Data;

    // 接收发送过来的数据保存
    Usart_Receive = recv;

	if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //进入APP转向控制界面
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//进入APP方向控制界面	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP摇杆控制界面命令
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP转向控制界面命令
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s
	  
    if(Usart_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
    if(Usart_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位
    if(Flag_PID==1)  //采集数据
    {
            Receive[i]=Usart_Receive;
            i++;
    }
    if(Flag_PID==2)  //分析数据
    {
            if(Receive[3]==0x50)               PID_Send=1;
            else if(Receive[1]!=0x23)
            {
                for(j=i;j>=4;j--)
                {
                    Data+=(Receive[j-1]-48)*pow(10,i-j);
                }
                switch(Receive[1])
                {
                    case 0x30:  RC_Velocity=Data;break;
                    case 0x31:  Velocity_KP=Data;break;
                    case 0x32:  Velocity_KI=Data;break;
                    case 0x33:  Turn90Angle=Data/100.0f;break;
                    case 0x34:  maxTurnAngle=Data/100.0f;break;
                    case 0x35:  midTurnAngle=Data/100.0f;break;
                    case 0x36:  minTurnAngle=Data/100.0f;break; //预留
                    case 0x37:  break; //预留
                    case 0x38:  break; //预留
                }
            }
                Flag_PID=0;
                i=0;
                j=0;
                Data=0;
                memset(Receive, 0, sizeof(uint8_t)*50);//数组清零
    }

}
