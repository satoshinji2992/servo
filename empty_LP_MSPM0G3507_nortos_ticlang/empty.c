/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
u8 Car_Mode=Diff_Car;
int Motor_Left,Motor_Right;                 //���PWM���� Ӧ��Motor��
u8 PID_Send;            //��ʱ�͵�����ر���
float RC_Velocity=200,RC_Turn_Velocity,Move_X,Move_Y,Move_Z,PS2_ON_Flag;               //ң�ؿ��Ƶ��ٶ�
float Velocity_Left,Velocity_Right; //�����ٶ�(mm/s)
u16 test_num,show_cnt;
float Voltage=0;

int main(void)
{
    // ϵͳ��ʼ��
    SYSCFG_DL_init();  // ��ʼ��ϵͳ����
    // �������������жϹ���״̬
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);    // ������A�ж�
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);    // ������B�ж�
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);  // UART0�����ж�
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);  // UART1�����ж�
    // ʹ�ܸ�������ж�
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);    // ����������A�ж�
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);    // ����������B�ж�
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // ����UART0�ж�
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN); // ����UART1�ж�
    // ��ʱ����ADC����ж�����
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);  // �����ʱ��0�жϹ���
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);        // ������ʱ��0�ж�
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
    OLED_Init();  // ��ʼ��OLED��ʾ��
    // ��ѭ��
    while (1) 
    {
       Voltage=2000;
		Voltage = Get_battery_volt();//����С����ǰ��ѹ
        BTBufferHandler();    // �����������ݻ�����
        oled_show();         //  OLED��ʾ����
        APP_Show();          //  APP��ʾ����

    }
}



