#ifndef __JY61_H
#define __JY61_H

#include <stdint.h>

// JY61 数据
extern float g_roll_jy61, g_pitch_jy61, g_yaw_jy61;
extern uint8_t rx_byte_jy61;

// 初始化
void jy61_init(void);

// 数据解析（在串口中断中调用）
void jy61_receive_data(uint8_t rx_data);

#endif /* __JY61_H */
