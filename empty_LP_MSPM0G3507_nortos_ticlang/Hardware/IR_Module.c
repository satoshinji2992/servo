#include "IR_Module.h"
// 引脚状态变量
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
//转向参数调参时放大100倍
float Turn90Angle  = 2.3f; // 直角弯转向角度（rad/s）
float maxTurnAngle = 1.03f; // 弯道最大转向角度（rad/s）
float midTurnAngle = 0.50f; // 弯道中等转向角度（rad/s）
float minTurnAngle = 0.25f; // 弯道最小转向角度（rad/s）
//调参时速度参数放大1000倍

// 定义方向常量
#define DIR_LEFT 1
#define DIR_RIGHT -1
#define DIR_STRAIGHT 0

void IRDM_line_inspection(void) {
    static int last_state = 0; // 上一控制周期的状态
    static int ten_time = 0;
    static int last_turn_direction = DIR_STRAIGHT; // 记录最后一次调整的方向
	float baseSpeed=RC_Velocity/1000.0f * 1.15f;      //巡线基础速度:略微提高小车速度
	    // 读取四个引脚的状态并强制转换为0或1
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
	
    int DH_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // 将传感器状态组合成一个整数
    switch (DH_state) {
        case 15: // 0000 四路传感器都没有检测到黑线
                Move_X = baseSpeed * 0.8f; // 适当降低速度
                if (last_turn_direction == DIR_LEFT) {
                    Move_Z = midTurnAngle; // 左转
                } else if (last_turn_direction == DIR_RIGHT) {
                    Move_Z = -midTurnAngle; // 右转
                } else { // 如果是直行，则尝试一个默认的微调方向，例如左转
                    Move_Z = midTurnAngle;
                }
            last_state = 1; // 这里的last_state可能需要更精确的定义，目前保持不变
            break;

        case 1: // 0001 左直角弯
            ten_time = 0;
            Move_X = baseSpeed * 0.3f;
            Move_Z = Turn90Angle;
            last_state = 2;
            last_turn_direction = DIR_LEFT;
            break;

        case 8: // 1000 右直角弯
            ten_time = 0;
            Move_X = baseSpeed * 0.3f;
            Move_Z = -Turn90Angle;
            last_state = 3;
            last_turn_direction = DIR_RIGHT;
            break;

        case 12:  // 0011 右侧两个传感器检测到黑线 - 立即右转
            ten_time = 0;
            Move_X = baseSpeed * 0.5f; // 降低速度以便急转
            Move_Z = -Turn90Angle;     // 大角度右转
            last_state = 10; // 新增状态，表示急右转
            last_turn_direction = DIR_RIGHT;
            break;

        case 3: // 1100 左侧两个传感器检测到黑线 - 立即左转
        case 7:  // 0111 (234检测到黑线) - 左转
            ten_time = 0;
            Move_X = baseSpeed * 0.5f; // 降低速度以便急转
            Move_Z = Turn90Angle;      // 大角度左转
            last_state = 9; // 新增状态，表示急左转
            last_turn_direction = DIR_LEFT;
            break;

        case 14: // 1110 (123检测到黑线) - 向右回正
            ten_time = 0;
            Move_X = baseSpeed * 0.8f; // 保持较高速度
            Move_Z = -minTurnAngle;    // 小角度右转回正
            last_state = 11; // 新增状态，表示向右回正
            last_turn_direction = DIR_RIGHT;
            break;

        case 11: // 1011 左微调
            ten_time = 0;
            Move_X = baseSpeed * 0.8f;
            Move_Z = minTurnAngle;
            last_state = 6;
            last_turn_direction = DIR_LEFT;
            break;

        case 13: // 1101 右微调
            ten_time = 0;
            Move_X = baseSpeed * 0.8f;
            Move_Z = -minTurnAngle;
            last_state = 7;
            last_turn_direction = DIR_RIGHT;
            break;

        case 9: // 1001 直行
            ten_time = 0;
            Move_X = baseSpeed;
            Move_Z = 0;
            last_state = 8;
            last_turn_direction = DIR_STRAIGHT;
            break;
        
        default: // 其他所有情况，保持停止或默认行为
            ten_time = 0;
            Move_X = 0;
            Move_Z = 0;
            last_state = 0;
            last_turn_direction = DIR_STRAIGHT; // 默认停止时方向为直行
            break;
    }
}
