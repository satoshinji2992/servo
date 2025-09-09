#ifndef _IR_MODULE_H
#define _IR_MODULE_H
#include "ti_msp_dl_config.h"
#include "board.h"
extern uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;
// 红外引脚读取宏定义
#define IR_DH1_Read()   (DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_17_PIN) ? 1 : 0)
#define IR_DH2_Read()   (DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_16_PIN) ? 1 : 0)
#define IR_DH3_Read()   (DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_12_PIN) ? 1 : 0)
#define IR_DH4_Read()   (DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_27_PIN) ? 1 : 0)
void IR_Module_Read(void);
void IRDM_line_inspection(void);

extern float Turn90Angle  ;// 直角弯转向角度（rad/s）
extern float maxTurnAngle ; // 弯道最大转向角度（rad/s）
extern float midTurnAngle ; // 弯道中等转向角度（rad/s）
extern float minTurnAngle ; // 弯道最小转向角度（rad/s）

#endif
