#ifndef _UART_CALLBACK_H_
#define _UART_CALLBACK_H_
#include "board.h"

void BT_DAMConfig(void);

void BTBufferHandler(void);

extern int Flag_Left, Flag_Right, Flag_Direction, Turn_Flag;

#endif 