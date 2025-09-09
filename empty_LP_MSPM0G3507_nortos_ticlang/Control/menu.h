#ifndef _MENU_H
#define _MENU_H

#include "board.h" // 假设board.h包含了按键相关的定义
#include "key.h"   // 假设key.h包含了key_scan函数声明
#include "oled.h"  // 假设oled.h包含了OLED显示函数

// 菜单状态枚举
typedef enum {
    MENU_MAIN,
    MENU_SUB_N
} MenuState_t;

// 菜单选项枚举
typedef enum {
    MAIN_OPTION_1,
    MAIN_OPTION_2
} MainOption_t;

// 函数原型
void Menu_Init(void);
void Menu_Loop(void);
void Display_MainMenu(int selected_option);
void Display_SubMenuN(int current_N);

#endif // _MENU_H
