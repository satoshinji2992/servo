#include "menu.h"
#include "oled.h" // 确保包含OLED头文件

// 菜单状态变量
static MenuState_t current_menu_state = MENU_MAIN;
static int main_menu_selected_option = MAIN_OPTION_1; // 默认选择主菜单选项1
static int N_value = 0; // 二级菜单的数字N

// 显示主菜单
void Display_MainMenu(int selected_option) {
    OLED_Clear();
    OLED_ShowString(0, 0, "Main Menu", 8);
    if (selected_option == MAIN_OPTION_1) {
        OLED_ShowString(0, 2, "> 1", 8);
        OLED_ShowString(0, 4, "  2", 8);
    } else {
        OLED_ShowString(0, 2, "  1", 8);
        OLED_ShowString(0, 4, "> 2", 8);
    }
    OLED_Refresh_Gram();
}

// 显示二级菜单N
void Display_SubMenuN(int current_N) {
    OLED_Clear();
    OLED_ShowString(0, 0, "Sub Menu N", 8);
    OLED_ShowString(0, 2, "N:", 8);
    OLED_ShowNum(24, 2, current_N, 4, 8); // 显示N的值
    OLED_Refresh_Gram();
}

// 菜单初始化
void Menu_Init(void) {
    OLED_Init(); // 初始化OLED
    Display_MainMenu(main_menu_selected_option); // 显示初始主菜单
}

// 菜单主循环
void Menu_Loop(void) {
    u8 key_val = key_scan(200); // 读取按键值

    switch (current_menu_state) {
        case MENU_MAIN:
            if (key_val == 1) { // Key1: 循环选择
                main_menu_selected_option = (main_menu_selected_option == MAIN_OPTION_1) ? MAIN_OPTION_2 : MAIN_OPTION_1;
                Display_MainMenu(main_menu_selected_option);
            } else if (key_val == 2) { // Key2: 确认选择
                if (main_menu_selected_option == MAIN_OPTION_1) {
                    current_menu_state = MENU_SUB_N; // 进入二级菜单N
                    N_value = 0; // 重置N为0
                    Display_SubMenuN(N_value);
                } else { // 选择2，结束菜单
                    OLED_Clear();
                    OLED_ShowString(0, 0, "Option 2 Selected", 8);
                    OLED_Refresh_Gram();
                    // 可以在这里添加其他结束菜单的逻辑，例如设置一个标志位
                    // 为了演示，这里只是显示信息，实际应用可能需要退出菜单循环
                }
            }
            break;

        case MENU_SUB_N:
            if (key_val == 1) { // Key1: N++
                N_value++;
                Display_SubMenuN(N_value);
            } else if (key_val == 2) { // Key2: 确认N的选项，结束菜单
                OLED_Clear();
                OLED_ShowString(0, 0, "N Confirmed:", 8);
                OLED_ShowNum(0, 2, N_value, 4, 8);
                OLED_Refresh_Gram();
                // 可以在这里添加其他结束菜单的逻辑，例如设置一个标志位
                // 为了演示，这里只是显示信息，实际应用可能需要退出菜单循环
            }
            break;
    }
}
