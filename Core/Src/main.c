/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t upper_angle = 240; // 上舵机角度 (中位)
uint16_t lower_angle = 135; // 下舵机角度 (中位)

// 串口交互变量
uint8_t rx_char;
char uart_buffer[32];
uint8_t uart_index = 0;
uint8_t command_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void servo_set_upper_angle(uint16_t angle);
void servo_set_lower_angle(uint16_t angle);
void process_uart_command(void);
void parse_servo_command(char *cmd);
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 上舵机控制函数：TIM2_CH2 (PA1) - TBS-K20 270度舵机
void servo_set_upper_angle(uint16_t angle)
{
    if (angle > 270)
        angle = 270;
    uint16_t pulse = 50 + (angle * 200) / 270; // 500μs-2500μs对应50-250
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

// 下舵机控制函数：TIM2_CH3 (PA2) - TBS-K20 270度舵机
void servo_set_lower_angle(uint16_t angle)
{
    if (angle > 270)
        angle = 270;
    uint16_t pulse = 50 + (angle * 200) / 270; // 500μs-2500μs对应50-250
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

// 串口命令处理函数
void process_uart_command(void)
{
    if (command_ready)
    {
        uart_buffer[uart_index] = '\0'; // 字符串结束符
        parse_servo_command(uart_buffer);
        uart_index = 0;
        command_ready = 0;
    }
}

// 解析舵机控制命令
void parse_servo_command(char *cmd)
{
    char response[128];

    // 检查命令长度
    if (strlen(cmd) == 6)
    {
        // 6位数字格式: 前3位上舵机，后3位下舵机
        char upper_str[4] = {cmd[0], cmd[1], cmd[2], '\0'};
        char lower_str[4] = {cmd[3], cmd[4], cmd[5], '\0'};

        int upper = atoi(upper_str);
        int lower = atoi(lower_str);

        if (upper >= 0 && upper <= 270 && lower >= 0 && lower <= 270)
        {
            upper_angle = upper;
            lower_angle = lower;
            servo_set_upper_angle(upper_angle);
            servo_set_lower_angle(lower_angle);

            sprintf(response, "OK: Upper=%d, Lower=%d\r\n", upper_angle, lower_angle);
        }
        else
        {
            sprintf(response, "ERROR: Angles out of range (0-270)\r\n");
        }
    }
    else if (strncmp(cmd, "status", 6) == 0)
    {
        // 状态查询命令
        sprintf(response, "Status: Upper=%d, Lower=%d\r\n", upper_angle, lower_angle);
    }
    else if (strncmp(cmd, "help", 4) == 0)
    {
        // 帮助命令
        sprintf(response, "Commands:\r\n"
                          "- 6-digits: Set angles (e.g. 135090)\r\n"
                          "- status: Show current angles\r\n"
                          "- help: Show this help\r\n"
                          "Range: 0-270 degrees\r\n");
    }
    else
    {
        sprintf(response, "ERROR: Invalid command. Type 'help' for usage.\r\n");
    }

    // 发送响应
    HAL_UART_Transmit(&huart1, (uint8_t *)response, strlen(response), 1000);
}

// 串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 回显字符
        HAL_UART_Transmit(&huart1, &rx_char, 1, 10);

        if (rx_char == '\r' || rx_char == '\n')
        {
            // 收到回车或换行，命令结束
            if (uart_index > 0)
            {
                command_ready = 1;
            }
        }
        else if (rx_char == '\b' || rx_char == 127) // 退格键
        {
            if (uart_index > 0)
            {
                uart_index--;
                // 发送退格序列
                char backspace[] = "\b \b";
                HAL_UART_Transmit(&huart1, (uint8_t *)backspace, 3, 10);
            }
        }
        else if (uart_index < sizeof(uart_buffer) - 1)
        {
            // 普通字符
            uart_buffer[uart_index++] = rx_char;
        }

        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_char, 1);
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 启动上舵机PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 启动下舵机PWM

    // 初始化舵机位置
    servo_set_upper_angle(upper_angle);
    servo_set_lower_angle(lower_angle);

    // 启动串口中断接收
    HAL_UART_Receive_IT(&huart1, &rx_char, 1);

    // 发送提示信息
    char welcome[] = "TBS-K20 Ready\r\nFormat: 6-digits (e.g. 135090)\r\nFirst 3: upper servo, Last 3: lower servo\r\nRange: 0-270\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)welcome, strlen(welcome), 1000);

    // 发送调试信息
    char debug_msg[] = "Debug: UART ready for commands\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 1000);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        process_uart_command(); // 处理串口命令
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
