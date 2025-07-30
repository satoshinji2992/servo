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
int pulse_delay_us = 400;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double upper_angle = 190.0; // 上舵机角度 (中位)
double lower_angle = 0.0;   // 当前角度，初始化为0.0

// 串口交互变量
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
uint8_t packet_ready = 0;
uint8_t rx_byte; // 接收缓冲区

// 协议解析相关
#define PROTOCOL_HEADER_SIZE 4
uint8_t protocol_header[PROTOCOL_HEADER_SIZE] = {0xAA, 0xCA, 0xAC, 0xBB};
uint8_t header_match_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void servo_set_upper_angle(double angle);
void servo_set_lower_angle(double angle);
void stepper1_set_dir(uint8_t dir);
void stepper1_step_once(void);

void process_uart_packet(void);
void parse_protocol_packet(uint8_t *data, uint16_t len);
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 上舵机控制函数：TIM2_CH2 (PA1) - TBS-K20 270度舵机
void servo_set_upper_angle(double angle)
{
    if (angle > 270)
        angle = 270;
    if (angle < 0)
        angle = 0;
    int pulse = 50 + (angle * 200) / 270; // 500μs-2500μs对应50-250
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

// 下舵机控制函数：步进电机控制
void servo_set_lower_angle(double angle)
{
    double delta = angle - lower_angle;
    if (delta == 0)
        return; // 如果角度没有变化，直接返回
    if (delta < 0)
    {
        stepper1_set_dir(0); // 反向
    }
    else
    {
        stepper1_set_dir(1); // 正向
    }
    int steps = (int)(abs(delta) / 0.1125); // 每步0.1125度
    for (int i = 0; i < steps; i++)
    {

        stepper1_step_once();
        lower_angle += (delta < 0) ? -0.1125 : 0.1125; // 更新当前角度
    }
}

// 串口数据包处理函数
void process_uart_packet(void)
{
    if (packet_ready)
    {
        // 停止串口接收，避免干扰
        HAL_UART_AbortReceive_IT(&huart1);

        parse_protocol_packet(rx_buffer, 28);

        // 重置并重新开始
        rx_index = 0;
        packet_ready = 0;
        header_match_count = 0;

        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// 简洁的协议解析函数
void parse_protocol_packet(uint8_t *data, uint16_t len)
{
    // 检查协议头 AA CA AC BB
    if (data[0] == 0xAA && data[1] == 0xCA &&
        data[2] == 0xAC && data[3] == 0xBB &&
        data[9] == 0x11) // 检查命令字节
    {
        // 提取舵机增量 (double, 小端序)
        double delta_y = 0, delta_x = 0;
        memcpy(&delta_y, &data[10], sizeof(double));
        memcpy(&delta_x, &data[18], sizeof(double));

        // 保留两位小数
        delta_y = (int)(delta_y * 100) / 100.0;
        delta_x = (int)(delta_x * 100) / 100.0;

        // 回显收到的增量数据
        char echo[80];
        sprintf(echo, "ECHO: delta_x=%d, delta_y=%d\r\n", (int)(delta_x * 100), (int)(delta_y * 100));
        HAL_UART_Transmit(&huart1, (uint8_t *)echo, strlen(echo), 100);

        // 更新角度
        double new_upper = upper_angle + delta_x;
        double new_lower = lower_angle + delta_y;

        // 限制范围
        if (new_upper < 0)
            new_upper = 0;
        if (new_upper > 270)
            new_upper = 270;

        // 更新舵机
        upper_angle = new_upper;
        servo_set_upper_angle(upper_angle);
        servo_set_lower_angle(new_lower);

        // 只输出最终结果
        // char msg[50];
        // sprintf(msg, "OK:%.1f,%.1f\r\n", upper_angle, lower_angle);
        // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);

        // 翻转PC13
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

// 串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 存储接收到的字节
        rx_buffer[rx_index++] = rx_byte;

        // 检查协议头匹配
        if (header_match_count < PROTOCOL_HEADER_SIZE)
        {
            if (rx_byte == protocol_header[header_match_count])
            {
                header_match_count++;
            }
            else
            {
                // 协议头不匹配，检查是否是新的开始
                if (rx_byte == protocol_header[0])
                {
                    rx_index = 1;
                    rx_buffer[0] = rx_byte;
                    header_match_count = 1;
                }
                else
                {
                    rx_index = 0;
                    header_match_count = 0;
                }
            }
        }

        // 检查是否接收完整数据包
        if (header_match_count >= PROTOCOL_HEADER_SIZE && rx_index >= 28)
        {
            packet_ready = 1;
            return; // 数据包完整，等待处理
        }

        // 防止缓冲区溢出
        if (rx_index >= sizeof(rx_buffer))
        {
            rx_index = 0;
            header_match_count = 0;
        }

        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// 步进电机1控制相关
void stepper1_enable(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOA, En1_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepper1_set_dir(uint8_t dir)
{
    HAL_GPIO_WritePin(GPIOA, Dir1_Pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void stepper1_step_once(void)
{
    // 产生一个脉冲
    HAL_GPIO_WritePin(GPIOA, Stp1_Pin, GPIO_PIN_SET);
    for (volatile int d = 0; d < pulse_delay_us * 10; d++)
        ;
    HAL_GPIO_WritePin(GPIOA, Stp1_Pin, GPIO_PIN_RESET);
    for (volatile int d = 0; d < pulse_delay_us * 10; d++)
        ;
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
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动触发器PWM (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 启动上舵机PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 启动下舵机PWM

    // 初始化舵机位置
    servo_set_upper_angle(upper_angle);
    servo_set_lower_angle(20.0);
    servo_set_lower_angle(0.0);

    // 启动串口中断接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    // 发送提示信息
    char welcome[] = "Protocol Parser Ready\r\nWaiting for binary protocol packets...\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)welcome, strlen(welcome), 1000);

    // 发送当前角度
    char status[100];
    sprintf(status, "Initial angles: Upper=%.1f, Lower=%.1f\r\n", upper_angle, lower_angle); // 修正格式化字符串
    HAL_UART_Transmit(&huart1, (uint8_t *)status, strlen(status), 1000);
    // 步进电机初始化
    stepper1_enable(1); // 使能（保持高电平）
    // 不再关闭使能，EN1 始终为高电平
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        process_uart_packet(); // 处理串口数据包
        // 可在此处添加步进电机控制逻辑
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
