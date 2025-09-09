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
int pulse_delay_us = 700;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
double upper_angle = 200.0; // 上舵机角度 (中位)
double lower_angle = 0.0;   // 当前角度，初始化为0.0
double last_yaw = 0.0;      // 上次yaw角度，用于计算增量
int count = 0;              // 用于计数，控制步进旋转

// 串口交互变量
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
uint8_t packet_ready = 0;
uint8_t rx_byte; // 接收缓冲区

// 新的协议状态
typedef enum
{
    STATE_WAIT_HEADER1,
    STATE_WAIT_HEADER2,
    STATE_WAIT_LEN,
    STATE_WAIT_DATA,
    STATE_WAIT_CHECKSUM
} ProtocolState;

static ProtocolState g_rx_state = STATE_WAIT_HEADER1;
static uint8_t g_data_len = 0;
static uint8_t g_checksum = 0;

// JY61P 解析相关
static uint8_t RxBuffer_JY61[11];
static volatile uint8_t RxState_JY61 = 0;
static uint8_t RxIndex_JY61 = 0;
float g_roll_jy61 = 0, g_pitch_jy61 = 0, g_yaw_jy61 = 0;
uint8_t rx_byte_jy61;
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
extern UART_HandleTypeDef huart2;
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
    double delta = angle - lower_angle; // 计算实际需要转动的角度
    double abs_delta = fabs(delta);     // 取绝对值
    if (abs_delta < 0.1125)
        return; // 如果角度变化小于0.1125度，直接返回
    if (abs_delta > 5.0)
    {
        pulse_delay_us = 200;
    }
    else
    {
        pulse_delay_us = 400; // 正常转动速度
    }
    if (delta < 0)
    {
        stepper1_set_dir(0); // 反向
    }
    else
    {
        stepper1_set_dir(1); // 正向
    }
    int steps = (int)(abs_delta / 0.1125); // 每步0.1125度
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

        parse_protocol_packet(rx_buffer, g_data_len); // 使用接收到的长度

        // 重置并重新开始
        rx_index = 0;
        packet_ready = 0;
        g_rx_state = STATE_WAIT_HEADER1; // 重置状态机

        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// 简洁的协议解析函数 - 带校验和版本
void parse_protocol_packet(uint8_t *data, uint16_t len)
{
    // 提取舵机增量 (double, 小端序)
    // Python: struct.pack("<dd", float(output_x), float(output_y))
    // C: 第一个double是delta_x, 第二个是delta_y
    double delta_x = 0, delta_y = 0;
    memcpy(&delta_x, &data[0], sizeof(double));
    memcpy(&delta_y, &data[8], sizeof(double));

    // 判断是否瞄准完成（收到500,500）
    if (delta_x == 500.0 && delta_y == 500.0)
    {
        HAL_GPIO_WritePin(GPIOA, Laser_Pin, GPIO_PIN_RESET); // 激光关闭
        return;
    }

    // 回显收到的增量数据
    // char echo[80];
    // sprintf(echo, "ECHO: delta_x=%d, delta_y=%d, yaw=%d\r\n", (int)(delta_x * 100), (int)(delta_y * 100), (int)(g_yaw_jy61 * 100));
    // HAL_UART_Transmit(&huart1, (uint8_t *)echo, strlen(echo), 100);

    // 更新角度
    double new_upper = upper_angle + delta_y;
    double new_lower = lower_angle + delta_x;

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

// JY61P 数据接收处理函数（无需头文件，直接放 main.c）
void jy61p_ReceiveData(uint8_t RxData)
{
    uint8_t i, sum = 0;
    if (RxState_JY61 == 0)
    {
        if (RxData == 0x55)
        {
            RxBuffer_JY61[RxIndex_JY61] = RxData;
            RxState_JY61 = 1;
            RxIndex_JY61 = 1;
        }
    }
    else if (RxState_JY61 == 1)
    {
        if (RxData == 0x53)
        {
            RxBuffer_JY61[RxIndex_JY61] = RxData;
            RxState_JY61 = 2;
            RxIndex_JY61 = 2;
        }
    }
    else if (RxState_JY61 == 2)
    {
        RxBuffer_JY61[RxIndex_JY61++] = RxData;
        if (RxIndex_JY61 == 11)
        {
            for (i = 0; i < 10; i++)
            {
                sum = sum + RxBuffer_JY61[i];
            }
            if (sum == RxBuffer_JY61[10])
            {
                g_roll_jy61 = ((uint16_t)((uint16_t)RxBuffer_JY61[3] << 8 | (uint16_t)RxBuffer_JY61[2])) / 32768.0f * 180.0f;
                g_pitch_jy61 = ((uint16_t)((uint16_t)RxBuffer_JY61[5] << 8 | (uint16_t)RxBuffer_JY61[4])) / 32768.0f * 180.0f;
                g_yaw_jy61 = ((uint16_t)((uint16_t)RxBuffer_JY61[7] << 8 | (uint16_t)RxBuffer_JY61[6])) / 32768.0f * 180.0f;
            }
            RxState_JY61 = 0;
            RxIndex_JY61 = 0;
        }
    }
}

// 串口中断回调函数 - 带校验和的状态机版本
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (g_rx_state)
        {
        case STATE_WAIT_HEADER1:
            if (rx_byte == 0xAA)
            {
                g_rx_state = STATE_WAIT_HEADER2;
            }
            break;
        case STATE_WAIT_HEADER2:
            if (rx_byte == 0xBB)
            {
                g_rx_state = STATE_WAIT_LEN;
            }
            else
            {
                g_rx_state = STATE_WAIT_HEADER1; // 帧头错误，复位
            }
            break;
        case STATE_WAIT_LEN:
            g_data_len = rx_byte;
            if (g_data_len > 0 && g_data_len <= sizeof(rx_buffer))
            {
                rx_index = 0;
                g_checksum = 0; // 重置校验和
                g_rx_state = STATE_WAIT_DATA;
            }
            else
            {
                g_rx_state = STATE_WAIT_HEADER1; // 长度错误，复位
            }
            break;
        case STATE_WAIT_DATA:
            if (rx_index < g_data_len)
            {
                rx_buffer[rx_index] = rx_byte;
                g_checksum += rx_byte; // 累加校验和
                rx_index++;
            }
            if (rx_index >= g_data_len)
            {
                g_rx_state = STATE_WAIT_CHECKSUM;
            }
            break;
        case STATE_WAIT_CHECKSUM:
            if (rx_byte == (g_checksum & 0xFF)) // 校验和匹配
            {
                packet_ready = 1;
                // 成功接收，等待主循环处理。处理完后状态机会复位。
                return; // 暂时不启动下一次接收
            }
            else
            {
                // 校验失败，复位状态机，丢弃数据
                g_rx_state = STATE_WAIT_HEADER1;
            }
            break;
        }

        // 只要数据包还没准备好，就继续启动下一次接收
        if (!packet_ready)
        {
            HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        }
    }
    else if (huart->Instance == USART2)
    {
        jy61p_ReceiveData(rx_byte_jy61);
        HAL_UART_Receive_IT(&huart2, &rx_byte_jy61, 1);
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
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动触发器PWM (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // 启动上舵机PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // 启动下舵机PWM

    // 初始化舵机位置
    servo_set_upper_angle(upper_angle);

    // 启动串口中断接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &rx_byte_jy61, 1);

    // 步进电机初始化
    stepper1_enable(1); // 使能（保持高电平）
    // 不再关闭使能，EN1 始终为高电平
    // 发送提示信息
    HAL_Delay(300); // 等待系统稳定
    char welcome[] = "Ready\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)welcome, strlen(welcome), 1000);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        count++;
        if (count == 3000)
        {
            double now_yaw = g_yaw_jy61; // 获取当前yaw角度
            // 计算增量(最小变化,如1到360是-2,359到4是5)
            double delta_yaw = now_yaw - last_yaw; // 计算当前yaw与上次的差值
            if (delta_yaw < -180)
                delta_yaw += 360; // 如果差值小于-180，调整为正向增量
            else if (delta_yaw > 180)
                delta_yaw -= 360;                           // 如果差值大于180，调整为负向增量
            last_yaw = now_yaw;                             // 更新上次yaw角度
            servo_set_lower_angle(lower_angle - delta_yaw); // 更新下舵机角度
            count = 0;                                      // 重置计数器
        }
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
