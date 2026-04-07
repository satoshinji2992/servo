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
#include "jy61.h"
#include "gimbal.h"
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
// 串口交互变量
uint8_t rx_buffer[32];
uint8_t rx_index = 0;
uint8_t packet_ready = 0;
uint8_t rx_byte;

// 协议状态机
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void process_uart_packet(void);
void parse_protocol_packet(uint8_t *data, uint16_t len);
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void process_uart_packet(void)
{
    if (packet_ready)
    {
        HAL_UART_AbortReceive_IT(&huart1);
        parse_protocol_packet(rx_buffer, g_data_len);
        rx_index = 0;
        packet_ready = 0;
        g_rx_state = STATE_WAIT_HEADER1;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void parse_protocol_packet(uint8_t *data, uint16_t len)
{
    double delta_x = 0, delta_y = 0;
    memcpy(&delta_x, &data[0], sizeof(double));
    memcpy(&delta_y, &data[8], sizeof(double));

    if (delta_x == 500.0 && delta_y == 500.0)
    {
        HAL_GPIO_WritePin(GPIOA, Laser_Pin, GPIO_PIN_RESET);
        return;
    }

    double new_upper = upper_angle + delta_y;
    double new_lower = lower_angle + delta_x;

    if (new_upper < 0)
        new_upper = 0;
    if (new_upper > 270)
        new_upper = 270;

    upper_angle = new_upper;
    servo_set_upper_angle(upper_angle);
    servo_set_lower_angle(new_lower);

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (g_rx_state)
        {
        case STATE_WAIT_HEADER1:
            if (rx_byte == 0xAA)
                g_rx_state = STATE_WAIT_HEADER2;
            break;
        case STATE_WAIT_HEADER2:
            if (rx_byte == 0xBB)
                g_rx_state = STATE_WAIT_LEN;
            else
                g_rx_state = STATE_WAIT_HEADER1;
            break;
        case STATE_WAIT_LEN:
            g_data_len = rx_byte;
            if (g_data_len > 0 && g_data_len <= sizeof(rx_buffer))
            {
                rx_index = 0;
                g_checksum = 0;
                g_rx_state = STATE_WAIT_DATA;
            }
            else
            {
                g_rx_state = STATE_WAIT_HEADER1;
            }
            break;
        case STATE_WAIT_DATA:
            rx_buffer[rx_index++] = rx_byte;
            if (rx_index >= g_data_len)
            {
                g_rx_state = STATE_WAIT_CHECKSUM;
            }
            break;
        case STATE_WAIT_CHECKSUM:
            {
                uint8_t sum = 0;
                for (uint8_t i = 0; i < g_data_len; i++)
                    sum += rx_buffer[i];
                if (sum == rx_byte)
                    packet_ready = 1;
                g_rx_state = STATE_WAIT_HEADER1;
            }
            break;
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        jy61_receive_data(rx_byte_jy61);
        HAL_UART_Receive_IT(&huart2, &rx_byte_jy61, 1);
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
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    jy61_init();
    servo_set_upper_angle(upper_angle);

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    HAL_UART_Receive_IT(&huart2, &rx_byte_jy61, 1);

    char welcome[] = "Protocol Parser Ready\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)welcome, strlen(welcome), 1000);

    stepper1_enable(1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE BEGIN 3 */
        process_uart_packet();
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
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
