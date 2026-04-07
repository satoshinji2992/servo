#include "jy61.h"

// JY61 数据
float g_roll_jy61 = 0, g_pitch_jy61 = 0, g_yaw_jy61 = 0;
uint8_t rx_byte_jy61 = 0;

// 内部状态
static uint8_t rx_buffer[11];
static volatile uint8_t rx_state = 0;
static uint8_t rx_index = 0;

void jy61_init(void)
{
    g_roll_jy61 = 0;
    g_pitch_jy61 = 0;
    g_yaw_jy61 = 0;
    rx_state = 0;
    rx_index = 0;
}

void jy61_receive_data(uint8_t rx_data)
{
    uint8_t i, sum = 0;

    if (rx_state == 0)
    {
        if (rx_data == 0x55)
        {
            rx_buffer[rx_index] = rx_data;
            rx_state = 1;
            rx_index = 1;
        }
    }
    else if (rx_state == 1)
    {
        if (rx_data == 0x53)
        {
            rx_buffer[rx_index] = rx_data;
            rx_state = 2;
            rx_index = 2;
        }
    }
    else if (rx_state == 2)
    {
        rx_buffer[rx_index++] = rx_data;
        if (rx_index == 11)
        {
            for (i = 0; i < 10; i++)
            {
                sum += rx_buffer[i];
            }
            if (sum == rx_buffer[10])
            {
                g_roll_jy61 = ((uint16_t)((uint16_t)rx_buffer[3] << 8 | rx_buffer[2])) / 32768.0f * 180.0f;
                g_pitch_jy61 = ((uint16_t)((uint16_t)rx_buffer[5] << 8 | rx_buffer[4])) / 32768.0f * 180.0f;
                g_yaw_jy61 = ((uint16_t)((uint16_t)rx_buffer[7] << 8 | rx_buffer[6])) / 32768.0f * 180.0f;
            }
            rx_state = 0;
            rx_index = 0;
        }
    }
}
