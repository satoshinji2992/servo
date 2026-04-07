# 电赛仓库

报告见 b25580_s.pdf
[电赛报告](b25580_s.pdf)

## GPIO 配置

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | TIM2_CH1 | 触发器 PWM |
| PA1 | TIM2_CH2 | 上舵机 PWM (270度舵机) |
| PA2 | USART2_TX | JY61 陀螺仪串口 |
| PA3 | USART2_RX | JY61 陀螺仪串口 |
| PA4 | Dir1 | 步进电机方向 |
| PA5 | Stp1 | 步进电机脉冲 |
| PA6 | En1 | 步进电机使能 |
| PA7 | Laser | 激光控制 |
| PA9 | USART1_TX | 上位机串口 |
| PA10 | USART1_RX | 上位机串口 |
| PC13 | LED | 状态指示灯 |

## 文件结构

```
Core/
├── Inc/
│   ├── main.h       # GPIO 定义
│   ├── jy61.h       # 陀螺仪 API
│   └── gimbal.h     # 云台控制 API
└── Src/
    ├── main.c       # 主循环 + 串口协议
    ├── jy61.c       # JY61 解析
    └── gimbal.c     # 舵机/步进电机控制

host/
├── control.py       # MaixCAM 上位机
└── touch_example.py # 触摸示例
```

## 编译

```bash
make clean && make
```

## 上位机

MaixCAM 上位机程序见 `host/control.py`
