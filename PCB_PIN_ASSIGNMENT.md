# STM32F407ZGT6 小车 PCB 完整引脚分配

## 1. 设计原则

- 保留当前已经完成实车测试的电机、编码器、循迹、OLED、视觉和激光测距引脚。
- 使用硬件定时器、硬件 UART 和硬件 I²C，不使用软件模拟通信。
- PA13、PA14、NRST 保留给 SWD；PH0、PH1 保留给 8 MHz 外部晶振。
- PC14、PC15 保留给 RTC 晶振，不连接普通外设。
- BOOT0 使用 10 kΩ 下拉，并预留跳线或测试点。
- 所有模块必须共地；不同电压域之间按本文要求增加电平保护。

## 2. 总引脚表

| 功能 | F407 引脚 | 硬件外设 | 方向 | PCB 网络名 | 备注 |
| --- | --- | --- | --- | --- | --- |
| 左前电机 PWM | PE9 | TIM1_CH1 | 输出 | MOTOR_A_PWM | 对应 D24A PWMA |
| 右前电机 PWM | PE11 | TIM1_CH2 | 输出 | MOTOR_B_PWM | 对应 D24A PWMB |
| 左后电机 PWM | PE13 | TIM1_CH3 | 输出 | MOTOR_C_PWM | 对应 D24A PWMC |
| 右后电机 PWM | PE14 | TIM1_CH4 | 输出 | MOTOR_D_PWM | 对应 D24A PWMD |
| 左前电机方向 1 | PD0 | GPIO | 输出 | MOTOR_A_IN1 | 对应 AIN1 |
| 左前电机方向 2 | PD1 | GPIO | 输出 | MOTOR_A_IN2 | 对应 AIN2 |
| 右前电机方向 1 | PD2 | GPIO | 输出 | MOTOR_B_IN1 | 对应 BIN1 |
| 右前电机方向 2 | PD3 | GPIO | 输出 | MOTOR_B_IN2 | 对应 BIN2 |
| 左后电机方向 1 | PD4 | GPIO | 输出 | MOTOR_C_IN1 | 对应 CIN1 |
| 左后电机方向 2 | PD5 | GPIO | 输出 | MOTOR_C_IN2 | 对应 CIN2 |
| 右后电机方向 1 | PD6 | GPIO | 输出 | MOTOR_D_IN1 | 对应 DIN1 |
| 右后电机方向 2 | PD7 | GPIO | 输出 | MOTOR_D_IN2 | 对应 DIN2 |
| 四路驱动待机 | PC0 | GPIO | 输出 | MOTOR_STBY | 低电平关闭全部电机 |
| 左前编码器 A | PB6 | TIM4_CH1 | 输入 | ENC_LF_A | E1A |
| 左前编码器 B | PB7 | TIM4_CH2 | 输入 | ENC_LF_B | E1B |
| 右前编码器 A | PA6 | TIM3_CH1 | 输入 | ENC_RF_A | E2A |
| 右前编码器 B | PA7 | TIM3_CH2 | 输入 | ENC_RF_B | E2B |
| 左后编码器 A | PC6 | TIM8_CH1 | 输入 | ENC_LR_A | E3A |
| 左后编码器 B | PC7 | TIM8_CH2 | 输入 | ENC_LR_B | E3B |
| 右后编码器 A | PA0 | TIM2_CH1 | 输入 | ENC_RR_A | E4A |
| 右后编码器 B | PA1 | TIM2_CH2 | 输入 | ENC_RR_B | E4B |
| 循迹地址 AD0 | PB0 | GPIO | 输出 | LINE_AD0 | CD4051 地址位 0 |
| 循迹地址 AD1 | PB1 | GPIO | 输出 | LINE_AD1 | CD4051 地址位 1 |
| 循迹地址 AD2 | PB2 | GPIO | 输出 | LINE_AD2 | CD4051 地址位 2；PB2 兼作 BOOT1 |
| 循迹数字输出 | PB10 | GPIO | 输入 | LINE_OUT | 模块 OUT，必须预留降压保护 |
| OLED 时钟 | PB8 | I2C1_SCL | 开漏双向 | OLED_SCL | 4.7 kΩ 上拉到 3.3 V |
| OLED 数据 | PB9 | I2C1_SDA | 开漏双向 | OLED_SDA | 4.7 kΩ 上拉到 3.3 V |
| 外置按键 1 | PA15 | GPIO/EXTI15 | 输入 | KEY1_N | 延续现有 KEY；按下接地 |
| 外置按键 2 | PE0 | GPIO/EXTI0 | 输入 | KEY2_N | 新增；按下接地 |
| 激光测距时钟 | PA8 | I2C3_SCL | 开漏双向 | LASER_SCL | 已验证 |
| 激光测距数据 | PC9 | I2C3_SDA | 开漏双向 | LASER_SDA | 已验证 |
| 激光测距使能 | PC8 | GPIO | 输出 | LASER_XSHUT | 低电平关闭 VL53L0X |
| 视觉模块发送 | PA9 | USART1_TX | 输出 | VISION_TX | 接视觉模块 RX |
| 视觉模块接收 | PA10 | USART1_RX | 输入 | VISION_RX | 接视觉模块 TX |
| 陀螺仪发送 | PC10 | USART3_TX | 输出 | IMU_TX | 接串口陀螺仪 RX |
| 陀螺仪接收 | PC11 | USART3_RX | 输入 | IMU_RX | 接串口陀螺仪 TX |
| 云台上方轴 PWM | PA2 | TIM5_CH3 | 输出 | GIMBAL_TILT_PWM | 接上方舵机黄线 |
| 云台下方轴 PWM | PA3 | TIM5_CH4 | 输出 | GIMBAL_PAN_PWM | 接下方舵机白线；暂不启用 |
| 板载状态灯 | PC13 | GPIO | 输出 | STATUS_LED_N | 低电平点亮，驱动能力较弱 |
| SWD 数据 | PA13 | SWDIO | 双向 | SWDIO | 不复用 |
| SWD 时钟 | PA14 | SWCLK | 输入 | SWCLK | 不复用 |
| 芯片复位 | NRST | NRST | 输入 | NRST | 接 ST-Link，并预留复位按键 |
| 外部晶振输入 | PH0 | OSC_IN | 输入 | HSE_IN | 8 MHz 晶振 |
| 外部晶振输出 | PH1 | OSC_OUT | 输出 | HSE_OUT | 8 MHz 晶振 |

## 3. D24A 四路 TB6612 接口对应关系

### J4：前轮驱动与前轮编码器

| D24A J4 针脚 | D24A 信号 | F407 引脚 |
| ---: | --- | --- |
| 1 | 3V3 | 不与 F407 3.3 V 直接并联，预留测试点 |
| 2 | STBY | PC0 |
| 3 | PWMB | PE11 |
| 4 | PWMA | PE9 |
| 5 | BIN2 | PD3 |
| 6 | AIN2 | PD1 |
| 7 | BIN1 | PD2 |
| 8 | AIN1 | PD0 |
| 9 | E2A | PA6 |
| 10 | E1A | PB6 |
| 11 | E2B | PA7 |
| 12 | E1B | PB7 |

### J6：后轮驱动与后轮编码器

| D24A J6 针脚 | D24A 信号 | F407 引脚 |
| ---: | --- | --- |
| 1 | ADC | 暂不连接，预留测试点 |
| 2 | GND | F407 GND |
| 3 | PWMD | PE14 |
| 4 | PWMC | PE13 |
| 5 | DIN2 | PD7 |
| 6 | CIN2 | PD5 |
| 7 | DIN1 | PD6 |
| 8 | CIN1 | PD4 |
| 9 | E4A | PA0 |
| 10 | E3A | PC6 |
| 11 | E4B | PA1 |
| 12 | E3B | PC7 |

## 4. 外设连接器定义

### OLED：J_OLED，4 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | 3V3 |
| 2 | GND |
| 3 | PB8 / OLED_SCL |
| 4 | PB9 / OLED_SDA |

### 两个外置按键：J_KEYS，4 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | PA15 / KEY1_N |
| 2 | GND |
| 3 | PE0 / KEY2_N |
| 4 | GND |

每个按键按下时将对应输入短接到 GND。PCB 上各放置 10 kΩ 上拉到 3.3 V 和 100 nF 对地电容；软件仍保留消抖。

### 八路循迹：J_LINE，6 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | 5V |
| 2 | GND |
| 3 | PB10 / LINE_OUT |
| 4 | PB0 / LINE_AD0 |
| 5 | PB1 / LINE_AD1 |
| 6 | PB2 / LINE_AD2 |

### 激光测距：J_LASER，6 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | 3V3 |
| 2 | GND |
| 3 | PA8 / LASER_SCL |
| 4 | PC9 / LASER_SDA |
| 5 | PC8 / LASER_XSHUT |
| 6 | NC，预留模块 INT |

### 视觉模块：J_VISION，3 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | GND |
| 2 | PA9 / VISION_TX，接视觉 RX |
| 3 | PA10 / VISION_RX，接视觉 TX |

视觉模块继续使用自身 Type-C 供电。连接器不引出 F407 电源，防止误供电和电源倒灌。

### 串口陀螺仪：J_IMU，4 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | 可选 3V3/5V，由跳线选择 |
| 2 | GND |
| 3 | PC10 / IMU_TX，接陀螺仪 RX |
| 4 | PC11 / IMU_RX，接陀螺仪 TX |

UART 电平必须是 3.3 V。若陀螺仪 TX 输出 5 V，必须经过电平转换后再进入 PC11。具体供电电压和波特率在确定陀螺仪型号后填写，不能直接假定。

### 二维云台：两个标准 3 针舵机接口

| 接口 | 针脚 1：信号 | 针脚 2：电源 | 针脚 3：地 |
| --- | --- | --- | --- |
| J_GIMBAL_TILT | PA2 / GIMBAL_TILT_PWM，上方轴黄线 | 5V_SERVO，上方轴红线 | GND，上方轴棕线 |
| J_GIMBAL_PAN | PA3 / GIMBAL_PAN_PWM，下方轴白线 | 5V_SERVO，下方轴红线 | GND，下方轴黑线 |

接口丝印明确标注 `S / + / -`，并增加防反插结构。PA2、PA3 各串联 1 kΩ 电阻。5V_SERVO 入口旁放置 1000 µF 电解电容和 100 nF 陶瓷电容。舵机电流不能经过 F407 的 5 V 引脚或细信号线。

### SWD：J_SWD，5 针

| 针脚 | 信号 |
| ---: | --- |
| 1 | 3V3 目标电压检测 |
| 2 | PA13 / SWDIO |
| 3 | PA14 / SWCLK |
| 4 | NRST |
| 5 | GND |

## 5. 电平与保护要求

1. **编码器输入**：D24A 给 GT50 编码器提供 5 V，E1～E4 的 8 路信号进入 F407 前统一经过 3.3 V 电平转换。推荐使用两个 74LVC245，方向固定为 D24A → F407；也可为每路预留分压电阻。
2. **循迹 OUT**：模块使用 5 V 供电。LINE_OUT 进入 PB10 前放置电平转换，或至少预留 10 kΩ 上臂、20 kΩ 下臂的分压焊盘。
3. **循迹地址线**：PB0～PB2 输出 3.3 V。当前实物直连已经工作；正式 PCB 建议预留 74AHCT125，将 3.3 V 地址信号转换为 5 V。
4. **电机控制线**：当前 3.3 V 直连 D24A 已通过实车测试。正式 PCB 可在 PD0～PD7、PE9/PE11/PE13/PE14、PC0 与 D24A 之间预留 74HCT245/125 缓冲位置，提高 5 V 逻辑裕量。
5. **UART**：视觉和陀螺仪 UART 均按 3.3 V TTL 设计，TX、RX 各串联 100～1 kΩ 保护电阻。不得连接 RS-232 电平。
6. **I²C**：OLED 的 PB8/PB9、激光的 PA8/PC9 分别使用独立的 4.7 kΩ 上拉到 3.3 V，不能上拉到 5 V。
7. **舵机电源**：D24A 5 V 可用于首次单舵机空载测试，但原理图未标注可持续输出电流。PCB 应预留独立 5 V 大电流降压模块接口和电源选择跳线。

## 6. 电源网络建议

| 电源网络 | 用途 | 说明 |
| --- | --- | --- |
| 12V_MOTOR | D24A 电机驱动输入 | 来自电池；不要连接任何传感器或舵机 |
| 5V_SERVO | 二维云台舵机 | 优先使用独立大电流降压；与逻辑系统共地 |
| 5V_SENSOR | 八路循迹模块 | 可与 D24A 5 V 共源，但建议经过保险丝或磁珠 |
| 5V_MCU | F407 核心板 5 V 输入 | 与 ST-Link 5 V 之间必须设置跳线、二极管或电源选择开关 |
| 3V3 | OLED、VL53L0X 和逻辑电路 | 不能给舵机和视觉模块供电 |

所有 GND 最终相连。电机与舵机的大电流回流应直接回到电源入口，不能先经过 F407 或传感器地线；建议使用星形接地或完整地平面。

## 7. 特殊引脚说明

- PA13、PA14 仅用于 SWD，不连接其他模块。
- PA15 原为 JTAG JTDI。项目只使用 SWD，因此继续作为 KEY1，既保留现有软件又不影响烧录。
- PB2 兼作 BOOT1。BOOT0 固定下拉后，PB2 可正常作为 LINE_AD2；循迹模块一侧是输入，不应在复位时反向驱动 PB2。
- PC13 只驱动板载 LED，不用于继电器、蜂鸣器或大电流负载。
- PH0、PH1 只连接 8 MHz 晶振和匹配电容。
- PC14、PC15 暂不使用，保留给 32.768 kHz RTC 晶振。

## 8. 外设资源检查

| 资源 | 用途 | 冲突检查 |
| --- | --- | --- |
| TIM1 CH1～CH4 | 四路电机 PWM | 独占，无冲突 |
| TIM2 CH1/CH2 | 右后编码器 | 独占，无冲突 |
| TIM3 CH1/CH2 | 右前编码器 | 独占，无冲突 |
| TIM4 CH1/CH2 | 左前编码器 | 独占，无冲突 |
| TIM5 CH3/CH4 | 云台上、下轴 | 同为 50 Hz，无冲突 |
| TIM8 CH1/CH2 | 左后编码器 | 独占，无冲突 |
| USART1 | 视觉模块 | PA9/PA10，独占 |
| USART3 | 串口陀螺仪 | PC10/PC11，独占 |
| I2C1 | OLED | PB8/PB9，独占总线 |
| I2C3 | VL53L0X | PA8/PC9，独占总线 |
| EXTI15 | KEY1 | PA15 |
| EXTI0 | KEY2 | PE0；PA0 仍只作 TIM2 输入 |

以上分配没有 GPIO 重复占用。若后续更换模块通信方式，应先更新本表，再修改原理图和软件，避免只改其中一处。
