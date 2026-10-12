# STBY 统一迁移到 PB15

日期：2026-10-12。四路 TB6612 的公共 STBY 从 PC0 改为 PB15，其余接线、速度、里程、IMU 转向控制、循迹和停车路线均不改。

这里的 11 个工程是当前 PB15 线束入口。旧 PC0 源码、固件、ZIP 和测试记录保留，不覆写；新版本只完成编译和离线检查，尚未烧录或实车复测。旧版成功记录说明行驶逻辑的来源，不代表 PB15 新版已经实测成功。

## 工程和固件

| 项目 | 主程序 | BIN | 启动与起点 |
| --- | --- | --- | --- |
| 1-1：A→B，到 B 停车 | [main.c](task1_1/main.c) | [PB15 固件](固件/F407ZGT6_Task1_AB_STBY_PB15.bin) | A 点朝 B，KEY |
| 1-2：A 点出发绕场一周回 A | [main.c](task1_2/main.c) | [PB15 固件](固件/F407ZGT6_Task1_OneLap_STBY_PB15.bin) | A 点朝 B，KEY |
| 2-1：A 点出发，BC 倒车入库后停车 | [main.c](task2_1/main.c) | [PB15 固件](固件/F407ZGT6_Task2_1_ReverseEntry_STBY_PB15.bin) | A 点朝 B，KEY；只保留 BC 库线 |
| 2-2：A 点出发，DA 侧方入库后停车 | [main.c](task2_2/main.c) | [PB15 固件](固件/F407ZGT6_Task2_2_SideEntry_STBY_PB15.bin) | A 点朝 B，KEY；只保留 DA 库线 |
| 任务三：BC 库内出发，固定到 A | [main.c](task3_A/main.c) | [PB15 固件](固件/F407ZGT6_Task3_ToA_STBY_PB15.bin) | BC 库内车头朝外，KEY 或 G；启动前探头全亮 |
| 任务三：BC 库内出发，固定到 B | [main.c](task3_B/main.c) | [PB15 固件](固件/F407ZGT6_Task3_ToB_STBY_PB15.bin) | BC 库内车头朝外，KEY 或 G；启动前探头全亮 |
| 任务三：BC 库内出发，固定到 C | [main.c](task3_C/main.c) | [PB15 固件](固件/F407ZGT6_Task3_ToC_STBY_PB15.bin) | BC 库内车头朝外，KEY 或 G；启动前探头全亮 |
| 任务三：BC 库内出发，固定到 D | [main.c](task3_D/main.c) | [PB15 固件](固件/F407ZGT6_Task3_ToD_STBY_PB15.bin) | BC 库内车头朝外，KEY 或 G；启动前探头全亮 |
| 10.7 正方形成功循迹，四弯后停车 | [main.c](square/main.c) | [PB15 固件](固件/F407ZGT6_FourCorner_GarageLine_Test_STBY_PB15.bin) | B 点朝 C，KEY 或 G，四弯一圈 |
| 串口单步转向和直行控制 | [main.c](manual/main.c) | [PB15 固件](固件/F407ZGT6_Manual_Control_STBY_PB15.bin) | 串口发送 L／R／F／B，KEY 或 X 停止 |
| OLED 菜单测试，电机始终禁用 | [main.c](oled/main.c) | [PB15 固件](固件/F407ZGT6_OLED_Menu_STBY_PB15.bin) | PA15 切换／PE6 确认，仅显示，电机禁用 |

ELF 和 BIN 同名，位于 [固件目录](固件/)。下载整个仓库再重编，不能只复制 main.c；共用 HAL、CMSIS、main.h 和链接脚本位于仓库根目录。

任务三仍是四个固定目标固件，不启用视觉；到 C 仍保留旧版 LINE_LOST 保护停机，到 B 仍保留 AB 里程上限保护停车。没有借改引脚的机会修改到点判定。各旧版验证范围见 [10.11 核对清单](../2026-10-11_基础任务上传核对.md)。

## 迁移内容

- MOTOR_STBY_PORT 定义为 GPIOB，MOTOR_STBY_PIN 定义为 GPIO_PIN_15。
- 先开启 GPIOB 时钟，预置输出低电平，再把 PB15 初始化为低速推挽输出、内部下拉。
- PC13 状态灯单独初始化；不再把 PC0 和 PC13 一起初始化。
- 行驶使能、正常停车、急停和错误处理都使用 PB15。PWM、方向引脚、PA15 KEY 不变。
- OLED 菜单也配置 PB15 为低，避免只改行车代码、菜单仍把驱动使能脚悬空。

## 编译和验证

本机运行：

```powershell
pwsh -NoProfile -File .\compile_all.ps1
python .\test_pb15.py
```

编译脚本在每个工程目录执行完整 make 编译，复制新 BIN／ELF 到固件目录，保存编译日志和 SHA256SUMS.txt。它不连接 SWD、串口，不烧录、不复位、不启动车辆。更换电脑时可用 -MakePath 指定 make 路径，或在单个工程目录执行 make TOOL_ROOT=实际ARM工具链目录。

离线检查逐一对照旧源码，要求主程序除 STBY 定义、写引脚和初始化外完全一致；启动文件、路线表及 C 弯接管头文件也必须原样一致。OLED 新副本与根目录同步。

## 接线与首次测试

1. 全部断电，把驱动板 STBY 从 PC0 移到 PB15。不要同时接 PC0、PB15、3V3 中的多个点。
2. 电机和所有模块仍共地；其余引脚见 [接线表](接线.md)。
3. 首次验证先架空车轮，烧录明确带 STBY_PB15 后缀的固件；确认复位后不自行转动、停止指令能关断电机，再落地测轨迹。
4. 本次未操作烧录器，小车芯片里的程序还是之前的版本。移线后，旧 PC0 固件不能控制 PB15。
