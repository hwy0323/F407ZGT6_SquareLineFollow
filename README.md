# 菜单工程

2026-10-10 当前已烧录版本的完整交付包。解压后在本目录编译和烧录，HAL、CMSIS、头文件和链接脚本已包含在 Support 中。

SSD1306 0.96 寸 128×64，PB8=SCL、PB9=SDA，模块按用户确认接 5VOUT 和 GND。ST-Link 烧录，PA15/PE6 为上拉输入、按下低电平。

## 菜单行为

顺序固定为：`1-1 → 1-2 → 2-1 → 2-2 → 3 → 4 → 5 → 6-1 → 6-2 → 6-3 → 6-4 → 1-1`。

- 每次只绘制三个项目。中间为选中项目，3 倍字、外加方框；上下为相邻项目，2 倍字。
- PA15 按下并松开：向下选择一项。原上项消失，原中项上移，原下项上移进入中间，新的下一项从底部进入，动画约 240 ms。
- 列表首尾相连。初始中间为 1-1，上方为 6-4，下方为 1-2。
- PE6 按下并松开：确认当前项目，画面只显示该项目编号，5 倍大字。
- 用户明确选择“先测试菜单”：当前确认后仅演示编号 2 秒，然后自动返回菜单，选项仍停在刚才确认的项目。
- 演示期间忽略选择/确认，按键事件不会排队到演示结束。运行中的按键必须重新松开、再按才能在菜单触发。
- 同时松开两个按键时，确认优先。两路各自消抖 30 ms，上电时已按住的键不会误触发。

本测试版只验证界面与按键。各编号尚未接入实际循迹/停车任务。主控将电机使能 PC0 保持关闭，PD0～PD7 保持低电平；先前出库和四轮测试工程另目录保留。

## 工程

`main.c`：按键、时钟、主循环及主控诊断。

`menu_ui.c/.h`：十一项循环菜单、滚动、确认及自动返回。接实际任务时，用任务开始/完成状态替换 MENU_PREVIEW 的 2 秒演示，不修改选中索引。

`oled.c/.h`：SSD1306 驱动、字体、裁剪大字与框线。动画允许部分字符出屏幕，像素坐标按有符号范围裁剪，避免越界像素从另一边绕回。

I2C1 使用 400 kHz，OLED 每次写入约 25 ms，滚动约 40 ms 一帧。菜单保持静态时只在画面改变或一秒刷新时写入。

```powershell
.\build_flash.ps1                       # 编译
.\build_flash.ps1 -Flash -Probe stlink  # 编译、烧录、读回校验
```

Makefile 使用包内 `Support` 的 HAL/CMSIS 和链接脚本。ST-Link 使用已经修正的 stlink-dap.cfg / dapdirect_swd，SWD 100 kHz。

## RAM 运行状态

`menu_test_status` 为 20 个 uint32。依次为魔数 0x4D454E55、程序阶段、运行毫秒数、OLED 初始化次数、地址、ready、HAL 状态、I2C 错误、刷新成功次数、刷新失败次数、选中索引（0～10）、界面状态（0菜单/1滚动/2演示）、PA15次数、PE6次数、待滚动步数、演示开始次数、完成次数、PA15电平、PE6电平、按键准备位图。

ST-Link 可只读查看状态；真正文字显示、滚动感受及物理按键效果由现场验收。菜单测试时不要求恢复已拔掉的 COM9。

## 验证记录

2026-10-10 已编译并通过 ST-Link 烧录，OpenOCD 返回 Verified OK。BIN SHA256：93383B46686E795527AE3D165004448C9CED258E075AC7DB722C9B0896AD0F93。

已只读确认运行魔数正确、阶段3、OLED地址0x3C、ready=1、HAL状态0、I2C错误0、刷新失败0；当前选中索引0，菜单状态0，PA15/PE6均处于松开高电平且就绪。RAM中的1024字节屏幕缓冲区已导出并生成 验证记录/menu_framebuffer.png，画面显示6-4、框内1-1、1-2。状态和原始图像见 验证记录/runtime_menu_status.log、验证记录/oled_framebuffer.bin。


## 本包编译、烧录和文件说明

主控为 STM32F407ZGT6。本包使用 STM32CubeIDE 自带的 ARM GCC 和 Make 从命令行构建；进入解压后的“菜单工程”目录执行 build_flash.ps1，直接编译已有 main.c、menu_ui.c 和 oled.c，无需额外生成源码。

默认 CubeIDE 路径为 D:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE。其他安装位置通过 -IdeRoot 指定，例如：

```powershell
.\build_flash.ps1 -IdeRoot 'C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE'
.\build_flash.ps1 -Flash -Probe stlink
.\build_flash.ps1 -FlashOnly -Probe stlink
```

- build/F407ZGT6_OLED_Menu.elf 和 .bin：可直接烧录的菜单测试固件；.map 为链接映射。
- Support：配套 HAL/CMSIS、系统时钟文件、头文件及 STM32F407ZGTx_FLASH.ld。
- 验证记录：原烧录校验记录、RAM状态和实际MCU屏幕缓冲区截图。
- SHA256SUMS.txt：包内文件的 SHA256 清单。

打包版只调整依赖路径和说明，没有修改 MCU 控制源码。重新编译的 BIN 与已烧录版本的 SHA256 一致，见验证记录/打包编译.log。本版仍仅用于菜单测试：PE6 确认显示编号 2 秒，不执行车辆运动任务。
