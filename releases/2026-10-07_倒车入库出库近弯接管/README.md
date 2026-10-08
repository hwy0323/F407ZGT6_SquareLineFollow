# 10.7 倒车入库、出库后近弯低速接管

这是 2026-10-07 实车验证的固定路线版本：小车从第一个真实直角弯前出发，循迹转弯，在倒车入库库线前停车；接着倒车入库、出库；出库末段直行 200 mm 后切换低速循迹，通过紧邻的下一处真实右直角弯并停车。现场反馈入库、出库、转弯位置正确，没有压车库线。

此流程由电脑运行 PowerShell 脚本，通过 DAP 串口和 SWD 依次切换三个固件。不是烧录单个 BIN、按一次 KEY 就能完成的独立程序。该成功结论只对应本次起点、方向、线束和胶带位置。

## 下载内容

[完整工程、固件和日志 ZIP](10.7倒车入库出库近弯接管_实测成功.zip) 的目录结构如下：

```text
项目/
  F407ZGT6_SquareLineFollow/             共用 HAL、CMSIS 和链接脚本
  F407ZGT6_OneCorner_To_ParkingStart/    首个真弯、库线前停车
  F407ZGT6_FullParking_F200_Handoff_2026-10-07/  入库、出库九步
  F407ZGT6_PostExit_SlowAlign_2026-10-07/        低速接管、下一弯停车
本次串口日志/                              三个阶段的原始 CSV
README.md
```

三个固件目录均含现场对应的 `main.c`、`Makefile`、启动代码、`build/*.elf` 和 `build/*.bin`。入库、出库目录还含 `route_data.h` 和整套执行脚本 `run_full_parking_then_corner_align.ps1`。压缩包没有收入同目录里未经实测确认的提速或九弯脚本。

## 实测流程

1. 首个真弯前启动 `F407ZGT6_OneCorner_To_ParkingStart`，过弯后在车库线前停车。本次日志的停车事件为 `PARKING_START_READY,turns=1`，车库线灰度 `0F`，距弯后零点约 1716 编码器 A 计数。
2. 脚本烧录 `F407ZGT6_FullParking_F200_Handoff_2026-10-07` 并执行 9 步：`F209 → L090 → B200 → G200 → F100 → R050 → F180 → R060 → F200`。F/B/G 单位为 mm，L/R 单位为度。第 4 步 `G200` 是低速向前寻找八路全黑 `FF`，上限 200 mm，并非固定直行 200 mm。每步停车约 0.2 秒；脚本还要检查串口结果，所以实际阶段间隔可能更长。
3. 最后一步结束时，探头读到 `0x18`（中间两路）。脚本切换 `F407ZGT6_PostExit_SlowAlign_2026-10-07`，预检中线和停机状态后低速启动。约 0.30 秒识别右弯，转弯后灰度由 `0x70`、`0x30` 回到 `0x18`，最后报告 `ONE_CORNER_COMPLETE,turns=1` 并停车。

对应原始证据：

- 首弯及库线停车：压缩包内 `本次串口日志/parking_start_20261007_223544.csv`
- 入库与出库九步：压缩包内 `本次串口日志/full_parking_20261007_223549.csv`
- 出库后低速循迹接弯：压缩包内 `本次串口日志/post_exit_20261007_223601.csv`
- 现场复测说明：压缩包内 `本次串口日志/第三轮低速复测记录.md`

这次没有发生丢线、超时或里程保护停车。此前 220 mm 试验曾停在真弯宽黑区 `0x3E`，被接管预检拒绝；本成功版本用 200 mm，并保留宽黑区预检。

## 核对固件

压缩包内实测 BIN 的 SHA-256：

| 阶段 | SHA-256 |
| --- | --- |
| 首弯、库线前停车 | `4DCBF3C356FA32DE016B3A3321502B4F9A21F1ED57DB361F2CFCB5972CEED27C` |
| 入库、出库九步 | `DDBCA608E1CBE7A06B5E7D087F9816E7487E2FFEADADAF642A1767B5106323E2` |
| 出库后低速接弯 | `1E4AEC335DBBB6C0BC09ABD6D06DE51885439CD9100A29A985A91BFD14827C8F` |

执行脚本会逐一核对这些哈希，文件变化时拒绝烧录。脚本 `-DryRun` 可查看执行顺序；它不会下载或驱动车辆。

## 编译和运行

保持 ZIP 内 `项目` 下四个目录为同级。在各固件目录使用 STM32CubeIDE 随附 GNU Arm 工具链和 GNU `make` 执行 `make -B`；`Makefile` 从相邻的 `F407ZGT6_SquareLineFollow` 加载共用文件。换电脑时需要修改三个 Makefile 的 `TOOL_ROOT` 安装路径。

整套脚本默认串口 `COM14`、115200 8N1，OpenOCD 路径指向原测试电脑的 `F:\cbide`，还固定了当时的 DAP 序列号。换设备时核对并修改脚本开头的串口、OpenOCD 路径与 DAP 序列号；重编固件后也须更新脚本中对应 BIN 哈希。实际复现时将小车放回相同的真弯前位置、车头沿主轨道朝向弯道，再在现场有人可立即断开电机电源的条件下运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\run_full_parking_then_corner_align.ps1 -PortName COM14 -DryRun
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\run_full_parking_then_corner_align.ps1 -PortName COM14
```

上述命令须在 `项目/F407ZGT6_FullParking_F200_Handoff_2026-10-07` 内执行。正式运行会连续烧录三个固件并启动车辆，执行后 MCU 中留下的是第三阶段的低速接弯固件。

接线沿用[仓库首页当前实测线束](../../README.md#当前实测线束)：电机 A/B/C/D 对应左前、左后、右后、右前；灰度 AD0/AD1/AD2/OUT 为 PB12/PB13/PB14/PC1；IMU 使用 UART4 的 PC10/PC11；DAP 串口使用 USART1 的 PA9/PA10。电机、主控、传感器与 DAP 共地。
