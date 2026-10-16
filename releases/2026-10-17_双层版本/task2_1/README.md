# 双层版本 2-1：A 点出发，BC 倒车入库

[BIN](固件/F407ZGT6_Task2_1_ReverseEntry_Remote_STBY_PB15.bin) · [ELF](固件/F407ZGT6_Task2_1_ReverseEntry_Remote_STBY_PB15.elf) · [main.c](main.c) · [路线表](route_data.h) · [接线](../接线.md)

从 A 点沿 AB 出发，转过 B 真弯后找 BC 库线；定位停车后执行 `F209 → L090 → B200`，入库后停止，不出库。F／B 单位为 mm，L／R 单位为度。保留 BC 倒车库线，撤除 DA 侧方库线。

按 PA15 KEY 或 DAP 单字节 G 启动，KEY／X 停止；上电保持停车。本版未启用视觉或菜单，不是任务三的从 BC 库内出发版本。

原始整套记录 [run_20261017_002432.csv](logs/run_20261017_002432.csv) 包含转 B、找库线、三步入库，最终 `EVENT,TASK2_1_DONE,garage_entry_only,motors_off`。用户现场认可。原控制器的步骤末端分别记有 DISTANCE_OVERSHOOT／OVERSHOOT，路线控制器按原逻辑继续并正常结束，不删除或重写这些事件。

来源为 PB15 任务 2-1，主要增加电脑串口启动方式，本次不改动实测入库动作。编译见 [总览](../README.md)。
