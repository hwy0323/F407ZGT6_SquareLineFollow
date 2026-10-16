# 双层版 3-D：BC 库内出发，到 D 停车

2026-10-17 整套执行后用户反馈“没问题”，本次双层整车到 D 现场通过。没有启用视觉，没有套用 3-C 的末段合并改动。

## 路线和操作

从 BC 倒车库内出发，车头朝外、八路全亮 FF，执行 F100→R50→F180→R60→F200→F190→B37→R68；F／B 单位 mm，R 单位度。之后低速 CD 循迹，探测到 D 弯时停车，不在 D 转弯、不继续去 A。

上电停机，短按并松开 PA15 KEY 或 DAP 串口发送 G 启动；X／KEY 停止。T 只适用于已经过 C 的 CD 起点，不能在库内发 T。READY 是 TASK3_MERGED68_OFFSET_TO_D。启动条件包括 FF、灰度及 IMU 数据新鲜。STBY 为 PB15，[接线表](../接线.md)。

## 本次记录

- [完整串口](logs/task3_D_20261017_045006.csv)；[烧录校验](logs/flash_20261017_045006.txt)。
- 八步完成 7340 ms，CD 接管灰度 10；接管后 4326 ms 报 D_REACHED_FROM_BC_GARAGE，电机命令为 0。用户随后认可现场结果。
- 实际三次右转约 51.33°／64.56°／72.70°；后两次 OVERSHOOT，各直线 DISTANCE_OVERSHOOT 均保留。这是旧控制器的既有处理，不是精度保证。

## 来源与编译

main.c、route_data.h 和启动文件原样来自 10.12 PB15 task3_D；固件只重命名，内容保持此次实车烧录文件不变。下载整个仓库，在此目录 make all，依赖上级 build.mk 和根目录 Core／Drivers／链接脚本。BIN／ELF 在固件目录。

BIN SHA256：1C732069D5D5232EB318DCF8FAE013BFA3CC7FA7A9D03CB080B35259C3902177。
