# STM32 独立巡线底盘

STM32F103RCT6（72 MHz，STM32CubeF1 HAL，Keil 工程 `MDK-ARM/ros_stm32.uvprojx`）。底盘独立完成巡线、过弯、计圈和停车：沿 100 cm 方形赛道逆时针跑，每个角左转 90°。8 路灰度测横向偏差和拐角，编码器做轮速闭环和里程，ICM-40607 的 gz 用来判断每个弯转了多少度。运动状态每 20 ms 用 `$C` 帧发给云台。

代码基础是作者自己的 ROS 2 小车固件（电机、编码器、IMU 和轮速环沿用）。原来的 USART2 速度命令入口还在（`CHASSIS_AUTONOMOUS_ENABLE=0`），本项目不用。

上电自启动和 PB5 模式按键见 [docs/GIMBAL_TUNING.md](../docs/GIMBAL_TUNING.md) 第 2 节，设计思路见 [docs/CHASSIS_SCHEME.md](../docs/CHASSIS_SCHEME.md)，`$C`/`$G` 帧格式见 [docs/GIMBAL_COMM_PROTOCOL.md](../docs/GIMBAL_COMM_PROTOCOL.md) 第 4 节。

## 接线与串口

| 用途 | 引脚 | 外设与说明 |
|---|---|---|
| 左电机（TB6612） | PA0 / PA1 | TIM2_CH1 / CH2，双 PWM 约 80 kHz；STBY 由电机板固定使能 |
| 右电机（TB6612） | PA6 / PA7 | TIM3_CH1 / CH2 |
| 左编码器 A / B | PB6 / PB7 | TIM4 编码器模式 |
| 右编码器 A / B | PC6 / PC7 | TIM8 编码器模式 |
| ICM-40607 | PB10 SCL / PB11 SDA | I2C2 400 kHz，地址 0x68，WHO_AM_I 为 0x38 |
| 巡线地址 AD0 / AD1 / AD2 | PC0 / PC1 / PC2 | 输出，选通 8 路中的一路 |
| 巡线 OUT | PB0 | 输入 |
| 云台链路 | PA2 TX → 云台 PA10 RX（`$C`）；PA3 RX ← 云台 PA9 TX（`$G`） | USART2，921600 8N1，共地；云台端是丝印 USART1 的 3 针口 |
| 调试 / 蓝牙 | PC12 TX / PD2 RX（H4 接口） | UART5，115200 8N1，无流控；PD2 开了内部上拉，模块拔掉后不会收到乱码 |
| 模式按键 | PB5 | 内部上拉，按下接地：上电自启动出发前切换打靶心 ↔ 画圆 |
| 蜂鸣器 | PB1 | 高电平响：模式按键响 1 声 = 打靶心，2 声 = 画圆 |
| RGB 灯 | PB4 红 / PC5 绿 / PC8 蓝 | 红 = 故障，绿 = 运行，蓝 = 待机或完成 |

H4：2 = PD2（接模块 TX），3 = PC12（接模块 RX），4 = GND，5 = 5V，1 / 6 未接。可以直接接 3.3 V TTL 的 USB 转串口，也可以接蓝牙或 ESP32 透传模块（设成 115200 8N1、无流控）。模块芯片本身不能直接用 5 V 供电或 5 V 串口电平。

`CHASSIS_DEBUG_UART=2` 时调试口改到 USART2（PA2 / PA3），云台链路自动改到 UART4（PC10 TX / PC11 RX，`CHASSIS_GIMBAL_UART 4`）。

巡线模块沿用旧驱动约定：黑线高电平、bit0 为最左、bit7 为最右；装车时用 STATUS 的 mask 核对实际极性和通道方向。`User/Bsp/bsp_line.c` 每路选通后至少等 500 µs 再读一个通道，不阻塞等待；完整 8 路才发布快照。主循环空闲 WFI、SysTick 1 ms，完整扫描通常约 8 ms 或更长，超过 30 ms 的快照无效。连续确认依据新扫描序号，不重复计数同一帧。

## 默认参数

任务参数在 `User/chassis_config.h`，车体参数在 `User/board_config.h`，每个宏旁边的注释写了选这个值的实测依据。表里有 `SET` 名字的，可以在停车时用调试口 `SET <名字> <值>` 改（只在 RAM，重新上电恢复默认），`PARAMS` 查看当前值。

| 宏 | 默认 | `SET` | 含义 |
|---|---|---|---|
| `CHASSIS_CRUISE_M_S` | 0.35 m/s | `cruise` | 直线速度 |
| `CHASSIS_TURN_M_S` | 0.14 m/s | `turn` | 过弯速度（也可以用 `TURN <cm/s>`） |
| `CHASSIS_FIRST_EDGE_M_S` | 0.20 m/s | — | 第 1 个弯之前限速（起点到第 1 个弯的距离未知）；0 = 不限 |
| `CHASSIS_RADIUS_M` | 0.062 m | — | 转弯半径 |
| `CHASSIS_ACCEL_M_S2` / `CHASSIS_DECEL_M_S2` | 0.8 / 0.6 m/s² | `accel` / `decel` | 线加速、减速 |
| `CHASSIS_ANGULAR_ACCEL_RAD_S2` | 8 rad/s² | `tacc` | 弯里的角加速度 |
| `CHASSIS_LINE_ANGULAR_ACCEL_RAD_S2` | 3 rad/s² | `wacc` | 直道上角速度的变化率，太大时车头会甩，云台误差翻倍 |
| `CHASSIS_LINE_KP` / `CHASSIS_LINE_KD` | 0.020 / 0.002 | `kp` / `kd` | 巡线 PD |
| `CHASSIS_LINE_KH` | 1.5 | `kh` | 航向保持增益（每条边方向 = 已过弯数 × 90°） |
| `CHASSIS_YAW_RATE_KG` | 2.0 | `kg` | 直道上的陀螺角速度环，抑制出弯后的残余转动；0 = 关 |
| `CHASSIS_EDGE_M` | 1.00 m | `edge` | 名义拐角触发间距，对应 100 cm 赛道（实测平均 1.063 m）；换赛道时修改，宏设 0 关闭提前减速 |
| `CHASSIS_CIRCLE` | 0 | `circle` | 上电默认任务：0 = 打靶心，1 = 画圆；PB5 按键可切换 |
| `CHASSIS_AUTOSTART_LAPS` | 1 | — | 上电自启动跑几圈；0 = 关闭，只能发 RUN |
| `CHASSIS_SENSOR_AHEAD_M` | 0.095 m | — | 探头排到驱动轮轴的距离（实测） |
| `CHASSIS_PROBE_SPACING_MM` | 11.14 mm | — | 探头间距（实测，ch0～ch7 共 78 mm） |
| `BOARD_WHEEL_SEPARATION_M` | 0.115 m | — | 轮距（实测） |
| `BOARD_WHEEL_CIRCUMFERENCE_LEFT_M` / `_RIGHT_M` | 0.22627 / 0.23155 m | — | 左右轮周长，分别标定 |

编码器 13 线 × 30 减速 × 4 倍频 = 每轮 1560 count。拐角前按「刹车距离 + 车体滞后余量（过弯速度 × 0.20 s）+ 0.04 m」最晚开始减速。

开关：

- `CHASSIS_AUTONOMOUS_ENABLE=1`：使用本地巡线任务；0 编译回原来的 USART2 速度命令入口。
- `CHASSIS_HARDWARE_CONFIRMED=1`：允许 RUN / DRIVE。新装的车先设 0，只看 STATUS，接线、方向和几何参数确认后再改回 1。
- `CHASSIS_KEYS_ENABLE=0`：不初始化三键（LAPS / RUN / STOP，引脚未分配；PC0～PC2 已用于巡线）。PB5 模式按键与它无关，始终启用（`User/Algorithm/mode_key.c` 去抖和蜂鸣节奏，按完至少等 `CHASSIS_AUTOSTART_KEY_HOLD_MS` 才出发）。
- `HARDWARE_TEST_MODE`（`Core/Src/main.c`）：硬件测试模式，有自己的运动逻辑，正常使用保持 `HARDWARE_TEST_MODE_NORMAL`。

## 操作与日志

调试口发送大写 ASCII 命令，以回车或换行结束。上电自启动之前发任何一条命令，都会取消这次自启动。

| 命令 | 作用 |
|---|---|
| `STATUS` | 立即输出状态（平时每秒输出一次） |
| `LAPS 1`～`LAPS 5` | 设置 RUN 的圈数，只在停车时有效；上电默认 1 |
| `RUN` / `STOP` | 开跑 / 停车并清除故障 |
| `DRIVE <cm/s> <cm>` | 直线走一段后停车，不需要黑线和云台链路，例如 `DRIVE 20 60` |
| `TURN <cm/s>` | 设置过弯速度（只在 RAM） |
| `SET <名字> <值>` / `PARAMS` | 修改 / 查看任务参数，名字见上表 |
| `CIRCLE <s>` | 车不动，画圆进度在 `<s>` 秒内走完 1 圈，用来静态检查画圆；`CIRCLE 0` 结束 |
| `TRACE` | 打印最近 32 次状态或 mask 变化（`状态/mask@秒`），故障后冻结，DONE 后保留 |
| `LOG ON` / `LOG OFF` | 开关运行日志 |

`LOG ON` 后每次运行先输出一行 `H,` 表头，运行中每 20 ms 输出一行 `D,` 数据（时间、状态、mask、原始/滤波巡线误差、v/w 指令与实测、航向误差、左右轮目标/实测、占空比，均为整数：mm/s、mrad/s、0.1 mm），停车那一刻再补一行。把串口输出存成文本后运行 `py -3.13 chassis/host/plot_run_log.py 文件.txt`，得到每个弯的出弯偏差、摆动次数、稳定时间和曲线图。

RUN 要求：`CHASSIS_HARDWARE_CONFIRMED=1`、没有锁存的故障、上电陀螺校准成功、IMU 与巡线数据新鲜、探头看到正常线（1～3 路相邻），以及到云台的串口近期发送完成。不满足时回复第一个不满足的条件。RUN 不等云台就绪（`$G` 只用于上电自启动），云台断电不妨碍底盘单独跑。FAULT 状态要先 STOP 清除再 RUN；跑完不会自动重启。

每秒输出两行：

- `AUTO armed=… state=… fault=… laps=… corners=… mask=… fresh=… scan=… tx_fail=… gimbal_rx=… gimbal_flags=… gimbal_ready=… autostart=… circle=… key=…`。mask 为十六进制：全白通常 00、中央两路为 18、全黑 FF。fresh=1 只说明采样新鲜，不代表 mask 是正常线。`gimbal_ready` 是云台 `$G` 的就绪位，`circle` 是当前任务，`key=1` 表示 PB5 正按着。
- `HW imu_found=… who_am_i=… calib=… imu_read=… link_ok=… enc_l=… enc_r=… rx_bytes=… rx_err=… cruise_mm_s=… turn_mm_s=… fw=<编译时间>`：硬件自检和当前烧录的版本。

## 控制与状态

10 ms 调度：编码器/IMU → 完整巡线快照 → 任务状态机 → 唯一 v/w → 原轮速环。主循环处理扫描、串口命令和云台心跳；每次最多解析 32 个调试字节，避免持续输入饿死控制。

| state | 名称与动作 |
|---|---|
| 0 | IDLE，停止 |
| 1 | LINE，负号 PD 巡线，左转角速度为正；左侧拐角图样连续 3 次确认；另加陀螺航向保持（每条边方向 = 已过弯数 × 90°，增益 CHASSIS_LINE_KH），抑制过弯后残留的车头偏角；从第 2 个弯起，按刹车距离在预计拐角前最晚减速到过弯速度（第 2 个弯用名义边长预测，之后用实测平均边长） |
| 2 | PRE_TURN，立即请求云台保持，减速前进；至少预告 60 ms，走完「前伸距离 − 半径 − 滞后提前量」后起转（提前量 = 实测速度 ×（0.20 s + 角速度爬升时间）），不再等速度降完 |
| 3 | TURN，积分本弯 gz，角速度按剩余角度与角加速度斜坡控制；剩余角度先减去「当前角速度 × 0.20 s」，补偿车体转动滞后（实车曾转到 105°～112°）；转弯角速度按编码器实测速度 ÷ 半径计算，速度没降下来时也保持实际半径 |
| 4 | REACQUIRE，转角至少 80°后找线：看到正常线就按「横向偏差 + 相对新边的车头偏角」引入（不要求落在中间两路），全丢时朝最后看到的一侧找，转过头时往回扳；线距中心 ≤1.5 个探头间距且车头偏角 ≤15°，连续 3 次后退出 |
| 5 | FINISH，达到 4N 弯后按统一里程基准返回起点并提前减速 |
| 6 | DONE，刹停，不自动重新启动 |
| 7 | FAULT，刹停，故障锁存 |

拐角按逆时针左转实现，适用于预期方形赛道。巡线正常图样为连续 1～3 路；模糊/无线路段最多低速保持 0.20 s，之后停车，不做盲目原地搜索。全部探头都看不到线时，按最后一次看到线的一侧以 1.0 rad/s 低速转向找线（线偏离中心不足半个探头间距时视为间断，保持直行）；有黑点但图样模糊时保持原角速度。计圈停车使用拐角触发到触发的平均里程；起点应在直线段，留足返回后的刹车距离。没有专用起点标记校正，打滑/不等长轨迹会影响停车位置。

| fault | 含义 |
|---|---|
| 0 | 无故障 |
| 1 | 参数不合法 |
| 2 | 传感器无效、快照过期或数据异常；IMU 连续无效超过 50 ms（期间用编码器差速估算角速度） |
| 3 | 实际控制周期不在 5～30 ms |
| 4 | 丢线/模糊线超过 0.20 s |
| 5 | 转弯超时（5 s）或角度异常（超过 130° 或反转超过 15°） |
| 6 | 预期运动但编码器持续无增量 |
| 7 | 路程/总时间越界（单边超过 1.5 m 或总时间超限）；起点已过或来不及刹车时不再报故障，改为立即刹停并进入 DONE |
| 8 | 到云台的串口长时间未完成发送 |
| 9 | 调试串口 RX 溢出；丢弃当前残帧，重新发换行、STOP 后才能 RUN |

IMU 失败不降级到编码器继续跑。运行期总超时为 `40*N+5` 秒，仅用于防止任务卡住，不是比赛限时判断。停车复用原 `app_motor_stop()` 的刹车与 PID 清零。

## 云台协议

每 20 ms 发一帧 `$C` 版本 3：序号、mode、`imu_valid`、底盘 gz（°/s，IMU 正常时 `imu_valid=1`；yaw/pitch/roll/gx/gy 不测，填 0）、编码器测得的前进速度、任务（0 打靶心 / 1 画圆）和跑圈进度（千分之一圈）。使用独立中断发送和持久缓冲区，发送完成前不覆盖；模式变化尽快发送，发送卡住超时后恢复。

`PRE_TURN/TURN/REACQUIRE/FAULT` 发 mode=1（请求云台保持、屏蔽视觉），其余状态发 0。FAULT 复用保持行为，云台不能仅凭 mode 判断故障类型。协议没有 ACK，60 ms 预告只是处理余量，不代表云台已经确认。

同一根线的反方向，云台以 10 Hz 发 `$G` 就绪状态；上电自启动要等它报告就绪（超过 500 ms 的就绪不算）。

## 新装车的检查顺序

1. 先设 `CHASSIS_HARDWARE_CONFIRMED=0`，串口看 STATUS；逐路遮挡确认 PC0/1/2 选通、PB0 输入、mask 方向及极性。
2. 实测并填入探头间距、探头到轮轴距离、轮距和左右轮周长，复核 IMU 安装方向；上电校准期间保持车静止。
3. 检查电机/编码器方向和 STOP 刹车，再改回 1；用 `DRIVE` 调直线，然后单弯，最后 1 圈及多圈，用 `LOG ON` 和 `plot_run_log.py` 看丢线、转角和停车误差。
4. 接上云台联调，见 [docs/GIMBAL_TUNING.md](../docs/GIMBAL_TUNING.md)。

## 验证与工程维护

主机测试：`Tests/run_autonomy_tests.ps1 -Compiler <gcc.exe路径>`，覆盖任务状态机、自启动、模式按键、命令行、DRIVE、轮速环、I/O 替身、里程计和调试串口；根目录 `tests/run_tests.ps1` 会一起调用。`Tests/*_sim.py` 是选参数时用的运动学仿真（过弯落点、出弯航向、巡线）。

`.ioc` 没有和源码同步：巡线 GPIO（PC0～PC2、PB0）、PB5 按键、UART4/UART5 中断、各串口波特率和 RX 上拉都只在 C 源码里。不要直接用 CubeMX 重新生成并覆盖；确实要重新生成时先备份，再把这些配置和 main/MSP/IRQ 里的集成代码补回去。正常编译使用仓库里的 C 源码。
