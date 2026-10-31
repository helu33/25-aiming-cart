# 跑圈分析脚本

实车跑圈后的离线分析。输入是 `gimbal/tools/telemetry_log.py run` 记录的一次跑圈：

- `RUN.csv`：云台蓝牙遥测（视觉误差、云台陀螺、底盘 gz 等，见 `telemetry_log.COLUMNS`）；
- `RUN_chassis.txt`：底盘 `LOG ON` 的 50 Hz 日志（`--chassis-log`）；
- `RUN_params.txt`：当时的云台参数，其中 `tsel` 决定遥测第 9、10 路记录什么（见 `docs/GIMBAL_TUNING.md` 第 4 节）。

命令行参数写 `RUN` 的路径前缀（不带扩展名），可以一次给多个，例如 `logs/2026-10/final`。脚本按自己的位置找 `gimbal/tools`，在哪个目录运行都可以。需要 Python 3（`py -3.13`）和 numpy。

| 脚本 | 需要的文件 | 输出 |
|---|---|---|
| `lap_time.py` | `_chassis.txt` | RUN 到 DONE 的用时、轮子里程、每个拐角的触发时刻 |
| `link_gaps.py` | `.csv`（tsel 0） | 云台收不到底盘 `$C` 的时段（≥ 0.3 s） |
| `worst_errors.py` | `.csv` + `_chassis.txt` | 误差最大的 5 个时刻、所在路段、之前 0.5 s 的视觉无效比例 |
| `corner_exit.py` | `.csv` + `_chassis.txt` | 每个弯出弯后 1 s 的 yaw / pitch 误差（rms / 最大） |
| `face_error_cm.py` | `.csv` + `_chassis.txt` | 换算到靶面的误差（cm）：p50 / p95 / p99 / 最大、超过 2 cm 的比例 |
| `circle_fit.py` | `.csv` + `_chassis.txt`（画圆的 1 圈） | 是否真的画了圆：拟合出的圆幅度（应约 60 mm）和相位，靶面误差 |
| `circle_static.py` | `.csv`（tsel 1） | 静止画圆（`CIRCLE <s>`）时云台 yaw / pitch 随时间的轨迹 |
| `track_geometry.py` | `.csv` + `_chassis.txt` | 由底盘里程计和云台方位反推靶的位置；`face_error_cm.py`、`circle_fit.py` 用它 |

`face_error_cm.py` 和 `circle_fit.py` 里的靶心高度 `H = 0.182 m`（靶心比相机高多少）、`track_geometry.py` 里的视觉延迟 `L_VIS` 和链路延迟 `LINK` 是 2026-10 这台车的实测值，换了车要改。

底盘串口的小工具在 `chassis/host/`：`chassis_cmd.py`（发命令、可重发到确认）、`listen_chassis.py`（只听不发，记录上电自启动的那一圈）、`plot_run_log.py`（底盘日志画图）。
