# K230 视觉程序

适用：亚博 K230 Vision Module、GC2093、未接显示屏。运行流程为灰度采图 → 黑框检测与外框角点直线拟合（亚像素） → 本帧靶面姿态 → 预测激光与靶面交点 → 角度误差 → `$V` 串口帧。默认瞄准靶心；云台转发底盘的画圆任务（`$K`）后，瞄准点沿靶心周围半径 60 mm 的圆移动，车跑 1 圈画 1 圈（`config.py` 的 `CIRCLE_RADIUS_MM` / `CIRCLE_DIRECTION`，协议见 `docs/GIMBAL_COMM_PROTOCOL.md` 第 4.2 节）。

板上固件以 CanMV IDE 里显示的版本为准；亚博资料中的 `CanMV_K230_YAHBOOM_micropython_v1.8-13` 是 OpenCV 教程配套版本，不能据此认定板上版本。

## 1. 文件和部署

把本目录以下文件复制到板上 `/sdcard/k230_vision/`。推荐用脚本一次复制（`E:\` 换成 SD 卡盘符）；本目录是唯一源码，不再维护单独的部署副本：

```bash
powershell -ExecutionPolicy Bypass -File vision/deploy.ps1 -Target E:\
```

也可以先生成到本地再手动拷贝：`-Target vision` 会生成 `vision/k230_vision/`（含 `部署说明.txt`），把这个文件夹整个复制到 SD 卡根目录即可。该目录是生成物，不要在里面改代码；改 `vision/` 下的源码后重新生成。

- `main.py`：采图、资源管理、日志和发送。
- `config.py`：端口、分辨率、检测阈值和时间限制。
- `detector.py`、`geometry.py`、`processor.py`：检测、几何、标定检查。
- `protocol.py`、`runtime.py`：协议和过期结果处理。
- 完成实测后再复制 `calibration.json`。`calibration.example.json` 是空模板，不能直接作为有效标定。

用 CanMV IDE 运行 `main.py` 之前，先把全部配套脚本复制到 **K230 的 SD 卡**；仅在电脑 IDE 中打开入口脚本不足以提供板端导入模块。入口依次检查脚本所在目录、`/sdcard/k230_vision`、板端当前工作目录及 `/sdcard`。建议按上述固定目录部署。标定与采图默认使用相对路径，跟随选中的脚本目录；也可在 `config.py` 中指定绝对路径。板上不用安装 pip 包，也不用复制 `tools/` 或电脑的 OpenCV。

若出现 `ImportError: no module named 'config'`，请确认 SD 卡上的 `k230_vision/` 中已经有 `config.py` 及其余全部脚本，不能只复制 `main.py`，也不能只保存在电脑的工程目录。SD 卡根目录在电脑上看到的 `k230_vision/`，挂载到板上就是 `/sdcard/k230_vision/`。复制更新后重启板端解释器再运行，以免沿用缓存的旧模块。

默认 `Sensor()`、640×480、`Sensor.GRAYSCALE` 直接出灰度；若实机灰度格式报错，把 `config.py` 的 `SENSOR_GRAYSCALE` 改为 `False`，回到 RGB565 加逐帧转灰度。`gc.collect()` 每 `GC_INTERVAL_FRAMES` 帧执行一次，若出现内存或缓冲区错误改为 1。不初始化 ST7701，不调用屏幕显示函数。只有实际固件无屏媒体初始化需要时才尝试 `USE_VIRTUAL_DISPLAY=True`，它使用 VIRT，不要求物理 LCD。缺少 `cv2` 或接口不兼容时打印异常并退出，云台按无效帧/超时处理。

### 电脑实时预览（USB，不接实体屏）

更新整套板端脚本，用 USB 数据线连接 K230 和电脑，在 CanMV IDE 连接板子。在**板上的** `config.py` 中设置：

```python
IDE_PREVIEW = True
PREVIEW_INTERVAL_MS = 100
```

重启板端解释器后运行 `main.py`，查看 IDE 的图像/帧缓冲预览区域，如该区域被关闭则启用图像预览。程序自动初始化 `Display.VIRT` 并通过 `to_ide=True` 和 `Display.show_image()` 输出；不用单独打开 `USE_VIRTUAL_DISPLAY`。

画面为摄像头原图：绿色四边形表示当前有效控制结果，黄色表示检测到候选但结果尚未有效（例如尚未标定或等待连续确认），青色十字为根据四角透视映射得到的几何靶心。没有候选时只显示原图。靶心十字是检测几何位置，不是实际激光光斑。具体无效原因看串口控制台的 `reason` 日志。

未标定也能看检测框。预览绘制在单独的 RGB888 图像上，不污染检测图及保存的原图。预览不会改变原有串口控制逻辑；若已有有效标定，仍按正常条件发送控制误差。USB 传图会降低循环速度，默认最多 10 帧/秒，可能达不到该上限。调试完把 `IDE_PREVIEW` 改回 `False` 再测试运行帧率。此路径已用接口替身验证，板上仍需核对实际固件的 VIRT/IDE 支持。接口依据：[CanMV Display 官方文档](https://github.com/kendryte/k230_canmv_docs/blob/main/en/api/mpp/K230_CanMV_Display_Module_API_Manual.md)。

调试日志中的 `fps` 是包含预览、存图等开销的循环帧率，预览自身受 `PREVIEW_INTERVAL_MS` 限制。`ms` 是本帧采集和处理耗时；`max_ms` 是本次日志窗口内的最大值；`preview_ms`、`save_ms` 分别是该窗口内预览、存图累计耗时，不是单帧平均值。`seen=18/20` 表示这个窗口中 20 帧有 18 帧返回了检测框；`det` 为本帧检测状态：`no_candidates` 没有合格候选，`association_rejected` 候选与上一帧变化过大，`confirming` 等待连续确认，`confirmed` 已确认。`source` 保留采图模式覆盖前的处理原因，因此即使 `reason=calibration_capture` 也能判断检测是否正常。

2026-10-17 实拍排查：`captures/targetcheck` 的 53 张原图中，阈值 110 找到候选框 7 张，改为 70 后找到 50 张。已将 70 设为本轮上板复测值；这是同一批静态照片上的调参结果，不是独立验证或实时跟踪成功率，也不保证其他光照适用。尚未检出的三张为 `frame_50058_0.bmp`、`frame_68970_0.bmp`、`frame_75358_0.bmp`。仍保留内外框校验和连续确认，未通过时不输出控制误差。

2026-10-18 光斑闭环（`spot.py`，`config.SPOT`）：认到靶后每 5 帧在灰度图的靶纸区域内找最亮的光斑核心，用本帧位姿反算激光的实际方向，低通学习一个对标定方向的修正（与预测偏差超过 3° 视为反光不采纳，总修正 ≤4°，找不到光斑时保持）。实拍验证：灰度光斑与彩色工具位置中位差 0.4 px，53/53 检出；52 张无激光照片无误报；在激光装偏的旧照片上，预测落点误差由平均 5.3 mm 降到 2.6 mm。日志行尾 `spot_ok corr=yaw/pitch` 为状态与当前修正（度）。板上若精简 cv2/ulab 缺少所需函数，状态显示 `spot_disabled:...`，瞄准照常进行。

2026-10-18 抗光照：70 改为起始值。某帧找不到靶时，下一帧额外试一个阈值（70，或画面平均亮度的 0.4/0.55/0.7/0.85 倍，轮流），找到就沿用，每帧最多多做一次处理。把这 53 张图亮度乘 0.4～1.6 模拟光照变化，连续 6 帧内检出数由 0～50 张提高到 35～52 张。日志行尾的 `thr` 是当前实际阈值。偶尔漏 1～2 帧（`miss_tolerance`）不再重新做 3 帧确认，超过才重新确认。

## 2. 接线和控制职责

| K230 | 云台控制板 | 用途 |
|---|---|---|
| GPIO9 / UART1 TX | PE2 / USART10 RX | 视觉误差 |
| GND | GND | 共地 |
| GPIO10 / UART1 RX | PE3 / USART10 TX | 画圆任务 `$K`（只打靶心时可不接） |

3.3 V TTL，921600、8N1。GPIO 编号按亚博引脚图识别，不按排针序号猜测。

发送 `$V,1,seq,valid,yaw_deg,pitch_deg*CRC\n`，CRC 和现有云台解析器一致。每次新图处理完成发送一帧，序号 16 位回绕。角度以度为单位，向右/向下需要修正为正；不在视觉端叠加电机方向、PID、死区或增益。协议详见 `../docs/GIMBAL_COMM_PROTOCOL.md`。

底盘接云台 USART1，转弯屏蔽视觉、巡线恢复视觉由云台判断；K230 不直接收底盘转弯信号，转弯期间仍可发送新结果。首次视觉接收有云台上电 5 秒屏蔽期。

无靶、确认未完成、标定无效、几何不可信时发 `valid=0`，角度归零。采集调用至运算结束超过 200 ms 也发无效帧。这个时间不含传感器内部曝光/缓存延迟，实际延迟仍需实测。进程停住由云台 300 ms 超时兜底；当前无目标时 Pitch 遵循云台已有的启动基准补偿逻辑，不能解释成锁住上一帧角度。

## 3. 检测和几何约定

练习靶外框 315×233 mm，胶带宽 20 mm，内框 275×193 mm，靶心 (157.5,116.5) mm（`config.py` 的 `TARGET_*`、`TAPE_WIDTH_MM`）。比赛用的 A4 靶是外框 297×210 mm、胶带 18 mm、靶心 (148.5,105.0) mm，换靶时改这几个值；尺寸不对时位姿残差会明显变大，大部分帧被拒绝。检测检查父子轮廓面积关系及内框四角在外框透视映射中的位置，连续 3 帧才确认。跟踪期间按四角位置及面积变化匹配上一帧，避免被附近更大的矩形抢走；连续 10 帧无法匹配才重新搜索；漏帧不超过 2 帧时保持已确认状态，超过后重新找到仍须 3 帧确认。没有合格内外框就报告无靶，不将普通外接矩形直接用于控制。面积、灰度阈值等在 `DETECTION` 中，需用实拍照片调整；过快的真实目标移动也可能触发重新搜索。

`geometry.py` 对角点去畸变，求平面单应性并分解姿态，再将标定激光射线与靶面求交。去畸变后重新投影核对原像素，残差超过 0.25 px 则拒绝本次计算，避免未收敛的迭代结果进入控制或激光标定。命中点与期望点均在靶平面毫米坐标内表达，角度在相机轴中计算。没有在板上生成整张拉正图，坐标变换已完成所需的透视几何计算；电脑预览工具可生成拉正图。

相机坐标 x 向右、y 向下、z 向前；图像不镜像/翻转，目标横放、上边朝上，相机横滚应较小。±55° 斜视的合成样本已检查，但这不等于实物在 60° 斜视下达到精度。黑框本身没有方向标记，不支持任意横滚后仍可靠识别纸张方向。

角度计算是小步视觉修正，默认相机轴和云台轴安装对应，未建模云台转轴到相机的机械偏移；大角度、近距离及运动中需验证收敛。默认的 3 px 重投影门限、65° 倾斜门限、250～4000 mm 深度范围都只是初始筛选参数。

## 4. 标定：先相机，再激光

本机的标定过程和结果见 [CAMERA_CALIBRATION_REPORT.md](CAMERA_CALIBRATION_REPORT.md)。换了相机、镜头或激光安装就要重新标定；`LASER_APPROX_ORIGIN_MM` 的 18 mm 安装间距只能作为初始参考，程序不会将空模板当成标定成功。

电脑端准备 Python 3 和依赖，在仓库根目录执行：

```powershell
python -m pip install -r vision/requirements-pc.txt
```

采图时把 `CAPTURE_INTERVAL_MS` 设为 `2000`，程序每 2 秒左右保存一张原图 BMP 到 `captures/`，整个采图模式均发送无效控制帧；同名文件会追加编号，结束后改回 `0`。无屏操作时通过 USB 控制台看保存路径和状态。

相机标定使用尺寸已知且平整的棋盘格，至少 12 张有效图片，建议 20～30 张，覆盖画面中心/边缘、不同距离及倾斜；不要全部正对拍摄。必须使用与运行时一致的分辨率、裁剪、镜像和镜头设置。例如棋盘内角点 9×6、每格 20 mm：

```powershell
python vision/tools/calibrate.py camera captures/chessboard --cols 9 --rows 6 --square-mm 20 --output camera.json
```

工具检查重投影误差并输出 `camera.json`，此时 `laser.calibrated` 仍为 false。数值门限通过不代表覆盖充分或实际测距正确，需要留出未参与标定的照片验证。

**推荐流程（自动提取）**：

1. 生成采图版并拷到 SD 卡：`powershell -ExecutionPolicy Bypass -File vision/deploy.ps1 -Target vision -LaserCapture`。它每 1.5 s 存一张**彩色** BMP 到板上 `captures/laser/`，并始终发送 valid=0。
2. 激光常开、光斑落在靶纸白色区域（不要落在黑胶带上），在约 0.5 / 1.0 / 1.5 m 各拍若干张，光斑位置尽量分散；相机与激光相对安装全程不动。
3. 把 BMP 拷到电脑 `vision/captures/laser/`，运行 `py -3.13 vision/tools/extract_laser_samples.py`：自动找外框四角和光斑，打印每张的距离与状态，并在 `captures/laser_check/` 输出标注图（绿框为外框，红十字为光斑）。认错的图删掉或用 `--exclude` 排除。
4. 确认无误后加 `--calibrate` 生成 `vision/calibration.json`，再不带 `-LaserCapture` 重新生成正式版烧回板上。

**手工流程**（自动提取失败时）：激光标定采集至少 6 组，覆盖约 500、1000、1500 mm 三档，每档至少两组，保持相机和激光相对安装不动，记录同一原图的外框四角和实际光斑中心。运行预览工具取得四角模板：

```powershell
python vision/tools/inspect_frame.py captures/spot01.bmp --output-prefix captures/check01
```

输出带 TL/TR/BR/BL 标记的原图、2 px/mm 的拉正图以及一个样本 JSON。检查绿色轮廓确实是胶带**外沿**，必要时人工修正四角。用图像编辑器读取**原图**光斑中心像素，填入 `spot_px`，不能填写拉正图坐标。多个样本合并为 `{"samples": [...]}`，结构见 `tools/laser_samples.example.json`。

```powershell
python vision/tools/calibrate.py laser camera.json laser_samples.json --output calibration.json
```

工具利用每帧平面姿态将实测光斑还原到相机三维坐标，再拟合激光直线，要求样本深度跨度≥200 mm、最大拟合残差≤5 mm。输出 `origin_mm` 是激光直线与相机 z=0 平面的交点，不是激光器壳体的安装位置。实际采样仍应覆盖全工作距离，不能只满足最低跨度。

标定工具拒绝覆盖已有输出文件。把结果复制到板上，重启程序；只有两项 `calibrated=true`、数据有效、分辨率一致时才允许有效控制帧。改变镜头、裁剪、分辨率或相机/激光安装后需要重新检查/标定。

## 5. 联调与验证

1. 先运行无标定版本，核对启动打印的固件、无屏采图和 UART；应保持 `valid=0`。每秒日志报告 FPS、处理耗时、状态和发送失败次数。
2. 用保存的实拍图调整检测参数，确认各种距离/斜视下选中胶带外框，遮挡、纯黑矩形等不应成为有效靶。
3. 补齐标定后检查靶框在画面中左右、上下移动时误差方向正确，再验证云台低速收敛。方向系数只在云台端改。
4. 遮靶、断视觉串口、底盘转弯及恢复巡线分别检查行为。K230 不伪造底盘心跳。
5. 实测 D1≤20 mm、光斑直径≤5 mm。行驶中的跟踪调参见 `docs/GIMBAL_TUNING.md`。

电脑验证（先构建 C 协议测试桥，再运行视觉测试）：

```powershell
powershell -ExecutionPolicy Bypass -File tests/run_tests.ps1
python -B tests/test_vision.py
```

测试用 CPython 3.13 和 `vision/requirements-pc.txt` 里的 OpenCV（相机标定一项需要 OpenCV 5）。覆盖合成黑框、几何姿态/视差、无标定和过期门控、激光拟合、模拟棋盘内参标定、画圆瞄准点、`$K` 接收、调试图片导出、CRC 与实际 C 解析器，以及邻近大矩形干扰、面积突变、去畸变未收敛和非有限内参等回归用例。采图资源生命周期和无屏路径使用硬件接口替身验证。电脑测试不能验证固件 cv2 子集、曝光、板上耗时及闭环精度，这些以实车测试为准。

接口依据：[亚博 OpenCV 说明](https://www.yahboom.net/study/K230/19717/)、[亚博轮廓例程](https://www.yahboom.net/study/K230/19725/)、[CanMV UART](https://www.kendryte.com/k230_canmv/en/main/api/machine/K230_CanMV_UART_API_Manual.html)、[图像接口](https://www.kendryte.com/k230_canmv/en/main/api/openmv/image.html)、[OpenCV 相机标定](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)。板上接口最终以实际亚博固件为准。
