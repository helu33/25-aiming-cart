# Copy the board-side vision files to a K230 SD card (or any staging folder).
# Usage: powershell -ExecutionPolicy Bypass -File vision/deploy.ps1 -Target E:\
#        add -LaserCapture to build the laser-calibration capture variant,
#        -FrameLog to log every frame to captures/framelog_<ticks>.csv (vision/framelog.py).
# Files land in <Target>\k230_vision\, which the board sees as /sdcard/k230_vision/.
param(
    [Parameter(Mandatory = $true)][string]$Target,
    [switch]$LaserCapture,
    [switch]$FrameLog
)
$ErrorActionPreference = 'Stop'

$boardFiles = @('main.py', 'config.py', 'protocol.py', 'runtime.py', 'detector.py', 'geometry.py', 'processor.py', 'spot.py', 'framelog.py')
$destination = Join-Path $Target 'k230_vision'
New-Item -ItemType Directory -Force $destination | Out-Null

foreach ($name in $boardFiles) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $destination -Force
}

# Power-on start: CanMV runs /sdcard/main.py at boot. This 1-line launcher goes to the SD
# root as main.py and runs the real program, so it never goes stale after a redeploy.
# No BOM: MicroPython rejects a byte-order mark at the start of a script.
$launcher = "# Copy to the SD card ROOT and rename to main.py (/sdcard/main.py): runs at power-on.`n" +
            "exec(open(`"/sdcard/k230_vision/main.py`").read())`n"
[System.IO.File]::WriteAllText((Join-Path (Resolve-Path $destination) 'autostart_main.py'), $launcher,
                               (New-Object System.Text.UTF8Encoding($false)))

if ($LaserCapture) {
    # Later assignments win in Python, so the source config stays the production one.
    # Colour frames: the violet spot is hard to separate from white paper in grayscale.
    $overrides = @(
        '',
        '# --- Laser calibration capture, appended by deploy.ps1 -LaserCapture ---',
        '# Saves a colour BMP every 1.5 s to captures/laser and forces valid=0.',
        'CAPTURE_INTERVAL_MS = 1500',
        'CAPTURE_DIRECTORY = "captures/laser"',
        'SENSOR_GRAYSCALE = False'
    )
    Add-Content -LiteralPath (Join-Path $destination 'config.py') -Value $overrides -Encoding UTF8
}

# Only a real calibration enables valid frames; never substitute the example template.
$calibration = Join-Path $PSScriptRoot 'calibration.json'
$calibrated = Test-Path -LiteralPath $calibration
if ($calibrated) {
    Copy-Item -LiteralPath $calibration -Destination $destination -Force
    Write-Output "Copied calibration.json"
} elseif (-not $LaserCapture) {
    Write-Warning 'vision/calibration.json not found: the board will send valid=0 until laser calibration is done.'
}

$notes = @(
    'K230 视觉程序（由 vision/deploy.ps1 生成，请勿在此处修改，改源码后重新生成）',
    '',
    '1. 把整个 k230_vision 文件夹复制到 SD 卡根目录，板上路径为 /sdcard/k230_vision/。',
    '2. 在 CanMV IDE 中打开并运行本目录的 main.py；8 个 .py 文件必须全部在同一目录。',
    '3. 接线：K230 UART1 GPIO9(TX) -> 云台 USART10 PE2(RX)，共地，921600 8N1，3.3V TTL。',
    '4. 串口日志每秒一行：valid/reason 为本帧结果，det 为检测状态，thr 为当前二值化阈值。',
    '5. 上电自启动：把本目录的 autostart_main.py 复制到 SD 卡根目录并改名为 main.py（板上 /sdcard/main.py）。',
    ''
)
if ($LaserCapture) {
    $notes += @(
        '【当前为激光标定采图版】每 1.5 秒存一张彩色图到 /sdcard/k230_vision/captures/laser/，始终发送 valid=0。',
        '拍摄：激光常开、光斑落在靶纸上；靶框完整在画面内；0.5 / 1.0 / 1.5 m 三个距离各拍若干张，光斑位置尽量不同。',
        '拍完把 captures/laser 里的 BMP 拷到电脑 vision/captures/laser/，再运行 vision/tools/extract_laser_samples.py。',
        $(if ($calibrated) { '已包含 calibration.json：日志里 source=tracking 且 yaw/pitch 稳定，说明上板检测和标定计算正常（采图模式下发给云台的仍是 valid=0）。' }
          else { '未包含 calibration.json：日志 source 会显示 laser_not_calibrated，属正常。' }),
        '标定完成后必须重新生成正式版（不带 -LaserCapture）烧回板上。'
    )
} elseif ($calibrated) {
    $notes += '标定：已包含 calibration.json，满足条件时会发送有效误差。'
} else {
    $notes += '标定：未包含 calibration.json（激光未标定），程序只会发送 valid=0，云台不会跟踪。'
}
Set-Content -LiteralPath (Join-Path $destination '部署说明.txt') -Value $notes -Encoding UTF8

if ($FrameLog) {
    $overrides = @(
        '',
        '# --- Per-frame log, appended by deploy.ps1 -FrameLog ---',
        'FRAME_LOG = True'
    )
    Add-Content -LiteralPath (Join-Path $destination 'config.py') -Value $overrides -Encoding UTF8
}

Write-Output "Deployed $($boardFiles.Count) scripts to $destination$(if ($LaserCapture) { ' (laser capture mode)' })$(if ($FrameLog) { ' (frame log on)' })"

