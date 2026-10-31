$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path $PSScriptRoot -Parent
$previousPath = $env:PATH
$env:PATH = (Split-Path (Get-Command gcc).Source) + ';' + $env:PATH
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force 'tests/.build' | Out-Null
    & gcc -std=c99 -Wall -Wextra -Werror -Igimbal/App/inc -Ichassis/User/Algorithm tests/test_protocol.c gimbal/App/gimbal_protocol.c chassis/User/Algorithm/chassis_protocol.c -lm -o tests/.build/test_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Protocol test compile failed' }
    & ./tests/.build/test_protocol.exe
    if ($LASTEXITCODE -ne 0) { throw 'Protocol test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -Igimbal/App/inc tests/test_gyro_bias.c -lm -o tests/.build/test_gyro_bias.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gyro bias test compile failed' }
    & ./tests/.build/test_gyro_bias.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gyro bias test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -Igimbal/App/inc tests/test_track_filter.c -lm -o tests/.build/test_track_filter.exe
    if ($LASTEXITCODE -ne 0) { throw 'Track filter test compile failed' }
    & ./tests/.build/test_track_filter.exe
    if ($LASTEXITCODE -ne 0) { throw 'Track filter test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -Igimbal/App/inc tests/test_rate_integral.c -lm -o tests/.build/test_rate_integral.exe
    if ($LASTEXITCODE -ne 0) { throw 'Rate integral test compile failed' }
    & ./tests/.build/test_rate_integral.exe
    if ($LASTEXITCODE -ne 0) { throw 'Rate integral test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -Igimbal/App/inc tests/test_telemetry.c -lm -o tests/.build/test_telemetry.exe
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry test compile failed' }
    & ./tests/.build/test_telemetry.exe
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -Itests/stubs -Igimbal/App/inc tests/test_gimbal.c gimbal/App/gimbal_protocol.c gimbal/App/gimbal_tune.c -lm -o tests/.build/test_gimbal.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal test compile failed' }
    & ./tests/.build/test_gimbal.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gimbal test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -DGIMBAL_YAW_LIMIT_ENABLE=1 -Itests/stubs -Igimbal/App/inc tests/test_gimbal.c gimbal/App/gimbal_protocol.c gimbal/App/gimbal_tune.c -lm -o tests/.build/test_yaw_limited.exe
    if ($LASTEXITCODE -ne 0) { throw 'Limited-yaw test compile failed' }
    & ./tests/.build/test_yaw_limited.exe
    if ($LASTEXITCODE -ne 0) { throw 'Limited-yaw test failed' }
    & gcc -std=c99 -Wall -Wextra -Werror -DGIMBAL_CAN_TX_ENABLE=0 -Itests/stubs -Igimbal/App/inc tests/test_gimbal.c gimbal/App/gimbal_protocol.c gimbal/App/gimbal_tune.c -lm -o tests/.build/test_disabled.exe
    if ($LASTEXITCODE -ne 0) { throw 'Disabled-CAN test compile failed' }
    & ./tests/.build/test_disabled.exe
    if ($LASTEXITCODE -ne 0) { throw 'Disabled-CAN test failed' }
    # Cross-check the encoder the chassis firmware actually ships against the gimbal decoder.
    & gcc -std=c99 -Wall -Wextra -Werror -shared -Igimbal/App/inc -Ichassis/User/Algorithm tests/encoder_bridge.c gimbal/App/gimbal_protocol.c chassis/User/Algorithm/chassis_protocol.c -lm -o tests/.build/encoders.dll
    if ($LASTEXITCODE -ne 0) { throw 'Sender example compile failed' }
    & python -B tests/test_telemetry_log.py
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry log tool test failed' }
    & python -B tests/test_run_report.py
    if ($LASTEXITCODE -ne 0) { throw 'Run report test failed' }
    & python -B tests/test_framelog.py
    if ($LASTEXITCODE -ne 0) { throw 'Frame log test failed' }
    & python -B tests/test_encoders.py
    if ($LASTEXITCODE -ne 0) { throw 'Sender example round-trip failed' }
} finally { $env:PATH = $previousPath; Pop-Location }

& (Join-Path $repoRoot 'chassis/Tests/run_autonomy_tests.ps1')
