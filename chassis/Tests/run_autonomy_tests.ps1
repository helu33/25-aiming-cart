param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$previousPath = $env:PATH
# MSYS2 gcc needs its own bin directory on PATH to find cc1/DLLs.
$env:PATH = (Split-Path (Get-Command $Compiler).Source) + ';' + $env:PATH
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force Tests/.build | Out-Null
    $common = @('-std=c99','-Wall','-Wextra','-Werror','-ITests/stubs','-IUser','-IUser/Algorithm','-IUser/App','-IUser/Bsp')
    $cases = @(
        @{Name='mission'; Sources=@('Tests/test_chassis_mission.c','User/Algorithm/chassis_mission.c')},
        @{Name='autostart'; Sources=@('Tests/test_autostart.c','User/Algorithm/autostart.c')},
        @{Name='mode_key'; Sources=@('Tests/test_mode_key.c','User/Algorithm/mode_key.c')},
        @{Name='command_line'; Sources=@('Tests/test_command_line.c','User/Algorithm/command_line.c')},
        @{Name='drive_test'; Sources=@('Tests/test_drive_test.c','User/Algorithm/drive_test.c')},
        @{Name='motor'; Sources=@('Tests/test_app_motor.c','User/App/app_motor.c','User/Algorithm/filter.c','User/Algorithm/pid.c')},
        @{Name='io'; Sources=@('Tests/test_chassis_io.c','User/Bsp/bsp_line.c','User/Bsp/bsp_gimbal_link.c','User/Algorithm/chassis_protocol.c')},
        @{Name='odometry'; Sources=@('Tests/test_odometry.c','User/Algorithm/odometry.c')},
        @{Name='stationary'; Sources=@('Tests/test_app_odometry_stationary.c','User/App/app_odometry.c','User/Algorithm/odometry.c')},
        @{Name='debug_uart5'; Sources=@('Tests/test_debug_uart.c','User/Bsp/bsp_uart.c')},
        @{Name='debug_usart2'; Options=@('-DCHASSIS_DEBUG_UART=2'); Sources=@('Tests/test_debug_uart.c','User/Bsp/bsp_uart.c')},
        @{Name='debug_legacy'; Options=@('-DCHASSIS_AUTONOMOUS_ENABLE=0'); Sources=@('Tests/test_debug_uart.c','User/Bsp/bsp_uart.c')}
    )
    foreach ($case in $cases) {
        $output = "Tests/.build/test_$($case.Name).exe"
        [string[]]$options = @()
        if ($case.Options) { $options = $case.Options }
        & $Compiler @common @options @($case.Sources) -lm -o $output
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case.Name)" }
        & "./$output"
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case.Name)" }
        Write-Output "PASS $($case.Name)"
    }
} finally { $env:PATH = $previousPath; Pop-Location }
