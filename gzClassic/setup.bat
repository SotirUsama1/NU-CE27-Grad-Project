@echo off
setlocal
echo =================================================
echo   Gazebo Classic Windows Environment Auto-Setup
echo =================================================
echo.

:: 1. Check and Install Docker Desktop
echo [1/3] Checking Docker Desktop...
where docker >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo Docker not found. Installing via Winget...
    winget install --id Docker.DockerDesktop -e --accept-package-agreements --accept-source-agreements
    echo [WARNING] Docker requires a system restart after setup completes.
) else (
    echo [OK] Docker is already installed.
)
echo.

:: 2. Check and Install VcXsrv
echo [2/3] Checking VcXsrv (X Server)...
if not exist "%ProgramFiles%\VcXsrv\vcxsrv.exe" (
    echo VcXsrv not found. Installing via Winget...
    winget install --id marha.VcXsrv -e --accept-package-agreements --accept-source-agreements
) else (
    echo [OK] VcXsrv is already installed.
)
echo.

:: 3. Generate the PowerShell Run Script
echo [3/3] Generating runGzClassic.ps1...
powershell -NoProfile -Command "$text = @'^
$ErrorActionPreference = ''Stop''^

if (!(Test-Path ''../my_models'')) { New-Item -ItemType Directory -Path ''../my_models'' | Out-Null }^
if (!(Test-Path ''../my_worlds'')) { New-Item -ItemType Directory -Path ''../my_worlds'' | Out-Null }^

if ($args.Count -eq 0) {^
    Write-Host ''[INFO] Defaulting to empty world.'' -ForegroundColor Cyan^
    $SET_COMMAND = ''worlds/empty.world''^
} else {^
    $SET_COMMAND = $args -join '' ''^
}^

$BashCmd = ''source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=$$GAZEBO_MODEL_PATH:/workspace/my_models && export GAZEBO_RESOURCE_PATH=$$GAZEBO_RESOURCE_PATH:/workspace/my_worlds && exec gazebo --verbose '' + $SET_COMMAND^

docker run -it --rm `^
    --gpus all `^
    --env=\"DISPLAY=host.docker.internal:0\" `^
    --env=\"QT_X11_NO_MITSHM=1\" `^
    --env=\"NVIDIA_DRIVER_CAPABILITIES=all\" `^
    --volume=\"${PWD}/../my_models:/workspace/my_models:ro\" `^
    --volume=\"${PWD}/../my_worlds:/workspace/my_worlds:ro\" `^
    gzclassic `^
    bash -c $BashCmd^
'; $text = $text.Replace('$$', '`$'); Set-Content -Path 'runGzClassic.ps1' -Value $text"

echo [OK] Created runGzClassic.ps1 in the current directory.
echo.
echo =================================================
echo   Setup Complete! Next Steps:
echo =================================================
echo 1. If Docker was just installed, RESTART YOUR COMPUTER.
echo 2. Open your Start Menu, search for "XLaunch" and run it.
echo 3. CRITICAL: On the last XLaunch screen, check "Disable access control".
echo 4. Open PowerShell in this folder and run: .\runGzClassic.ps1 worlds/cafe.world
echo.
pause
