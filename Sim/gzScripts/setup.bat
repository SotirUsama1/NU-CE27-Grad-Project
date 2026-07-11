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

:: Use pure Batch to write the file to avoid PowerShell parsing errors
(
echo # runGzClassic.ps1
echo $ErrorActionPreference = 'Stop'
echo.
echo # Resolve the .. into clean, absolute Windows paths so Docker doesn't crash
echo $ModelsDir = [System.IO.Path]::GetFullPath^("$PSScriptRoot\..\models"^)
echo $WorldsDir = [System.IO.Path]::GetFullPath^("$PSScriptRoot\..\worlds"^)
echo.
echo # 1. Ensure local directories exist on the host
echo if ^(!^(Test-Path $ModelsDir^)^) { New-Item -ItemType Directory -Path $ModelsDir ^| Out-Null }
echo if ^(!^(Test-Path $WorldsDir^)^) { New-Item -ItemType Directory -Path $WorldsDir ^| Out-Null }
echo.
echo # 2. Handle default argument
echo if ^($args.Count -eq 0^) {
echo     Write-Host "[INFO] No simulation arguments provided. Defaulting to an empty world." -ForegroundColor Cyan
echo     $SET_COMMAND = "worlds/empty.world"
echo } else {
echo     $SET_COMMAND = $args -join " "
echo }
echo.
echo Write-Host "[INFO] Ensure VcXsrv is running with 'Disable access control' checked!" -ForegroundColor Yellow
echo Write-Host "[INFO] Launching Gazebo Classic Docker Container on Windows..." -ForegroundColor Cyan
echo Write-Host "[INFO] Passing arguments to gazebo: $SET_COMMAND" -ForegroundColor Cyan
echo.
echo # 3. Construct the bash execution string
echo $BashCommand = "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=`$GAZEBO_MODEL_PATH:/workspace/my_models && export GAZEBO_RESOURCE_PATH=`$GAZEBO_RESOURCE_PATH:/workspace/my_worlds && exec gazebo --verbose $SET_COMMAND"
echo.
echo # 4. Execute the Docker command
echo docker run -it --rm `
echo     --gpus all `
echo     --env="DISPLAY=host.docker.internal:0" `
echo     --env="QT_X11_NO_MITSHM=1" `
echo     --env="NVIDIA_DRIVER_CAPABILITIES=all" `
echo     --volume="${ModelsDir}:/workspace/my_models:ro" `
echo     --volume="${WorldsDir}:/workspace/my_worlds:ro" `
echo     sotirusama/gzclassic:latest `
echo     bash -c $BashCommand
) > runGzClassic.ps1

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