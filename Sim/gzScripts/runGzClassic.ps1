# runGzClassic.ps1
$ErrorActionPreference = 'Stop'

# Resolve the .. into clean, absolute Windows paths so Docker doesn't crash
$ModelsDir = [System.IO.Path]::GetFullPath("$PSScriptRoot\..\models")
$WorldsDir = [System.IO.Path]::GetFullPath("$PSScriptRoot\..\worlds")

# 1. Ensure local directories exist on the host
if (!(Test-Path $ModelsDir)) { New-Item -ItemType Directory -Path $ModelsDir | Out-Null }
if (!(Test-Path $WorldsDir)) { New-Item -ItemType Directory -Path $WorldsDir | Out-Null }

# 2. Handle default argument
if ($args.Count -eq 0) {
    Write-Host "[INFO] No simulation arguments provided. Defaulting to an empty world." -ForegroundColor Cyan
    $SET_COMMAND = "worlds/empty.world"
} else {
    $SET_COMMAND = $args -join " "
}

Write-Host "[INFO] Ensure VcXsrv is running with 'Disable access control' checked!" -ForegroundColor Yellow
Write-Host "[INFO] Launching Gazebo Classic Docker Container on Windows..." -ForegroundColor Cyan
Write-Host "[INFO] Passing arguments to gazebo: $SET_COMMAND" -ForegroundColor Cyan

# 3. Construct the bash execution string
$BashCommand = "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=`$GAZEBO_MODEL_PATH:/workspace/my_models && export GAZEBO_RESOURCE_PATH=`$GAZEBO_RESOURCE_PATH:/workspace/my_worlds && exec gazebo --verbose $SET_COMMAND"

# 4. Execute the Docker command
# Strip interactive TTY flags if running in headless CI (GitHub Actions)
if ($env:GITHUB_ACTIONS) {
    docker run --rm `
        --gpus all `
        --env="DISPLAY=host.docker.internal:0" `
        --env="QT_X11_NO_MITSHM=1" `
        --env="NVIDIA_DRIVER_CAPABILITIES=all" `
        --volume="${ModelsDir}:/workspace/my_models:ro" `
        --volume="${WorldsDir}:/workspace/my_worlds:ro" `
        sotirusama/gzclassic:latest `
        bash -c $BashCommand
} else {
    docker run -it --rm `
        --gpus all `
        --env="DISPLAY=host.docker.internal:0" `
        --env="QT_X11_NO_MITSHM=1" `
        --env="NVIDIA_DRIVER_CAPABILITIES=all" `
        --volume="${ModelsDir}:/workspace/my_models:ro" `
        --volume="${WorldsDir}:/workspace/my_worlds:ro" `
        sotirusama/gzclassic:latest `
        bash -c $BashCommand
}