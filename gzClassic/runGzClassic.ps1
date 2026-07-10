# runGzClassic.ps1

$ErrorActionPreference = 'Stop'

# 1. Ensure local directories exist on the host
if (!(Test-Path "../my_models")) { New-Item -ItemType Directory -Path "../my_models" | Out-Null }
if (!(Test-Path "../my_worlds")) { New-Item -ItemType Directory -Path "../my_worlds" | Out-Null }

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
# The backticks (`) prevent PowerShell from evaluating the Gazebo variables, 
# ensuring they are evaluated safely inside the Linux container.
$BashCommand = "source /usr/share/gazebo/setup.sh && export GAZEBO_MODEL_PATH=`$GAZEBO_MODEL_PATH:/workspace/my_models && export GAZEBO_RESOURCE_PATH=`$GAZEBO_RESOURCE_PATH:/workspace/my_worlds && exec gazebo --verbose $SET_COMMAND"

# 4. Execute the Docker command
docker run -it --rm `
    --gpus all `
    --env="DISPLAY=host.docker.internal:0" `
    --env="QT_X11_NO_MITSHM=1" `
    --env="NVIDIA_DRIVER_CAPABILITIES=all" `
    --volume="${PWD}/../my_models:/workspace/my_models:ro" `
    --volume="${PWD}/../my_worlds:/workspace/my_worlds:ro" `
    gzclassic `
    bash -c $BashCommand
