# Simulation Run Scripts

These scripts start Gazebo Classic in the project Docker image. They mount the project's `Sim/models` and `Sim/worlds` directories into the container, then add those paths to Gazebo's model and resource search paths. Changes to those host directories are visible inside the container.

## Requirements

- Docker must be installed and running.
- Build or pull the image tag required by the script (see [Docker image instructions](../Docker/readme.md)).
- The host needs a working graphical display and compatible GPU support for the script's display and GPU options.
- On WSL, use WSL2 with WSLg and Docker Desktop WSL integration enabled. See [WinInstallation.md](WinInstallation.md) for Windows and WSL setup details.

## Ubuntu / Linux with X11

Run `runGzClassic-Ubuntu` on a Linux desktop with an X11 display:

```bash
cd /path/to/NU-CE27-Grad-Project
./Sim/runScripts/runGzClassic-Ubuntu
```

The script uses `sotirusama/gzclassic:test`. It passes the host display and X11 socket into the container, mounts the user's `.Xauthority`, grants local Docker clients access with `xhost` when available, and requests host networking, privileged access, and NVIDIA GPU support.

## Windows with WSL2 and WSLg

From the project directory in the Ubuntu WSL distribution, run:

```bash
./Sim/runScripts/runGzClassic-WSL.sh
```

This script uses `sotirusama/gzclassic:latest`. It passes WSLg's display/audio environment and mounts WSLg paths, `/usr/lib/wsl`, and `/dev/dxg` for GPU access. It does not run `xhost` or mount `.Xauthority`, because WSLg provides the display authorization for this setup.

## Choosing a world or opening a shell

With no arguments, both scripts ask Gazebo to open `worlds/empty.world`. To open a project world, pass its path relative to `Sim` (the container's `/main` working directory):

```bash
./Sim/runScripts/runGzClassic-Ubuntu worlds/Grad_warehouse/Grad_warehouse.world
./Sim/runScripts/runGzClassic-WSL.sh worlds/Structured_Warehouse/Structured_Warehouse.world
```

To start an interactive Bash shell in the configured container instead of launching Gazebo, pass `bash` as the first argument:

```bash
./Sim/runScripts/runGzClassic-Ubuntu bash
./Sim/runScripts/runGzClassic-WSL.sh bash
```

The containers are removed when they exit (`--rm`), so persistent project files should be kept in the mounted `Sim/models` and `Sim/worlds` directories.

## Script differences

| Script | Image tag | Display/GPU setup |
|---|---|---|
| `runGzClassic-Ubuntu` | `sotirusama/gzclassic:test` | X11 socket, `DISPLAY`, `.Xauthority`, NVIDIA container settings |
| `runGzClassic-WSL.sh` | `sotirusama/gzclassic:latest` | WSLg mounts and environment, `/dev/dxg`, NVIDIA adapter selection |

If you change an image tag in a launcher, build or pull that same tag before running it. The Ubuntu launcher requests `--gpus all`; the WSL launcher additionally requires `/dev/dxg` and the WSLg paths present on the host.
