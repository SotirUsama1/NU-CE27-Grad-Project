# Gazebo Docker Images

This directory contains two Dockerfiles used by the Gazebo Classic simulation setup. `BaseImage/Dockerfile` builds the shared ROS 2 and Gazebo environment. The top-level `Dockerfile` builds the project image on top of that base and adds navigation and SLAM packages.

## Image layers

### Base image: `sotirusama/gzbase:latest`

Build this image first. It is based on Ubuntu 22.04 (Jammy) and installs:

- ROS 2 Humble `ros-base`
- Gazebo ROS packages
- `rqt` and `turtlesim`
- English UTF-8 locale support

The ROS apt repository defaults to the Tsinghua ROS mirror. Override it at build time with `ROS_APT_REPOSITORY` if needed.

From the repository root:

```bash
docker build -f Sim/Docker/BaseImage/Dockerfile \
  --build-arg ROS_APT_REPOSITORY=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu \
  -t sotirusama/gzbase:latest .
```

The ROS and Gazebo setup scripts are sourced in root's `.bashrc` for interactive shells.

### Simulation image: `sotirusama/gzclassic`

The top-level `Dockerfile` uses `sotirusama/gzbase:latest` and adds:

- ROS 2 Navigation (`navigation2` and `nav2-bringup`)
- `slam-toolbox`
- `cartographer-ros`

After building the base, build the simulation image from the repository root:

```bash
docker build -f Sim/Docker/Dockerfile -t sotirusama/gzclassic:latest .
```

The Ubuntu launcher currently uses the `latest` tag, so build that tag if you plan to use `Sim/runScripts/runGzClassic-Ubuntu`:

```bash
docker build -f Sim/Docker/Dockerfile -t sotirusama/gzclassic:latest .
```


## Running the simulation

The launch scripts in `Sim/runScripts` run the image with a graphical display and GPU options, and mount the project `Sim/models` and `Sim/worlds` directories into `/main/models` and `/main/worlds`. They set Gazebo's model and resource paths to include those mounts.

On Ubuntu, for example:

```bash
./Sim/runScripts/runGzClassic-Ubuntu
```

Pass a world path to launch a specific world, or pass `bash` as the first argument to open a shell in the container:

```bash
./Sim/runScripts/runGzClassic-Ubuntu worlds/Grad_warehouse/Grad_warehouse.world
./Sim/runScripts/runGzClassic-Ubuntu bash
```

Use `runGzClassic-WSL.sh` under WSL2; it configures WSLg display and GPU access. Both scripts assume Docker and compatible host GPU/display support are already configured.

## Rebuilding

When changing the base image, rebuild `sotirusama/gzbase:latest` first, then rebuild `sotirusama/gzclassic` with the tag used by the launcher on your platform.
