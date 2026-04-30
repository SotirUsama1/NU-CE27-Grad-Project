@echo off
docker run -it ^
  --gpus all ^
  --env="NVIDIA_DRIVER_CAPABILITIES=all" ^
  --env="DISPLAY=host.docker.internal:0.0" ^
  --env="QT_X11_NO_MITSHM=1" ^
  --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" ^
  --volume="%cd%\worlds:/root/worlds:rw" ^
  gazebo:latest ^
  gazebo /root/worlds/%1