bash
export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export LIBGL_DRIVERS_PATH=/usr/lib/wsl/lib
export LD_LIBRARY_PATH=/usr/lib/wsl/lib

glxinfo | grep -i "OpenGL renderer"