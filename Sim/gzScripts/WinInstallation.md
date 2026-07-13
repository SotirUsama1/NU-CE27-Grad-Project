# GPU-Accelerated Gazebo Classic 11 in Docker (Windows 11 + WSL2 + WSLg)

This sets up `sotirusama/gzclassic:latest` to run with **real NVIDIA GPU acceleration**
on a Windows 11 laptop with hybrid graphics (Intel integrated + NVIDIA discrete),
using WSL2's built-in WSLg display server. No VcXsrv, no manual X11 forwarding,
no `xhost` permissions to manage.

## How this works (short version)

Docker Desktop on Windows runs Linux containers inside a WSL2 VM. WSL2 ships with
**WSLg**, which gives that VM a working Wayland/X11 display server and a GPU device
(`/dev/dxg`) that's paravirtualized straight to your Windows GPU driver — no
translation layer, no software fallback required. We mount that device and WSLg's
sockets into the container and tell Mesa explicitly which physical GPU to use,
since hybrid laptops default to the integrated GPU otherwise.

---

## Prerequisites

- Windows 11, fully updated (Settings → Windows Update)
- A laptop/PC with an NVIDIA GPU and up-to-date **Windows-side** NVIDIA driver
  (installed the normal way, from nvidia.com or GeForce Experience — you do
  **not** install a separate Linux NVIDIA driver anywhere in this setup)
- Administrator access

---

## Step 1 — Enable the required Windows features

Open **PowerShell as Administrator**:

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

Reboot after this completes.

---

## Step 2 — Install WSL2 and Ubuntu

```powershell
wsl --update
wsl --set-default-version 2
wsl --install -d Ubuntu
```

Launch Ubuntu from the Start menu and complete the first-run username/password setup.

### If `wsl --install` fails with `Wsl/Service/E_UNEXPECTED`

This is a known, fixable service-registration fault — it does **not** mean your
hardware or BIOS virtualization is broken.

```powershell
wsl --shutdown
wsl --update
```
Then check the WSL service (name varies by install method — don't worry if
`LxssManager` specifically doesn't exist, that's normal on Store-based WSL):
```powershell
Get-Service *lxss*,*wsl* -ErrorAction SilentlyContinue
```
Clear any partial previous install and retry:
```powershell
wsl --unregister Ubuntu
wsl --install -d Ubuntu
```
(This does not touch Docker Desktop's own internal `docker-desktop` /
`docker-desktop-data` distros — leave those alone.)

---

## Step 3 — Verify WSLg (display pipeline) works, before touching Docker

Inside the **Ubuntu** terminal:

```bash
sudo apt update && sudo apt install -y x11-apps
xclock
```

A clock window should appear on your Windows desktop. If it doesn't, stop here
and resolve WSLg first — everything below depends on this working.

> **Clock-drift gotcha:** if `apt update` fails with `Release file ... is not
> valid yet`, your WSL2 VM's clock has drifted (common after sleep/resume).
> Fix: `sudo hwclock -s`, or `wsl --shutdown` from PowerShell and reopen Ubuntu.

---

## Step 4 — Install Docker Desktop and enable WSL integration

1. Download and install **Docker Desktop for Windows** from docker.com, using
   default settings (WSL2 backend, not Hyper-V).
2. Open Docker Desktop → **Settings → Resources → Advanced**, Uncheck `Enable Resource Saver`.
3. Open Docker Desktop → **Settings → Resources → WSL Integration**, Mark Checked `Enable integration with my default WSL distro`.
4. Enable the toggle for **Ubuntu** specifically.
5. **Apply & Restart.**
6. From the **Ubuntu terminal**, confirm Docker is reachable:
   ```bash
   docker ps
   ```
   If you get `permission denied ... docker.sock`:
   ```bash
   sudo usermod -aG docker $USER
   newgrp docker
   ```

---

## Step 5 — Confirm the GPU is visible to WSL and to Docker

Inside Ubuntu:
```bash
nvidia-smi
```
Your NVIDIA GPU should appear (no Linux driver install needed — this is
paravirtualized from the Windows host driver).

Then confirm Docker containers can see it too:
```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

---

## Step 6 — Confirm GPU-accelerated *rendering* (not just compute) works in a container

This is the step that actually matters for Gazebo's GUI, and it's worth testing
in isolation before running the real simulation.

```bash
mkdir -p ~/wslg-test && cd ~/wslg-test
cat > Dockerfile.glxinfo << 'EOF'
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt upgrade -y && apt install -y mesa-utils
ENV LD_LIBRARY_PATH=/usr/lib/wsl/lib
CMD /usr/bin/glxinfo -B
EOF

docker build -t glxinfo -f Dockerfile.glxinfo .

docker run -it --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/wslg:/mnt/wslg \
  -v /usr/lib/wsl:/usr/lib/wsl \
  --device=/dev/dxg \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e PULSE_SERVER=$PULSE_SERVER \
  -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  --gpus all \
  glxinfo
```

Check the output for:
```
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 3050 Laptop GPU)
```
(or your specific NVIDIA model). If it instead shows your Intel GPU or
`llvmpipe`, see **Troubleshooting** below.

---

## Step 7 — Clone the project repo
 
Inside the **Ubuntu** terminal (not PowerShell — keep the repo on the Linux
filesystem, not `/mnt/c/...`, for the same performance/permission reasons
noted earlier):
 
```bash
cd ~
git clone https://github.com/SotirUsama1/NU-CE27-Grad-Project.git
cd NU-CE27-Grad-Project/Sim/gzScripts
```
 
---

---

## Step 8 — The launch script

```bash
./runGzClassic-WSL.sh                       # empty world
./runGzClassic-WSL.sh worlds/cafe.world     # a specific world
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `bad interpreter: bash\r` or `!/usr/bin/env: No such file or directory` | Script was edited/saved on Windows — has CRLF line endings and/or a UTF-8 BOM | `sed -i '1s/^\xEF\xBB\xBF//' script.sh && dos2unix script.sh && chmod +x script.sh` — or just create the file directly inside WSL (`nano`) instead of copying from Windows |
| `permission denied ... docker.sock` | Your WSL user isn't in the `docker` group yet | `sudo usermod -aG docker $USER && newgrp docker` |
| `/usr/bin/docker: Input/output error` | Docker Desktop ↔ WSL2 communication glitch, often after sleep/resume | `docker ps` to confirm it's not script-related first; if it persists, `wsl --shutdown` from PowerShell, restart Docker Desktop, retry |
| `Release file ... is not valid yet` on `apt update` | WSL2 VM clock drift | `sudo hwclock -s`, or `wsl --shutdown` + reopen Ubuntu |
| `glxinfo` shows Intel GPU instead of NVIDIA | Hybrid-laptop Mesa defaults to the first-enumerated (integrated) GPU | Set `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (already in the script above) |
| `glxinfo` shows `llvmpipe` (no D3D12 at all) | Base image's Mesa version predates the `d3d12` Gallium driver (common on Ubuntu 18.04-based images) | Build a derived image adding a newer Mesa PPA, e.g. `ppa:kisak/kisak-mesa`, on top of `sotirusama/gzclassic:latest` |
| `LxssManager` service not found | Normal on Store-delivered WSL — it doesn't always register a classic Windows service | Use `Get-Service *lxss*,*wsl*` to check under alternate names; not itself an error |

---

## Notes

- Verify GPU usage during a real sim run with `nvidia-smi` in a second
  terminal, and check FPS via Gazebo's own GUI (Window → Show FPS).
