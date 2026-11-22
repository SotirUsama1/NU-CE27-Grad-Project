# Our MiR 250 + UR Arm

What we run in the warehouse sim. Workspace: `Sim/ros2_ws`.

---

## Robot

| Part | Ours |
|------|------|
| Base | **MiR 250** (lasers `f_scan`/`b_scan`, IMU) |
| Arm | **UR10e** (also `ur5e` / `ur16e` via launch) |
| Gripper | **Vacuum suction** — not Robotiq |
| Camera | RealSense D435i **off** in our sim |

**Arm mount** on MiR `base_link`: `(0.135, 0, 0.79)` m, yaw **+90°**. Joint prefix: `ur_`.

**Grippers**

| | Large (default) | Small |
|--|-----------------|-------|
| Size | 30×40 cm, 12 cups | 30×18 cm, 6 cups |
| Use | 8 kg boxes | 2 / 4 kg boxes |
| Entity | `mir_ur` | `mir_light` |

Vacuum on/off: `/suction/set_vacuum`.

---

## What we customized

Upstream base: `Sim/ros2_ws/src/UR-MiR-mobile-manipulator/`  
Our package: `Sim/ros2_ws/src/mir_ur_warehouse/`

| Change | Why |
|--------|-----|
| Suction EE (large/small) | Carton pick instead of Robotiq |
| Two robots in one world | Big + small gripper |
| RealSense removed | Sim speed |
| `park_base` | Arm motion won’t shake the base when parked |
| Forked ros2_control plugin | Two robots, two controller managers |
| Arm **position-only** cmds | Velocity cmds spin the MiR |
| Mount frame = `arm_mount_frame` | Avoid Gazebo `world` name clash |

---

## Our files (`mir_ur_warehouse`)

| File | Role |
|------|------|
| `urdf/mir_ur.urdf.xacro` | MiR + UR + suction + plugins |
| `launch/mir_ur_warehouse.launch.py` | Spawn both robots in warehouse |
| `config/controllers.yaml` | Robot 1 (wheels + arm) |
| `config/controllers_light.yaml` | Robot 2 |
| `config/ur*_joint_limits.yaml` | Arm joint limits (`ur_*` names) |
| `src/vacuum_gripper.cpp` | Suction physics |
| `src/park_base.cpp` | Hold base still when idle |
| `src/gazebo_ros2_control_plugin.cpp` | Dual-robot control bridge |

Mount numbers live in upstream:  
`.../mir_description/urdf/include/mir_250_v1.urdf.xacro`

---

## Run

```bash
docker build -f Sim/gzScripts/Dockerfile.mir_ur -t gzclassic-mir-ur:latest Sim/gzScripts
./Sim/gzScripts/runMirUrWarehouse.sh
docker exec -it mir_ur_sim bash
```

Demos: `./Sim/gzScripts/runMirUrDemo.sh` · `./Sim/gzScripts/runMirUrHeavyDemo.sh`
