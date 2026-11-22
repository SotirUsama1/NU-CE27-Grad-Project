# Hetero warehouse world (`hetero-warehouse-world`)

Rebuilt from **`origin/devvv`**, then only the Warehouse World MiR/suction stack and
the edited `Our_Structured_Warehouse` (thinner boxes, `aisles.yaml`, aisle naming)
were brought over — not the full `Feature/Warehouse_World` history.

## Base vs added

| Layer | Source |
|-------|--------|
| Husky / Jackal / `runGzClassic-Robots` / Nav maps in `warehouse_navigation` | `origin/devvv` (unchanged) |
| MiR 250 + UR + vacuum suction (`mir_ur_warehouse`) | from Warehouse World |
| Edited warehouse world + `aisles.yaml` + `maps/our_structured_warehouse_*` | from Warehouse World |
| Hetero bring-up (`Dockerfile.hetero`, dual namespaced MiRs) | this branch |

## Robots

| Robot | Gripper / notes | Default spawn |
|-------|-----------------|---------------|
| `mir_ur` | Twelve-cup suction | dock west (−4.5, 23.2) |
| `mir_light` | Six-cup suction | dock east (2.5, 24.6) |
| Husky | Mobile base (`devvv` stack) | launch args (default 0,0) |
| Jackal | Mobile base (`devvv` stack) | (4.0, 2.0) via spawn helper |

MiRs use `/mir_ur` and `/mir_light` so Husky can keep `/controller_manager`.

```bash
ros2 service call /mir_ur/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
ros2 service call /mir_light/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
```

## Run

```bash
docker pull sotirusama/gzclassic:devvv
docker build -f Sim/gzScripts/Dockerfile.hetero -t gzclassic-hetero:latest Sim/gzScripts
./Sim/gzScripts/runHeteroWarehouseWorld.sh
```

Add Husky / Jackal into the same Gazebo:

```bash
docker exec -it hetero_warehouse_sim bash
source /opt/ros/humble/setup.bash
source /opt/ros2_robots/install/setup.bash
source /main/ros_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 launch warehouse_husky husky.launch.py start_gazebo:=false
ros2 launch warehouse_husky jackal_spawn.launch.py
```

World only (no robots): `./Sim/gzScripts/runGzClassic-Robots`

MiRs-only (original `/suction/...` topics): `./Sim/gzScripts/runMirUrWarehouse.sh`

## Suction on Husky / Jackal

Husky and Jackal here are **bases without arms**. Reusable suction lives in
`suction_gripper.xacro`; standalone UR + suction in `ur_with_suction.urdf.xacro`.
