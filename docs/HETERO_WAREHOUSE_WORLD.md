# Hetero warehouse world (`hetero-warehouse-world`)

Rebuilt from **`origin/devvv`**, then only the Warehouse World MiR/suction stack and
the edited `Our_Structured_Warehouse` (thinner boxes, `aisles.yaml`, aisle naming)
were brought over — not the full `Feature/Warehouse_World` history.

## Base vs added

| Layer | Source |
|-------|--------|
| Husky / Jackal / `runGzClassic-Robots` / Nav maps in `warehouse_navigation` | `origin/devvv` (extended with UR+suction) |
| MiR 250 + UR + vacuum suction (`mir_ur_warehouse`) | from Warehouse World |
| Edited warehouse world + `aisles.yaml` + `maps/our_structured_warehouse_*` | from Warehouse World |
| Hetero bring-up (`Dockerfile.hetero`, dual namespaced MiRs) | this branch |

## Robots

| Robot | Arm / gripper | Controllers | Default spawn |
|-------|---------------|-------------|---------------|
| `mir_ur` | UR10e + twelve-cup suction | `/mir_ur/controller_manager` | dock west (−4.5, 23.2) |
| `mir_light` | UR10e + six-cup suction | `/mir_light/controller_manager` | dock east (2.5, 24.6) |
| Husky | UR10e + twelve-cup suction | `/controller_manager` | (−1.5, 2.0) |
| Jackal | UR5e + six-cup suction | `/jackal/controller_manager` | (0.0, 8.0) |

```bash
ros2 service call /mir_ur/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
ros2 service call /mir_light/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
ros2 service call /suction/set_vacuum std_srvs/srv/SetBool "{data: true}"          # Husky (root)
ros2 service call /jackal/suction/set_vacuum std_srvs/srv/SetBool "{data: true}"
```

## Run

```bash
docker pull sotirusama/gzclassic:devvv
docker build -f Sim/gzScripts/Dockerfile.hetero -t gzclassic-hetero:latest Sim/gzScripts
./Sim/gzScripts/runHeteroWarehouseWorld.sh
```

Spawns Gazebo + both MiRs + Husky (−1.5, 2.0) + Jackal (0.0, 8.0). Override poses with
`HUSKY_X` / `JACKAL_X` (etc.) env vars if needed.

World only (no robots): `./Sim/gzScripts/runGzClassic-Robots`

MiRs-only (original `/suction/...` topics): `./Sim/gzScripts/runMirUrWarehouse.sh`

## Conveyor demo (four robots)

After the sim is up:

```bash
./Sim/gzScripts/runHeteroBoxDemo.sh
```

Each robot picks a different carton size from the dock shelves and places it on the
conveyor toward the truck (same flow as `runMirUrHeavyDemo.sh`, extended to Husky/Jackal).
Jackal takes the low-shelf 4 kg long carton (`box_D_001`); Husky takes the mid-shelf 4 kg.

Physics is 2 ms / 500 Hz (stable UR arms, better RTF than the old 1 ms / 1000 Hz).
Restart the sim after world/controller changes.

## URDF extras

| File | Role |
|------|------|
| `mir_ur_warehouse/urdf/clearpath_arm_extras.xacro` | Shared UR + suction macro |
| `mir_ur_warehouse/urdf/husky_ur_suction.urdf.xacro` | Husky `urdf_extras` (lidar + UR10e + large) |
| `mir_ur_warehouse/urdf/jackal_ur_suction.urdf.xacro` | Jackal `JACKAL_URDF_EXTRAS` (UR5e + small) |
