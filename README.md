# NU-CE27-Grad-Project Blueprint

## System Overview
Multi-robot autonomous warehouse simulation and coordination system. The project integrates Clearpath mobile robot bases (Husky, Jackal) and Universal Robots manipulators with ROS 2 Humble Nav2 autonomous navigation inside Gazebo Classic 11. It provides deterministic operational routing, obstacle dilation, AMCL localization with patched deadlock-free transform buffers, dynamic multi-robot costmap injection, and containerized runtime environments.

## Subdirectories & System Boundaries
- [Sim](./Sim/README.md) - Simulation Subsystem containing Docker runtime scripts, Gazebo worlds and models, and the ROS 2 colcon workspace (`warehouse_navigation`, `warehouse_husky`, `warehouse_jackal`, and `tf2_ros`).

## Global Invariants
- All simulated nodes and controllers must operate with `use_sim_time: true` synchronized to the Gazebo simulation clock (`/clock`).
- Robot topics, TF streams, and controller nodes must be strictly namespaced (e.g., `/husky1`, `/husky2`, `/jackal1`) with `/tf` and `/tf_static` remapped locally to prevent inter-robot frame collisions.
- Odometry TF (`odom -> base_link`) is exclusively broadcast by `robot_localization` EKF filter nodes; controller odometry TF broadcasting must remain disabled.
- Manual teleoperation (`/cmd_vel`, priority 7) strictly preempts autonomous Nav2 velocity commands (`/cmd_vel_nav`, priority 5) via `twist_mux`.
- The ROS 2 `tf2_ros` package in `Sim/ros_ws/src/tf2_ros` includes a verified project patch in `src/buffer.cpp` preventing an AB-BA mutex deadlock between AMCL laser scan callbacks and `TransformListener` updates.

## Global Side Effects
- Launching the simulation mounts host directories `Sim/models`, `Sim/worlds`, and `Sim/ros_ws` into the container at `/main/models`, `/main/worlds`, and `/main/ros_ws`.
- Interactive container execution compiles workspace packages via `colcon build`, generating `build`, `install`, and `log` directories on the host filesystem.
- The simulation modifies host X11 permissions (`xhost +local:docker`) and binds host graphics drivers for OpenGL rendering.

## System Verification
- Run: `./Sim/gzScripts/runGzClassic-Robots bash`

---

## Running the robots in simulation

The Docker image `sotirusama/gzclassic:devvv` ships ROS 2 Humble, Gazebo Classic 11 and these robots, built and ready to launch:

- **Clearpath Husky** and **Clearpath Jackal** (mobile bases)
- **Universal Robots arms** (UR3 to UR30)
- **TurtleBot3** (burger, waffle, waffle_pi)

### Open a world on its own

To look at a world without a robot, run from the repository root:

```bash
./Sim/gzScripts/runGzClassic-Robots
```

With no argument this opens `Our_Structured_Warehouse`. To open another world, pass its path, for example `./Sim/gzScripts/runGzClassic-Robots worlds/empty.world`.

A robot cannot be added to a world opened this way, because the plain `gazebo` command does not load the ROS plugins needed to spawn one. To use a robot, follow the steps below; the robot's launch file opens the world itself.

### 1. Start the container

From the repository root:

```bash
./Sim/gzScripts/runGzClassic-Robots bash
```

This opens a shell inside the container, with `Sim/models`, `Sim/worlds` and `Sim/ros_ws` mounted at `/main/models`, `/main/worlds` and `/main/ros_ws`.

It first builds the project's ROS packages in `Sim/ros_ws/src` with `colcon build --symlink-install`. Edits to their launch, config and URDF files apply on the next launch; restart the container after adding a new file or package. The `build`, `install` and `log` folders it creates are ignored by git.

### 2. Launch a robot

Run only one simulation at a time; they all use the same Gazebo port.

**Husky** in the warehouse, set up for mapping and navigation
```bash
ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
```

This is the project's Husky (`Sim/ros_ws/src/warehouse_husky`), built on Clearpath's:

- a 2D lidar on `/scan`
- the `odom` → `base_link` transform from an EKF that combines the wheel speed with the IMU gyroscope (the skid-steer wheels slip in turns, so on their own they misjudge how far the robot turned)
- Nav2 can drive it on `/cmd_vel_nav`; keyboard teleop on `/cmd_vel` overrides it

Extra arguments: `gui:=false` runs without the Gazebo window, and `x:=`, `y:=`, `yaw:=` set the spawn pose. Clearpath's stock Husky (no lidar) is still available with `ros2 launch husky_gazebo gazebo.launch.py world_path:=...`.

**Jackal** in the warehouse
```bash
ros2 launch jackal_gazebo gazebo.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
```

**UR arm** (opens its own empty world)
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e
```

`world_path` is required: without it the robot opens in an empty world. It is looked up inside `Sim/worlds`, so other worlds work the same way, for example `world_path:=Grad_warehouse/Grad_warehouse.world` or `world_path:=worlds/empty.world` for an empty world.

### 3. Drive the robot

Open a second shell in the same container from a new host terminal. Find the container name with `docker ps`, then:

```bash
docker exec -it <container_name> bash
```

**Husky or Jackal:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep this terminal focused while pressing keys: `i` forward, `j` / `l` turn, `k` stop.

**UR arm:** send a test trajectory:

```bash
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

### 4. The warehouse map (Husky)

Nav2 needs a map of the warehouse. It is already in the repository, in `Sim/ros_ws/src/warehouse_navigation/maps` (`warehouse.pgm` is the image, `warehouse.yaml` its scale and origin), so there is nothing to do unless the world changes. A second map, `warehouse_lidar`, shows the warehouse as the lidar sees it (rack posts one by one instead of solid racks); only localization uses it.

**Open the map image** on your computer (not in the container), from the repository root:

```bash
xdg-open Sim/ros_ws/src/warehouse_navigation/maps/warehouse.pgm
```

Black is an obstacle, white is free floor. If no viewer opens, install one with `sudo apt install eog`.

**View the map** with the Husky and its lidar on top:

1. Terminal 1: launch the Husky in the warehouse (step 2).
2. Terminal 2 (`docker exec -it <container_name> bash`):
   ```bash
   ros2 launch warehouse_navigation view_map.launch.py
   ```
   RViz opens with the map (black = obstacle, white = free), the robot, and the lidar scan in red. The red points should sit on the black walls, pallets and rack edges. If you spawned the Husky somewhere else, pass the same pose here, for example `x:=2.0 y:=1.0 yaw:=1.57`.

The robot is placed on the map at its spawn pose and then follows its wheel odometry, so it drifts slowly as it drives; Nav2's localization (AMCL) will correct this.

The map is generated from `Our_Structured_Warehouse.world` rather than built with SLAM: every obstacle in the world is a collision box, so the script draws walls, pallets, pillars, the dock and the truck from their exact positions, and each rack as one solid block. Labelled boxes are left out because they move.

**After changing the world**, regenerate the maps inside the container and commit all four files:

```bash
python3 /main/ros_ws/src/warehouse_navigation/scripts/generate_map.py
chown $(stat -c %u:%g /main/ros_ws/src) /main/ros_ws/src/warehouse_navigation/maps/warehouse*
```

`chown` gives the files back to your host user instead of root. If the script fails with `No module named 'PIL'`, your image is older than the Pillow addition: run `docker pull sotirusama/gzclassic:devvv`.

SLAM mapping (`ros2 launch warehouse_navigation slam.launch.py` while driving) is still available, but it does not work well in this warehouse: the lidar sees through the racks, and the identical aisles make it lose track.

### 5. Localization (Husky)

AMCL keeps track of where the robot is on the map: it compares the lidar scan with the map and corrects the wheel odometry as the robot drives.

1. Terminal 1: launch the Husky in the warehouse (step 2).
2. Terminal 2:
   ```bash
   ros2 launch warehouse_navigation localization.launch.py
   ```
3. Terminal 3: drive with the keyboard (step 3).

RViz shows the map, the robot, the lidar scan in red and AMCL's guesses as green arrows. While the robot drives, the green arrows should stay in a tight cluster under the robot and the red points on the black obstacles.

If you spawned the Husky somewhere else, pass the same pose, for example `x:=2.0 y:=1.0 yaw:=1.57`. If the robot is drawn in the wrong place, click **2D Pose Estimate** in RViz's toolbar, then click its real position on the map and drag in the direction it faces.

Run either this or `view_map.launch.py`, not both: they both place the robot on the map.

### 6. Navigation (Husky)

Instead of `localization.launch.py` (it is included), run in terminal 2:

```bash
ros2 launch warehouse_navigation navigation.launch.py
```

Wait for `Managed nodes are active` to appear twice (about 10 s). Then in RViz click **2D Goal Pose**, click a free spot on the map and drag to choose the direction the robot should face. The robot plans a route (green line) and drives there by itself.

What runs, all in `config/nav2_params.yaml`:

| Part | Job |
|---|---|
| Global costmap | the whole map with a 1 m safety zone around every obstacle, plus what the lidar sees; routes avoid passages narrower than about 1.25 m, which the 1.0 m long Husky could only enter perfectly lined up |
| Planner (NavFn) | the cheapest route from the robot to the goal on the global costmap, recomputed every second |
| Local costmap | a 6 x 6 m window around the robot, updated from the lidar 5 times a second |
| Controller (MPPI) | 20 times a second tries about 2000 short motions, keeps those whose whole footprint stays clear, and drives the best one on `/cmd_vel_nav` |
| Recovery behaviors | when stuck: clear the costmaps, spin 90 degrees, wait 5 s or back up 0.3 m, then try again (up to 6 times) |
| BT Navigator | takes the goal and runs the behavior tree that uses all of the above |

RViz shows the costmaps (red/purple = obstacle, blue to cyan = safety zone) and the robot's footprint as a cyan rectangle. A box added in Gazebo in front of the robot appears in the costmaps and the route goes around it. Keyboard teleop on `/cmd_vel` always overrides navigation.

A goal can also be sent from a terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 28.0, y: 25.6}, orientation: {w: 1.0}}}}"
```

The goal must leave room for the whole robot (1.0 x 0.69 m): a spot right against a wall is never reached.

### 7. Drive to the furthest point (Husky)

With navigation running (step 6), in another terminal:

```bash
ros2 run warehouse_navigation go_far.py
```

It finds the point of the warehouse furthest from the robot (in a straight line) that the robot can reach and that leaves room to turn on the spot (0.75 m clear all round), sends it to Nav2, and prints progress until the robot arrives. Add `--ros-args -p dry_run:=true` to only print the point. From the default spawn pose it is (28.2, 25.9), 38 m away; the drive takes about 2 minutes.

### 8. Stop

Press `Ctrl+C` in the launch terminal. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.

---

## Backend Orchestrator & Multi-Robot Coordination

The CyberScape backend orchestrator coordinates multi-robot warehouse fleets through a decoupled architecture:
- **FastAPI Fast-Loop Orchestrator**: Manages fleet status, REST endpoints with `X-API-Key` authentication middleware, and background task loops.
- **PostgreSQL 16 Task Queue**: Concurrency task claiming via `SELECT FOR UPDATE SKIP LOCKED`, task prerequisite gating, and robot session affinity.
- **Redis 7 Segment Locking**: Atomic Lua-based undirected edge locking, junction slot reservation, and rolling leases to prevent corridor deadlocks.
- **Simulation-Time Heartbeat Monitor**: Evaluates robot liveness against Gazebo `/clock` simulation timestamps (Tier 1 `UNRESPONSIVE` hold, Tier 2 `QUARANTINED` isolation).
- **LangGraph Multi-Agent Worker**: Decoupled natural language parsing grounded to warehouse landmarks, deterministic primitive sequencing (`NAVIGATE_TO`, `WAIT_FOR_LOAD`, `WAIT_FOR_UNLOAD`, `REPORT_STATUS`), and supervisory Nav2 failure triage.

### 1. Start Infrastructure with Docker Compose

From the repository root (or inside `backend/`):

```bash
docker compose up -d
```

This boots:
- PostgreSQL 16 on port `5432` (schema initialized from `backend/sql/schema.sql`)
- Redis 7 on port `6379`
- Backend Orchestrator on `http://localhost:8000`
- LangGraph Agent Worker running in the background

To view service logs:
```bash
docker compose logs -f orchestrator agent-worker
```

### 2. Run Backend Test Suite

To run all unit, concurrency, and integration tests across the backend:

```bash
cd backend
pytest -v
```

To run a specific test subsystem:
```bash
pytest tests/test_api.py -v             # REST API & authentication integration tests
pytest tests/test_langgraph.py -v       # LangGraph multi-agent workflow tests
pytest tests/test_redis_locking.py -v   # Redis atomic Lua segment locking tests
pytest tests/test_dispatcher.py -v      # Task claiming & route dispatching tests
pytest tests/test_heartbeat.py -v       # Simulation-time liveness monitor tests
pytest tests/test_db.py -v              # Database concurrency & SKIP LOCKED tests
```

### 3. Dispatch Missions to Robots via REST API

Submit a natural language mission string with the `X-API-Key` authentication header:

```bash
curl -X POST http://localhost:8000/missions \
     -H "Content-Type: application/json" \
     -H "X-API-Key: cyberscape-secret-key" \
     -d '{"text": "Move box A-001 from rack A2 to shipping dock"}'
```

Response (HTTP 201 Created):
```json
{
  "id": "e4b3c2a1-0000-0000-0000-000000000000",
  "text": "Move box A-001 from rack A2 to shipping dock",
  "status": "SUBMITTED",
  "created_at": "2026-11-12T15:00:00Z"
}
```

### 4. Monitor Mission Status & Task Sequence

Poll the mission execution state and its sequenced primitives:

```bash
curl -H "X-API-Key: cyberscape-secret-key" http://localhost:8000/missions/{mission_id}
```

### 5. Fleet Management

Inspect the current status, location, and telemetry of all robots in the fleet:

```bash
curl -H "X-API-Key: cyberscape-secret-key" http://localhost:8000/robots
```

Register a newly spawned robot into the fleet:

```bash
curl -X POST http://localhost:8000/robots \
     -H "Content-Type: application/json" \
     -H "X-API-Key: cyberscape-secret-key" \
     -d '{"robot_id": "husky1", "current_node": "junction_A_1_1"}'
```

Re-commission a robot from `MAINTENANCE` back to `IDLE` after manual inspection:

```bash
curl -X POST http://localhost:8000/robots/husky1/resume \
     -H "X-API-Key: cyberscape-secret-key"
```

### 6. Emergency Safety Override

Lift an active Tier 2 quarantine zone (node and its incident corridor edges) using the administrative token:

```bash
curl -X POST http://localhost:8000/override \
     -H "Content-Type: application/json" \
     -d '{"node": "junction_A_1_1", "admin_token": "cyberscape-secret-key"}'
```

