# NU-CE27-Grad-Project

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

To run the template Amazon world:

```bash
./Sim/gzScripts/runGzClassic.sh worlds/no_roof_small_warehouse/no_roof_small_warehouse.world
```

A robot cannot be added to a world opened this way, because the plain `gazebo` command does not load the ROS plugins needed to spawn one. To use a robot, follow the steps below; the robot's launch file opens the world itself.

### 1. Start the container

From the repository root:

```bash
./Sim/gzScripts/runGzClassic-Robots bash
```

This opens a shell inside the container, with `Sim/models`, `Sim/worlds` and `Sim/ros_ws` mounted at `/main/models`, `/main/worlds` and `/main/ros_ws`.

ROS 2 traffic stays on this computer through the loopback-only UDP profile in `warehouse_husky/config/fastdds_udp.xml`. The container applies it to simulation, Nav2, RViz and command-line tools. `ROS_LOCALHOST_ONLY=0` is intentional: Humble's Fast DDS integration adds shared-memory transport back when this variable is `1`, overriding the intended UDP-only setup. The XML interface whitelist (`127.0.0.1`) keeps teammates' simulations off this ROS graph.

It first builds the project's ROS packages in `Sim/ros_ws/src` with `colcon build --symlink-install`. Edits to their launch, config and URDF files apply on the next launch; restart the container after adding a new file or package. The `build`, `install` and `log` folders it creates are ignored by git.

### 2. Launch a robot

Run only one simulation at a time; they all use the same Gazebo port.

**Husky** in the warehouse, set up for mapping and navigation
```bash
ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
```

This is the project's Husky (`Sim/ros_ws/src/warehouse_husky`), built on Clearpath's:

- a 2D lidar on `/scan`
- the `odom` → `base_link` transform from an EKF (`robot_localization`) that fuses wheel speed with the IMU heading, because the skid-steer wheels slip when turning (`/odometry/filtered`)
- Nav2 drives it on `/cmd_vel`; keyboard teleop on `/cmd_vel_teleop` overrides it

Extra arguments: `gui:=false` runs without the Gazebo window, and `x:=`, `y:=`, `yaw:=` set the spawn pose (default `-1.5, 2.0, 0`: open floor 2 m from the racks). Clearpath's stock Husky (no lidar) is still available with `ros2 launch husky_gazebo gazebo.launch.py world_path:=...`.

**Jackal** in the warehouse
```bash
ros2 launch jackal_gazebo gazebo.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
```

**UR arm** (opens its own empty world)
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e
```

`world_path` is required: without it the robot opens in an empty world. It is looked up inside `Sim/worlds`, so other worlds work the same way, for example `world_path:=Grad_warehouse/Grad_warehouse.world` or `world_path:=worlds/empty.world` for an empty world.

### 3. Drive the robot manually (Teleoperation)

Open a second shell in the same container from a new host terminal:

```bash
docker exec -it $(docker ps -q | head -n 1) bash
```

**Husky:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

**Jackal:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep this terminal focused while pressing keys: `i` forward, `j` / `l` turn, `k` stop.

**UR arm:** send a test trajectory:

```bash
ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
```

---

### 4. Autonomous Navigation with Nav2 & RViz2

The warehouse includes two pre-generated 2D occupancy grid maps and a 3D semantic database (`maps/Our_Structured_Warehouse_semantic.json`):

- `maps/our_structured_warehouse_map.yaml` + `.pgm`: racks drawn solid (the robot cannot drive under them). The Nav2 costmaps plan on it.
- `maps/our_structured_warehouse_map_scan.yaml` + `.pgm`: only what the lidar sees. The racks are open frames, so the lidar sees just their posts. AMCL localizes on it.

#### Step 1: Start the Docker Container & Launch Husky with Gazebo GUI
In your **First Host Terminal**:
```bash
./Sim/gzScripts/runGzClassic-Robots bash
```
Once inside the container shell, launch the warehouse world and Husky:
```bash
ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world gui:=true
```
*(Gazebo Classic 3D window will open with the Husky spawned at `(-1.5, 2.0)` facing the warehouse aisles).*

#### Step 2: Launch Nav2 Stack (Tuned MPPI Controller - Default)
In your **Second Host Terminal**, connect to the running container:
```bash
docker exec -it $(docker ps -q | head -n 1) bash
```
Inside the container, launch the Nav2 navigation stack:
```bash
ros2 launch warehouse_husky nav2.launch.py
```
*(This loads the map servers, AMCL localization, global/local costmaps, and the Model Predictive Path Integral [MPPI] controller tuned for narrow warehouse corridors).*

*(Optional)* To benchmark alternative controllers and robot platforms:
- Husky with RPP: `ros2 launch warehouse_husky nav2.launch.py controller:=rpp`
- Husky with DWB: `ros2 launch warehouse_husky nav2.launch.py controller:=dwb`
- Jackal with MPPI: `ros2 launch warehouse_husky nav2.launch.py robot:=jackal controller:=mppi`
- Jackal with RPP: `ros2 launch warehouse_husky nav2.launch.py robot:=jackal controller:=rpp`
- Custom params: `ros2 launch warehouse_husky nav2.launch.py params_file:=/path/to/custom_params.yaml`

#### Step 3: Launch RViz2 (Nav2 Graphical Interface)
In your **Third Host Terminal**, connect to the running container:
```bash
docker exec -it $(docker ps -q | head -n 1) bash
```
Inside the container, launch RViz2:
```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

#### Step 4: Operating Nav2 in RViz2
1. **Initial Robot Pose:**
   * AMCL automatically initializes at spawn pose `(-1.5, 2.0)`, matching the red laser scan points against warehouse walls and rack posts.
   * If you spawned the robot at a custom pose, click **`2D Pose Estimate`** in the top toolbar and click/drag on the map to set the initial pose.
2. **Send an Autonomous Navigation Goal:**
   * In the top toolbar, click **`Nav2 Goal`** (or press `g`).
   * Click inside any picking aisle (for example, at `(2.47, 3.5)` in Aisle 1) facing North ($90^\circ$).
   * The MPPI controller will predictively sample trajectories, center the Husky within the narrow 1.45 m corridor, and follow the path with sub-15 cm accuracy.

---

### 5. Running the Local Planner Stress Benchmark Suite

To run the automated endurance benchmark comparing MPPI vs. RPP over 150 test scenarios across narrow corridors, dead-ends, and slaloms:

```bash
# Execute headless benchmark on host (auto-manages containers and domain isolation)
./scripts/run_automated_benchmarks.sh

# Generate the formatted Markdown comparison report from JSON logs
python3 scripts/generate_benchmark_report.py
```

*Detailed analysis, failure mode breakdowns, and controller theory:* see [`docs/NAV2_BENCHMARK_REPORT.md`](file:///home/ubuntu/Documents/Grad/NU-CE27-Grad-Project/docs/NAV2_BENCHMARK_REPORT.md).

---

### 6. Regenerating Maps (Optional)

If the warehouse world or models are modified, you can regenerate the maps directly:

* **2D Occupancy Grid Maps (the solid navigation map and the `_scan` map, each `.pgm` + `.yaml` + `.png` preview):**
  ```bash
  python3 scripts/generate_occupancy_grid.py
  ```
  It reads the collision boxes from the `.world` and the models it includes, and needs `Pillow` and `PyYAML`. If you move the lidar, update `LIDAR_Z` in the script to match `urdf/husky_lidar.urdf.xacro`.
* **3D Semantic Database (`.json` with all 626 objects, box heights, and racks):**
  ```bash
  python3 scripts/generate_semantic_db.py
  ```

---

### 7. Stop

Press `Ctrl+C` in each launch terminal: Nav2 first, then the robot. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.

---

### 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| RViz shows an empty map, no robot; Nav2 log repeats `Detected jump back in time` | Another ROS 2 simulation on the network shares your graph. Check `ros2 node list`: duplicate `/gazebo` or `/twist_mux` entries confirm it. Start the container with `./Sim/gzScripts/runGzClassic-Robots bash`, which sets `ROS_LOCALHOST_ONLY=1`. |
| Nav2 stops partway through startup, no `map` frame | AMCL has no initial pose. It is set automatically for the default spawn; if you spawned elsewhere, click **2D Pose Estimate** in RViz. |
| Goal aborts at once, log shows `No valid trajectories` | The robot is within about 0.4 m of a rack. Drive it into open floor with teleop and send the goal again. |
| Robot does not move for keyboard teleop | Run the Husky teleop command from Step 3, which publishes on `/cmd_vel_teleop`. |

---

### 9. warehouse_navigation (map, localization, navigation)

`Sim/ros_ws/src/warehouse_navigation` is a second Husky navigation stack, with its own map in `maps/` (`warehouse.pgm` and `warehouse.yaml`, plus `warehouse_lidar` for what the lidar sees). It drives on `/cmd_vel_nav`. Keyboard teleop stays the command from step 3 (`/cmd_vel_teleop`), which overrides it. Run this stack or `ros2 launch warehouse_husky nav2.launch.py`, not both.

**View the map** after launching the Husky (step 2), in a second shell:

```bash
ros2 launch warehouse_navigation view_map.launch.py
```

**Localization** (AMCL) in that second shell instead:

```bash
ros2 launch warehouse_navigation localization.launch.py
```

**Navigation** (includes localization). Wait for `Managed nodes are active` twice, then use **2D Goal Pose** in RViz:

```bash
ros2 launch warehouse_navigation navigation.launch.py
```

**Drive to the furthest reachable point:**

```bash
ros2 run warehouse_navigation go_far.py
```

**After changing the world**, regenerate that package's maps inside the container:

```bash
python3 /main/ros_ws/src/warehouse_navigation/scripts/generate_map.py
chown $(stat -c %u:%g /main/ros_ws/src) /main/ros_ws/src/warehouse_navigation/maps/warehouse*
```

Pass the same `x:=`, `y:=` and `yaw:=` you gave the Husky if it was not spawned at the default `(-1.5, 2.0)`. Run `view_map.launch.py` or `localization.launch.py`, not both: both place the robot on the map.

