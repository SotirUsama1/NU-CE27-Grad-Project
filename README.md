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
- the `odom` → `base_link` transform from the wheel odometry
- Nav2 drives it on `/cmd_vel`; keyboard teleop on `/cmd_vel_teleop` overrides it

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

#### Step A: Launch Nav2 Bringup with the Warehouse Map
In a new host terminal connected to the container (`docker exec -it $(docker ps -q | head -n 1) bash`):

```bash
ros2 launch warehouse_husky nav2.launch.py
```

*(This loads the map server, AMCL localization, global/local costmaps, and path planners).*

The container and launch files select loopback-only UDP transport for Fast DDS using `config/fastdds_udp.xml`. Shared-memory communication stalled AMCL transforms even while its lifecycle state remained active. Restart the container after changing transport settings so every ROS process uses the same configuration. If you added this XML file to an existing workspace, rebuild `warehouse_husky` first. Keep just one RViz instance open to avoid unnecessary rendering load.

This runs Nav2's `bringup_launch.py` with the Husky's settings ([nav2_params.yaml](Sim/ros_ws/src/warehouse_husky/config/nav2_params.yaml)) and both maps. Nav2's own default parameters are for a TurtleBot3 and would read odometry from `/odom`, which the Husky does not publish. The Husky file also sets the Husky's size and speed, and starts AMCL at the Husky's default spawn pose `(-1.5, 2.0)`, 2 m from the racks. If you spawn the robot elsewhere (`x:=`, `y:=`, `yaw:=`), set the initial pose in RViz as in Step C, and keep clear of the racks: Nav2 cannot plan a turn within about 0.4 m of one.

#### Step B: Launch RViz2 (Nav2 Graphical Interface)

AMCL uses 120 laser samples per scan and 500–2,000 particles. Updates occur after 0.1 m of translation or 0.1 rad of rotation. Check scan alignment while driving and turning before using longer routes; an active lifecycle state alone does not establish localization accuracy. DWB's minimum translation and rotation speeds are configured together to avoid turn commands too weak to overcome Husky's skid friction.

In another terminal connected to the container (`docker exec -it $(docker ps -q | head -n 1) bash`):

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

#### Step C: Operating Nav2 in RViz2
1. **Check the Initial Robot Pose:**
   * AMCL starts at the spawn point, so the robot and laser scan should already sit on the map. Set the pose by hand only if they don't line up, or if you spawned the robot elsewhere:
   * In the top toolbar, click **`2D Pose Estimate`**.
   * Click and drag on the map near the robot's spawn point (`-1.5, 2.0`), pointing in the direction the robot faces.
   * AMCL will snap the laser scan dots (in red) directly onto the warehouse walls and racks.
2. **Activate Navigation:**
   * In the bottom-left **Navigation 2** panel, ensure **`Localization`** is green (`active`).
   * If **`Navigation`** is `inactive`, click the **`Startup`** (or **`Reset`**) button.
3. **Send an Autonomous Goal:**
   * In the top toolbar, click **`Nav2 Goal`** (or **`2D Goal Pose`**).
   * Click on any open white floor in an aisle to set the destination.
   * Nav2 will calculate a global path and autonomously steer the Husky to the target while dodging obstacles.

---

### 5. Regenerating Maps (Optional)

If the warehouse world or models are modified, you can regenerate the maps directly:

* **2D Occupancy Grid Maps (the solid navigation map and the `_scan` map, each `.pgm` + `.yaml` + `.png` preview):**
  ```bash
  python3 scripts/generate_occupancy_grid.py
  ```
* **3D Semantic Database (`.json` with all 626 objects, box heights, and racks):**
  ```bash
  python3 scripts/generate_semantic_db.py
  ```

---

### 6. Stop

Press `Ctrl+C` in the launch terminal. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.
