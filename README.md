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

### 3. Drive the robot manually (Teleoperation)

Open a second shell in the same container from a new host terminal:

```bash
docker exec -it $(docker ps -q | head -n 1) bash
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

---

### 4. Autonomous Navigation with Nav2 & RViz2

The warehouse includes a pre-generated 2D occupancy grid map (`maps/our_structured_warehouse_map.yaml` + `.pgm`) and a 3D semantic database (`maps/Our_Structured_Warehouse_semantic.json`).

#### Step A: Launch Nav2 Bringup with the Warehouse Map
In a new host terminal connected to the container (`docker exec -it $(docker ps -q | head -n 1) bash`):

```bash
ros2 launch nav2_bringup bringup_launch.py \
    map:=/main/maps/our_structured_warehouse_map.yaml \
    use_sim_time:=true
```

*(This loads the map server, AMCL localization, global/local costmaps, and path planners).*

#### Step B: Launch RViz2 (Nav2 Graphical Interface)
In another terminal connected to the container (`docker exec -it $(docker ps -q | head -n 1) bash`):

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

#### Step C: Operating Nav2 in RViz2
1. **Set Initial Robot Pose:**
   * In the top toolbar, click **`2D Pose Estimate`**.
   * Click and drag on the map near the robot's spawn point (`0, 0`), pointing in the direction the robot faces.
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

* **2D Occupancy Grid Map (`.pgm` + `.yaml`):**
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
