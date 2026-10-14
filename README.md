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

Nav2 needs a map of the warehouse. It is already in the repository, in `Sim/ros_ws/src/warehouse_navigation/maps` (`warehouse.pgm` is the image, `warehouse.yaml` its scale and origin), so there is nothing to do unless the world changes.

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

**After changing the world**, regenerate the map inside the container and commit both files:

```bash
python3 /main/ros_ws/src/warehouse_navigation/scripts/generate_map.py
chown $(stat -c %u:%g /main/ros_ws/src) /main/ros_ws/src/warehouse_navigation/maps/warehouse.*
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

### 6. Costmaps (Husky)

A costmap marks how dangerous each spot is for the robot's centre: obstacles are lethal, and a safety zone around them (0.7 m) gets a cost that falls with distance. The **global costmap** covers the whole map and is used to plan routes; the **local costmap** is a 6 x 6 m window around the robot, updated from the lidar, used to follow the route and avoid new obstacles.

Instead of `localization.launch.py` (it is included), run in terminal 2:

```bash
ros2 launch warehouse_navigation costmaps.launch.py
```

RViz now also shows the costmaps (red/purple = obstacle, blue to cyan = safety zone) and the robot's footprint as a cyan rectangle. Drive the robot with the keyboard: the local costmap moves with it. A box added in Gazebo in front of the robot appears in the costmaps, and disappears again after it is removed.

### 7. Stop

Press `Ctrl+C` in the launch terminal. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.
