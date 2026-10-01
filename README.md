# NU-CE27-Grad-Project

## Running the robots in simulation

The Docker image `sotirusama/gzclassic:devvv` ships ROS 2 Humble, Gazebo Classic 11 and these robots, built and ready to launch:

- **Clearpath Husky** and **Clearpath Jackal** (mobile bases)
- **Universal Robots arms** (UR3 to UR30)
- **TurtleBot3** (burger, waffle, waffle_pi)

### 1. Start the container

From the repository root:

```bash
./Sim/gzScripts/runGzClassic-Robots bash
```

This opens a shell inside the container, with `Sim/models` and `Sim/worlds` mounted at `/main/models` and `/main/worlds`.

### 2. Launch a robot

Run only one simulation at a time; they all use the same Gazebo port.

**Husky**
```bash
ros2 launch husky_gazebo gazebo.launch.py world_path:=/usr/share/gazebo-11/worlds/empty.world
```

**Jackal**
```bash
ros2 launch jackal_gazebo gazebo.launch.py world_path:=/usr/share/gazebo-11/worlds/empty.world
```

**UR arm** (opens its own empty world)
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e
```

To use the warehouse instead of the empty world, pass the world file path:

```bash
world_path:=/main/worlds/Grad_warehouse/Grad_warehouse.world
```

### 3. Drive the robot

Open a second shell in the same container from the host:

```bash
docker exec -it $(docker ps -q --filter ancestor=sotirusama/gzclassic:devvv) bash
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

### 4. Stop

Press `Ctrl+C` in the launch terminal. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.
