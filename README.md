# NU-CE27-Grad-Project

## Automated checks (GitHub Actions)

Every branch push and pull request runs these checks. They can also be started
manually from the repository's **Actions** tab:

- **CI Tests / Python tests (3.10)** installs `requirements-test.txt` and runs the
  mapping, coordinate, memory, and HTTP API tests without ROS or Gazebo. The JUnit
  report is available as the `python-test-results` artifact for 14 days.
- **CI Tests / ROS 2 Humble build** builds both project packages inside
  `sotirusama/gzclassic:devvv`. It installs missing ROS system dependencies and the
  declared Python dependencies, checks imports and shared libraries, loads both
  launch files, expands the Husky model, and starts the installed semantic node
  to query its real HTTP server. Build logs and the runtime test report are
  retained for 14 days. The Docker Hub image must be publicly pullable and provide
  ROS Humble, Gazebo Classic, rosdep, and the upstream robot workspace.
- **Check Scripts** checks the Bash launchers and map-saving script with `bash -n`
  and compiles the Python sources to check syntax. It does not launch a GUI or
  require a GPU.

The mapping tests use a synthetic occupancy grid when a saved SLAM map is absent;
these checks do not validate a running Gazebo simulation.

To run the Python tests locally from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-test.txt
PYTHONPATH="$PWD/Sim/ros_ws/src/semantic_world_manager" \
  python -m pytest tests Sim/ros_ws/src/semantic_world_manager/test -v
```

The separate **Build and Push Docker Image** workflow publishes
`sotirusama/gzclassic` when its Dockerfile or dependency files change on `main`, or when manually
started with a version. It requires the repository Actions secret
`DOCKERHUB_TOKEN` for the `sotirusama` account. The existing publishing workflow
does not update the `devvv` image tag automatically on a `main` push; use `devvv`
as the manual version when intentionally refreshing the image used by CI.

### Dependency declarations and audit

- `Sim/ros_ws/src/semantic_world_manager/requirements.txt` declares FastAPI,
  Pydantic, and Uvicorn. The Python package metadata, test requirements, simulation
  requirements, Docker build, and CI all use this same file.
- `requirements-test.txt` adds pytest, HTTPX, Pillow, and PyYAML. The ROS runtime
  test is skipped in the plain Python job and runs in the ROS job.
- `requirements-sim.txt` adds the mesh and transformation libraries. NumPy is
  kept below 2 for Humble's compiled `cv_bridge`; transforms3d must be at least
  0.4.2 because Ubuntu 22.04's 0.3.1 uses removed NumPy APIs.
- Each ROS package's `package.xml` declares its ROS/system dependencies, which
  are checked with rosdep. The newer API Python dependencies are installed with
  pip; Jammy's apt API packages do not satisfy the declared versions.

To audit an existing simulation container, mount the complete repository at
`/workspace`, build the ROS workspace, and run from `/workspace`:

```bash
source /opt/ros/humble/setup.bash
source /opt/ros2_robots/install/setup.bash
rosdep install --from-paths Sim/ros_ws/src --ignore-src --rosdistro humble -y
python3 -m pip install -r requirements-test.txt -r requirements-sim.txt
python3 -m pip check
rosdep check --from-paths Sim/ros_ws/src --ignore-src --rosdistro humble
source Sim/ros_ws/install/setup.bash
python3 scripts/check_dependencies.py
WORKSPACE_ROOT="$PWD" ROS_LOCALHOST_ONLY=1 \
  python3 -m pytest tests/test_ros_runtime.py -v
```

These commands assume the repository is mounted with its `Sim/` directory intact;
mount it at `/workspace` as CI does when reproducing the audit. The standard
launcher mounts individual directories instead. Rebuild its image after changes
to any dependency file so interactive runs receive the same fixes.

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

When the Dockerfile changes, rebuild the local image before starting the container:

```bash
docker build -t sotirusama/gzclassic:devvv -f Sim/gzScripts/Dockerfile .
```

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

### 4. Stop

Press `Ctrl+C` in the launch terminal. Do not use `Ctrl+P` `P` (detach): it leaves the simulation half-stopped.

## Generating Maps for the Environment (2.5D Mapping)

The project uses a 2.5D mapping architecture: a 2D physical SLAM map (X/Y) fused with a 3D Semantic JSON Database (X/Y/Z and object bounds). If you modify `Our_Structured_Warehouse` or want to map a new world, follow these steps to regenerate both maps.

### 1. Extract the Semantic 3D Database (JSON)
On your host machine, parse the 3D `.world` file to automatically generate the bounding boxes, heights (Z), and labels for all semantic objects:
```bash
python3 scripts/generate_semantic_db.py --world Our_Structured_Warehouse
```
*This creates `maps/Our_Structured_Warehouse_semantic.json`.*

### 2. Generate the 2D Physical Map (SLAM)
Open 4 separate terminals inside your container:

**Terminal 1 (Gazebo):**
```bash
ros2 launch warehouse_husky husky.launch.py world_path:=Our_Structured_Warehouse/Our_Structured_Warehouse.world
```

**Terminal 2 (SLAM Toolbox):**
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=/main/ros_ws/src/warehouse_husky/config/mapper_params_online_async.yaml use_sim_time:=true
```

**Terminal 3 (Semantic State Manager):**
```bash
ros2 launch semantic_world_manager semantic_manager.launch.py
```

**Terminal 4 (Keyboard Teleop):**
*(Keep this window focused to drive the robot around)*
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save the Map:** Once the map looks complete, open a 5th terminal and save it:
```bash
ros2 run nav2_map_server map_saver_cli -f /main/maps/our_structured_warehouse_map
```
*This saves `our_structured_warehouse_map.pgm` and `.yaml` to the `maps/` directory.*
