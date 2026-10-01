# Custom_Robot_Warehouse

`Our_Structured_Warehouse` plus four custom robots:
- `picker_1`, `picker_2`: HaiPick-style case-handling pickers (lifting tray, telescopic side arms, 4 storage slots).
- `transport_1`, `transport_2`: roller-deck transport robots that unload onto the conveyor.

Topics and commands: `warehouse_robots.yaml`.

## Run

From the repository root:

```bash
./Sim/plugins/build_plugins.sh              # once, builds the robot and conveyor plugins
./Sim/gzScripts/runCustomRobotWarehouse.sh  # starts Gazebo with ROS 2
```

## Demo

In a second terminal, while the simulation runs:

```bash
./Sim/gzScripts/runCustomRobotDemo.sh
```

The picker takes a box from a shelf in the dock zone and places it on a transport robot. The transport robot then rolls it onto the conveyor, and the conveyor carries it into the truck. The demo can be run again.

## Regenerate

```bash
python3 Sim/tools/build_warehouse_robots.py   # robot models + this world (built from Our_Structured_Warehouse)
```
