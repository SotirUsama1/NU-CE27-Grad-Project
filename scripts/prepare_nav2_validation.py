#!/usr/bin/env python3
"""Create an instrumented warehouse world inside the project's ROS container.

Run after sourcing ROS and Gazebo, with no simulation running:
  python3 /main/scripts/prepare_nav2_validation.py --output /main/ros_ws/log/nav2-test
The production world and robot physics remain unchanged. This adds ground-truth
poses and a contact filter restricted to the Husky, for validate_nav2.py.
"""
import argparse
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--world', type=Path,
                        default=Path('/main/worlds/Our_Structured_Warehouse/Our_Structured_Warehouse.world'))
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    library = output / 'libnav2_contacts.so'
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gazebo'], text=True))
    subprocess.run(['g++', '-shared', '-fPIC', str(Path(__file__).with_name('nav2_contact_monitor.cpp')),
                    '-o', str(library), *flags, '-std=c++17'], check=True)
    tree = ET.parse(args.world)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('Expected an SDF file containing a world')
    plugin = ET.SubElement(world, 'plugin', name='nav2_validation_state', filename='libgazebo_ros_state.so')
    ET.SubElement(plugin, 'update_rate').text = '10.0'
    ET.SubElement(world, 'plugin', name='nav2_validation_contacts', filename=str(library))
    target = output / 'warehouse_test.world'
    tree.write(target, encoding='unicode')
    print(f'Prepared {target}')
    print(f'ros2 launch warehouse_husky husky.launch.py world_path:={target}')


if __name__ == '__main__':
    main()
