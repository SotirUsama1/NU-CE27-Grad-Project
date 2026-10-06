#!/usr/bin/env python3
"""Audit the built ROS/Gazebo environment without starting a simulation.

Run after sourcing Humble, /opt/ros2_robots/install/setup.bash, and the
project's install/setup.bash. Report all failures, then exit nonzero.
"""

import importlib
from importlib.metadata import version
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PYTHON_MODULES = {
    "fastapi": "fastapi",
    "pydantic": "pydantic",
    "uvicorn": "uvicorn",
    "httpx": "httpx",
    "PIL": "Pillow",
    "yaml": "PyYAML",
    "pytest": "pytest",
    "numpy": "numpy",
    "transforms3d": "transforms3d",
    "trimesh": "trimesh",
    "collada": "pycollada",
    "rclpy": None,
    "ament_index_python.packages": None,
    "launch": None,
    "launch_ros": None,
    "gazebo_msgs.msg": None,
    "tf_transformations": None,
    "cv_bridge": None,
    "semantic_world_manager.world_state_node": None,
}
GAZEBO_PLUGINS = {
    "gazebo_ros": ["libgazebo_ros_init.so", "libgazebo_ros_factory.so"],
    "gazebo_plugins": ["libgazebo_ros_ray_sensor.so"],
    "gazebo_ros2_control": ["libgazebo_ros2_control.so"],
}


def main():
    failures = []

    def check(label, operation):
        try:
            detail = operation()
        except Exception as exc:
            failures.append(label)
            print(f"FAIL {label}: {exc}", flush=True)
        else:
            print(f"OK   {label}" + (f": {detail}" if detail else ""), flush=True)

    def check_import(module, distribution):
        importlib.import_module(module)
        return version(distribution) if distribution else None

    for module, distribution in PYTHON_MODULES.items():
        check(module, lambda m=module, d=distribution: check_import(m, d))

    def check_package(package):
        from ament_index_python.packages import get_package_prefix
        return get_package_prefix(package)

    packages = set()
    for manifest in sorted((PROJECT_ROOT / "Sim/ros_ws/src").glob("*/package.xml")):
        root = ET.parse(manifest).getroot()
        packages.add(root.findtext("name"))
        for tag in ("depend", "exec_depend", "buildtool_depend"):
            packages.update(element.text.strip() for element in root.findall(tag))
    # System Python packages are checked by rosdep and pip check.
    for package in sorted(packages):
        if not package.startswith("python3-"):
            check(f"ROS package {package}", lambda p=package: check_package(p))

    def check_command(command):
        path = shutil.which(command)
        if path is None:
            raise RuntimeError("executable is missing from PATH")
        return path

    for command in ("ros2", "colcon", "xacro", "gzserver", "gzclient"):
        check(command, lambda c=command: check_command(c))

    def check_library(path):
        if not path.is_file():
            raise FileNotFoundError(path)
        result = subprocess.run(
            ["ldd", str(path)], capture_output=True, text=True, timeout=15
        )
        if result.returncode or "not found" in result.stdout:
            raise RuntimeError(result.stdout.strip() + "\n" + result.stderr.strip())
        return str(path)

    def check_plugin(package, filename):
        return check_library(Path(check_package(package)) / "lib" / filename)

    for package, filenames in GAZEBO_PLUGINS.items():
        for filename in filenames:
            check(filename, lambda p=package, f=filename: check_plugin(p, f))
    for library in sorted((PROJECT_ROOT / "Sim/models").glob("*/plugins/*.so")):
        check(str(library.relative_to(PROJECT_ROOT)), lambda p=library: check_library(p))

    def check_image_conversion():
        import numpy as np
        from cv_bridge import CvBridge
        bridge = CvBridge()
        pixels = np.zeros((2, 2, 3), dtype=np.uint8)
        message = bridge.cv2_to_imgmsg(pixels, encoding="bgr8")
        recovered = bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
        np.testing.assert_array_equal(recovered, pixels)

    check("cv_bridge image conversion (NumPy ABI)", check_image_conversion)
    print(f"\nDependency audit: {len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())