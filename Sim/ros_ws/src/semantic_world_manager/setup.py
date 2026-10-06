from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "semantic_world_manager"
runtime_requirements = (
    Path(__file__).resolve().with_name("requirements.txt").read_text(encoding="utf-8").splitlines()
)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"] + runtime_requirements,
    zip_safe=True,
    maintainer="MohamedAbubakr22",
    maintainer_email="mohamedabubakr450@gmail.com",
    description="Gazebo semantic world-state cache and compact HTTP query API.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "world_state_node = semantic_world_manager.world_state_node:main",
        ],
    },
)
