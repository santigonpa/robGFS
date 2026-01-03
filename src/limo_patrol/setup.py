from setuptools import find_packages, setup
from glob import glob
import os

package_name = "limo_patrol"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/limo_patrol"]),
        ("share/limo_patrol", ["package.xml"]),
        (os.path.join("share", "limo_patrol", "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="Limo patrol package with waypoint generation and obstacle avoidance",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "patrol_node = limo_patrol.patrol_node:main",
            "obstacle_spawner = limo_patrol.obstacle_spawner:main",
            "waypoint_generator = limo_patrol.waypoint_generator:main",
            "waypoint_navigator = limo_patrol.waypoint_navigator:main",
            "potential_field_navigator = limo_patrol.potential_field_navigator:main",
        ],
    },
)
