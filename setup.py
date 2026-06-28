from setuptools import find_packages, setup
import os
from glob import glob

package_name = "bb8_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob(os.path.join("behavior_trees", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "description"),
            glob(os.path.join("description", "*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="banoffee",
    maintainer_email="pedrolunkesvillela@usp.br",
    description="Exploração autônoma por fronteiras (SLAM + Nav2 + m-explore) e coleta de flag",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "controle_robo = bb8_control.controle_robo:main",
            "vision_processor = bb8_control.vision_processor:main",
            "gripper_server = bb8_control.gripper_server:main",
            "odom_gt_publisher = bb8_control.odom_gt_publisher:main",
        ],
    },
)
