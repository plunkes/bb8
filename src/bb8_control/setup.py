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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="banoffee",
    maintainer_email="pedrolunkesvillela@usp.br",
    description="Controle autônomo – Wall Follower FSM + detecção visual de bandeira",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "controle_robo = bb8_control.controle_robo:main",
            "vision_processor = bb8_control.vision_processor:main",
            "gripper_server = bb8_control.gripper_server:main",
            "scan_masker = bb8_control.scan_masker:main",
        ],
    },
)
