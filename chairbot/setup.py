from setuptools import find_packages, setup
import os
from glob import glob

package_name = "chairbot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@example.com",
    description="OAK-D Lite RGB-D SLAM with rtabmap_ros, visualized via Rerun.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rerun_bridge = chairbot.rerun_bridge:main",
        ],
    },
)
