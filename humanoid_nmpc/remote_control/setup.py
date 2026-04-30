import os
from glob import glob
from setuptools import find_packages, setup

package_name = "remote_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Manuel Yves Galliker",
    maintainer_email="manuel.galliker@gmx.ch",
    description="Ros2 remote control interfaces",
    license="BSD-3",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_velocity_controller_gui = remote_control.base_velocity_controller_gui:main",
        ],
    },
)
