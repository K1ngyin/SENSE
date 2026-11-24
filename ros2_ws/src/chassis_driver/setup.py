from glob import glob
import os

from setuptools import setup


package_name = "chassis_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SENSE project",
    maintainer_email="robot@example.com",
    description="Serial feedback and velocity driver for the STM32 mobile robot chassis.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "chassis_driver = chassis_driver.serial_driver_node:main",
        ],
    },
)
