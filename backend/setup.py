import os
from glob import glob
from setuptools import find_packages, setup


package_name = "backend"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="darp-note",
    maintainer_email="flaniel.arp@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "rosbridge_node = backend.rosbridge_node:main",
            "trigger_service = backend.trigger_service:main",
            "solenoid_manager = backend.solenoid_manager:main",
        ],
    },
)
