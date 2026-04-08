from setuptools import find_packages, setup
from glob import glob
import os

package_name = "rammp_prototype_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "control_node = rammp_prototype_driver.MEBot_control_node:main",
            "control_node_test = rammp_prototype_driver.MEBot_control_node_test:main",
            "luci_node = rammp_prototype_driver.luci_heartbeat:main",
        ],
    },
)
