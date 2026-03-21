from glob import glob

from setuptools import find_packages, setup

package_name = "rammp_prototype_behavior"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
    ],
    install_requires=["setuptools", "transitions"],
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
            "test_nn_monitor = rammp_prototype_behavior.test_nn_monitor:main",  # just for testing
            "test_node1 = rammp_prototype_behavior.test_node1:main",  # just for testing
            "test_node2 = rammp_prototype_behavior.test_node2:main",  # just for testing
            "system_control = rammp_prototype_behavior.SystemControl:main",
            "mock_arm_driver = rammp_prototype_behavior.mocks.mock_arm_driver:main",
        ],
    },
)
