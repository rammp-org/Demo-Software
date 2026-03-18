from setuptools import find_packages, setup

package_name = "cmu_door_opener"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["*.pt"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/cmu_door_opener.launch.py"]),
    ],
    install_requires=["setuptools", "pybullet", "numpy"],
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
            "button_detector = cmu_door_opener.button_detector:main",
            "button_push_controller = cmu_door_opener.button_push_controller:main",
        ],
    },
)
