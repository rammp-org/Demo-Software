import os
from glob import glob

from setuptools import find_packages, setup

package_name = "cornell_feeding"


def collect_package_data(package_dir):
    """
    Return all non-Python files under ``package_dir``, as paths relative to it.

    The vendored ``rammp`` package ships URDFs, meshes, scene-config YAMLs and
    perception models alongside its code. RAMMP resolves these relative to
    ``__file__`` at runtime, so they must be installed into the package tree.
    Paths are returned relative to the package directory (setuptools installs
    them under that package, preserving the subdirectory layout).
    """
    data = []
    for root, _dirs, files in os.walk(package_dir):
        for name in files:
            if name.endswith((".py", ".pyc", ".pyo")):
                continue
            data.append(os.path.relpath(os.path.join(root, name), package_dir))
    return data


setup(
    name=package_name,
    version="0.0.0",
    # Picks up both the cornell_feeding ROS2 package and the vendored, top-level
    # `rammp` package (empriselab/RAMMP) copied into this folder so the node is
    # fully self-contained and installable in the Jetson container. The RAMMP code
    # dirs that originally shipped without __init__.py have had one added so they are
    # discoverable here; rammp/assets stays data-only (shipped via package_data).
    packages=find_packages(exclude=["test", "test.*"]),
    package_data={"rammp": collect_package_data("rammp")},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Cornell drinking module: DrinkAction servers backed by feeding_deployment.",
    license="BSD-3-Clause",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "drink_action_server = cornell_feeding.drink_action_server:main",
        ],
    },
)
