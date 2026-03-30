import os
from glob import glob
from setuptools import find_packages, setup

package_name = "rammp_prototype_bringup"
pkg_dir = os.path.dirname(os.path.abspath(__file__))


def local_glob(pattern):
    orig = os.getcwd()
    os.chdir(pkg_dir)
    result = glob(pattern)
    os.chdir(orig)
    return result


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), local_glob("launch/*.py")),
        (
            os.path.join("share", package_name, "environment"),
            local_glob("env_hooks/*.dsv"),
        ),
        (
            "share/ament_index/resource_index/packages_with_environment_hooks",
            ["resource/" + package_name],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="Bringup launch files for the RAMMP prototype robot",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": []},
)
