import os
from glob import glob

from setuptools import find_packages, setup

package_name = "limo_rviz"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (
            os.path.join("share", package_name, "models", "turtlebot3_square"),
            [
                filepath
                for filepath in glob("urdf/turtlebot3_square/*", recursive=True)
                if os.path.isfile(filepath)
            ],
        ),
        (
            os.path.join("share", package_name, "models", "box1"),
            [
                filepath
                for filepath in glob("urdf/box1/*", recursive=True)
                if os.path.isfile(filepath)
            ],
        ),
        (
            os.path.join("share", package_name, "models", "limo_four_diff"),
            [
                filepath
                for filepath in glob("urdf/limo_four_diff/*", recursive=True)
                if os.path.isfile(filepath)
            ],
        ),
        (
            os.path.join("share", package_name, "models", "limo_four_diff", "meshes"),
            [
                filepath
                for filepath in glob("urdf/limo_four_diff/meshes/*", recursive=True)
                if os.path.isfile(filepath)
            ],
        ),
        (os.path.join("share", package_name, "world"), glob("world/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="shuvro.mahmud79@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
