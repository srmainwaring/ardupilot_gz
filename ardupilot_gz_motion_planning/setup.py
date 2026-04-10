import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ardupilot_gz_motion_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.parm")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rhys Mainwaring",
    maintainer_email="rhys.mainwaring@me.com",
    description="Motion planning for the Iris with 6DoF Arm",
    license="GPLv3",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "motion_planning = ardupilot_gz_motion_planning.motion_planning:main",
            "move_to_goal = ardupilot_gz_motion_planning.move_to_goal:main",
        ],
    },
)
