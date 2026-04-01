"""
Generate a urdf robot description from sdf for the iris with arm.

This script used the ros2 package sdf_to_urdf to generate a urdf file from
a sdf robot description.

Usage

python ./generate_urdf.py

Notes

Like `sdformat_urdf` which it uses, `sdf_to_urdf` resolves dependent packages
using the ROS 2 style `package://` prefix rather than Gazebo's `model://`.

The output file is named `model.urdf`. This file should not be edited.

"""

import os
import subprocess
import tempfile

from lxml import etree
from pathlib import Path


def generate_urdf_model():
    model_sdf_file = Path("model.sdf")
    model_urdf_file = Path("model.urdf")

    # TODO: remove when environment hooks are available in upstream packages
    # Ensure `SDF_PATH` is populated, Required by `sdformat_urdf`
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path

    # open orignal sdf model
    with open(model_sdf_file) as file:
        model_sdf = file.read()

    # replace model:// prefix with package location
    replace_str = f"model://iris_with_standoffs"
    with_str = f"package://ardupilot_gazebo/models/iris_with_standoffs"
    model_sdf = model_sdf.replace(replace_str, with_str)

    replace_str = f"model://so_arm_100"
    with_str = f"package://ardupilot_gz_description/models/so_arm_100"
    model_sdf = model_sdf.replace(replace_str, with_str)

    # write to temporary file needed by sdf_to_urdf
    temp_sdf_file = tempfile.NamedTemporaryFile(delete=False, suffix=".sdf")
    temp_sdf_file_name = temp_sdf_file.name

    with open(temp_sdf_file_name, "w") as file:
        file.write(model_sdf)

    # convert from sdf to urdf, output is to `model_urdf_file`
    with subprocess.Popen(
        [
            "ros2",
            "run",
            "sdf_to_urdf",
            "sdf_to_urdf",
            f"{temp_sdf_file_name}",
            f"{model_urdf_file}",
        ],
        stdout=subprocess.PIPE,
    ) as proc:
        print(proc.stdout.read().decode())


def main():
    generate_urdf_model()


if __name__ == "__main__":
    main()
