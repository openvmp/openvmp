#!/usr/bin/python3

import os
import sys
from glob import glob
import shutil
import tempfile
import xacro

robot_dir_prefix = "src/openvmp_robot_"

if __name__ == "__main__":
    robot_names = []
    if len(sys.argv) > 1:
        robot_names.append(sys.argv[1])
    else:
        robot_dirs = glob(robot_dir_prefix + "*", recursive=False)
        for robot_dir in robot_dirs:
            robot_names.append(robot_dir[len(robot_dir_prefix) :])
    # print(robot_names)

    for robot_name in robot_names:
        path = os.getenv("HOME", ".") + "/.gazebo/models/openvmp_" + robot_name + "/"
        os.makedirs(path, exist_ok=True)

        config_dst = path + "model.config"
        config_src = robot_dir_prefix + robot_name + "/config/gazebo.config"
        shutil.copyfile(config_src, config_dst)

        model_tmp = tempfile.NamedTemporaryFile(delete=False)
        model_dst = path + "model.sdf"
        model_src = (
            robot_dir_prefix
            + robot_name
            + "/models/openvmp_robot_"
            + robot_name
            + ".urdf"
        )
        model_xacro = xacro.process_file(model_src)
        model_urdf = model_xacro.toprettyxml(indent="  ")
        model_tmp.write(model_urdf.encode())
        model_tmp_filename = model_tmp.name
        model_tmp.close()
        # print("copy from: ")
        # print(model_tmp_filename)
        # print("copy to: ")
        # print(model_dst)
        shutil.copyfile(model_tmp_filename, model_dst)
