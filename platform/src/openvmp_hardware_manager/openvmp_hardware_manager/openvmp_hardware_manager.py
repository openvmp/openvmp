import sys
import os
import time
import subprocess
import importlib
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from openvmp_hardware_configuration_py import parser
from openvmp_hardware_configuration_py import config

drivers_map = {
    "bus": {
        "modbus_rtu": [
            "modbus_rtu",
            "modbus_rtu_standalone",
            "--param",
            "modbus_prefix:=$PATH",
        ],
        "serial_bus": [
            "serial_bus",
            "serial_bus_standalone",
            "--param",
            "serial_bus_prefix:=$PATH",
        ],
    },
    "camera": {
        # "fake": [
        #     "usb_camera_driver",
        #     "usb_camera_driver_node",
        #     "--param",
        #     "fps:=25.0",
        #     "--param",
        #     "camera_calibration_file:=package://openvmp_robot/config/fake_video.ini",
        # ],
        # "usb": [
        #     "usb_camera_driver",
        #     "usb_camera_driver_node",
        # ],
    },
    "brake": {
        "fake": ["brake", "fake", "--param", "brake_prefix:=$PATH"],
        "switch": [
            "brake_switch",
            "brake_switch_standalone",
            "--param",
            "brake_prefix:=$PATH",
        ],
    },
    "encoder": {
        # "fake": ["encoder", "fake"],
        "amt21": ["encoder_amt21", "encoder_amt21_standalone"],
    },
    "actuator": {
        # "puldir": ["stepper_driver_puldir", "stepper_driver_puldir_standalone"],
        "em2rs": [
            "stepper_driver_em2rs",
            "stepper_driver_em2rs_standalone",
            "--param",
            "stepper_prefix:=$PATH",
        ],
    },
}


class HardwareManagerNode(Node):
    processes = {}

    def __init__(self, cfg: config.Config, name="hardware_manager"):
        super().__init__(name)
        self.declare_parameter("use_fake_hardware", False)
        self.use_fake_hardware = self.get_parameter("use_fake_hardware").value

        # Buses
        buses = cfg.get_buses()
        index = 0
        for bus in buses:
            self.driver_instantiate("bus" + str(index), "bus", bus)
            index = index + 1

        # Cameras
        cameras = cfg.get_cameras()
        index = 0
        for camera in cameras:
            self.driver_instantiate("camera" + str(index), "camera", camera)
            index = index + 1

        # Joints
        joints = cfg.get_joints()
        for joint in joints:
            if "brake" in joint:
                self.driver_instantiate(
                    "joint_" + joint["name"] + "_brake", "brake", joint["brake"]
                )
            if "encoder" in joint:
                self.driver_instantiate(
                    "joint_" + joint["name"] + "_encoder", "encoder", joint["encoder"]
                )
            if "actuator" in joint:
                self.driver_instantiate(
                    "joint_" + joint["name"] + "_actuator",
                    "actuator",
                    joint["actuator"],
                )

        # try:
        #     while len(self.processes) > 0:
        #         # TODO(clairbee): see if all processes are alive
        #         #                 and restart if needed
        #         time.sleep(1)
        # except KeyboardInterrupt:
        #     for process in self.processes:
        #         process["proc"].terminate()

    def driver_instantiate(self, id, driver_class, obj):
        name = id
        if "name" in obj:
            name = obj["name"]
        path = "/" + name
        if "path" in obj:
            path = obj["path"]
        path = path.replace("$DRIVER_NAME", name)

        self.get_logger().info("Launching a driver for {}".format(obj))

        # Determine ROS2 node launch parameters
        params = []
        init = []
        if self.use_fake_hardware or not "driver" in obj:
            driver = "fake"
            if not "fake" in drivers_map[driver_class]:
                self.get_logger().error(
                    "No need to instantiate a fake driver of {} class".format(
                        driver_class
                    )
                )
                return
        else:
            driver_config = obj["driver"]
            driver = driver_config["type"]
            if "init" in driver_config:
                init = driver_config["init"]

        if "driver" in obj:
            driver_config = obj["driver"]
            for param in driver_config:
                if param != "type" and param != "init":
                    value = str(driver_config[param])
                    params.append("--param")
                    params.append(param + ":=" + value)
        driver_pkg = drivers_map[driver_class][driver][0]
        driver_exe = drivers_map[driver_class][driver][1]
        for extra_param in drivers_map[driver_class][driver][2:]:
            params.append(extra_param)

        params_resolved = []
        for param in params:
            param = param.replace("$NAMESPACE", self.get_namespace())
            param = param.replace("$DRIVER_NAME", name)
            param = param.replace("$PATH", path)
            params_resolved.append(param)

        # Launch the process
        node_name = "driver_" + name
        namespace = self.get_namespace()

        cmd = (
            [
                "ros2",
                "run",
                driver_pkg,
                driver_exe,
            ]
            + [
                "--ros-args",
                "-r",
                "__node:=" + node_name,
                "-r",
                "__ns:=" + namespace,
            ]
            + params_resolved
        )
        self.get_logger().info("Executing the command: {}".format(cmd))
        proc = subprocess.Popen(cmd)
        self.processes[id] = {}
        self.processes[id]["id"] = id
        self.processes[id]["driver_class"] = driver_class
        self.processes[id]["obj"] = obj
        self.processes[id]["process"] = proc

        for step in init:
            service = step["service"]
            service_type = step["type"]

            init_path = namespace + path + service
            self.get_logger().info("Initializing: {}".format(init_path))

            parts = service_type.split("/")
            if len(parts) == 2:
                parts = [parts[0], "srv", parts[1]]
            module = importlib.import_module(".".join(parts[:-1]))
            srv_name = parts[-1]
            srv_module = getattr(module, srv_name)
            req = srv_module.Request()
            for key in step:
                if key == "service" or key == "type":
                    continue
                setattr(req, key, step[key])

            client = self.create_client(srv_module, init_path)
            client.wait_for_service()
            f = client.call_async(req)
            rclpy.spin_until_future_complete(self, f)
            client.destroy()


def main(args=None):
    if len(sys.argv) < 2:
        print("Syntax: openvmp_hardware_manager <harware-config-file>")
        return

    rclpy.init(args=sys.argv)

    cfg = parser.parse(sys.argv[1])
    hardware_manager = HardwareManagerNode(cfg)

    rclpy.spin(hardware_manager)
    hardware_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
