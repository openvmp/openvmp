import sys
import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from openvmp_hardware_configuration_py import parser
from openvmp_hardware_configuration_py import config

drivers_map = {
    "bus": {
        "modbus_rtu": ["modbus_rtu", "modbus_rtu_standalone"],
    },
    "brake": {
        "fake": ["brake", "fake"],
        "switch": ["brake_switch", "brake_switch_standalone"],
    },
    "encoder": {
        # "fake": ["encoder", "fake"],
        # "amt21": ["encoder_amt21", "encoder_amt32_standalone"],
    },
    "actuator": {
        # "puldir": ["stepper_driver_puldir", "stepper_driver_puldir_standalone"],
        "em2rs": ["stepper_driver_em2rs", "stepper_driver_em2rs_standalone"],
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

        # Joints
        joints = cfg.get_joints()
        for joint in joints:
            if "brake" in joint:
                self.driver_instantiate(
                    joint["name"] + "-brake", "brake", joint["brake"]
                )
            if "encoder" in joint:
                self.driver_instantiate(
                    joint["name"] + "-encoder", "encoder", joint["encoder"]
                )
            if "actuator" in joint:
                self.driver_instantiate(
                    joint["name"] + "-actuator", "actuator", joint["actuator"]
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
        self.get_logger().info("Launching a driver for {}".format(obj))

        # Determine ROS2 node launch parameters
        params = []
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
            for param in driver_config:
                if param != "type" and param != "namespace":
                    params.append(param + ":=" + str(driver_config[param]))
        driver_pkg = drivers_map[driver_class][driver][0]
        driver_exe = drivers_map[driver_class][driver][1]

        # Determine
        path = obj["path"]
        node_name = os.path.basename(path)
        namespace = self.get_namespace() + path

        proc = subprocess.Popen(
            [
                "ros2",
                "run",
                driver_pkg,
                driver_exe,
            ]
            + params
            + [
                "--ros-args",
                "-r",
                "__node:=" + node_name,
                "-r",
                "__ns:=" + namespace,
            ]
        )
        self.processes[id] = {}
        self.processes[id]["id"] = id
        self.processes[id]["driver_class"] = driver_class
        self.processes[id]["obj"] = obj
        self.processes[id]["process"] = proc


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
