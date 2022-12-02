import sys
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
    def __init__(self, cfg: config.Config, name="hardware_manager"):
        super().__init__(name)
        self.declare_parameter("use_fake_hardware", False)
        self.use_fake_hardware = self.get_parameter("use_fake_hardware").value

        # Buses
        buses = cfg.get_buses()
        for bus in buses:
            self.driver_instantiate("bus", bus)

        # Joints
        joints = cfg.get_joints()
        for joint in joints:
            if "brake" in joint:
                self.driver_instantiate("brake", joint["brake"])
            if "encoder" in joint:
                self.driver_instantiate("encoder", joint["encoder"])
            if "actuator" in joint:
                self.driver_instantiate("actuator", joint["actuator"])

        # while True:
        #     # TODO(clairbee): see if all processes are alive
        #     #                 and restart if needed
        #     time.sleep(1)

    def driver_instantiate(self, driver_class, obj):
        print("Launching a driver for")
        print(obj)

        params = []
        if self.use_fake_hardware or not "driver" in obj:
            driver = "fake"
            if not "fake" in drivers_map[driver_class]:
                print("No need to instantiate a fake driver of this class")
                return
        else:
            driver_config = obj["driver"]
            driver = driver_config["type"]
            for param in driver_config:
                if param != "type" and param != "namespace":
                    params.append(param + ":=" + driver_config[param])
        driver_pkg = drivers_map[driver_class][driver][0]
        driver_exe = drivers_map[driver_class][driver][1]

        # TODO(clairbee): refactor the below to wrap each subprocess with a thread
        #                 to restart them in case of a crash
        subprocess.run(
            [
                "ros2",
                "run",
                driver_pkg,
                driver_exe,
            ]
            + params
            + [
                "--ros-args",
                "-p",
                "__ns:=" + obj["path"],
            ]
        )


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
