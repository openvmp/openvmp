import sys
import subprocess

from openvmp_hardware_configuration_py import parser


def main():
    if len(sys.argv) < 2:
        print("Syntax: openvmp_hardware_manager <harware-config-file>")
        return

    config = parser.parse(sys.argv[1])

    # TODO(clairbee): refactor the below to wrap each subprocess with a thread
    #                 to restart them in case of a crash

    # Modbus RTU
    modbus_rtu = config.get_modbus_rtu()
    for rtu in modbus_rtu:
        print("Launching a driver for")
        print(rtu)
        result = subprocess.run(
            [
                "ros2",
                "run",
                "modbus_rtu",
                "modbus_rtu_standalone",
                ...,
            ]
        )


if __name__ == "__main__":
    main()
