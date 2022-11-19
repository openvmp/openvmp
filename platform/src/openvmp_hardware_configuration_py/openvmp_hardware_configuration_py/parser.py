import yaml
import sys

from openvmp_hardware_configuration_py.config import Config


def parse(filename):
    with open(filename, "r") as stream:
        try:
            parsed = yaml.safe_load(stream)
            return Config(parsed)
        except yaml.YAMLError as exc:
            print(exc)
    return {}


def main():
    if len(sys.argv) < 2:
        print("Syntax: openvmp_hardware_manager <harware-config-file>")
        return 1

    # TODO(clairbee): perform sanity check of the configuration file
    return 0


if __name__ == "__main__":
    main()
