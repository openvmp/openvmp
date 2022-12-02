from setuptools import setup

package_name = "openvmp_hardware_configuration_py"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Roman Kuzmenko",
    maintainer_email="openvmp@proton.me",
    description="Python parser for OpenVMP hardware configuration files",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["parser = openvmp_hardware_configuration_py.parser:main"],
    },
)
