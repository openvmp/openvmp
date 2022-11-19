from setuptools import setup

package_name = "openvmp_hardware_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Roman Kuzmenko",
    maintainer_email="openvmp@proton.me",
    description="Hardware Manager for OpenvMP robots",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "openvmp_hardware_manager = openvmp_hardware_manager.openvmp_hardware_manager:main"
        ],
    },
)
