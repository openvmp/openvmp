from setuptools import setup

package_name = "openvmp_motion_control_py"

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
    description="Motion control scripts and libraries for Python",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "stand = openvmp_motion_control_py.stand:main",
            "stand2 = openvmp_motion_control_py.stand2:main",
            "hug = openvmp_motion_control_py.hug:main",
            "walk = openvmp_motion_control_py.walk:main",
            "step = openvmp_motion_control_py.step:main",
        ],
    },
)
