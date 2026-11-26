from setuptools import setup

setup(
    name="core_robot",
    version="0.1.0",
    packages=["core_robot", "core_robot.fastening"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_robot"]),
        ("share/core_robot", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "robot_node = core_robot.robot_node:main",
            "fasten_test = core_robot.fastening.fasten_test:main",
            "slot_view = core_robot.fastening.slot_view:main",
        ],
    },
)
