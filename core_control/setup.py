from setuptools import setup

setup(
    name="core_control",
    version="0.1.0",
    packages=["core_control"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_control"]),
        ("share/core_control", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ['control = core_control.control:main', 'control_interface = core_control.control_interface:main', 'monitor_gui = core_control.monitor_gui:main']},
)
