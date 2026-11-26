from glob import glob
from setuptools import setup

setup(
    name="core_bringup",
    version="0.1.0",
    packages=["core_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_bringup"]),
        ("share/core_bringup", ["package.xml"]),
        ("share/core_bringup/launch", glob("launch/*.launch.py")),
        ("share/core_bringup/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": []},
)
