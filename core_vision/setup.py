from setuptools import setup

setup(
    name="core_vision",
    version="0.1.0",
    packages=["core_vision"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_vision"]),
        ("share/core_vision", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ['vision_node = core_vision.vision_node:main', 'vision1_node = core_vision.vision1_node:main', 'vision1_calibrate = core_vision.vision1_calibrate:main']},
)
