from setuptools import setup

setup(
    name="core_common",
    version="0.1.0",
    packages=["core_common"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_common"]),
        ("share/core_common", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": []},
)
