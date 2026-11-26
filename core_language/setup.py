from setuptools import setup

setup(
    name="core_language",
    version="0.1.0",
    packages=["core_language"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/core_language"]),
        ("share/core_language", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": ['voice_node = core_language.voice_node:main', 'llm_node = core_language.llm_node:main']},
)
