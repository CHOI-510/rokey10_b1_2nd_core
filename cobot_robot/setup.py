from setuptools import find_packages, setup

package_name = 'cobot_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='Cobot2 core test: screw fastening (no vision)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'fasten_test = cobot_robot.fasten_test:main',
            'slot_view = cobot_robot.slot_view:main',
        ],
    },
)
