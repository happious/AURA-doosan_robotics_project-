from setuptools import find_packages, setup

package_name = 'robot1_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot_control_team',
    maintainer_email='team@example.com',
    description='robot1 응급/AED 대응 미션 FSM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot1_mission_fsm_node = robot1_control.robot1_mission_fsm_node:main',
        ],
    },
)
