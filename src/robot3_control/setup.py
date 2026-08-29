from setuptools import find_packages, setup

package_name = 'robot3_control'

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
    description='robot3 가이드/러기지 어시스트 로컬 자율 스택',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'guide_rear_tracker_node = robot3_control.guide_rear_tracker_node:main',
            'guide_motion_node = robot3_control.guide_motion_node:main',
            'luggage_rgbd_tracker_node = robot3_control.luggage_rgbd_tracker_node:main',
            'luggage_follower_node = robot3_control.luggage_follower_node:main',
            'robot3_patrol_node = robot3_control.robot3_patrol_node:main',
            'robot3_mission_fsm_node = robot3_control.robot3_mission_fsm_node:main',
        ],
    },
)
