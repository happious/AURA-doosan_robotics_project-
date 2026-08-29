from setuptools import find_packages, setup

package_name = 'fleet_central'

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
    description='AURA 중앙 계층 노드 (dispatcher / db_manager / crowd_keepout)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_dispatcher_node = fleet_central.fleet_dispatcher_node:main',
            'db_manager_node = fleet_central.db_manager_node:main',
            'crowd_keepout_mask_node = fleet_central.crowd_keepout_mask_node:main',
        ],
    },
)
