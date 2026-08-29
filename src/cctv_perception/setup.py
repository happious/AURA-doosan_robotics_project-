from setuptools import find_packages, setup

package_name = 'cctv_perception'

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
    description='CCTV 쓰러짐/혼잡/인원수 감지 노드',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cctv_detector_node = cctv_perception.cctv_detector_node:main',
        ],
    },
)
