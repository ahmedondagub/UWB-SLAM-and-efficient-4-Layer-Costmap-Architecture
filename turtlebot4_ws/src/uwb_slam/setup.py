from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uwb_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'psutil'],
    zip_safe=True,
    maintainer='yakubiantechnologist',
    maintainer_email='yakubiantechnologist@todo.todo',
    description='UWB-integrated SLAM with 4-layer map architecture and ESKF sensor fusion for TurtleBot 4',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trilateration_node=uwb_slam.trilateration_node:main',
            'uwb_sim_node=uwb_slam.uwb_sim_node:main',
            'eskf_fusion_node=uwb_slam.eskf_fusion_node:main',
            'autonomous_motion_node=uwb_slam.autonomous_motion_node:main',
            'lidar_pose_adapter_node=uwb_slam.lidar_pose_adapter_node:main',
        ],
    },
)
