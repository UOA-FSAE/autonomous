from setuptools import setup
from glob import glob
import os

package_name = 'fsae_lidar_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py*'))),
        # parameter files
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FSAE Autonomous',
    maintainer_email='nithukrishnaa034@gmail.com',
    description='Camera-LiDAR sensor fusion: refines camera cone detections using the Velodyne LiDAR.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_node = fsae_lidar_fusion.fusion_node:main',
        ],
    },
)
