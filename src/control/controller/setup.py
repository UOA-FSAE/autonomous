from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package metadata
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sivasriram16',
    maintainer_email='ssri357@aucklanduni.ac.nz',
    description='ROS 2 Controller package with multiple controllers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanley = controller.stanley_controller:main',
            'head_to_goal = controller.head_to_goal_controller.main',
            'pure_pursuit = controller.pure_pursuit_controller.main',
        ],
    },
)
