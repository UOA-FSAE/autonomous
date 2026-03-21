from setuptools import setup
from glob import glob
import os

package_name = 'path_planning'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tanish',
    maintainer_email='Tanish.Bhatt@fsae.co.nz',
    description='Package for path planning implementation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centerline_planner = path_planning.centerline_planner:main',
            'center_line = path_planning.simple_centerline_planner:main',
            'fasttube = path_planning.fasttube_planner:main',
            'fasttube_without_kalman = path_planning.fasttube_without_kalman:main'
        ],
    },
)
