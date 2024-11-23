from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planning_visualiser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='Tanish.Bhatt@fsae.co.nz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualiser = path_planning_visualiser.visualise_trajectories:main',
            'visualiser2 = path_planning_visualiser.visualise_trajectories_demo:main',
        ],
    },
)
