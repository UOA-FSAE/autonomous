from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pure_pursuit_visualiser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                # include the launch directory
        (os.path.join('share', package_name, 'pure_pursuit_visualiser'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dyu056',
    maintainer_email='yudaniel888@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'visualiser = pure_pursuit_visualiser.visualise_pure_pursuit:main',
        ],
    },
)
