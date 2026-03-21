from setuptools import setup
from glob import glob
import os

package_name = 'gocart_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chrisgraham@gmail.com',
    description='Hardware drivers for the go-kart: CAN decoder',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'can_decoder_jnano = gocart_driver.can_decoder_jnano:main',
        ]
    },
)
