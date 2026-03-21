from setuptools import setup
from glob import glob
import os

package_name = 'gocart_control'

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
    maintainer_email='chrisgraham908@gmail.com',
    description='Go-kart control nodes: Ackermann-to-CAN, system status, trajectory following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ack_to_can_node = gocart_control.ack_to_can:main',
            'as_status_node = gocart_control.sys_status:main',
            'trajectory_follower = gocart_control.trajectory_follower_p_controller:main',
            'joystick_teleop = gocart_control.joystick_teleop:main',
            'mock_stimulus = gocart_control.mock_stimulus:main',
        ],
    },
)
