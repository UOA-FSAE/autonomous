from setuptools import find_packages, setup

package_name = 'steer_torque_from_ackermann'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Yu',
    maintainer_email='daniel.yu@fsae.co.nz',
    description='Controller node for simulation car to control its speed and steering angle',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'steer_torque_from_ackermann = steer_torque_from_ackermann.converter:main',
        ],
    },
)
