from setuptools import setup

package_name = 'moa_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Keating',
    maintainer_email='adam.keating@fsae.co.nz',
    description='Contains all the controllers for the car that integrate hardware and software',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ack_to_can_node = moa_controllers.ack_to_can:main',
            'as_status_node = moa_controllers.sys_status:main',
            'trajectory_follower = moa_controllers.trajectory_follower_p_controller:main'
        ],
    },
)
