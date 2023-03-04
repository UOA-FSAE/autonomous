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
    description='Receives stamped ackermann msg from a topic and converts \
        it to CAN msg for motec to read and publishes CAN msg to service',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ack_to_can_node = moa_controllers.ackermann_to_motec:main'
        ],
    },
)
