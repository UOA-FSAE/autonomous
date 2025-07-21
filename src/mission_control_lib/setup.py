from setuptools import setup

package_name = 'mission_control_lib'

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
    maintainer='clob269',
    maintainer_email='chris.lobo@fsae.co.nz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
	entry_points={
		    'console_scripts': [
                    'mission_control = mission_control_lib.mission_control:main',
                    'dummy_node = mission_control_lib.dummy_node:main',
                    'dummy_publisher = mission_control_lib.dummy_publisher:main',
		    ],
	},
)
