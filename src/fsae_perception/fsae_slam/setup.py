from setuptools import setup

package_name = 'fsae_slam'

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
    maintainer='dyu056',
    maintainer_email='daniel.yu@fsae.co.nz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
	entry_points={
		    'console_scripts': [
                    'kalman_filter = fsae_slam.cone_landmark_mapper:main',
		    ],
	},
)
