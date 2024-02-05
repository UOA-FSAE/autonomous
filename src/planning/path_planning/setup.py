from setuptools import setup

package_name = 'path_planning'

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
    maintainer='Tanish',
    maintainer_email='Tanish.Bhatt@fsae.co.nz',
    description='Package for path planning implementation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planning = path_planning.path_optimisation:main'
        ],
    },
)
