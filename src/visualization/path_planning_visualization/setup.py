from setuptools import find_packages, setup

package_name = 'path_planning_visualization'

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
    maintainer='root',
    maintainer_email='Tanish.Bhatt@fsae.co.nz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualize = path_planning_visualization.visualise_trajectories:main',
            'visualize2 = path_planning_visualization.visualise_trajectories_demo:main',
        ],
    },
)
