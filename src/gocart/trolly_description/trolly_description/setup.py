import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'trolly_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tommy',
    maintainer_email='tommyduan2020@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = trolly_description.state_publisher:main'
        ],
    },
)
