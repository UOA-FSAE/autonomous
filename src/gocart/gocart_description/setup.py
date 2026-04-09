from setuptools import setup
import os
from glob import glob

package_name = "gocart_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="chris",
    maintainer_email="chrisgraham908@gmail.com",
    description="URDF model and meshes for the go-kart (gocart) platform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
