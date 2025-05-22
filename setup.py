from glob import glob

from setuptools import find_packages, setup

package_name = "ezrospy"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.py")),
        ("share/" + package_name + "/config/", glob("config/*")),
        ("share/" + package_name + "/scripts/", glob("scripts/*")),
        ("share/" + package_name + "/modules/", glob("modules/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Devson Butani",
    maintainer_email="dbutani@ltu.edu",
    description="Ezrospy package for scripting with ROS",
    license="MIT",
    entry_points={
        "console_scripts": [
            f"ezrospy_gui = {package_name}.gui:main",
        ],
    },
)
