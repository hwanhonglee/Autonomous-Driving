from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

# HH_260811 - Install the failover nodes, launch files, parameters, and package documentation.

package_name = "autoware_gnss_failover_selector"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hwanhong Lee",
    maintainer_email="hwanhong57@gmail.com",
    # HH_260811 - Describe HEADING2 readiness separately from INS orientation output.
    description=(
        "Fail-closed GNSS selection, OEM7 HEADING2 readiness, and INS orientation."
    ),
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gnss_failover_selector = "
            "autoware_gnss_failover_selector.failover_node:main",
            "novatel_inspvax_orientation = "
            "autoware_gnss_failover_selector.novatel_orientation_node:main",
        ],
    },
)
