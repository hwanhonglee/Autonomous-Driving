from glob import glob
from setuptools import find_packages, setup

# HH_260811 - Install the client with a publishable placeholder config; live secrets stay in ~/.config.

package_name = "pc3_ntrip_client"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        # HH_260811 - Install only the tracked placeholder; the external mode-0600 file is never copied.
        ("share/" + package_name + "/config", ["config/ntrip_config.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    # HH_260811 - Record the vehicle integration maintainer in distributable package metadata.
    maintainer="Hwanhong Lee",
    maintainer_email="hwanhong57@gmail.com",
    description="Secure NTRIP client and validated RTCM3 serial bridge for vehicle PC3.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ntrip_client_node = pc3_ntrip_client.node:main",
        ],
    },
)
