import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ros_audio_io"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pib",
    maintainer_email="pib@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "audio_streamer = ros_audio_io.audio_streamer:main",
            "doa_publisher = ros_audio_io.doa_publisher:main",
            "doa_listener = ros_audio_io.doa_listener:main",            
        ],
    },
)
