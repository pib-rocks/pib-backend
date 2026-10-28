from setuptools import find_packages, setup

package_name = "smart_home"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pib",
    maintainer_email="pib@todo.todo",
    description="ROS 2 bridge for pib SmartHome",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "smart_home_node = smart_home.smart_home_node:main",
        ],
    },
)
