import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pib',
    maintainer_email='team@pib.rocks',
    description='Recieving msgs from cerebra via rosbridge and writing to pib motors using Tinkerforge APIs',
    license='Agpl3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'motor_control = motors.motor_control:main',
                'motor_current = motors.motor_current:main'
        ],
    },
)
