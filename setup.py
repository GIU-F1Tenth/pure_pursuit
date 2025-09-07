import os
from glob import glob
from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='1.1.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include path files
        (os.path.join('share', package_name, 'path'), glob('path/*.csv')),
        # Include license
        (os.path.join('share', package_name), ['LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata, George Halim, Karim Shousha',
    maintainer_email='fam@awadlouis.com, georgehany064@gmail.com',
    description='Pure Pursuit path following controller for F1TENTH autonomous racing',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main'
        ],
    },
)
