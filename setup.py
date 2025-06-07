from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (
            'share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]
        ),
        # Include ALL launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include ALL config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='georgehany064@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main'
        ],
    },
)
