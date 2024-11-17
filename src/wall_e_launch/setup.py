from setuptools import setup
import os
from glob import glob

package_name = 'wall_e_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        # Include the package.xml file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Launch files for Wall-E robot',
    license='YOUR_LICENSE',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No executables in this package
        ],
    },
)