from setuptools import find_packages, setup

package_name = 'power'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'smbus2'
    ],
    zip_safe=True,
    maintainer='mneuhaus',
    maintainer_email='marc@neuhaus.nrw',
    description='Power monitoring package for Wall-E robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'power_monitor = power.power_monitor:main',
        ],
    },
)
