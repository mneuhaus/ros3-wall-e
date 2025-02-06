from setuptools import find_packages, setup
import os

package_name = 'wall_e_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', [os.path.join('web', f) for f in os.listdir('web') if os.path.isfile(os.path.join('web', f))]),  # Include only files, not directories
        ('share/' + package_name + '/launch', ['launch/rosbridge_launch.py']),
        ('share/' + package_name + '/launch', ['launch/all_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mneuhaus',
    maintainer_email='marc@neuhaus.nrw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serve_web = wall_e_web.serve_web:main',
        ],
    },
)
