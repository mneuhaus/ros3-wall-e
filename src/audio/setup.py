import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'audio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/sounds',
         glob('resource/sounds/**/*.mp3', recursive=True))
    ],
    install_requires=['setuptools', 'pygame'],
    zip_safe=True,
    maintainer='mneuhaus',
    maintainer_email='marc@neuhaus.nrw',
    description='Audio playback package for Wall-E robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_node = audio.audio_node:main',
        ],
    },
)
