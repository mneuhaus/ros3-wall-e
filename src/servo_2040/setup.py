from setuptools import setup
import os
import subprocess
from setuptools.command.install import install

package_name = 'servo_2040'

class CustomInstallCommand(install):
    """Custom install command to upload firmware during colcon build."""
    def run(self):
        # Run the standard install process
        install.run(self)

        # Run the custom post-build step to upload firmware
        print("Running custom post-build step: Uploading firmware...")
        firmware_uploader = os.path.join(os.path.dirname(__file__), 'scripts/upload_main.py')
        try:
            subprocess.run(['python3', firmware_uploader], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Firmware upload failed: {e}")
            exit(1)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/all_launch.py']),
        ('share/' + package_name + '/firmware', ['firmware/main.py']),
    ],
    install_requires=['setuptools', 'pyserial', 'adafruit-ampy'],
    zip_safe=True,
    maintainer='mneuhaus',
    maintainer_email='marc@neuhaus.nrw',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_2040 = servo_2040.servo_2040:main',
        ],
    },
    cmdclass={
        'install': CustomInstallCommand,  # Add the custom install command
    },
)
