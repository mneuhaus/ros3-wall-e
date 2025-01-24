from setuptools import setup
import os
import subprocess
from setuptools.command.install import install

package_name = 'servo_2040'

class CustomInstallCommand(install):
    """Custom install command to build and flash firmware during colcon build."""
    def run(self):
        # Run the standard install process
        install.run(self)

        # Build the C firmware
        print("Building C firmware...")
        firmware_dir = os.path.join(os.path.dirname(__file__), 'firmware')
        build_dir = os.path.join(firmware_dir, 'build')
        flash_script = os.path.join(os.path.dirname(__file__), 'scripts/flash_firmware.py')
        
        try:
            # Create build directory
            os.makedirs(build_dir, exist_ok=True)
            
            # Run CMake
            subprocess.run(['cmake', '..'], 
                         cwd=build_dir, 
                         check=True)
            
            # Run Make
            subprocess.run(['make'], 
                         cwd=build_dir, 
                         check=True)
            
            print("C firmware built successfully!")
            
            # Flash the firmware
            print("\nAttempting to flash firmware...")
            subprocess.run(['python3', flash_script], check=True)
            
        except subprocess.CalledProcessError as e:
            print(f"Build/flash process failed: {e}")
            exit(1)
        except Exception as e:
            print(f"Error during build/flash: {e}")
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
        ('share/' + package_name + '/firmware', ['firmware/main.c', 'firmware/CMakeLists.txt']),
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
