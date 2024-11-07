from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'contact_estimation_testbed'

# Collect all files under the 'urdf' directory, preserving subdirectory structure
urdf_xml_files = []
for dirpath, dirnames, filenames in os.walk('urdf'):
    for filename in filenames:
        file_path = os.path.join(dirpath, filename)
        # Calculate the target directory in 'share'
        install_dir = os.path.join('share', package_name, dirpath)
        urdf_xml_files.append((install_dir, [file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        *urdf_xml_files
    ],
    install_requires=['setuptools', 'rclpy', 'mujoco', 'numpy', 'glfw'],
    zip_safe=True,
    maintainer='dyuman',
    maintainer_email='dyuman2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'contact_estimation_testbed = contact_estimation_testbed.contact_estimation_testbed:main'
        ],
    },
)
