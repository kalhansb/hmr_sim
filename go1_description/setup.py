from glob import glob
import os
from setuptools import setup

package_name = 'go1_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duarte Cruz',
    maintainer_email='duarte.cruz@isr.uc.pt',
    description='Simple ROS 2 package to launch robot_state_publisher from a URDF file.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
