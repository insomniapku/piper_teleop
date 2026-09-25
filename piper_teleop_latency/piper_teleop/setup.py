from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'piper_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='zktitan',
    maintainer_email='zktitan@todo.todo',
    description='Piper robot teleoperation using Pico XR controller',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'piper_teleop_node = piper_teleop.piper_teleop_node:main',
            'full_teleop_node = piper_teleop.full_teleop_node:main',
            'xr_interface_node = piper_teleop.xr_interface:main',
            'xr_test_node = piper_teleop.xr_test_node:main',
            'xr_monitor = piper_teleop.xr_monitor:main',
            'data_collection_node = piper_teleop.data_collection_node:main',
        ],
    },
)
