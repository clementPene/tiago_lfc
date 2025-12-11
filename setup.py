from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tiago_lfc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installer TOUS les launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Installer TOUS les config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpene',
    maintainer_email='cpene@todo.todo',
    description='TODO: Package description',
    license='Apache2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_loader = tiago_lfc.test_loader:main',
            'test_mpc_pure = tiago_lfc.test_mpc_pure:main',
            'mpc_node = tiago_lfc.mpc_node:main',
        ],
    },
)
