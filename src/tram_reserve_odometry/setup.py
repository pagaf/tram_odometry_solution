from setuptools import setup
from glob import glob
import os

package_name = 'tram_reserve_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hackathon-team',
    maintainer_email='team@example.com',
    description='Robust nonlinear reserve odometry for tram using wheel speeds and driver controller input.',
    license='MIT',
    entry_points={'console_scripts': ['reserve_odometry = tram_reserve_odometry.node:main']},
)
