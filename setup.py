from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab08_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config YAML files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'matplotlib',
        'numpy',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='Eric Keetz',
    maintainer_email='eck27@psu.edu',
    description='Crazyflie Lab 08 - Real drone PID control, trajectory, and logging',
    license='MIT',
    entry_points={
        'console_scripts': [
            'goal_controller = lab08_control.goal_controller:main',
            'trajectory_publisher = lab08_control.trajectory_publisher:main',
            'plotter = lab08_control.plotter:main',
            'control_services = lab08_control.control_services:main',
        ],
    },
)
