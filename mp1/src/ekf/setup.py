import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ekf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psi',
    maintainer_email='Pascal.Siekiera@gmail.com',
    description='Mini-Project 1: EKF, SLAM (slam_toolbox) and AMCL on a turtlebot3',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ekf_node = ekf.ekf_node:main',
            'ground_truth_node = ekf.ground_truth_node:main',
            'evaluator_node = ekf.evaluator_node:main',
            'compare_maps = ekf.compare_maps:main',
        ],
    },
)
