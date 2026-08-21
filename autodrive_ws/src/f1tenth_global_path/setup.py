from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'f1tenth_global_path'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'paths'), glob('paths/*.csv')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan Gallo',
    maintainer_email='juangallo@example.com',
    description='Publica el mapa y la trayectoria global D* para F1TENTH.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'path_publisher = f1tenth_global_path.path_publisher:main',
        ],
    },
)
