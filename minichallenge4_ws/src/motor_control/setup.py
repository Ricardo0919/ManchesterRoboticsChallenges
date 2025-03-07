from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),  # Archivos de lanzamiento (*.launch.py, *.launch.xml)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),  # Archivos de configuración (*.yaml, *.yml)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ricardosierra',
    maintainer_email='rickisierra03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Input = motor_control.Input:main',  # Script para el generador de puntos de ajuste
        ],
    },
)
