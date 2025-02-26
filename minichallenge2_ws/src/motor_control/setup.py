from setuptools import find_packages, setup
import os
from glob import glob

# Definición del nombre del paquete
package_name = 'motor_control'

setup(
    name=package_name,  # Nombre del paquete
    version='0.0.0',  # Versión del paquete
    packages=find_packages(exclude=['test']),  # Buscar todos los paquetes excepto el directorio 'test'
    
    data_files=[  # Archivos de datos que se incluyen en el paquete
        ('share/ament_index/resource_index/packages',  # Ruta donde se guardará el índice de recursos
            ['resource/' + package_name]),  # El archivo relacionado con el paquete
        ('share/' + package_name, ['package.xml']),  # Archivo de metadatos del paquete
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),  # Archivos de lanzamiento (*.launch.py, *.launch.xml)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),  # Archivos de configuración (*.yaml, *.yml)
    ],
    
    install_requires=['setuptools'],  # Paquete requerido para la instalación
    zip_safe=True,  # El paquete puede ser comprimido y almacenado de manera segura
    maintainer='ricardosierra',  # Nombre del mantenedor
    maintainer_email='rickisierra03@gmail.com',  # Correo electrónico del mantenedor
    description='This package generates a DC motor simulation, Set Point Generator and Controller nodes for simulating and controlling a DC Motor',  # Descripción del paquete
    license='TODO: License declaration',  # Declaración de licencia (por completar)
    tests_require=['pytest'],  # Requiere pytest para realizar pruebas
    entry_points={  # Puntos de entrada para ejecutar scripts desde la línea de comandos
        'console_scripts': [
            'dc_motor = motor_control.dc_motor:main',  # Script para la simulación del motor de corriente continua
            'set_point = motor_control.set_point:main',  # Script para el generador de puntos de ajuste
            'controller = motor_control.controller:main'  # Script para el controlador del motor
        ],
    },
)
