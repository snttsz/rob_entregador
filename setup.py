from setuptools import setup
import os
from glob import glob

package_name = 'rob_entregador'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@email.com',
    description='Pacote para o robô de entrega com Nav2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = rob_entregador.vision_node:main',
            'controller_node = rob_entregador.controller_node:main',
            'statistics_collector = rob_entregador.statistics_collector:main'
        ],
    },
)