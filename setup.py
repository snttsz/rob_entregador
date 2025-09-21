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
        # Adiciona os arquivos launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Adiciona os arquivos de mundo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
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
            'entregador_node = rob_entregador.entregador_node:main',
        ],
    },
)