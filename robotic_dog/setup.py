import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robotic_dog'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF e XACRO
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # PASTA MESHES (ESSA LINHA É O QUE FALTAVA!)
        (os.path.join('share', package_name, 'urdf/meshes'), glob('urdf/meshes/*')),
        # Configs (se tiver)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pato',
    maintainer_email='pato@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'ik_publisher = robotic_dog.ik_publisher_node:main',
            'robotic_dog_node = robotic_dog.robotic_dog_node:main',
        ],
    },
)
