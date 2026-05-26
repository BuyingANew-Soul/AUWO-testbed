from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auwo_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nameless',
    maintainer_email='taz.920722@gmail.com',
    description='Perception nodes for AUWO testbed',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_tracker = auwo_perception.object_tracker:main',
            'image_relay    = auwo_perception.image_relay:main',
        ],
    },
)