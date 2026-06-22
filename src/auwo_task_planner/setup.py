from setuptools import find_packages, setup

package_name = 'auwo_task_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nameless',
    maintainer_email='taz.920722@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'excavation_server = auwo_task_planner.excavation_server:main',
            'sim_target_bridge = auwo_task_planner.sim_target_bridge:main',
            'grasp_node = auwo_task_planner.grasp_node:main',
            'pick_place_server = auwo_task_planner.pick_place_server:main'
        ],
    },
)
