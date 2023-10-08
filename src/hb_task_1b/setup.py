import os 
from glob import glob
from setuptools import find_packages, setup

package_name = 'hb_task_1b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='furquan',
    maintainer_email='furquan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=[
        'scripts/controller.py',
        'scripts/service_node.py'
    ],
    entry_points={
        'console_scripts': [
            'robot_state_publisher=hb_task_1b.robot_state_publisher:main',
            'joint_state_publisher=hb_task_1b.joint_state_publisher:main',
            'spawn_entity=hb_task_1b.spawn_entity:main'
        ],
    },
)
