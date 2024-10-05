from setuptools import setup
from glob import glob
import os

package_name = 'my_oakd_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'my_oakd_launch.Nodo_tracking',
        'my_oakd_launch.Nodo_transform',
        'my_oakd_launch.oakd_publisher',
        'my_oakd_launch.nav_to_target',
        'my_oakd_launch.goal_pose_manual',
        'my_oakd_launch.goal_pose_flag',
        'my_oakd_launch.coke_tracker'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Nodo_tracking = my_oakd_launch.Nodo_tracking:main',
            'Nodo_transform = my_oakd_launch.Nodo_transform:main',
            'oakd_publisher = my_oakd_launch.oakd_publisher:main',
            'nav_to_target = my_oakd_launch.nav_to_target:main',
            'goal_pose_manual = my_oakd_launch.goal_pose_manual:main',
            'goal_pose_flag = my_oakd_launch.goal_pose_flag:main',
            'coke_tracker = my_oakd_launch.coke_tracker:main',
        ],
    },
)
