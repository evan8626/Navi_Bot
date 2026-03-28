from setuptools import setup
import os
from glob import glob

package_name = 'navi_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.sensors', f'{package_name}.control', f'{package_name}.utils'],
    data_files=[
        ('share/ament/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evan Osborn',
    maintainer_email='evan.a.osborn@gmail.com',
    description='Autonomous navigational robot with real-time constraints.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':[
            'robot_controller = navi_bot.robot_controller:main',
            'path_planner = navi_bot.path_planner:main',
            'state_machine = navi_bot.state_machine:main',
        ],
    },
)