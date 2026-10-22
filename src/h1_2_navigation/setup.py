from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'h1_2_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml'),
        ),
        (
            os.path.join('share', package_name, 'policy'),
            glob('policy/*'),
        ),
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu',
    maintainer_email='liu@todo.todo',
    description='Mid-360 localization and planning for the H1-2 Gazebo model.',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mid360_square_localizer = '
            'h1_2_navigation.mid360_square_localizer:main',
            'nav2_velocity_gate = '
            'h1_2_navigation.nav2_velocity_gate:main',
            'gazebo_upright_stabilizer = '
            'h1_2_navigation.gazebo_upright_stabilizer:main',
            'v12_asset_audit = '
            'h1_2_navigation.v12_asset_audit:main',
            'v12_gazebo_controller = '
            'h1_2_navigation.v12_gazebo_controller:main',
            'unitree_h12_velocity_gate = '
            'h1_2_navigation.unitree_h12_velocity_gate:main',
            'unitree_h12_gazebo_controller = '
            'h1_2_navigation.unitree_h12_gazebo_controller:main',
            'incubator_auto_navigator = '
            'h1_2_navigation.incubator_auto_navigator:main',
        ],
    },
)
