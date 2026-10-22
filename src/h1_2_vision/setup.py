from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'h1_2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu',
    maintainer_email='liu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cube_detector = h1_2_vision.cube_detector:main',
            'depth_point_cloud = h1_2_vision.depth_point_cloud:main',
            'environment_scene = h1_2_vision.environment_scene:main',
            'tag_localizer = h1_2_vision.tag_localizer:main',
            'dynamic_scene = h1_2_vision.dynamic_scene:main',
            'candidate3_replanner = h1_2_vision.candidate3_replanner:main',
            'candidate3_worker = h1_2_vision.candidate3_worker:main',
            'candidate3_middle_worker = '
            'h1_2_vision.candidate3_middle_worker:main',
            'candidate3_middle_capture = '
            'h1_2_vision.candidate3_middle_capture:main',
            'candidate3_middle_admission = '
            'h1_2_vision.candidate3_middle_admission:main',
            'candidate3_matrix = h1_2_vision.candidate3_matrix:main',
            'candidate3_base_residual_matrix = '
            'h1_2_vision.candidate3_base_residual_matrix:main',
            'candidate3_post_nav_open_worker = '
            'h1_2_vision.candidate3_post_nav_open_worker:main',
            'candidate3_post_nav_open_matrix = '
            'h1_2_vision.candidate3_post_nav_open_matrix:main',
            'candidate3_execution_supervisor = h1_2_vision.candidate3_execution_supervisor:main',
            'candidate3_q0_recovery = h1_2_vision.candidate3_q0_recovery:main',
            'candidate3_handle_recovery = h1_2_vision.candidate3_handle_recovery:main',
            'live_pose_dataset = h1_2_vision.live_pose_dataset:main',
        ],
    },
)
