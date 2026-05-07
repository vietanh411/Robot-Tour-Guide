from glob import glob
from setuptools import find_packages, setup

PACKAGE_NAME = 'robot_tour_guide'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Tour Guide Team',
    maintainer_email='claudevippro@gmail.com',
    description='Hybrid tour-guide stack for the OU TurtleBot 4.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector       = robot_tour_guide.aruco_detector:main',
            'semantic_perception  = robot_tour_guide.semantic_perception:main',
            'world_model          = robot_tour_guide.world_model:main',
            'tour_planner         = robot_tour_guide.tour_planner:main',
            'executive            = robot_tour_guide.executive:main',
            'narrator             = robot_tour_guide.narrator:main',
            'safety_monitor       = robot_tour_guide.safety_monitor:main',
            'landmark_announcer   = robot_tour_guide.landmark_announcer:main',
            'pattern_tour         = robot_tour_guide.pattern_tour:main',
            'measure_pattern      = robot_tour_guide.measure_pattern:main',
        ],
    },
)
