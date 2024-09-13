from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'imav24'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dronkab',
    maintainer_email='perrusquia832@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            "px4_driver = imav24.px4_driver:main",
            "start_msg_node = imav24.start_msg_node:main",
            "line_follower = imav24.line_follower:main",
            "camera_pub = imav24.camera_pub:main",
            "aruco_control = imav24.aruco_control:main",
            "joystick_control = imav24.joystick_control:main",
            "state1 = imav24.state1:main",
            "state2 = imav24.state2:main",
            "indoor_smach = imav24.indoor_smach:main",
            "send_command = imav24.send_command:main",
            "go_to_waypoint = imav24.go_to_waypoint:main",
            "SetPhoto = imav24.SetPhoto:main"
        ],
    },
)
