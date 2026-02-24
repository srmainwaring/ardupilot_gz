from setuptools import find_packages, setup

package_name = 'ardupilot_gz_arm_moveit_gz_bridge'

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
    maintainer='Maintainer',
    maintainer_email='maintainer@email.com',
    description='A MoveIt to Gazebo bridge for the LeArm',
    license='GPL-3.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'moveit_to_gz_bridge = ardupilot_gz_arm_moveit_gz_bridge.moveit_to_gz_bridge:main',
        ],
    },
)
