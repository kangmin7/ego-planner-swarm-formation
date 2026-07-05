from setuptools import find_packages, setup

package_name = 'px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kangmin7',
    maintainer_email='kangmin7@gmail.com',
    description='PX4 SITL <-> ego_planner bridge: odometry/point-cloud in, offboard setpoints out.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_cloud_bridge = px4_bridge.odom_cloud_bridge:main',
            'offboard_bridge = px4_bridge.offboard_bridge:main',
            'truth_odom_bridge = px4_bridge.truth_odom_bridge:main',
        ],
    },
)
