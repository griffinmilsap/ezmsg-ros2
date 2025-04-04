from setuptools import find_packages, setup

package_name = 'ezmsg_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Griffin Milsap',
    maintainer_email='griffin.milsap@jhuapl.edu',
    description='command-line topic tools for ezmsg and ros2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "to_ros = ezmsg_ros2.to_ros:main",
            # "from_ros = ezmsg_ros2.command:from_ros",
        ],
    },
)
