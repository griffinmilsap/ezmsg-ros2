from setuptools import find_packages, setup

package_name = 'ezmsg_ros2_examples'

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
    description='Examples for ezmsg-ros2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sub = ezmsg_ros2_examples.sub:main",
            "pub = ezmsg_ros2_examples.pub:main",
            "params = ezmsg_ros2_examples.params:main"
        ],
    },
)
