from setuptools import setup
package_name = 'bno055_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='ROS2 node for BNO055 on Raspberry Pi 5',
    license='Apache-2.0',
    entry_points={'console_scripts': ['BNO = bno055_ros.bno_node:main']},
)
