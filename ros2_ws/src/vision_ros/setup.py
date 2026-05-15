from setuptools import setup

package_name = 'vision_ros'

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
    description='Nodo ROS 2 que publica la clasificación de señales.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision = vision_ros.vision_node:main',
        ],
    },
)
