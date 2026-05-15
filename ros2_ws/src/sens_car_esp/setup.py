from setuptools import setup

package_name = 'sens_car_esp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Serial bridge for ESP logs and commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'Sens_Car_ESP = sens_car_esp.serial_node:main',
        ],
    },
)
