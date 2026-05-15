from setuptools import setup

package_name = 'control_car'

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
    description='Nodo de control que suscribe sensores y publica comandos.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control = control_car.control_node:main',
        ],
    },
)

