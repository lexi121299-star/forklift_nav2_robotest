from glob import glob

from setuptools import setup

package_name = 'forklift_vehicle_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pl',
    maintainer_email='user@example.com',
    description='Simulation and Curtis vehicle interface adapters for shared forklift commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_command_bridge = forklift_vehicle_interface.sim_command_bridge:main',
            'curtis_vehicle_interface = forklift_vehicle_interface.curtis_vehicle_interface:main',
        ],
    },
)
