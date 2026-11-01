from setuptools import setup, find_packages

package_name = 'simulation_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Alipázaga Lazo',
    maintainer_email='alipazaga.daniel@gmail.com',
    description='Our own kinematic simulator: publishes pose and synthetic ranges with the same API as cf_adapter.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = simulation_adapter.sim_node:main',
            'sim_crazyflie = simulation_adapter.sim_crazyflie_node:main',
            'sim_sensors = simulation_adapter.sim_sensors_node:main',
        ],
    },
)
