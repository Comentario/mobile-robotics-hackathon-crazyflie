from setuptools import setup

package_name = 'simulation_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Alipázaga Lazo',
    maintainer_email='alipazaga.daniel@gmail.com',
    description='Simulador cinemático propio: publica pose y rangos sintéticos con la misma API que cf_adapter.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_node = simulation_adapter.sim_node:main',
            # Dron simulado con la API de crazyflie_server (Crazyswarm2).
            'sim_crazyflie = simulation_adapter.sim_crazyflie_node:main',
        ],
    },
)
