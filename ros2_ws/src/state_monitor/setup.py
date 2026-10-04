from setuptools import setup

package_name = 'state_monitor'

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
    description='Normaliza el estado del dron, publica TF map->base_link y evalúa la salud de la localización.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_monitor_node = state_monitor.state_monitor_node:main',
        ],
    },
)
