from setuptools import setup

package_name = 'telemetry'

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
    description='Métricas de la misión y resumen por ejecución para la demo y la test matrix.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_node = telemetry.telemetry_node:main',
        ],
    },
)
