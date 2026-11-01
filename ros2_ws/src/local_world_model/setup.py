from setuptools import setup

package_name = 'local_world_model'

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
    description='Rolling 2D occupancy grid around the drone, with obstacle inflation and decay.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_world_model_node = local_world_model.local_world_model_node:main',
        ],
    },
)
