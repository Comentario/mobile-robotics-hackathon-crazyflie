from setuptools import setup

package_name = 'global_planner'

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
    description='Global planner: direct route (P0) or A* over the local map (P1), with replanning.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_node = global_planner.global_planner_node:main',
        ],
    },
)
