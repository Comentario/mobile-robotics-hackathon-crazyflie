from setuptools import setup

package_name = 'safety_supervisor'

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
    description='Safety supervisor independent of the planner: it can override any command.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_supervisor_node = safety_supervisor.safety_supervisor_node:main',
        ],
    },
)
