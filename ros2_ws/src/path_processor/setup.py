from setuptools import setup

package_name = 'path_processor'

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
    description='Turns the discrete route into executable references: simplification and spacing.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_processor_node = path_processor.path_processor_node:main',
        ],
    },
)
