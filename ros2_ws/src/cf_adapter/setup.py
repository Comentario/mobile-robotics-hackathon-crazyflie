from setuptools import setup

package_name = 'cf_adapter'

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
    description='Adapter between our own API (/hk/...) and Crazyswarm2. The only package that knows Crazyswarm2 names.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cf_adapter_node = cf_adapter.cf_adapter_node:main',
        ],
    },
)
