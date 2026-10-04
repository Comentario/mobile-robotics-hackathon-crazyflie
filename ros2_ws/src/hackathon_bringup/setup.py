from glob import glob

from setuptools import setup

package_name = 'hackathon_bringup'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/launch', glob('launch/*.launch.py')),
        (share + '/config', glob('config/*.yaml')),
        (share + '/config/worlds', glob('config/worlds/*.yaml')),
        (share + '/config/missions', glob('config/missions/*.yaml')),
        (share + '/config/scenarios', glob('config/scenarios/*.yaml')),
        (share + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Alipázaga Lazo',
    maintainer_email='alipazaga.daniel@gmail.com',
    description='Launch files y configuración del stack.',
    license='TODO',
    tests_require=['pytest'],
)
