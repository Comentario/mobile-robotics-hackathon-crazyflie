from setuptools import setup

package_name = 'hackathon_tests'

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
    description='Evaluador de escenarios de simulación: compara métricas con el resultado esperado (pass/fail).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_scenario = hackathon_tests.run_scenario:main',
        ],
    },
)
