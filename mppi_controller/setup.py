from setuptools import find_packages, setup

package_name = 'mppi_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mppi_controller_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stanislaw Piasecki',
    maintainer_email='spiasecki@ethz.ch',
    description='MPPI controller for a walking robot',
    license='BSD',
    entry_points={
        'console_scripts': [
            'mppi_controller_node = mppi_controller.main:main'
        ],
    },
)
