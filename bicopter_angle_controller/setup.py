from setuptools import setup
from glob import glob
import os

package_name = 'bicopter_angle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebnerl',
    maintainer_email='ebnerl@ethz.ch',
    description='bicopter angle controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bicopter_angle_controller = bicopter_angle_controller.bicopter_angle_controller:main'
        ],
    },
)
