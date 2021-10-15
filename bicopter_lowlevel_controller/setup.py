from setuptools import setup

package_name = 'bicopter_lowlevel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebnerl',
    maintainer_email='ebnerl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bicopter_lowlevel_controller_node = bicopter_lowlevel_controller.bicopter_lowlevel_controller_node:main',
        ],
    },
)
