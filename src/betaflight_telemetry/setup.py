from setuptools import find_packages, setup

package_name = 'betaflight_telemetry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simas',
    maintainer_email='simas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_node = betaflight_telemetry.telemetry_node:main',
            'telemetry_gui = betaflight_telemetry.telemetry_gui:main',
            'motor_setter = betaflight_telemetry.motor_setter:main',
            'rc_setter = betaflight_telemetry.rc_setter:main'
        ],
    },
)
