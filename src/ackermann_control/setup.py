from setuptools import find_packages, setup

package_name = 'ackermann_control'

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
    maintainer='Ashish',
    maintainer_email='krishnachinnari108@gmail.com',
    description='Ackermann motor control vis /cmd_vel',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_motor_controller = ackermann_control.ackermann_motor_controller:main',
        ],
    },
)
